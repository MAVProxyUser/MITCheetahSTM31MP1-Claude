#!/usr/bin/env python3
"""
Conductor - a fleet control panel for the star/oval/atom/dash missions.

Cannibalised from a colleague's drone-swarm "Conductor" (parrot_CHUCK,
commit 69b8381: fleet lifecycle, restart-world, live status cards) - stealing
its dark ops-console look, its play-card/fleet-card vocabulary, and its
"freeze the config, then launch" discipline. Left out: everything that is
drone-specific and has no quadruped analogue - MAVLink, ArduPilot profiles,
wind corridors, IAMSAR search patterns, terrain/DEM, rail launch, camera
bridges. What's kept is the part that transfers directly: a small always-on
local server driving real subprocesses, with one page that shows what they
are doing right now.

DESIGN RULE, taken directly from this session's own post-mortem: a run must
be launched with ONE locked configuration and never touched again until it
finishes. /api/launch refuses a second call while a fleet is active - the
UI has no "edit mid-run" path at all, because "code changes mid-test mean
rerunning the test" is exactly the mistake this exists to make impossible.

Stdlib only. No build step, no framework - this repo's tests do not need one
more moving part than the C++ toolchain and Python venv it already has.
"""
import http.server
import json
import os
import re
import socketserver
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
GAZEBO_DIR = os.path.abspath(os.path.join(HERE, ".."))
REPO_ROOT = os.path.abspath(os.path.join(GAZEBO_DIR, "..", ".."))
HOST_RUN = os.path.join(REPO_ROOT, "host-run")
RUN_DIR = "/tmp/cheetah_conductor"
PARTITION = "cheetah_fleet"
PORT = 8420

PYBIN = ("/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/"
         "ground/gazebo_bridge/venv/bin/python3")
OPMODELS = ("/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/"
            "ground/gazebo_bridge/models")

# ---------------------------------------------------------------------------
# THE HARD SPEED CAP. Not a UI suggestion - enforced here, server-side, on
# every path that sets SIM_VX, because the browser's <input max> can be
# edited in devtools and the intent ("stop pushing toward peak spec") has to
# survive that. 4.7 m/s (17 km/h) is Unitree's sprint/marketing peak; 3.5-3.7
# is the real sustained envelope (Pro/EDU rated max). 3.9 leaves a hair of
# headroom over EDU's 3.7 without ever approaching the peak number.
# ---------------------------------------------------------------------------
HARD_SPEED_CAP = 3.9

# Per-mission-kind recipe: gait, and the extra knobs THIS SESSION measured to
# be the best validated configuration for that course shape (see CLAUDE.md,
# "CAMPAIGN RESULTS"). The UI can override gait and speed; it does not expose
# a way to change these, on purpose - they are not tunables, they are the
# answer.
RECIPES = {
    "star": dict(gait=5, speed=3.5, extra="WP_ACCEPT=1.5 WP_ALAT=3.25",
                 note="trotRunning, lateral budget 3.25 - 38.25s @ 6/6"),
    "oval": dict(gait=5, speed=3.5, extra="WP_ANALYZER=1 WP_VSUS=2.6",
                 note="trotRunning, analyzer, sustained cap 2.6 - 30.48s @ 6/6"),
    "atom": dict(gait=9, speed=2.1, extra="",
                 note="trotting, bare (analyzer adds nothing here) - 58.97s @ 6/6"),
    "dash": dict(gait=5, speed=3.0, extra="",
                 note="trotRunning straight-line - UNDER REVIEW, see README"),
}
GAITS = {"trotting": 9, "trotRunning": 5, "walking": 20, "walking2": 21, "pacing": 8}


def mission_kind(spec):
    return spec.split(":", 1)[0]


def clamp_speed(v, cap):
    v = max(0.3, min(float(v), HARD_SPEED_CAP))
    cap = max(0.3, min(float(cap), HARD_SPEED_CAP))
    return round(min(v, cap), 2)


# ---------------------------------------------------------------------------
# Fleet manager - owns every subprocess and the polled state derived from
# their logs.
# ---------------------------------------------------------------------------
class Fleet:
    def __init__(self):
        self.lock = threading.Lock()
        self.phase = "idle"          # idle | launching | running | done | error
        self.slots = []              # locked config, set at launch
        self.status = []             # per-slot live status, updated by poller
        self.log = []                # tail of orchestration events
        self.procs = []              # every Popen this run started
        self.started_at = None
        self._poll_thread = None

    def _note(self, msg):
        self.log.append("[%s] %s" % (time.strftime("%H:%M:%S"), msg))
        self.log = self.log[-200:]
        print(msg, flush=True)

    def snapshot(self):
        with self.lock:
            return {
                "phase": self.phase,
                "slots": self.slots,
                "status": self.status,
                "log": self.log[-60:],
                "hard_cap": HARD_SPEED_CAP,
                "recipes": RECIPES,
                "gaits": GAITS,
            }

    # ---- launch ----------------------------------------------------------
    def launch(self, slots, speed_cap):
        with self.lock:
            if self.phase in ("launching", "running"):
                return False, "a fleet is already active - stop it first"
            if not (1 <= len(slots) <= 3):
                return False, "1 to 3 slots only"
            self.phase = "launching"
            self.log = []
            self.procs = []

        # Freeze the configuration NOW. Nothing below this point reads the
        # request again - a second call while running is refused above.
        locked = []
        for i, s in enumerate(slots):
            spec = s["mission"]
            kind = mission_kind(spec)
            recipe = RECIPES.get(kind, dict(gait=5, speed=2.5, extra=""))
            gait = GAITS.get(s.get("gait", ""), recipe["gait"])
            speed = clamp_speed(s.get("speed", recipe["speed"]), speed_cap)
            locked.append(dict(index=i, mission=spec, kind=kind, gait=gait,
                                speed=speed, extra=recipe["extra"],
                                note=recipe["note"]))
        with self.lock:
            self.slots = locked
            self.status = [dict(index=s["index"], phase="pending", text="",
                                 t="", waypoints="") for s in locked]

        threading.Thread(target=self._run, args=(locked,), daemon=True).start()
        return True, "launching %d dog(s)" % len(locked)

    def _run(self, locked):
        try:
            os.makedirs(RUN_DIR, exist_ok=True)
            self._note("building fleet world: %s"
                        % ", ".join(s["mission"] for s in locked))
            world_out = os.path.join(RUN_DIR, "fleet.sdf")
            r = subprocess.run(
                [sys.executable, os.path.join(HERE, "fleet_world.py"),
                 os.path.join(GAZEBO_DIR, "worlds/go1_speedway.sdf"),
                 world_out] + [s["mission"] for s in locked],
                capture_output=True, text=True)
            if r.returncode != 0:
                self._note("world build FAILED: " + r.stderr[-500:])
                with self.lock:
                    self.phase = "error"
                return
            for line in r.stdout.strip().splitlines():
                self._note(line)

            env = os.environ.copy()
            env["GZ_SIM_RESOURCE_PATH"] = "%s/unitree_ros/robots:%s/models:%s" % (
                GAZEBO_DIR, GAZEBO_DIR, OPMODELS)
            env["GZ_PARTITION"] = PARTITION
            env["PATH"] = "/opt/homebrew/bin:" + env.get("PATH", "")

            self._note("starting Gazebo server")
            gz_log = open(os.path.join(RUN_DIR, "gz.log"), "w")
            p = subprocess.Popen(["gz", "sim", "-s", "-r", world_out],
                                  cwd=GAZEBO_DIR, env=env, stdout=gz_log,
                                  stderr=subprocess.STDOUT)
            self.procs.append(p)

            self._note("waiting for %d dog(s) to advertise sensors" % len(locked))
            deadline = time.time() + 30
            ready = set()
            while time.time() < deadline and len(ready) < len(locked):
                topics = subprocess.run(["gz", "topic", "-l"], env=env,
                                         capture_output=True, text=True,
                                         timeout=5).stdout
                for s in locked:
                    if "/go1_%d/imu" % s["index"] in topics:
                        ready.add(s["index"])
                time.sleep(1)
            if len(ready) < len(locked):
                self._note("only %d/%d dogs came up - continuing anyway"
                            % (len(ready), len(locked)))

            self._note("opening Gazebo GUI")
            gui_log = open(os.path.join(RUN_DIR, "gui.log"), "w")
            gp = subprocess.Popen(["gz", "sim", "-g"], cwd=GAZEBO_DIR, env=env,
                                   stdout=gui_log, stderr=subprocess.STDOUT)
            self.procs.append(gp)
            time.sleep(3)

            for s in locked:
                i = s["index"]
                name = "go1_%d" % i
                senv = env.copy()
                senv["SIM_INSTANCE"] = str(i)
                senv["SIM_MODEL"] = name
                blog = open(os.path.join(RUN_DIR, "bridge_%d.log" % i), "w")
                bp = subprocess.Popen(
                    [PYBIN, "-u", "cheetah_gazebo_bridge.py"],
                    cwd=GAZEBO_DIR, env=dict(senv, BRIDGE_CONV="mit"),
                    stdout=blog, stderr=subprocess.STDOUT)
                self.procs.append(bp)
                self._note("dog%d bridge up (%s, ports %d/%d)"
                            % (i, name, 9100 + 10 * i, 9101 + 10 * i))

            time.sleep(2)

            # Trail lane offsets must match fleet_world.py's own bin-packing
            # exactly, or the planned track is drawn under the wrong dog.
            sys.path.insert(0, HERE)
            from fleet_world import layout  # noqa: E402
            placed = layout([s["mission"] for s in locked])

            for s, (spec, north, east, bbox) in zip(locked, placed):
                i = s["index"]
                tenv = env.copy()
                tenv["SIM_MODEL"] = "go1_%d" % i
                tlog = open(os.path.join(RUN_DIR, "trail_%d.log" % i), "w")
                tp = subprocess.Popen(
                    [PYBIN, "-u", "trail_daemon.py", spec, "900",
                     str(i), str(east)],
                    cwd=GAZEBO_DIR, env=tenv, stdout=tlog, stderr=subprocess.STDOUT)
                self.procs.append(tp)

            with self.lock:
                self.phase = "running"
                self.started_at = time.time()

            for s in locked:
                i = s["index"]
                cenv = env.copy()
                cenv["SIM_INSTANCE"] = str(i)
                cmd = (
                    "env DYLD_LIBRARY_PATH=. SIM_INSTANCE=%d SIM_GAIT=%d SIM_VX=%s "
                    "SIM_VX_DELAY_S=4 SIM_VX_RAMP_S=8 WP_MISSION=%s WP_PLANNER=1 "
                    "WP_MAX_YAWRATE=1.2 %s timeout 220 ./mit_ctrl_sim 127.0.0.1 "
                    "stm32mp1-defaults.yaml mc-mit-ctrl-user-parameters.yaml"
                    % (i, s["gait"], s["speed"], s["mission"], s["extra"])
                )
                clog = open(os.path.join(RUN_DIR, "ctrl_%d.log" % i), "w")
                cp = subprocess.Popen(["bash", "-c", cmd], cwd=HOST_RUN,
                                       env=cenv, stdout=clog, stderr=subprocess.STDOUT)
                self.procs.append(cp)
                self._note("dog%d LOCKED: %s gait=%d cmd=%.2f m/s (cap %.2f) %s"
                            % (i, s["mission"], s["gait"], s["speed"],
                               HARD_SPEED_CAP, s["extra"]))

            self._start_poller(locked)

        except Exception as e:  # noqa: BLE001 - report, don't crash the server
            self._note("launch error: %r" % e)
            with self.lock:
                self.phase = "error"

    # ---- live status, parsed from the controller logs --------------------
    def _start_poller(self, locked):
        def poll():
            done = set()
            while True:
                with self.lock:
                    if self.phase not in ("running",):
                        return
                for s in locked:
                    i = s["index"]
                    if i in done:
                        continue
                    path = os.path.join(RUN_DIR, "ctrl_%d.log" % i)
                    try:
                        with open(path, errors="ignore") as f:
                            text = f.read()
                    except FileNotFoundError:
                        continue
                    st = dict(index=i, phase="running", text="", t="", waypoints="")
                    m = re.findall(r"\[nav\] wp(\d+)/(\d+).*?d=([\d.]+).*?v=([\d.]+)",
                                    text)
                    if m:
                        wp, tot, d, v = m[-1]
                        st["waypoints"] = "%s/%s" % (wp, tot)
                        st["text"] = "d=%sm v=%sm/s" % (d, v)
                    if "MISSION COMPLETE" in text:
                        tm = re.search(r"MISSION COMPLETE t=([\d.]+)s", text)
                        st["phase"] = "complete"
                        st["t"] = tm.group(1) + "s" if tm else ""
                        done.add(i)
                        self._note("dog%d COMPLETE t=%s" % (i, st["t"]))
                    elif "[FALL]" in text:
                        st["phase"] = "fell"
                        done.add(i)
                        self._note("dog%d FELL" % i)
                    with self.lock:
                        for j, old in enumerate(self.status):
                            if old["index"] == i:
                                self.status[j] = st
                if len(done) == len(locked):
                    with self.lock:
                        self.phase = "done"
                    self._note("fleet run complete")
                    return
                time.sleep(1)
        threading.Thread(target=poll, daemon=True).start()

    # ---- teardown ----------------------------------------------------------
    def stop(self):
        with self.lock:
            procs = list(self.procs)
            self.procs = []
            self.phase = "idle"
        for p in procs:
            try:
                p.terminate()
            except Exception:  # noqa: BLE001
                pass
        time.sleep(1)
        for p in procs:
            try:
                p.kill()
            except Exception:  # noqa: BLE001
                pass
        self._note("fleet stopped")


FLEET = Fleet()


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=os.path.join(HERE, "static"), **kw)

    def log_message(self, fmt, *args):
        pass  # the on-page log is the log that matters here

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/api/state":
            return self._json(FLEET.snapshot())
        return super().do_GET()

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length else b"{}"
        try:
            body = json.loads(raw)
        except json.JSONDecodeError:
            return self._json({"ok": False, "error": "bad json"}, 400)

        if self.path == "/api/launch":
            ok, msg = FLEET.launch(body.get("slots", []),
                                    body.get("speed_cap", HARD_SPEED_CAP))
            return self._json({"ok": ok, "message": msg})
        if self.path == "/api/stop":
            FLEET.stop()
            return self._json({"ok": True})
        return self._json({"ok": False, "error": "no such route"}, 404)


def main():
    os.makedirs(RUN_DIR, exist_ok=True)
    with socketserver.ThreadingTCPServer(("127.0.0.1", PORT), Handler) as httpd:
        print("Conductor on http://127.0.0.1:%d" % PORT, flush=True)
        httpd.serve_forever()


if __name__ == "__main__":
    main()
