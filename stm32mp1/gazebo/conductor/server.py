#!/usr/bin/env python3
"""
Conductor - a fleet control panel for the star/oval/atom/dash missions.

Cannibalised from a colleague's drone-swarm "Conductor" (parrot_CHUCK,
commit 69b8381: fleet lifecycle, restart-world, live status cards) - stealing
its dark ops-console look, its play-card/fleet-card vocabulary, its
"freeze the config, then launch" discipline, and (per-dog front + nadir camera
tiles) its `.ahrs-cam` bring-up. Left out: everything drone-specific with no
quadruped analogue - MAVLink, ArduPilot profiles, wind corridors, IAMSAR
search patterns, rail launch.

RENDERING, v2. The first version drove `gz sim -g` (the native GUI) and had
trail_daemon.py post gz Markers into that scene - i.e. it still asked Gazebo
to do the showing. The first live 3-dog run under that setup collapsed all
three dogs, and the leading suspect was exactly that: GUI render load and
marker-service traffic landing on the SAME headless engine the controllers
depend on for physics, in a configuration (3 heterogeneous missions, one
engine) never tried before. So: Conductor now does its OWN rendering, the way
the original parrot_CHUCK actually works (`#flock-map`, drawn in the page,
not the sim's window) - it drives `gz sim -s -r` HEADLESS and nothing else
touches that process's transport bus for visualization. This server (running
under the venv Python so it can use gz.transport13 directly) subscribes to
world pose itself, in-process, and serves positions + the precomputed planned
paths as JSON; the browser draws both on a canvas. Gazebo now only ever does
physics for this tool.

What's kept from the original is the part that transfers directly regardless
of rendering strategy: a small always-on local server driving real
subprocesses, with one page that shows what they are doing right now.

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
import math
import os
import re
import socketserver
import subprocess
import sys
import threading
import time
import urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
GAZEBO_DIR = os.path.abspath(os.path.join(HERE, ".."))
REPO_ROOT = os.path.abspath(os.path.join(GAZEBO_DIR, "..", ".."))
HOST_RUN = os.path.join(REPO_ROOT, "host-run")
RUN_DIR = "/tmp/cheetah_conductor"
PARTITION = "cheetah_fleet"
WORLD = "go1_world"
PORT = 8420

PYBIN = ("/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/"
         "ground/gazebo_bridge/venv/bin/python3")
OPMODELS = ("/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/"
            "ground/gazebo_bridge/models")

# GZ_PARTITION has to be set in THIS process's own environment before the
# transport library initialises, or the Node this server creates for its own
# pose subscription lands on the default partition while gz sim runs on
# "cheetah_fleet" - they never see each other's traffic. Every subprocess
# below gets GZ_PARTITION through its own env dict; this line is what makes
# the SERVER's in-process subscriber agree with them. Found by testing: the
# topic was confirmed alive and publishing correctly, this process's own
# subscription just never received anything - a partition mismatch prints no
# error on either side, so it must be reasoned about, not caught.
os.environ["GZ_PARTITION"] = PARTITION

# mission_waypoints() is pure geometry with no gz imports triggered at call
# time, but the FILE it lives in imports gz.transport13 at module level - safe
# to import here only because this server now runs under PYBIN (see
# conductor.sh), which has those bindings. Do not run this under system
# python3 any more.
sys.path.insert(0, GAZEBO_DIR)
from trail_daemon import mission_waypoints  # noqa: E402
import gz.transport13 as _gz_transport      # noqa: E402
from gz.msgs10.pose_v_pb2 import Pose_V     # noqa: E402
from gz.msgs10.image_pb2 import Image as _GzImage  # noqa: E402
import base64                               # noqa: E402
import io                                   # noqa: E402
from PIL import Image as _PILImage          # noqa: E402

CAMERAS = ("front_cam", "nadir_cam", "chase_cam")
CAM_FLAG_KEYS = {"front_cam": "cam_front", "nadir_cam": "cam_nadir", "chase_cam": "cam_chase"}
# Cameras default OFF. Nine live feeds were repeatedly implicated in
# host-load fleet failures (GPU 44-49% with them, 0% without), and a server
# restart used to silently reset drafts back to all-on - which re-armed
# them under the operator mid-session ("KILL THE CAMERAS"). Fail dark;
# checking a box is deliberate.
DEFAULT_CAM_SLOT = dict(cam_front=False, cam_nadir=False, cam_chase=False,
                         chase_distance=3.0, chase_height=1.2, chase_degree=90.0)
import terrain  # noqa: E402 - conductor/terrain.py, procedural heightmaps

# ---------------------------------------------------------------------------
# THE HARD SPEED CAP. Not a UI suggestion - enforced here, server-side, on
# every path that sets SIM_VX, because the browser's <input max> can be
# edited in devtools and the intent ("stop pushing toward peak spec") has to
# survive that. 4.7 m/s (17 km/h) is Unitree's sprint/marketing peak; 3.5-3.7
# is the real sustained envelope (Pro/EDU rated max). 3.9 leaves a hair of
# headroom over EDU's 3.7 without ever approaching the peak number.
# ---------------------------------------------------------------------------
HARD_SPEED_CAP = 3.9

# Per-model hard cap, independent of and in ADDITION to HARD_SPEED_CAP above -
# a real Go1 Air cannot do 3.5 m/s no matter what the mission asks for, so
# picking a model must clamp the same way changing the mission's speed field
# does, enforced here server-side (never trust the browser alone). EDU's
# figure is Unitree's own "peak sprint, optimised test conditions" number
# (17 km/h) rather than its sustained rating (3.7) - kept as the ceiling
# because it is still below HARD_SPEED_CAP's ceiling and a user who
# explicitly picks EDU has already opted into its top end.
MODEL_MAX_SPEED = {"air": 2.5, "pro": 3.5, "edu": 4.7}
DEFAULT_MODEL = "edu"

# Per-mission-kind recipe: gait, and the extra knobs THIS SESSION measured to
# be the best validated configuration for that course shape (see CLAUDE.md,
# "CAMPAIGN RESULTS"). The UI can override gait and speed; it does not expose
# a way to change these, on purpose - they are not tunables, they are the
# answer.
RECIPES = {
    "star": dict(gait=5, speed=3.5, extra="WP_ACCEPT=1.5 WP_ALAT=3.25 WP_CORRIDOR_MIN=0.1",
                 note="trotRunning, lateral budget 3.25, graded corridor - UNDER RE-TUNE"),
    # VSUS 2.6 was the campaign-era edge and no longer holds on the current
    # dynamics lineage: a controlled bisect (last night's binary vs HEAD,
    # semi-interleaved, solo) showed BOTH fall mid-180 at ~50-75% with the
    # yaw command saturated at the lateral cap - the cell was always
    # marginal, the old 6/6 belongs to a much older build. 2.4 is the
    # re-measured envelope: its first run under the operator's own eyes ran
    # the curves clean AND the full stop/lie-down/stand-up/dash sequence.
    "oval": dict(gait=5, speed=3.5, extra="WP_ANALYZER=1 WP_VSUS=2.4",
                 note="trotRunning, analyzer, sustained cap 2.4 (re-swept 2026-08-24; 2.6 is past the current envelope)"),
    "atom": dict(gait=9, speed=2.1, extra="",
                 note="trotting, bare (analyzer adds nothing here) - 58.97s @ 6/6"),
    "dash": dict(gait=5, speed=3.0, extra="",
                 note="trotRunning straight-line - UNDER REVIEW, see README"),
}
GAITS = {"trotting": 9, "trotRunning": 5, "walking": 20, "walking2": 21, "pacing": 8}
GAIT_NAMES = {v: k for k, v in GAITS.items()}


def _gname(num_str):
    try:
        return GAIT_NAMES.get(int(num_str), num_str)
    except ValueError:
        return num_str


# Discrete events worth a line in the orchestration log, matched against NEW
# ctrl_%d.log text each poll tick (see _start_poller). This is deliberately
# NOT the continuous per-tick telemetry ([nav] wp.../d=.../v=... already
# drives the live fleet-card text) - only state transitions: init, standing
# up, gait engagement, mission pre-planning, gait switches, the dash
# interlude, and anything that trips a safety/governor path. Each entry is
# (compiled regex, formatter(match) -> message). Order matters only in that
# earlier patterns are tried first per line; every line matches at most one.
EVENT_PATTERNS = [
    (re.compile(r"\[mit-sim\] Gazebo SITL \| peer=(\S+) robot=(\S+) user=(\S+)"),
     lambda m: "initialising Cheetah MIT controller (peer=%s, %s / %s)" % m.groups()),
    (re.compile(r"\[stm32mp1\] REAL ESTIMATOR: (.+)"),
     lambda m: "estimator: %s" % m.group(1)),
    (re.compile(r"\[sim\] control_mode -> STAND_UP"),
     lambda m: "standing up"),
    (re.compile(r"\[sim\] control_mode -> BALANCE_STAND"),
     lambda m: "balance stand - settling before locomotion"),
    (re.compile(r"\[sim\] control_mode -> 4\b"),
     lambda m: "entering LOCOMOTION"),
    (re.compile(r"\[nav\] LOCOMOTION engaged - holding ([\d.]+) s"),
     lambda m: "gait engaged - holding %ss for it to settle before nav takes over" % m.group(1)),
    (re.compile(r"\[nav\] taking the stick at t=([\d.]+)s \(mission (\S+)\)"),
     lambda m: "nav taking the stick - mission %s under way" % m.group(2)),
    (re.compile(r"\[plan\] (\d+) pts, ([\d.]+) m, tightest R=([\d.]+) m -> ([\d.]+) m/s "
                r"\(cruise ([\d.]+), a_lat ([\d.]+), corridor ([\d.]+)\)"),
     lambda m: ("pre-planning mission: %s pts, %sm path, tightest corner "
                "R=%sm -> %sm/s (cruise %sm/s, a_lat %s)" % m.groups()[:6])),
    (re.compile(r"\[mission\] (\d+) segments over ([\d.]+) m"),
     lambda m: "mission analyzer: %s segments over %sm" % m.groups()),
    (re.compile(r"\[mission\] ([\d.]+) s lost to constraints; costliest FEATURE "
                r"([+\-\d.]+) s at s=([\d.]+) \((\S+), R=([\d.]+) m\)"),
     lambda m: ("analyzer: %ss lost to constraints - costliest feature %ss at "
                "s=%sm (%s, R=%sm)" % m.groups())),
    (re.compile(r"\[mission\] (\d+) sustained-curve segments, (\d+) gait changes planned"),
     lambda m: "analyzer plan: %s sustained-curve segments, %s gait changes scheduled" % m.groups()),
    (re.compile(r"\[gait\] (\d+) -> (\d+) at v=([\d.]+) \(planned ([\d.]+)\) t=([\d.]+)s"),
     lambda m: "gait change: %s -> %s at v=%sm/s (planned %sm/s)" % (
         _gname(m.group(1)), _gname(m.group(2)), m.group(3), m.group(4))),
    (re.compile(r"\[mission\] gait (\d+) -> (\d+) entering (\S+) at s=([\d.]+) "
                r"\(R=([\d.]+), cost ([+\-\d.]+) s\) t=([\d.]+)s"),
     lambda m: "gait change (pre-planned): %s -> %s entering %s terrain (R=%sm)" % (
         _gname(m.group(1)), _gname(m.group(2)), m.group(3), m.group(5))),
    (re.compile(r"\[HGOV\] .*dep=([+\-\d.]+)"),
     lambda m: "height governor active (departure %sm) - trading speed for stance height" % m.group(1)),
    (re.compile(r"\[nav\] loop complete at t=([\d.]+)s - stop, lie down, "
                r"stand back up before the (\S+) dash"),
     lambda m: "loop complete - stopping, lying down before the dash finish"),
    (re.compile(r"\[nav\] back up at t=([\d.]+)s - dashing the final leg"),
     lambda m: "back on its feet - dashing the final leg"),
    (re.compile(r"\[mission\] settle: z=([\d.]+) roll=([\d.]+) pitch=([\d.]+) -> (ok|BAD)"),
     lambda m: "settled on its feet: z=%sm roll=%s pitch=%s -> %s" % m.groups()),
    (re.compile(r"\[mission\] laydown: z=([\d.]+) roll=([\d.]+) pitch=([\d.]+) -> (ok|BAD)"),
     lambda m: "lying down: z=%sm roll=%s pitch=%s -> %s" % m.groups()),
    (re.compile(r"\[mission\] RESULT: (PASS|FAIL)"),
     lambda m: "mission result: %s" % m.group(1)),
    (re.compile(r"Orientation safety check failed!"),
     lambda m: "SAFETY CHECK FAILED - orientation exceeded the trip limit"),
    (re.compile(r"STATE ESTIMATE WENT NON-FINITE"),
     lambda m: "state estimate went non-finite - reinitialising"),
]


def mission_kind(spec):
    return spec.split(":", 1)[0]


def clamp_speed(v, cap, model=None):
    v = max(0.3, min(float(v), HARD_SPEED_CAP))
    cap = max(0.3, min(float(cap), HARD_SPEED_CAP))
    if model is not None:
        cap = min(cap, MODEL_MAX_SPEED.get(model, HARD_SPEED_CAP))
    return round(min(v, cap), 2)


def read_host_load():
    """CPU 1-minute load average (normalised by core count, as a 0-100+ %
    figure - can exceed 100 under real contention, which is the honest
    answer) and GPU device utilisation, both stdlib/no-sudo on macOS. Called
    from a background sampler, not per-request - ioreg is a subprocess spawn,
    cheap but not free, and nothing about this needs sub-second freshness.
    Same motivation as the MISSED CONTROL DEADLINES section in CLAUDE.md:
    this project has already been bitten once by machine load silently
    explaining an "unexplained" sim failure, so make it visible instead of
    assumed quiet."""
    try:
        load1, _, _ = os.getloadavg()
        cpu_pct = round(100.0 * load1 / max(1, os.cpu_count()), 1)
    except (OSError, AttributeError):
        cpu_pct = None
    gpu_pct = None
    try:
        out = subprocess.run(["ioreg", "-r", "-d", "1", "-c", "IOAccelerator"],
                              capture_output=True, text=True, timeout=2).stdout
        m = re.search(r'"Device Utilization %"\s*=\s*(\d+)', out)
        if m:
            gpu_pct = int(m.group(1))
    except (subprocess.SubprocessError, OSError):
        pass
    return dict(cpu_pct=cpu_pct, gpu_pct=gpu_pct)


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
        self.planned = {}            # index -> [[x,y], ...] world frame, frozen at launch
        self.positions = {}          # index -> {"x","y","z","yaw","speed","trail":[[x,y],...]}
        self._name_to_index = {}     # "go1_2" -> 2, for the pose subscriber
        self._last_pose = {}         # index -> (x, y, t) for the speed EMA
        self._gz_node = None         # keep the Node alive - gc'ing it drops the subscription
        # DRAFT config, mutated by /api/slots* and /api/speed_cap. This is
        # what "+ Add dog", editing a field, or dragging the cap slider does
        # in the browser - moved server-side so every one of those actions is
        # independently REST-automatable, not just the final Launch call.
        # dash=100: per direct instruction, the 100 m dash is how a course
        # ENDS, not an opt-in toggle - stop, lie down, stand back up, then
        # sprint 100 m on the closing heading, after every loop mission.
        # cam_front/cam_nadir/cam_chase default all-on (matches what already
        # shipped); chase_distance/height/degree are the side-view default
        # ("side view chase camera... hover over the dogs chasing").
        self.draft_slots = [
            dict(mission="star:10.514:5", gait="trotRunning", speed=3.5, dash=100,
                 model=DEFAULT_MODEL, **DEFAULT_CAM_SLOT),
            dict(mission="oval:40:5.0", gait="trotRunning", speed=3.5, dash=100,
                 model=DEFAULT_MODEL, **DEFAULT_CAM_SLOT),
            dict(mission="atom:9.0:6", gait="trotting", speed=2.1, dash=100,
                 model=DEFAULT_MODEL, **DEFAULT_CAM_SLOT),
        ]
        self.draft_cap = 3.5
        # Terrain, from terrain.py. "flat" reproduces the EXACT ground_plane
        # every campaign result was measured on; anything else is new,
        # unvalidated ground and stays opt-in for exactly that reason.
        self.draft_terrain = "flat"
        self.cameras = {}             # index -> {"front_cam": "data:...", "nadir_cam": "data:..."}
        self._gz_cam_nodes = []       # one Node per camera subscription, kept alive
        self.host_load = read_host_load()
        threading.Thread(target=self._host_load_loop, daemon=True).start()

    def _host_load_loop(self):
        while True:
            load = read_host_load()
            with self.lock:
                self.host_load = load
            time.sleep(1.5)

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
                "model_max_speed": MODEL_MAX_SPEED,
                "recipes": RECIPES,
                "gaits": GAITS,
                "planned": self.planned,
                "positions": self.positions,
                "draft_slots": self.draft_slots,
                "draft_cap": self.draft_cap,
                "draft_terrain": self.draft_terrain,
                "terrain_types": terrain.TERRAIN_TYPES,
                "cameras": self.cameras,
                "host_load": self.host_load,
            }

    # ---- draft editing: one method per interactive element ---------------
    # Every one of these is what a click/edit in the browser does; each is
    # also its own REST route, so the panel and a script hit the exact same
    # code path and can never drift apart.
    def draft_add_slot(self):
        with self.lock:
            if len(self.draft_slots) >= 3:
                return False, "max 3 slots"
            used = {mission_kind(s["mission"]) for s in self.draft_slots}
            nxt = next((k for k in RECIPES if k != "dash" and k not in used),
                       "star")
            r = RECIPES[nxt]
            self.draft_slots.append(dict(
                mission={"star": "star:10.514:5", "oval": "oval:40:5.0",
                         "atom": "atom:9.0:6"}.get(nxt, "star:10.514:5"),
                gait=next(g for g, n in GAITS.items() if n == r["gait"]),
                speed=r["speed"], dash=100, model=DEFAULT_MODEL, **DEFAULT_CAM_SLOT))
            return True, self.draft_slots

    def draft_remove_slot(self, i):
        with self.lock:
            if not (0 <= i < len(self.draft_slots)):
                return False, "no such slot"
            if len(self.draft_slots) <= 1:
                return False, "at least one slot required"
            self.draft_slots.pop(i)
            return True, self.draft_slots

    def draft_set_slot(self, i, fields):
        with self.lock:
            if not (0 <= i < len(self.draft_slots)):
                return False, "no such slot"
            s = self.draft_slots[i]
            if "mission" in fields:
                s["mission"] = str(fields["mission"])
            if "gait" in fields and fields["gait"] in GAITS:
                s["gait"] = fields["gait"]
            if "model" in fields and fields["model"] in MODEL_MAX_SPEED:
                s["model"] = fields["model"]
                # Re-clamp immediately - switching to a slower model with an
                # already-fast speed set must visibly drop it, not silently
                # cap it only at launch time.
                s["speed"] = clamp_speed(s["speed"], HARD_SPEED_CAP, s["model"])
            if "speed" in fields:
                s["speed"] = clamp_speed(fields["speed"], HARD_SPEED_CAP, s.get("model", DEFAULT_MODEL))
            if "dash" in fields:
                # 0/None/"" = no finish. The loop mission is unchanged either
                # way - this only appends one more waypoint after it closes.
                try:
                    v = float(fields["dash"])
                except (TypeError, ValueError):
                    v = 0.0
                s["dash"] = max(0.0, min(v, 200.0))
            for flag in ("cam_front", "cam_nadir", "cam_chase"):
                if flag in fields:
                    s[flag] = bool(fields[flag])
            for key, lo, hi in (("chase_distance", 0.5, 10.0),
                                 ("chase_height", 0.1, 5.0),
                                 ("chase_degree", -360.0, 360.0)):
                if key in fields:
                    try:
                        s[key] = max(lo, min(float(fields[key]), hi))
                    except (TypeError, ValueError):
                        pass
            if "extra" in fields:
                # Per-slot env overrides, appended AFTER the recipe's extra so
                # they win (env A=1 A=2 keeps the last). Needed for A/B work
                # (e.g. WP_VSUS sweeps) without editing RECIPES. STRICTLY
                # token-validated - this string is interpolated into a
                # bash -c command line, so only bare KEY=VALUE survives.
                toks = str(fields["extra"]).split()
                ok_toks = [t for t in toks
                           if re.fullmatch(r"[A-Z][A-Z0-9_]*=[-A-Za-z0-9_.:]*", t)]
                s["extra"] = " ".join(ok_toks)
            return True, s

    def draft_set_cap(self, value):
        with self.lock:
            self.draft_cap = max(0.3, min(float(value), HARD_SPEED_CAP))
            return True, self.draft_cap

    def draft_set_terrain(self, kind):
        with self.lock:
            if kind not in terrain.TERRAIN_TYPES:
                return False, "unknown terrain %r - choices: %s" % (
                    kind, list(terrain.TERRAIN_TYPES))
            self.draft_terrain = kind
            return True, self.draft_terrain

    # ---- launch ----------------------------------------------------------
    def launch(self, slots=None, speed_cap=None, terrain_kind=None):
        # No body (or an empty one) launches whatever is currently in the
        # draft - i.e. exactly what the panel is showing right now. An
        # explicit body still works for direct automation that wants to skip
        # the draft entirely and specify a full config in one call.
        with self.lock:
            if slots is None:
                slots = self.draft_slots
            if speed_cap is None:
                speed_cap = self.draft_cap
            if terrain_kind is None:
                terrain_kind = self.draft_terrain
            if terrain_kind not in terrain.TERRAIN_TYPES:
                return False, "unknown terrain %r" % terrain_kind
            if self.phase in ("launching", "running"):
                return False, "a fleet is already active - stop it first"
            # TIME MACHINE GATE (operator-diagnosed): hourly backups on this
            # Mac start around :38, and the two same-wall-second multi-dog
            # stall kills landed at 14:38:09 and 16:44:09 (six minutes into
            # the 16:38 backup). A backup's I/O burst is exactly the kind of
            # host stall that applies 2 ms control forces for 18 ms. Refuse
            # to launch while one is in flight; `sudo tmutil disable` (or
            # `tmutil stopbackup`) is the operator-side fix for a session.
            try:
                tm = subprocess.run(["tmutil", "status"], capture_output=True,
                                     text=True, timeout=5).stdout
                if "Running = 1" in tm:
                    return False, ("Time Machine backup in progress - it has "
                                   "killed fleets before (I/O stall = 9x force "
                                   "impulse). Wait for it, or `tmutil "
                                   "stopbackup` / `sudo tmutil disable`.")
            except Exception:  # noqa: BLE001 - tmutil missing/slow never blocks
                pass
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
            gait_name = s.get("gait") if s.get("gait") in GAITS else next(
                (g for g, n in GAITS.items() if n == gait), str(gait))
            # Model cap is enforced HERE, not only in draft_set_slot - this is
            # the one path that cannot be bypassed by a direct /api/launch
            # call carrying its own "slots" body with an unclamped speed.
            speed = clamp_speed(s.get("speed", recipe["speed"]), speed_cap,
                                 s.get("model", DEFAULT_MODEL))
            # Slot-level overrides go AFTER the recipe's own extra: with
            # `env A=1 A=2 cmd` the later assignment wins, so a slot can
            # override a recipe knob (WP_VSUS etc.) for A/B work.
            extra = (recipe["extra"] + " " + str(s.get("extra") or "")).strip()
            dash = float(s.get("dash") or 0.0)
            if dash > 0:
                # Appended after whatever the recipe's own mission builds -
                # the loop runs exactly as validated, then keeps going straight
                # for `dash` more metres on the heading it closes on.
                extra = (extra + " WP_DASH=%.1f" % dash).strip()
            locked.append(dict(index=i, mission=spec, kind=kind, gait=gait,
                                gait_name=gait_name, speed=speed, extra=extra,
                                dash=dash, note=recipe["note"],
                                cam_front=bool(s.get("cam_front", True)),
                                cam_nadir=bool(s.get("cam_nadir", True)),
                                cam_chase=bool(s.get("cam_chase", True)),
                                chase_distance=float(s.get("chase_distance", 3.0)),
                                chase_height=float(s.get("chase_height", 1.2)),
                                chase_degree=float(s.get("chase_degree", 90.0))))
        with self.lock:
            self.slots = locked
            self.status = [dict(index=s["index"], phase="pending", text="",
                                 t="", waypoints="") for s in locked]
            self.planned = {}
            self.positions = {}

        threading.Thread(target=self._run, args=(locked, terrain_kind),
                         daemon=True).start()
        return True, "launching %d dog(s) on %s terrain" % (len(locked), terrain_kind)

    def _run(self, locked, terrain_kind="flat"):
        try:
            os.makedirs(RUN_DIR, exist_ok=True)
            self._note("building fleet world: %s (terrain=%s)"
                        % (", ".join(s["mission"] for s in locked), terrain_kind))
            world_out = os.path.join(RUN_DIR, "fleet.sdf")
            cam_cfgs = [dict(front=s["cam_front"], nadir=s["cam_nadir"],
                              chase=s["cam_chase"], distance=s["chase_distance"],
                              height=s["chase_height"], degree=s["chase_degree"])
                        for s in locked]
            r = subprocess.run(
                [sys.executable, os.path.join(HERE, "fleet_world.py"),
                 "--terrain=%s" % terrain_kind,
                 "--cam_config=%s" % json.dumps(cam_cfgs),
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

            # Planned paths, frozen now, in WORLD frame (local mission coords
            # + this slot's own spawn offset from the SAME layout() call
            # fleet_world.py used to place the robots - so a drawn path lines
            # up with where the robot actually is, not an independently
            # recomputed guess). Locked before anything moves, matching the
            # "config is frozen at launch" rule for everything else here.
            sys.path.insert(0, HERE)
            from fleet_world import layout  # noqa: E402
            placed = layout([s["mission"] for s in locked])
            planned = {}
            for s, (spec, north, east, bbox) in zip(locked, placed):
                pts = mission_waypoints(spec)
                if s.get("dash") and len(pts) >= 2:
                    # Mirrors WaypointNav::appendDash() exactly (same source
                    # of truth this whole overlay already leans on) so the
                    # dim planned line shows the finish before the dog ever
                    # runs it, not just after, in the flown trail. TWO points,
                    # not one: an explicit return to wp0 (closing the shape
                    # for real - "it never went back to the first waypoint"),
                    # then the dash outward from there. Heading for the final
                    # leg is last-waypoint -> wp0 (the course's own closing
                    # tangent), computed before appending wp0 since it's the
                    # same two points either way - see the C++ comment for why
                    # not the raw final-leg vector (a star tip's own heading
                    # shoots the dash out at the tip's oblique angle instead
                    # of continuing the shape).
                    (n2, e2), (n0, e0) = pts[-1], pts[0]
                    dn, de = n0 - n2, e0 - e2
                    length = math.hypot(dn, de)
                    if length > 1e-3:
                        dn, de = dn / length, de / length
                        dash = float(s["dash"])
                        pts = pts + [(n0, e0), (n0 + dn * dash, e0 + de * dash)]
                planned[s["index"]] = [[e + east, n + north] for (n, e) in pts]
            with self.lock:
                self.planned = planned

            # STRAGGLER GUARD. A leftover `gz sim` from a previous launch shares
            # this fixed PARTITION and keeps publishing its OWN "go1_0" etc. on
            # the same pose topic - measured consequence: two zombie sims left
            # a trail buffer alternating between two frozen points, which
            # rendered as a scrambled scribble that looked like a tracking bug
            # and was not one. Refuse to proceed until nothing is left.
            for i in range(20):
                stale = subprocess.run(
                    ["pgrep", "-f", "gz[ ]sim -s -r"],
                    capture_output=True, text=True).stdout.split()
                if not stale:
                    break
                for pid in stale:
                    subprocess.run(["kill", "-9", pid], capture_output=True)
                time.sleep(0.5)
            else:
                self._note("could not clear a stale gz sim - aborting launch")
                with self.lock:
                    self.phase = "error"
                return

            env = os.environ.copy()
            env["GZ_SIM_RESOURCE_PATH"] = "%s/unitree_ros/robots:%s/models:%s" % (
                GAZEBO_DIR, GAZEBO_DIR, OPMODELS)
            env["GZ_PARTITION"] = PARTITION
            env["PATH"] = "/opt/homebrew/bin:" + env.get("PATH", "")

            self._note("starting Gazebo HEADLESS (no GUI, no marker traffic - "
                       "this page renders the fleet itself)")
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

            self._name_to_index = {"go1_%d" % s["index"]: s["index"] for s in locked}
            self._subscribe_pose(env)
            self._subscribe_cameras(locked)

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

            with self.lock:
                self.phase = "running"
                self.started_at = time.time()

            for s in locked:
                i = s["index"]
                cenv = env.copy()
                cenv["SIM_INSTANCE"] = str(i)
                # STAGGER THE RAMP ACROSS THE FLEET. Every dog used a fixed
                # 4 s delay, so N dogs launched together reached commanded
                # cruise in the SAME instant - and gz is ONE process serving
                # all of them, so that instant is the fleet's peak physics
                # demand. Measured 2026-08-24 18:28: three dogs, three
                # different courses, three different waypoints, all down at
                # t=12.4-12.5 s of their own clocks, with every per-dog
                # instrument clean (loop max 3.0 ms, 0 over-4 ms, bridges at
                # a steady 500 cmd/s). Per-dog instruments CANNOT see a
                # shared-simulator stall: the control loops are wall-clock
                # timed, so when physics falls behind, forces computed for a
                # 2 ms step get integrated over a longer one and every dog
                # is hit at once. Staggering by 5 s per slot spreads that
                # peak; a solo dog is unaffected (i=0 keeps the validated 4 s).
                delay_s = 4 + 5 * i
                cmd = (
                    "env DYLD_LIBRARY_PATH=. SIM_INSTANCE=%d SIM_GAIT=%d SIM_VX=%s "
                    "SIM_VX_DELAY_S=%d SIM_VX_RAMP_S=8 WP_MISSION=%s WP_PLANNER=1 "
                    "WP_MAX_YAWRATE=1.2 %s timeout 240 ./mit_ctrl_sim 127.0.0.1 "
                    "stm32mp1-defaults.yaml mc-mit-ctrl-user-parameters.yaml"
                    % (i, s["gait"], s["speed"], delay_s, s["mission"], s["extra"])
                )
                clog = open(os.path.join(RUN_DIR, "ctrl_%d.log" % i), "w")
                cp = subprocess.Popen(["bash", "-c", cmd], cwd=HOST_RUN,
                                       env=cenv, stdout=clog, stderr=subprocess.STDOUT)
                self.procs.append(cp)
                dash_note = (" +dash %.0fm" % s["dash"]) if s.get("dash") else ""
                self._note("dog%d LOCKED: %s gait=%s cmd=%.2f m/s (cap %.2f) %s%s"
                            % (i, s["mission"], s["gait_name"], s["speed"],
                               HARD_SPEED_CAP, s["extra"], dash_note))

            self._start_poller(locked)

        except Exception as e:  # noqa: BLE001 - report, don't crash the server
            self._note("launch error: %r" % e)
            with self.lock:
                self.phase = "error"

    # ---- Conductor's OWN rendering: subscribe to world pose ourselves -----
    def _subscribe_pose(self, env):
        """One subscription covers every dog - Gazebo publishes ALL models'
        poses in a single message on this topic (trail_daemon.py already
        relied on that, filtering to one name; here we keep every name we
        placed). Runs for the life of the fleet; gz.transport13 fires the
        callback on its own thread, so this just has to register once."""
        node = _gz_transport.Node()
        self._gz_node = node  # keep alive - a gc'd Node drops the subscription
        SEG_MIN = 0.15        # m between recorded trail points, matches
                               # trail_daemon's own decimation logic
        TRAIL_MAX = 4000

        def on_pose(msg):
            now = time.time()
            with self.lock:
                for p in msg.pose:
                    idx = self._name_to_index.get(p.name)
                    if idx is None:
                        continue
                    x, y, z = p.position.x, p.position.y, p.position.z
                    # orientation is a quaternion, not Euler - yaw about world Z
                    o = p.orientation
                    yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                                      1.0 - 2.0 * (o.y * o.y + o.z * o.z))
                    # Ground speed from consecutive fixes, not a field Pose_V
                    # carries - lightly EMA'd (alpha 0.3) so the per-dog
                    # readout does not flicker at whatever rate gz publishes,
                    # while staying responsive enough to show a real change.
                    speed = 0.0
                    prev_t = self._last_pose.get(idx)
                    cur = self.positions.get(idx)
                    if prev_t is not None:
                        px, py, pt = prev_t
                        dt_s = now - pt
                        if 1e-3 < dt_s < 1.0:  # skip the first fix after a launch/gap
                            raw = math.hypot(x - px, y - py) / dt_s
                            prev_speed = cur["speed"] if cur else raw
                            speed = 0.3 * raw + 0.7 * prev_speed
                    self._last_pose[idx] = (x, y, now)
                    trail = cur["trail"] if cur else []
                    if not trail or ((x - trail[-1][0]) ** 2
                                      + (y - trail[-1][1]) ** 2) >= SEG_MIN ** 2:
                        trail = (trail + [[round(x, 3), round(y, 3)]])[-TRAIL_MAX:]
                    self.positions[idx] = dict(x=round(x, 3), y=round(y, 3),
                                                z=round(z, 3), yaw=round(yaw, 3),
                                                speed=round(speed, 2), trail=trail)

        ok = node.subscribe(Pose_V, "/world/%s/dynamic_pose/info" % WORLD, on_pose)
        self._note("pose subscriber %s" % ("up" if ok else "FAILED to register"))

    def _subscribe_cameras(self, locked):
        """Front / nadir / chase feed per dog, Chuck-UI style (.ahrs-cam
        tiles) - only the ones each slot's checkboxes actually enabled.
        A camera the world never spawned (see fleet_world.apply_camera_config)
        has no topic to subscribe to, so this must match that exactly or the
        subscribe call just silently gets nothing; both read the same
        cam_front/cam_nadir/cam_chase slot fields. One gz.transport13
        subscription per enabled camera topic, each decoding the latest
        frame to a JPEG data URL. Deliberate choices:
          - keep only the LATEST frame, never a queue - this is a live tile,
            not a recording, and a queue would only let the browser fall
            behind and then catch up on stale frames;
          - encode on receipt at the sensor's own 10 Hz rather than on
            request, so a slow poller never blocks the transport callback.
        """
        for s in locked:
            i = s["index"]
            for cam in CAMERAS:
                if not s.get(CAM_FLAG_KEYS[cam], True):
                    continue
                node = _gz_transport.Node()
                self._gz_cam_nodes.append(node)

                def on_image(msg, idx=i, camname=cam):
                    # DYNAMIC per-frame gate: the checkbox edits the DRAFT
                    # slot, and this reads it live, so unchecking a camera
                    # mid-run stops (and un-publishes) its stream instantly
                    # and re-checking resumes it. Launch-time flags still
                    # decide whether the sensor exists in the world at all -
                    # a camera disabled at launch has no topic and can never
                    # be re-enabled mid-run, only mid-run muted/unmuted.
                    with self.lock:
                        d = self.draft_slots[idx] if idx < len(self.draft_slots) else {}
                        if not d.get(CAM_FLAG_KEYS[camname], True):
                            self.cameras.get(idx, {}).pop(camname, None)
                            return
                    try:
                        img = _PILImage.frombytes(
                            "RGB", (msg.width, msg.height), msg.data)
                        buf = io.BytesIO()
                        img.save(buf, format="JPEG", quality=60)
                        url = "data:image/jpeg;base64," + base64.b64encode(
                            buf.getvalue()).decode("ascii")
                    except Exception as e:  # noqa: BLE001 - one bad frame must
                        print("[camera] %s dog%d error: %r" % (camname, idx, e),
                              flush=True)
                        return                                # not kill the feed
                    with self.lock:
                        self.cameras.setdefault(idx, {})[camname] = url

                topic = "/go1_%d/%s" % (i, cam)
                ok = node.subscribe(_GzImage, topic, on_image)
                if not ok:
                    self._note("camera subscribe FAILED: %s" % topic)

    # ---- live status, parsed from the controller logs --------------------
    def _start_poller(self, locked):
        def poll():
            done = set()
            seen_bytes = {}   # index -> bytes of ctrl_%d.log already scanned for EVENT_PATTERNS
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
                    # Incremental scan: only NEW complete lines since last
                    # tick, so a discrete event is logged once, not re-noted
                    # every poll while it sits in the file. A trailing
                    # partial line (still being written) is left for the
                    # next tick rather than matched half-formed.
                    start = seen_bytes.get(i, 0)
                    new_text = text[start:]
                    cut = new_text.rfind("\n")
                    if cut >= 0:
                        for line in new_text[:cut].splitlines():
                            for pat, fmt in EVENT_PATTERNS:
                                mo = pat.search(line)
                                if mo:
                                    self._note("dog%d: %s" % (i, fmt(mo)))
                                    break
                        seen_bytes[i] = start + cut + 1
                    st = dict(index=i, phase="running", text="", t="", waypoints="")
                    m = re.findall(r"\[nav\] wp(\d+)/(\d+).*?d=([\d.]+).*?v=([\d.]+)",
                                    text)
                    if m:
                        wp, tot, d, v = m[-1]
                        st["waypoints"] = "%s/%s" % (wp, tot)
                        st["text"] = "d=%sm v=%sm/s" % (d, v)
                    if "[mission] RESULT" in text:
                        # A dog is DONE at its JUDGE line, not at "MISSION
                        # COMPLETE": the judge prints ~9 s later, after the
                        # end-of-mission settle + lie-down. Keying done on
                        # MISSION COMPLETE made the teardown-on-done kill
                        # single-dog runs (and the last fleet finisher)
                        # mid-lie-down, truncating their verdict - two
                        # perfect 114.2 s star guards were reported as
                        # failures by exactly this.
                        tm = re.search(r"MISSION COMPLETE t=([\d.]+)s", text)
                        st["phase"] = "complete"
                        st["t"] = tm.group(1) + "s" if tm else ""
                        done.add(i)
                        self._note("dog%d COMPLETE t=%s" % (i, st["t"]))
                    elif "MISSION COMPLETE" in text:
                        # Loop+dash finished; lie-down/judge still running -
                        # show it, but do NOT count it done yet.
                        tm = re.search(r"MISSION COMPLETE t=([\d.]+)s", text)
                        st["phase"] = "finishing"
                        st["t"] = tm.group(1) + "s" if tm else ""
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
                        procs = list(self.procs)
                        self.procs = []
                        # Clear the SUBSCRIPTION state too, exactly as stop()
                        # does - killing the processes but keeping _gz_node
                        # left the next launch holding a stale subscription
                        # to a dead gz, so its pose feed never came up: the
                        # canvas froze on the previous run's final positions
                        # while the new controllers ran blind ("the interface
                        # was out of sync" - it was the server, not the
                        # browser).
                        self._name_to_index = {}
                        self._last_pose = {}
                        self._gz_node = None
                        self._gz_cam_nodes = []
                    self._note("fleet run complete")
                    # KILL THE SIM ON DONE. Leaving gz alive "for the next
                    # launch" left it idling at ~a full core simulating an
                    # empty world - which held ambient load at 3.8-4.3 (and
                    # got blamed on the operator's web browser), and once
                    # stacked a SECOND physics engine under a new launch,
                    # producing upside-down dogs and below-floor estimates
                    # that read exactly like a code regression. A finished
                    # run owns nothing; tear it all down.
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
                    self._note("sim torn down (gz + bridges + controllers)")
                    return
                time.sleep(1)
        threading.Thread(target=poll, daemon=True).start()

    # ---- teardown ----------------------------------------------------------
    def stop(self):
        with self.lock:
            procs = list(self.procs)
            self.procs = []
            self.phase = "idle"
            self._name_to_index = {}
            self._last_pose = {}       # a stale (x,y,t) would spike the next launch's speed EMA
            self._gz_node = None       # drops the pose subscription
            self._gz_cam_nodes = []    # drops every camera subscription
            self.cameras = {}
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
        # /docs (no extension) -> the static docs.html file. Explicit rather
        # than relying on SimpleHTTPRequestHandler's directory-index rules,
        # which only kick in for a trailing-slash directory path - this way
        # /docs works exactly like typing it looks like it should.
        if self.path in ("/docs", "/docs/"):
            self.path = "/docs.html"
            return super().do_GET()
        m = re.match(r"^/api/logs/(\d+)(?:\?(.*))?$", self.path)
        if m:
            return self._logs(int(m.group(1)), m.group(2) or "")
        return super().do_GET()

    def _logs(self, i, query):
        """Raw log for dog `i` - the FULL text mit_ctrl_sim/the bridge wrote,
        not just the curated one-line-per-event orchestration log /api/state
        already carries. ?kind=ctrl|bridge (default ctrl) picks which
        process; ?tail=<n> returns only the last n lines (default 500 - full
        logs run to thousands of lines on a long mission, and most uses want
        "what just happened", not the whole file); ?full=1 overrides tail
        and returns everything."""
        params = urllib.parse.parse_qs(query)
        kind = params.get("kind", ["ctrl"])[0]
        if kind not in ("ctrl", "bridge"):
            return self._json({"ok": False, "error": "kind must be ctrl or bridge"}, 400)
        path = os.path.join(RUN_DIR, "%s_%d.log" % (kind, i))
        try:
            with open(path, errors="ignore") as f:
                lines = f.readlines()
        except FileNotFoundError:
            return self._json({"ok": False, "error": "no log for dog %d (not launched yet, "
                                "or run_dir was cleared)" % i}, 404)
        full = params.get("full", ["0"])[0] == "1"
        if not full:
            tail = int(params.get("tail", ["500"])[0])
            lines = lines[-tail:]
        return self._json({"ok": True, "index": i, "kind": kind,
                            "lines": len(lines), "text": "".join(lines)})

    def _body(self):
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length else b"{}"
        return json.loads(raw or b"{}")

    def do_POST(self):
        try:
            body = self._body()
        except json.JSONDecodeError:
            return self._json({"ok": False, "error": "bad json"}, 400)

        m = re.match(r"^/api/slots/(\d+)$", self.path)
        if self.path == "/api/launch":
            ok, msg = FLEET.launch(body.get("slots"), body.get("speed_cap"),
                                    body.get("terrain"))
            return self._json({"ok": ok, "message": msg})
        if self.path == "/api/stop":
            FLEET.stop()
            return self._json({"ok": True})
        if self.path == "/api/slots/add":
            ok, res = FLEET.draft_add_slot()
            return self._json({"ok": ok, "slots": res if ok else None,
                                "message": None if ok else res})
        if m:
            ok, res = FLEET.draft_set_slot(int(m.group(1)), body)
            return self._json({"ok": ok, "slot": res if ok else None,
                                "message": None if ok else res})
        if self.path == "/api/speed_cap":
            ok, res = FLEET.draft_set_cap(body.get("value", HARD_SPEED_CAP))
            return self._json({"ok": ok, "speed_cap": res})
        if self.path == "/api/terrain":
            ok, res = FLEET.draft_set_terrain(body.get("value", "flat"))
            return self._json({"ok": ok, "terrain": res if ok else None,
                                "message": None if ok else res})
        return self._json({"ok": False, "error": "no such route"}, 404)

    def do_DELETE(self):
        m = re.match(r"^/api/slots/(\d+)$", self.path)
        if m:
            ok, res = FLEET.draft_remove_slot(int(m.group(1)))
            return self._json({"ok": ok, "slots": res if ok else None,
                                "message": None if ok else res})
        return self._json({"ok": False, "error": "no such route"}, 404)


def main():
    os.makedirs(RUN_DIR, exist_ok=True)
    socketserver.ThreadingTCPServer.allow_reuse_address = True
    with socketserver.ThreadingTCPServer(("127.0.0.1", PORT), Handler) as httpd:
        print("Conductor on http://127.0.0.1:%d" % PORT, flush=True)
        httpd.serve_forever()


if __name__ == "__main__":
    main()
