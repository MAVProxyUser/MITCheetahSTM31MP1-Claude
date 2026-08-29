#!/usr/bin/env python3
"""OPEN-7 part 1: the per-terrain CAPABILITY ENVELOPE - which gait holds
which speed, and which corner angle, on which ground.

Operator's framing (2026-08-28): "open 7 seems like you can work it to
the point of simply documenting which gaits work at which max speeds,
and angles on the different terrains to build logic for the path planner
to be aware of said terrain ... there are obviously certain limitations
that we have to live with, we just need to document them, and add the
rest to the pre-planner for each terrain type and gait on said terrain."

Two sweeps, one file each, both resumable and both gated on the SAME
ground truth the panel uses (mission_runner's METRICS line: flown/planned
ratio from the gz pose feed, so a dog that believes it navigated while
standing still reads INVALID, never PASS):

  --sweep speed  dash:30 at a speed LADDER per (terrain, gait). The
                 ladder runs LOW to HIGH and does not stop at the first
                 failure - it measures every rung, because this project
                 has twice recorded a gait's ceiling from a stop-at-first-
                 failure ladder and twice had to retract it (a marginal
                 cell that fails one rung and passes the next is the
                 normal case here, not an anomaly).

  --sweep angle  corner:25:<angle> per (terrain, angle) at the corner
                 probe's own validated recipe. The corner mission exists
                 precisely so an angle can be swept without inheriting
                 another course's tuning.

Both write the highest PASSING rung per row into the CSV, so the planner
table (BodyLimits::v_terrain_max, and later a per-angle cap) can be read
straight out of the measured data rather than guessed from mu.
"""
import argparse
import csv
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RUNNER = os.path.join(ROOT, "stm32mp1/gazebo/conductor/mission_runner.py")
SPEED_CSV = os.path.join(HERE, "terrain_envelope_speed.csv")
ANGLE_CSV = os.path.join(HERE, "terrain_envelope_angle.csv")

METRICS_RE = re.compile(
    r"METRICS dog0 terrain=(\S+) mission=(\S+) flown=([\d.]+) plan=([\d.]+) "
    r"ratio=([\d.]+) xtrack_max=([-\d.]+) verdict=(\S+)")

# The geometry kinds - what the surface matrix could not reach. flat is
# the control: every one of its cells must reproduce a known-good result
# or the sweep is measuring the harness, not the ground.
GEOM_TERRAINS = ["flat", "rough", "rolling"]
# One representative surface at each end of the measured friction ladder,
# so the speed envelope carries a mu axis without re-running all nine.
SURF_TERRAINS = ["concrete", "mud"]

# Low to high, EVERY rung measured - never stop at the first failure. This
# project has twice recorded a ceiling from a stop-at-first-failure ladder
# and twice had to retract it, because a cell that fails one rung and passes
# the next is the normal case here. Rungs above each gait's established base
# speed are where a terrain-specific ceiling would actually show up; the
# base rungs are the control.
GAIT_LADDER = {
    "walking":     [1.0, 1.5, 2.0, 2.5],
    "trotting":    [1.5, 2.0, 2.5, 3.0, 3.5],
    "trotRunning": [2.0, 3.0, 3.5, 4.0, 4.5],
}
ANGLES = [45, 90, 135]
CORNER_EXTRA = ("WP_ACCEPT=1.5 WP_CORRIDOR_MIN=0.07 WP_ALON=0.4 "
                "WP_TURN_SOFT=0.3 WP_TURN_HARD=2.0 WP_CLOSE_LEG=0")


def run_cell(terrain, mission, gait, speed, extra):
    cmd = [sys.executable, RUNNER, "--terrain", terrain, "--slot", mission,
           "--gait", gait, "--speed", str(speed), "--dash", "0",
           "--extra", extra]
    t0 = time.time()
    p = subprocess.run(cmd, capture_output=True, text=True)
    wall = time.time() - t0
    out = p.stdout + p.stderr
    if p.returncode == 3 or "LAUNCH ABORTED BY THE SERVER" in out:
        return dict(verdict="ABORTED", wall=wall)   # never ran - not a verdict
    if "launch refused" in out:
        return dict(verdict="REFUSED", wall=wall)
    mo = METRICS_RE.search(out)
    tm = re.search(r"COMPLETE t=([\d.]+)s", out)
    verdict = "TIMEOUT" if p.returncode == 2 else (
        mo.group(7) if mo else ("FELL" if "FELL" in out else "FAIL"))
    return dict(verdict=verdict, wall=wall,
                ratio=mo.group(5) if mo else "", xtrack=mo.group(6) if mo else "",
                t=tm.group(1) if tm else "",
                desync=len(re.findall(r"DESYNC:", out)))


def recycle(reason):
    sys.path.insert(0, os.path.join(ROOT, "stm32mp1/gazebo/conductor"))
    import conductor_ctl
    conductor_ctl.restart_server(reason)


def server_healthy():
    """Is the conductor actually answering? A sweep that keeps launching at
    a dead or wedged server writes a column of FAIL rows that look exactly
    like robot results - which is what happened the night this was added:
    a 'only 0/1 dogs came up' launch took the server down and the next NINE
    cells were recorded as failures without a mission ever running."""
    try:
        import urllib.request
        with urllib.request.urlopen("http://127.0.0.1:8420/api/state",
                                     timeout=5) as f:
            json.load(f)
        return True
    except Exception:  # noqa: BLE001
        return False


def cell_with_retries(terrain, mission, gait, speed, extra):
    """NOFEED is infrastructure (OPEN-21), not a robot verdict - recycle
    the conductor and re-run rather than recording a fake failure. Same for
    a server that has stopped answering at all."""
    for _ in range(6):
        if not server_healthy():
            print("[gate] conductor not answering - recycling before this "
                  "cell (no verdict recorded)", flush=True)
            recycle("server unreachable")
            if not server_healthy():
                print("[gate] STILL not answering - stopping rather than "
                      "recording fake failures", flush=True)
                return dict(verdict="ABORTED-NO-SERVER", wall=0.0)
        r = run_cell(terrain, mission, gait, speed, extra)
        if r["verdict"] == "ABORTED":
            print("[gate] launch ABORTED (never ran) - recycling, retrying",
                  flush=True)
            recycle("launch aborted")
            continue
        if r["verdict"] == "NOFEED":
            recycle("NOFEED (OPEN-21)")
            continue
        if r["verdict"] != "REFUSED":
            return r
        print("[gate] refused (Time Machine?) - waiting 60s", flush=True)
        time.sleep(60)
    return r


def done_rows(path, keys):
    seen = set()
    if os.path.exists(path):
        with open(path) as f:
            for row in csv.DictReader(f):
                seen.add(tuple(row[k] for k in keys))
    return seen


def sweep_speed(terrains):
    keys = ("terrain", "gait", "speed")
    seen = done_rows(SPEED_CSV, keys)
    new = not os.path.exists(SPEED_CSV)
    with open(SPEED_CSV, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["terrain", "gait", "speed", "verdict", "t_s", "wall_s",
                        "ratio", "xtrack_max_m", "desync", "when"])
        for t in terrains:
            for gait, ladder in GAIT_LADDER.items():
                for spd in ladder:
                    if (t, gait, "%g" % spd) in seen:
                        print("[skip] %s %s @%g (measured)" % (t, gait, spd), flush=True)
                        continue
                    print("[cell] %s %s @%g ..." % (t, gait, spd), flush=True)
                    r = cell_with_retries(t, "dash:30", gait, spd,
                                          "WP_CLOSE_LEG=0")
                    w.writerow([t, gait, "%g" % spd, r["verdict"], r.get("t", ""),
                                "%.1f" % r["wall"], r.get("ratio", ""),
                                r.get("xtrack", ""), r.get("desync", 0),
                                time.strftime("%Y-%m-%d %H:%M:%S")])
                    f.flush()
                    print("[cell] %-9s %-12s @%-4g -> %-8s ratio=%s xtrack=%sm"
                          % (t, gait, spd, r["verdict"], r.get("ratio", "?"),
                             r.get("xtrack", "?")), flush=True)
                    time.sleep(4)
    print("SPEED_SWEEP_DONE", flush=True)


def sweep_angle(terrains):
    keys = ("terrain", "angle")
    seen = done_rows(ANGLE_CSV, keys)
    new = not os.path.exists(ANGLE_CSV)
    with open(ANGLE_CSV, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["terrain", "angle_deg", "gait", "speed", "verdict",
                        "t_s", "wall_s", "ratio", "xtrack_max_m", "when"])
        for t in terrains:
            for a in ANGLES:
                if (t, str(a)) in seen:
                    print("[skip] %s corner %d (measured)" % (t, a), flush=True)
                    continue
                print("[cell] %s corner:%d ..." % (t, a), flush=True)
                r = cell_with_retries(t, "corner:25:%d" % a, "trotting", 1.5,
                                      CORNER_EXTRA)
                w.writerow([t, a, "trotting", 1.5, r["verdict"], r.get("t", ""),
                            "%.1f" % r["wall"], r.get("ratio", ""),
                            r.get("xtrack", ""),
                            time.strftime("%Y-%m-%d %H:%M:%S")])
                f.flush()
                print("[cell] %-9s corner %3d -> %-8s ratio=%s xtrack=%sm"
                      % (t, a, r["verdict"], r.get("ratio", "?"),
                         r.get("xtrack", "?")), flush=True)
                time.sleep(4)
    print("ANGLE_SWEEP_DONE", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep", choices=["speed", "angle", "both"], default="both")
    ap.add_argument("--terrains", default=None,
                    help="comma list; default = geometry kinds (+2 surfaces for speed)")
    args = ap.parse_args()
    if args.sweep in ("speed", "both"):
        sweep_speed(args.terrains.split(",") if args.terrains
                    else GEOM_TERRAINS + SURF_TERRAINS)
    if args.sweep in ("angle", "both"):
        sweep_angle(args.terrains.split(",") if args.terrains else GEOM_TERRAINS)


if __name__ == "__main__":
    main()
