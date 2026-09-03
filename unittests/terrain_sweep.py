#!/usr/bin/env python3
"""Systematic per-terrain characterization sweep (operator-ordered,
2026-08-28: "start systematically perfecting each texture... you could do
a dash on each terrain as the perfect test... I suspect oval would handle
with similar interest... keep track of the friction vibe in each world
and how much it makes the runs deviate").

One solo run per (terrain, course) cell through mission_runner, which now
prints a per-dog METRICS line (flown/planned ratio + worst cross-track
deviation) and enforces the flown-vs-planned ground-truth gate; the
server's live desync monitor flags belief-vs-world divergence in the
orchestration log as it happens. Rows land in
unittests/terrain_friction.csv - resumable exactly like corner_sweep
(measured cells skip; REFUSED (Time Machine) and TIMEOUT retry).

Courses: the dash (pure longitudinal traction: launch, cruise, planned
braking to a stop) and the oval (sustained R=5 curves - LATERAL friction
demand, where mu should start writing itself into the deviation numbers).
Both at their validated recipes' own gait/speed so 'flat' rows reproduce
known-good baselines and every other terrain reads as a delta from them.
"""
import argparse
import csv
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RUNNER = os.path.join(ROOT, "gazebo/conductor/mission_runner.py")
CSV_PATH = os.path.join(HERE, "terrain_friction.csv")

TERRAINS = ["flat", "concrete", "asphalt", "grass", "dirt", "gravel",
            "sand", "mud", "rock", "ice"]
# (mission, gait, speed) - dash at the walking probe config used all day;
# oval at its own shipping recipe (trotRunning cap-only rides in the
# recipe's extra, resolved server-side when gait/speed are given).
COURSES = [("dash:30", "walking", "1.5"),
           ("oval:40:5.0", "trotRunning", "3.5")]

METRICS_RE = re.compile(
    r"METRICS dog0 terrain=(\S+) mission=(\S+) flown=([\d.]+) plan=([\d.]+) "
    r"ratio=([\d.]+) xtrack_max=([-\d.]+) verdict=(\S+)")


def done_cells():
    cells = set()
    if os.path.exists(CSV_PATH):
        with open(CSV_PATH) as f:
            for row in csv.DictReader(f):
                if row["verdict"] not in ("TIMEOUT", "REFUSED", "NOFEED"):
                    cells.add((row["terrain"], row["mission"]))
    return cells


def run_cell(terrain, mission, gait, speed):
    cmd = [sys.executable, RUNNER, "--terrain", terrain, "--slot", mission,
           "--gait", gait, "--speed", speed, "--dash", "0",
           "--extra", "WP_CLOSE_LEG=0"]
    t0 = time.time()
    p = subprocess.run(cmd, capture_output=True, text=True)
    wall = time.time() - t0
    out = p.stdout + p.stderr
    if "launch refused" in out:
        return dict(verdict="REFUSED", wall=wall)
    mo = METRICS_RE.search(out)
    desync = len(re.findall(r"DESYNC:", out))
    tm = re.search(r"COMPLETE t=([\d.]+)s", out)
    verdict = "TIMEOUT" if p.returncode == 2 else (
        mo.group(7) if mo else ("FELL" if "FELL" in out else "FAIL"))
    return dict(verdict=verdict, wall=wall,
                flown=mo.group(3) if mo else "", plan=mo.group(4) if mo else "",
                ratio=mo.group(5) if mo else "", xtrack=mo.group(6) if mo else "",
                t=tm.group(1) if tm else "", desync=desync)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--terrains", default=None,
                     help="comma list; default = the full surface set")
    ap.add_argument("--courses", default=None,
                     help="dash | oval | both (default both)")
    args = ap.parse_args()
    terrains = args.terrains.split(",") if args.terrains else TERRAINS
    courses = COURSES
    if args.courses == "dash":
        courses = COURSES[:1]
    elif args.courses == "oval":
        courses = COURSES[1:]
    seen = done_cells()

    new_file = not os.path.exists(CSV_PATH)
    with open(CSV_PATH, "a", newline="") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(["terrain", "mission", "gait", "speed", "verdict",
                        "t_mission_s", "wall_s", "flown_m", "plan_m",
                        "ratio", "xtrack_max_m", "desync_events", "when"])
        for mission, gait, speed in courses:
            for t in terrains:
                if (t, mission) in seen:
                    print("[skip] %s on %s (measured)" % (mission, t), flush=True)
                    continue
                for attempt in range(30):
                    print("[cell] %s on %s ..." % (mission, t), flush=True)
                    r = run_cell(t, mission, gait, speed)
                    if r["verdict"] == "NOFEED":
                        sys.path.insert(0, os.path.join(ROOT, "gazebo/conductor"))
                        import conductor_ctl
                        conductor_ctl.restart_server("NOFEED (OPEN-21)")
                        continue
                    if r["verdict"] != "REFUSED":
                        break
                    print("[gate] refused - waiting 60s", flush=True)
                    time.sleep(60)
                w.writerow([t, mission, gait, speed, r["verdict"],
                            r.get("t", ""), "%.1f" % r["wall"],
                            r.get("flown", ""), r.get("plan", ""),
                            r.get("ratio", ""), r.get("xtrack", ""),
                            r.get("desync", 0),
                            time.strftime("%Y-%m-%d %H:%M:%S")])
                f.flush()
                print("[cell] %s on %-9s -> %-7s t=%ss ratio=%s xtrack=%sm desync=%s"
                      % (mission, t, r["verdict"], r.get("t", "?"),
                         r.get("ratio", "?"), r.get("xtrack", "?"),
                         r.get("desync", 0)), flush=True)
                time.sleep(4)
    print("TERRAIN_SWEEP_DONE", flush=True)


if __name__ == "__main__":
    main()
