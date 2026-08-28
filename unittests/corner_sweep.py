#!/usr/bin/env python3
"""OPEN-8: the per-gait x per-angle cornering envelope, as data collection.

Runs `corner:<leg>:<angle>` probes through the conductor's own
mission_runner (REST only - structurally cannot kill a legit run), one
solo run per (gait, angle) cell, and appends one CSV row per cell to
unittests/corner_envelope.csv. Cells already present in the CSV are
SKIPPED, so the sweep is resumable / extendable in later sessions and in
finer notches (start at a 15-degree grid, add 5-degree notches around
whatever transitions appear - the stretch goal's "5 degree notches"
resolution is a refinement pass, not the first pass).

Course-shape choices, deliberate:
  - WP_CLOSE_LEG=0: the probe is ONE corner with an approach and an exit;
    a walk-home leg would add a second, sharper corner and stop measuring
    the angle under test.
  - dash=0: no sprint finish.
  - leg 25 m (the corner: mission's validated geometry, recipe'd 45/90/135).
Verdict vocabulary matches mission_runner: PASS / FAIL / FELL, plus
TIMEOUT (exit 2: harness gave up - NOT a mission verdict; re-run before
believing it, per the project's own rule).

Usage:
  python3 unittests/corner_sweep.py                  # default tranche
  python3 unittests/corner_sweep.py --gait bounding:1.0 --angles 85,90,95
"""
import argparse
import csv
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RUNNER = os.path.join(ROOT, "stm32mp1/gazebo/conductor/mission_runner.py")
CSV_PATH = os.path.join(HERE, "corner_envelope.csv")

# Default tranche: the three flight gaits whose 90-147.5deg "mid-band"
# question this sweep settles for good, plus the two flagship gaits.
DEFAULT_GAITS = ["bounding:1.0", "galloping:0.8", "pronking:0.6",
                  "trotRunning:3.5", "trotting:2.5"]
DEFAULT_ANGLES = [30, 45, 60, 75, 90, 105, 120, 135, 150, 165]


def done_cells():
    cells = set()
    if os.path.exists(CSV_PATH):
        with open(CSV_PATH) as f:
            for row in csv.DictReader(f):
                # A TIMEOUT is not a verdict - retry those cells.
                if row["verdict"] != "TIMEOUT":
                    cells.add((row["gait"], float(row["speed"]),
                               int(float(row["angle"]))))
    return cells


def run_cell(gait, speed, angle, leg):
    spec = "corner:%g:%g" % (leg, angle)
    cmd = [sys.executable, RUNNER, "--slot", spec, "--gait", gait,
           "--speed", str(speed), "--dash", "0",
           "--extra", "WP_CLOSE_LEG=0"]
    t0 = time.time()
    p = subprocess.run(cmd, capture_output=True, text=True)
    wall = time.time() - t0
    out = p.stdout + p.stderr
    if p.returncode == 2:
        verdict = "TIMEOUT"
    elif "mission result: PASS" in out:
        verdict = "PASS"
    elif "FELL" in out or "fell" in out.lower():
        verdict = "FELL"
    else:
        verdict = "FAIL"
    # One line of forensic context for a non-pass, grepped from the stream.
    detail = ""
    for line in out.splitlines():
        if any(k in line for k in ("FELL", "fell", "orientation", "RESULT")):
            detail = line.strip()[-120:]
    return verdict, wall, detail


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gait", action="append", default=[],
                     help="gait:speed (repeatable); default = the standard tranche")
    ap.add_argument("--angles", default=None,
                     help="comma-separated degrees; default 30..165 step 15")
    ap.add_argument("--leg", type=float, default=25.0)
    args = ap.parse_args()

    gaits = args.gait or DEFAULT_GAITS
    angles = ([int(a) for a in args.angles.split(",")] if args.angles
              else DEFAULT_ANGLES)
    seen = done_cells()

    new_file = not os.path.exists(CSV_PATH)
    with open(CSV_PATH, "a", newline="") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(["gait", "speed", "angle", "verdict", "wall_s",
                        "detail", "when"])
        for gs in gaits:
            gait, speed = gs.split(":")
            speed = float(speed)
            for angle in angles:
                if (gait, speed, angle) in seen:
                    print("[skip] %s@%g angle=%d (already measured)"
                          % (gait, speed, angle), flush=True)
                    continue
                print("[cell] %s@%g angle=%d ..." % (gait, speed, angle),
                      flush=True)
                verdict, wall, detail = run_cell(gait, speed, angle, args.leg)
                w.writerow([gait, speed, angle, verdict, "%.1f" % wall,
                            detail, time.strftime("%Y-%m-%d %H:%M:%S")])
                f.flush()
                print("[cell] %s@%g angle=%d -> %s (%.0fs)"
                      % (gait, speed, angle, verdict, wall), flush=True)
                time.sleep(3)
    print("SWEEP_DONE", flush=True)


if __name__ == "__main__":
    main()
