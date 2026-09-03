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
sys.path.insert(0, os.path.join(ROOT, 'gazebo/conductor'))
import campaign as _campaign   # noqa: E402  (panel progress)
RUNNER = os.path.join(ROOT, "gazebo/conductor/mission_runner.py")
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
                # TIMEOUT (harness gave up) and REFUSED (launch gate - e.g.
                # a Time Machine backup) are not verdicts - retry those.
                if row["verdict"] not in ("TIMEOUT", "REFUSED"):
                    cells.add((row["gait"], float(row["speed"]),
                               int(float(row["angle"]))))
    return cells


def nonpass_cells():
    """Cells whose ONLY row is FELL/FAIL - the second tranche. Every one of
    them was measured N=1, and on a conductor that was leaking ~1 thread per
    run (fixed 2026-08-31), which both dropped launches and depressed pass
    rates - i.e. biased toward exactly these verdicts. A cell with several
    rows already has its repeats and is left alone."""
    rows = {}
    if os.path.exists(CSV_PATH):
        with open(CSV_PATH) as f:
            for row in csv.DictReader(f):
                if row["verdict"] in ("TIMEOUT", "REFUSED", "ABORTED"):
                    continue
                k = (row["gait"], float(row["speed"]), int(float(row["angle"])))
                rows.setdefault(k, []).append(row["verdict"])
    return sorted(k for k, v in rows.items()
                  if len(v) == 1 and v[0] in ("FELL", "FAIL"))


def server_healthy():
    """Is the conductor answering at all? See the call site."""
    try:
        import json as _json
        import urllib.request
        with urllib.request.urlopen("http://127.0.0.1:8420/api/state",
                                     timeout=5) as f:
            _json.load(f)
        return True
    except Exception:  # noqa: BLE001
        return False


def run_cell(gait, speed, angle, leg):
    spec = "corner:%g:%g" % (leg, angle)
    cmd = [sys.executable, RUNNER, "--slot", spec, "--gait", gait,
           "--speed", str(speed), "--dash", "0",
           # Sit out a closed launch gate (Time Machine, fleet tearing down)
           # instead of recording the refusal as a robot verdict - a refused
           # launch consumed 15 reps in 51 s on another harness before
           # mission_runner grew this flag and LAUNCH_REFUSED_EXIT (5).
           "--wait-for-gate", "3600",
           "--extra", "WP_CLOSE_LEG=0"]
    t0 = time.time()
    p = subprocess.run(cmd, capture_output=True, text=True)
    wall = time.time() - t0
    out = p.stdout + p.stderr
    if p.returncode == 3 or "LAUNCH ABORTED BY THE SERVER" in out:
        # The launch never ran (world build / gz discovery). Not a verdict.
        return "ABORTED", wall, "launch aborted by the server - re-run"
    if p.returncode == 5 or "launch refused" in out:
        # The conductor's launch gate said no (a Time Machine backup, a
        # fleet still active, ...). NOT a mission verdict, and it burned an
        # hour of this sweep's first attempt: the 2026-08-28 first run
        # instant-"FAIL"ed 10 straight cells against the hourly backup.
        verdict = "REFUSED"
    elif p.returncode == 2:
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
        if any(k in line for k in ("FELL", "fell", "orientation", "RESULT",
                                    "refused")):
            detail = line.strip()[-120:]
    return verdict, wall, detail


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gait", action="append", default=[],
                     help="gait:speed (repeatable); default = the standard tranche")
    ap.add_argument("--angles", default=None,
                     help="comma-separated degrees; default 30..165 step 15")
    ap.add_argument("--leg", type=float, default=25.0)
    ap.add_argument("--redo-nonpass", action="store_true",
                     help="ignore --gait/--angles; re-run every cell whose "
                          "only row is FELL/FAIL, --reps more times each, "
                          "APPENDING rows (the tally aggregates per cell)")
    ap.add_argument("--reps", type=int, default=2,
                     help="repeats per cell in --redo-nonpass mode")
    ap.add_argument("--list", action="store_true",
                     help="print the cells that would run, then exit")
    args = ap.parse_args()

    if args.redo_nonpass:
        # (gait, speed, angle) triples, each repeated --reps times; the skip
        # set is emptied so existing rows do not suppress the repeat.
        cells = [c for c in nonpass_cells() for _ in range(args.reps)]
        seen = set()
    else:
        gaits = args.gait or DEFAULT_GAITS
        angles = ([int(a) for a in args.angles.split(",")] if args.angles
                  else DEFAULT_ANGLES)
        cells = [(g.split(":")[0], float(g.split(":")[1]), a)
                 for g in gaits for a in angles]
        seen = done_cells()
    if args.list:
        todo = [c for c in cells if c not in seen]
        for c in todo:
            print("%s@%g angle=%d" % c)
        print("%d runs (%d already measured, skipped)"
              % (len(todo), len(cells) - len(todo)))
        return
    _total = len(cells)
    _done = 0

    new_file = not os.path.exists(CSV_PATH)
    with open(CSV_PATH, "a", newline="") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(["gait", "speed", "angle", "verdict", "wall_s",
                        "detail", "when"])
        for gait, speed, angle in cells:
            if True:   # keeps the original loop body's indentation intact
                if (gait, speed, angle) in seen:
                    print("[skip] %s@%g angle=%d (already measured)"
                          % (gait, speed, angle), flush=True)
                    continue
                print("[cell] %s@%g angle=%d ..." % (gait, speed, angle),
                      flush=True)
                _done += 1
                _campaign.set_stage("OPEN-8 cornering envelope",
                                    "%s @%g" % (gait, speed), _done, _total,
                                    "angle %d deg" % angle)
                # A REFUSED launch (Time Machine et al.) is transient: wait
                # it out and retry the SAME cell, up to ~30 min, instead of
                # burning the rest of the grid against a closed gate.
                for attempt in range(30):
                    # A conductor that has stopped answering must never be
                    # recorded as a robot verdict. Nine cells were written as
                    # FAIL that way on 2026-08-29 without a mission ever
                    # running; recycle instead, and abort rather than
                    # manufacture results if it stays down.
                    if not server_healthy():
                        print("[gate] conductor not answering - recycling",
                              flush=True)
                        sys.path.insert(0, os.path.join(
                            ROOT, "gazebo/conductor"))
                        import conductor_ctl
                        conductor_ctl.restart_server("server unreachable")
                        if not server_healthy():
                            print("[gate] STILL down - stopping rather than "
                                  "recording fake failures", flush=True)
                            return
                    verdict, wall, detail = run_cell(gait, speed, angle,
                                                     args.leg)
                    if verdict == "ABORTED":
                        print("[gate] %s@%g angle=%d launch ABORTED (never "
                              "ran) - recycling and retrying"
                              % (gait, speed, angle), flush=True)
                        sys.path.insert(0, os.path.join(
                            ROOT, "gazebo/conductor"))
                        import conductor_ctl
                        conductor_ctl.restart_server("launch aborted")
                        continue
                    if verdict != "REFUSED":
                        break
                    print("[gate] %s@%g angle=%d REFUSED (%s) - waiting 60s"
                          % (gait, speed, angle, detail or "launch gate"),
                          flush=True)
                    time.sleep(60)
                w.writerow([gait, speed, angle, verdict, "%.1f" % wall,
                            detail, time.strftime("%Y-%m-%d %H:%M:%S")])
                f.flush()
                print("[cell] %s@%g angle=%d -> %s (%.0fs)"
                      % (gait, speed, angle, verdict, wall), flush=True)
                time.sleep(3)
    _campaign.clear()
    print("SWEEP_DONE", flush=True)


if __name__ == "__main__":
    main()
