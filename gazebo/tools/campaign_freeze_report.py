#!/usr/bin/env python3
"""Per-campaign harness-fall report: which FAIL rows were preceded by a sensor
freeze (the state the controller consumed bit-identical for >= 3 ticks under
motion in the second before the first 30 deg excursion). Prints one line per
fall and a per-arm split of genuine vs harness falls, so a campaign's verdict
counts can be read with the harness's own falls taken out.

  campaign_freeze_report.py NAME [--min-ms 20]
"""
import argparse, csv, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from snapio import load_json
from state_freeze_scan import freezes, event_window, T_SETTLE


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("campaign")
    ap.add_argument("--min-ms", type=float, default=20.0, help="a freeze at least this long counts as a harness fall")
    a = ap.parse_args()
    data = os.environ.get("CHEETAH_DATA", os.path.expanduser("~/Desktop/Cheetah/rundata"))
    rows = list(csv.DictReader(open(os.path.join(data, "campaigns", a.campaign + ".csv"))))
    per = {}
    for r in rows:
        arm = r.get("course") or r.get("arm") or "?"
        d = per.setdefault(arm, dict(runs=0, passes=0, falls=0, harness=0, gappy=0))
        if r.get("verdict") == "NONE":      # no run happened (launch refused, timeout): not a fall, not a rep
            d["none"] = d.get("none", 0) + 1
            continue
        d["runs"] += 1
        # the sim's stream (OPEN-35's third class): the harness records the run's
        # worst IMU gap from the bridge's own line; over ~15 ms the run is not
        # the robot's evidence whether it passed or fell
        try:
            gap = float(r.get("imu_gap_max_ms") or 0)
        except Exception:
            gap = 0.0
        if gap > 15.0:
            d["gappy"] += 1
        if r.get("verdict") == "PASS":
            d["passes"] += 1
            continue
        d["falls"] += 1
        snap = r.get("snapshot")
        if not snap:
            print("  %s rep %s %s run %s: no snapshot" % (arm, r.get("rep"), r.get("verdict"), r.get("run_id")))
            continue
        try:
            dd = load_json(snap)
            R = [x for x in dd["records"] if isinstance(x, dict) and "t" in x and "roll" in x and x["t"] >= T_SETTLE]
        except Exception as e:  # noqa
            print("  %s rep %s run %s: unreadable snapshot (%s)" % (arm, r.get("rep"), r.get("run_id"), e))
            continue
        exc, lo, hi = event_window(R, dd.get("text_log"))
        fz = [f for f in freezes(R, 3) if exc is not None and f["t0"] <= hi and f["t1"] >= lo and f["ms"] >= a.min_ms]
        tag = "HARNESS (freeze %.0f ms at t=%.2f, loop period max %.2f ms)" % (fz[0]["ms"], fz[0]["t0"], fz[0]["period_max"]) if fz else "genuine"
        if fz:
            d["harness"] += 1
        print("  %s rep %s %s run %s: event at %s -> %s" % (arm, r.get("rep"), r.get("verdict"), r.get("run_id"),
                                                          ("%.2f" % exc) if exc is not None else "-", tag))
    print("== %s: per arm PASS/runs, then PASS/(runs - harness falls)" % a.campaign)
    for arm, d in per.items():
        clean = d["runs"] - d["harness"]
        print("  %-10s %d/%d  ->  %d/%d with %d harness fall(s) removed%s%s" % (arm, d["passes"], d["runs"], d["passes"], clean, d["harness"],
              ("  (+%d NONE row(s): no run)" % d["none"]) if d.get("none") else "",
              ("  [%d run(s) with an IMU-stream gap > 15 ms]" % d["gappy"]) if d.get("gappy") else ""))


if __name__ == "__main__":
    main()
