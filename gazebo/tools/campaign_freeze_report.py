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
from state_freeze_scan import freezes, first_excursion, T_SETTLE


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
        d = per.setdefault(arm, dict(runs=0, passes=0, falls=0, harness=0))
        d["runs"] += 1
        if r.get("verdict") == "PASS":
            d["passes"] += 1
            continue
        d["falls"] += 1
        snap = r.get("snapshot")
        if not snap:
            print("  %s rep %s %s run %s: no snapshot" % (arm, r.get("rep"), r.get("verdict"), r.get("run_id")))
            continue
        try:
            R = [x for x in load_json(snap)["records"] if isinstance(x, dict) and "t" in x and "roll" in x and x["t"] >= T_SETTLE]
        except Exception as e:  # noqa
            print("  %s rep %s run %s: unreadable snapshot (%s)" % (arm, r.get("rep"), r.get("run_id"), e))
            continue
        exc = first_excursion(R)
        fz = [f for f in freezes(R, 3) if exc is not None and f["t0"] <= exc and f["t1"] >= exc - 1.0 and f["ms"] >= a.min_ms]
        tag = "HARNESS (freeze %.0f ms at t=%.2f, loop period max %.2f ms)" % (fz[0]["ms"], fz[0]["t0"], fz[0]["period_max"]) if fz else "genuine"
        if fz:
            d["harness"] += 1
        print("  %s rep %s %s run %s: 30deg at %s -> %s" % (arm, r.get("rep"), r.get("verdict"), r.get("run_id"),
                                                          ("%.2f" % exc) if exc is not None else "-", tag))
    print("== %s: per arm PASS/runs, then PASS/(runs - harness falls)" % a.campaign)
    for arm, d in per.items():
        clean = d["runs"] - d["harness"]
        print("  %-10s %d/%d  ->  %d/%d with %d harness fall(s) removed" % (arm, d["passes"], d["runs"], d["passes"], clean, d["harness"]))


if __name__ == "__main__":
    main()
