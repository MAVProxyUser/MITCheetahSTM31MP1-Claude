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


def event_second_deficit(run_id, data):
    """Fewest IMU samples/s the bridge saw in the last two CRUISE seconds
    (cmd_rx >= 400/s) of the run's archived bridge log - the controller exits on
    a fall (SIM_FALL_EXIT=1 under the conductor), so the last cruise second is
    the event second. None when there is no bridge log for the run."""
    import glob as _g, re as _re
    if not run_id:
        return None
    logs = _g.glob(os.path.join(data, "conductor", "archive", "*run%s_bridge_0.log" % run_id))
    if not logs:
        return None
    rx = []
    try:
        for line in open(logs[0], errors="replace"):
            m = _re.search(r"cmd_rx=(\d+)/s.*imu_rx=(\d+)/s", line)
            if m and int(m.group(1)) >= 400:
                rx.append(int(m.group(2)))
    except Exception:
        return None
    return min(rx[-2:]) if rx else None


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
        # OPEN-39 (2026-09-13): the same stream can lose twenty samples in one
        # second as several SHORT gaps - none over 15 ms, none a held sample the
        # freeze scan below would see. Both locomotionSafe leg-speed trips on
        # record sat in such a second (480/s and 482/s) against one cruise
        # second in 2859. The CSV carries the run's minimum (imu_rx_min); for a
        # fall the bridge log itself says whether the EVENT second was short.
        deficit_s = event_second_deficit(r.get("run_id"), data)
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
        if not fz and deficit_s is not None and deficit_s <= 485:
            tag = "HARNESS (sample deficit: %d IMU samples/s in the event second, OPEN-39)" % deficit_s
        if fz or (deficit_s is not None and deficit_s <= 485):
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
