#!/usr/bin/env python3
"""Scan shm_trace snapshots for STATE FREEZES: stretches where the state the
controller consumed (orientation + foot kinematics) did not change for several
consecutive 2 ms ticks while the control loop itself kept its period.

A freeze is the sensor path stalling under a running controller - the OPEN-28
harness signature (loopback delivery stalls, bridge receive-thread freezes,
host contention). The controller's own period counter cannot see it; this does.
See feedback: "a stall counter measures one thread".

usage:
  state_freeze_scan.py SNAPSHOT.json [...]          one line per snapshot, details for long freezes
  state_freeze_scan.py --campaign NAME [--min-ms 20] all snapshots of $CHEETAH_DATA/campaigns/NAME.csv
  state_freeze_scan.py --dir DIR --since 'YYYYMMDD_HHMM' every snapshot in DIR newer than the stamp
"""
import os as _os, sys as _sys; _sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))  # noqa: E702
from snapio import load_json  # archive snapshots may be compacted to .json.zst; this resolves either
import argparse, csv, glob, json, math, os, sys

DEG = 180.0 / math.pi
KEYS = ("roll", "pitch", "yaw", "foot_z0", "foot_z1", "foot_z2", "foot_z3", "kin_z")
MOVING_VX = 0.3      # m/s: a body at rest has bit-identical state legitimately; only count freezes under motion
T_SETTLE = 2.0       # s: records before the controller has initialised carry garbage orientation


def freezes(records, min_ticks):
    """maximal runs of identical state; returns list of dicts."""
    out = []
    i, n = 1, len(records)
    while i < n:
        a = records[i - 1]
        if all(records[i].get(k) == a.get(k) for k in KEYS):
            j = i
            while j < n and all(records[j].get(k) == a.get(k) for k in KEYS):
                j += 1
            ticks = j - i + 1                       # records i-1 .. j-1 identical
            moving = abs(records[i - 1].get("vx") or 0.0) >= MOVING_VX
            if ticks >= min_ticks and moving:
                t0, t1 = records[i - 1]["t"], records[j - 1]["t"]
                per = max(r.get("period_ms", 0.0) for r in records[i - 1:j])
                jump = {}
                if j < n:
                    b = records[j]
                    jump = {k: (b[k] - a[k]) * DEG for k in ("roll", "pitch", "yaw")}
                out.append(dict(t0=t0, t1=t1, ms=(t1 - t0) * 1000.0, ticks=ticks, period_max=per,
                                vx_in=records[i - 1].get("vx"), vx_out=records[j - 1].get("vx"),
                                jump=jump, roll=a["roll"] * DEG, pitch=a["pitch"] * DEG))
            i = j
        else:
            i += 1
    return out


def first_excursion(records, deg=30.0, z_low=0.18, z_up=0.24):
    """t of the first departure from upright running: |roll| or |pitch| past
    `deg`, or the body sinking below `z_low` after it had stood (z > `z_up`) -
    the level collapse (run 5058) never crossed 30 deg."""
    stood = False
    for r in records:
        if r["t"] < T_SETTLE or not r.get("finite", True):
            continue
        z = r.get("z")
        if z is not None and z > z_up:
            stood = True
        if abs(r["roll"]) * DEG >= deg or abs(r["pitch"]) * DEG >= deg:
            return r["t"]
        if stood and z is not None and z < z_low:
            return r["t"]
    return None


def scan(path, min_ticks, detail_ms):
    d = load_json(path)
    R = d["records"]
    R = [r for r in R if isinstance(r, dict) and "t" in r and "roll" in r and r["t"] >= T_SETTLE]
    fz = freezes(R, min_ticks)
    span = d.get("span_s") or (R[-1]["t"] - R[0]["t"] if R else 0)
    tot = sum(f["ms"] for f in fz)
    mx = max((f["ms"] for f in fz), default=0.0)
    exc = first_excursion(R)
    pre = [f for f in fz if exc is not None and f["t0"] <= exc and f["t1"] >= exc - 1.0]   # overlaps the second before the event (the collapse can begin inside the freeze)
    name = os.path.basename(path)
    print("%-90s span %6.1fs moving-freezes>=%dt: %3d  total %6.1f ms  max %6.1f ms  30deg@ %s  freeze<=1s before: %s"
          % (name[:90], span, min_ticks, len(fz), tot, mx,
             ("%.2f" % exc) if exc is not None else "-",
             ("%d (max %.0f ms)" % (len(pre), max(f["ms"] for f in pre))) if pre else "none"))
    for f in fz:
        if f["ms"] >= detail_ms:
            j = f["jump"]
            print("    freeze t=%.3f..%.3f  %5.1f ms (%d ticks)  loop period max %.2f ms  vx %.2f->%.2f  "
                  "state roll %.1f pitch %.1f  jump after: roll %+.1f pitch %+.1f yaw %+.1f deg"
                  % (f["t0"], f["t1"], f["ms"], f["ticks"], f["period_max"], f["vx_in"] or 0, f["vx_out"] or 0,
                     f["roll"], f["pitch"], j.get("roll", 0), j.get("pitch", 0), j.get("yaw", 0)))
    return dict(path=path, n=len(fz), total_ms=tot, max_ms=mx, pre=pre, exc=exc)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("snapshots", nargs="*")
    ap.add_argument("--campaign")
    ap.add_argument("--dir")
    ap.add_argument("--since", default="")
    ap.add_argument("--min-ticks", type=int, default=4, help="identical consecutive records to count (default 4 = 6 ms)")
    ap.add_argument("--detail-ms", type=float, default=20.0, help="print each freeze at least this long")
    a = ap.parse_args()
    paths = list(a.snapshots)
    if a.campaign:
        data = os.environ.get("CHEETAH_DATA", os.path.expanduser("~/Desktop/Cheetah/rundata"))
        with open(os.path.join(data, "campaigns", a.campaign + ".csv")) as fh:
            for row in csv.DictReader(fh):
                if row.get("snapshot"):
                    paths.append(row["snapshot"])
    if a.dir:
        for p in sorted(glob.glob(os.path.join(a.dir, "*.json"))):
            if os.path.basename(p) >= a.since:
                paths.append(p)
    if not paths:
        ap.error("no snapshots")
    res = [scan(p, a.min_ticks, a.detail_ms) for p in paths]
    n_fall = sum(1 for r in res if r["exc"] is not None)
    n_fall_pre = sum(1 for r in res if r["pre"])
    print("== %d snapshots; %d with a >=30deg excursion, %d of those with a freeze in the second before it; "
          "runs with any freeze >= 20 ms: %d" % (len(res), n_fall, n_fall_pre,
                                                sum(1 for r in res if r["max_ms"] >= 20)))


if __name__ == "__main__":
    main()
