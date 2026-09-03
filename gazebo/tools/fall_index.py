#!/usr/bin/env python3
"""fall_index.py - distil the fall archive into one small CSV.

WHY: OPEN-26's entire evidence base is 424 shm_trace snapshots totalling
2.9 GB under /tmp/cheetah_conductor/archive/shm_trace. /tmp is volatile -
macOS purges it - and 2.9 GB does not belong in git. Every number OPEN-26
quotes (the 68% level-collapse split, the 0.33 s / 0.102 m sink, the 2.5x
disagreement between the estimator's position and velocity blocks) is
derived from these files, so if they go, the findings become unreproducible
claims. This writes ONE ROW PER FALL with the derived quantities, small
enough to commit, so the analysis can be re-checked without the raw rings.

  python3 gazebo/tools/fall_index.py [out.csv]
"""
import json, glob, csv, sys, os

AR = "/private/tmp/cheetah_conductor/archive/shm_trace"
OUT = sys.argv[1] if len(sys.argv) > 1 else "gazebo/conductor/data/fall_index.csv"

def classify(r):
    ro, pi = abs(r["roll"])*57.2958, abs(r["pitch"])*57.2958
    if ro < 10 and pi < 15: return "LEVEL"
    if ro > 25 or pi > 25:  return "TIP"
    return "MIXED"

rows = []
for f in sorted(glob.glob(os.path.join(AR, "*.json"))):
    try: d = json.load(open(f))
    except Exception: continue
    R = [x for x in (d.get("records") or []) if x.get("z") is not None]
    if len(R) < 200: continue
    last = R[-1]
    # descent window: last crossing of 0.20 m down to 0.10 m
    start = end = None
    for i in range(len(R)-1, -1, -1):
        if R[i]["z"] <= 0.10 and end is None: end = i
        if end is not None and R[i]["z"] >= 0.20: start = i; break
    rec = dict(
        file=os.path.basename(f), reason=d.get("reason"), run_id=d.get("run_id"),
        n_records=len(R), span_s=round(d.get("span_s") or 0, 2),
        kind=classify(last),
        end_roll_deg=round(last["roll"]*57.2958, 2),
        end_pitch_deg=round(last["pitch"]*57.2958, 2),
        end_z=round(last["z"], 4),
    )
    if start is not None and end is not None and end > start:
        w = R[start:end+1]
        dt = w[-1]["t"] - w[0]["t"]; dz = w[0]["z"] - w[-1]["z"]
        rec.update(desc_dt=round(dt, 4), desc_dz=round(dz, 4),
                   desc_rate=round(dz/dt, 4) if dt > 0 else "",
                   desc_vz_min=round(min(x["vz"] for x in w), 4),
                   desc_contact_mean=round(
                       sum(x["c%d" % i] for x in w for i in range(4))/(4*len(w)), 4))
    rows.append(rec)

cols = ["file","reason","run_id","kind","n_records","span_s","end_roll_deg",
        "end_pitch_deg","end_z","desc_dt","desc_dz","desc_rate","desc_vz_min",
        "desc_contact_mean"]
os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore"); w.writeheader()
    for r in rows: w.writerow(r)
print("%d snapshots -> %s" % (len(rows), OUT))
from collections import Counter
# Split by reason: the archive also holds deliberate PASS dumps (c21/c22),
# and those end lying down, so they classify as LEVEL. Counting them into
# the fall split would inflate it - report the FALL population separately.
falls = [r for r in rows if (r.get("reason") or "").startswith("FALL")]
c = Counter(r["kind"] for r in falls); tot = sum(c.values()) or 1
print("  FALL snapshots: %d" % len(falls))
for k, v in c.most_common(): print("    %-6s %3d  %4.0f%%" % (k, v, 100*v/tot))
other = Counter((r.get("reason") or "?") for r in rows if r not in falls)
if other: print("  other dumps: " + ", ".join("%s=%d" % kv for kv in other.most_common()))
