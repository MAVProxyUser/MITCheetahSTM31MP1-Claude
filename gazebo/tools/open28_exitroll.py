#!/usr/bin/env python3
"""Peak roll in the window after the reversal exit - OPEN-28's continuous endpoint.

Every hp_gap20 run is pushed to ~14 deg of roll by the exit from the 180 deg
reversal (14.06 crossers vs 14.08 non-crossers, identical). 23% keep going to
SafetyChecker's 28.65 and 77% recover, split by nothing measurable before it.
A treatment that widens the margin should move the WHOLE distribution of this
number down, not just the rare tail - and a continuous endpoint on every run
is what makes a 25-rep arm decisive where a 30-rep count of crossings was not.

Window: from the reversal apex (minimum speed strictly between the first and
last cruise samples - bounded that way because the global minimum of a passing
run is its final stop, which silently dropped every pass from an earlier
version of this) to apex + 2.5 s, which covers the p90 of when crossers leave
10 deg (apex + 2.06 s).

Usage: open28_exitroll.py --csv .../open28_subcourse.csv
"""
import os as _os, sys as _sys; _sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))  # noqa: E702
from snapio import load_json  # archive snapshots may be compacted to .json.zst; this resolves either
import csv, json, os, math, argparse, statistics as st, collections

R2D = 57.2958
ap = argparse.ArgumentParser()
ap.add_argument("--csv", required=True)
ap.add_argument("--after", type=float, default=2.5)
ap.add_argument("--limit", type=float, default=28.65)
a = ap.parse_args()

rows, seen = [], set()
for r in csv.DictReader(open(a.csv)):
    rid = r.get("run_id")
    if rid and rid in seen:
        continue
    if rid:
        seen.add(rid)
    rows.append(r)


def score(path):
    try:
        d = load_json(path)
    except Exception:
        return None
    R = [x for x in d.get("records", []) if x.get("vx") is not None
         and x.get("roll") is not None and x["t"] > 6.0]
    if len(R) < 600:
        return None
    k = next((i for i in range(1, len(R)) if R[i]["op_mode"] == 2 and R[i-1]["op_mode"] != 2), None)
    pre = R[:k] if k else R
    sp = [math.hypot(x["vx"], x["vy"]) for x in pre]
    fast = [i for i, v in enumerate(sp) if v > 1.5]
    if len(fast) < 50 or fast[-1] - fast[0] < 200:
        return None
    ia = min(range(fast[0], fast[-1]), key=lambda i: sp[i])
    if sp[ia] > 0.8:
        return None
    ta = pre[ia]["t"]
    W = [x for x in pre if 0 <= x["t"] - ta <= a.after]
    if len(W) < 50:
        return None
    # mean exit acceleration, apex -> 1.5 m/s, to confirm the slew actually bit
    ie = next((i for i in range(ia, len(sp)) if sp[i] >= 1.5), None)
    acc = (sp[ie] - sp[ia]) / max(0.05, pre[ie]["t"] - ta) if ie else float("nan")
    return dict(roll=max(abs(x["roll"]) * R2D for x in W),
                att=max(max(abs(x["roll"]), abs(x["pitch"])) * R2D for x in W),
                acc=acc,
                crossed=max(max(abs(x["roll"]), abs(x["pitch"])) * R2D for x in pre) >= a.limit)


by = collections.defaultdict(list)
for r in rows:
    p = r.get("snapshot", "")
    if not p or p == "NONE" or not os.path.exists(p):
        continue
    s = score(p)
    if s:
        by[r.get("course", "?")].append(s)


def mw(x, y):
    allv = sorted([(v, 0) for v in x] + [(v, 1) for v in y])
    ranks, i = {}, 0
    while i < len(allv):
        j = i
        while j + 1 < len(allv) and allv[j+1][0] == allv[i][0]:
            j += 1
        for t in range(i, j + 1):
            ranks[t] = (i + j) / 2 + 1
        i = j + 1
    RA = sum(ranks[i] for i, (_, s) in enumerate(allv) if s == 0)
    na, nb = len(x), len(y)
    if not na or not nb:
        return 0.0, 1.0
    U = RA - na * (na + 1) / 2
    sd = (na * nb * (na + nb + 1) / 12) ** 0.5
    z = (U - na * nb / 2) / sd if sd else 0.0
    return z, math.erfc(abs(z) / 2 ** 0.5)


print(f"\n  peak |roll| in [apex, apex+{a.after}s] - the quantity the exit produces")
print(f"  {'arm':<24} {'n':>4} {'exit accel':>11} {'med roll':>9} {'p90 roll':>9} {'crossed':>9}")
keys = sorted(by)
for k in keys:
    g = by[k]
    rl = sorted(q["roll"] for q in g)
    ac = [q["acc"] for q in g if q["acc"] == q["acc"]]
    print(f"  {k:<24} {len(g):>4} {st.median(ac) if ac else float('nan'):>11.2f} "
          f"{st.median(rl):>9.1f} {rl[min(len(rl)-1, int(0.9*len(rl)))]:>9.1f} "
          f"{sum(1 for q in g if q['crossed']):>4}/{len(g):<4}")
base = next((k for k in keys if k.endswith("0") or k.endswith("=0")), keys[0] if keys else None)
if base and len(keys) > 1:
    print(f"\n  against {base}:")
    for k in keys:
        if k == base:
            continue
        z, p = mw([q["roll"] for q in by[k]], [q["roll"] for q in by[base]])
        print(f"    {k:<24} peak exit roll  z={z:+.2f}  p={p:.4f}"
              f"{'   <-- MOVED' if p < 0.01 else ''}")
