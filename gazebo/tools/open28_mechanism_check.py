#!/usr/bin/env python3
"""Does the weight change act on the mechanism I claimed, or merely help?

The OPEN-28 story so far: a real second mode (69x tail excess within one
course family), entered by an UNCOMMANDED yaw disturbance - body 1.70 rad/s
against a 0.30 rad/s command, ratio 6.4x, leading the attitude excursion by
>=2 s in 29 of 29 entries. Capping the commanded yaw does nothing, and neither
does giving the attitude rates non-zero weight. Unitree's full cost vector DOES
reduce crossings.

"A knob helps" and "I understand why" are different claims, and a campaign that
only counts crossings cannot separate them. Two mechanisms predict the same
crossing count and different traces:

  A. IT SUPPRESSES THE DISTURBANCE. Then the yaw excursions themselves are
     smaller in the treated arm, measured over every run - not just the ones
     that went on to cross.

  B. IT IMPROVES REJECTION. Then the yaw excursions are the same size, but
     fewer of them escalate: P(attitude crossing | a big yaw event happened)
     drops while P(big yaw event) does not.

Reporting both is the difference between a tuning result and an explanation.
If neither moves while crossings do, the effect is real but the mechanism
story is wrong and should be said so.

Usage: open28_mechanism_check.py --csv .../open28_subcourse.csv [--yaw 1.0]
"""
import os as _os, sys as _sys; _sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))  # noqa: E702
from snapio import load_json  # archive snapshots may be compacted to .json.zst; this resolves either
import csv, json, os, math, argparse, statistics as st, collections

R2D = 57.2958
ap = argparse.ArgumentParser()
ap.add_argument("--csv", required=True)
ap.add_argument("--yaw", type=float, default=1.0, help="rad/s that counts as a big yaw event")
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

by = collections.defaultdict(list)
for r in rows:
    p = r.get("snapshot", "")
    if not p or p == "NONE" or not os.path.exists(p):
        continue
    try:
        d = load_json(p)
    except Exception:
        continue
    R = [x for x in d.get("records", []) if x.get("wz") is not None
         and x.get("pitch") is not None and x.get("vx") is not None]
    if len(R) < 600:
        continue
    # cruise only: the stand, the stop and the lie-down have their own dynamics
    C = [x for x in R if x["t"] > 6.0 and math.hypot(x["vx"], x["vy"]) > 1.0]
    if len(C) < 300:
        continue
    peak_att = max(max(abs(x["pitch"]), abs(x["roll"])) * R2D for x in R)
    peak_wz = max(abs(x["wz"]) for x in C)
    # how much of the run is spent in a big yaw event
    frac = sum(1 for x in C if abs(x["wz"]) >= a.yaw) / len(C)
    by[r.get("course", "?")].append(dict(att=peak_att, wz=peak_wz, frac=frac,
                                         big=peak_wz >= a.yaw,
                                         crossed=peak_att >= a.limit))

if not by:
    raise SystemExit("  no usable traces")


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


print(f"\n  big yaw event = |wz| >= {a.yaw} rad/s ({a.yaw*R2D:.0f} deg/s) during cruise")
print(f"\n  {'arm':<16} {'n':>4} {'A: peak |wz|':>14} {'time in event':>14} "
      f"{'B: P(cross|big)':>17} {'crossed':>9}")
S = {}
for k in sorted(by):
    g = by[k]
    big = [q for q in g if q["big"]]
    pc = sum(1 for q in big if q["crossed"]) / len(big) if big else float("nan")
    S[k] = g
    print(f"  {k:<16} {len(g):>4} {st.median([q['wz'] for q in g]):>14.2f} "
          f"{st.median([q['frac'] for q in g]):>14.3f} "
          f"{pc:>16.2f} {sum(1 for q in g if q['crossed']):>4}/{len(g):<4}")

base = next((k for k in S if k.endswith("Q0")), None)
if base:
    print(f"\n  against {base}:")
    for k in sorted(S):
        if k == base:
            continue
        zA, pA = mw([q["wz"] for q in S[k]], [q["wz"] for q in S[base]])
        bk = [q for q in S[k] if q["big"]]
        bb = [q for q in S[base] if q["big"]]
        ck = sum(1 for q in bk if q["crossed"])
        cb = sum(1 for q in bb if q["crossed"])
        print(f"    {k:<16} A disturbance size z={zA:+.2f} p={pA:.3f}   "
              f"B escalation {ck}/{len(bk)} vs {cb}/{len(bb)}")
    print(f"\n  A moving  => the change SUPPRESSES the disturbance.")
    print(f"  B moving  => it improves REJECTION of the same disturbance.")
    print(f"  Neither   => the crossing effect is real but this mechanism story")
    print(f"               is wrong, and that is the finding.")
