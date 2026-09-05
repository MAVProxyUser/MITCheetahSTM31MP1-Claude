#!/usr/bin/env python3
"""Score the POST-STOP window: what the attitude does after the dog has stopped.

The finish-line face-plant is a claim about what happens between "the robot is
stopped" and "the robot is lying down".  Everything before that is the run, and
the run has its own well-characterised failure mode (OPEN-26: attitude crosses
SafetyChecker's 0.5 rad limit, the FSM E-stops, leg commands go to zero, the
body drops).  Counting whole-run PASS/FAIL mixes the two and mostly measures
the run - which is how the first stop-fix A/B produced a confident number about
a treatment it never applied to a single failing run.

The window is found from the TRACE, not from a log line.  Two clocks live in
these traces - on a long course the [nav] lines run ~18 s behind the record and
[stm32mp1] clock - so keying off "MISSION COMPLETE t=..." silently windows the
wrong three seconds.  Instead: take the last sample above cruise, then the
first sample after it under 0.15 m/s.  That instant is "stopped" on the same
clock as the attitude being scored, on any mission shape.

The endpoint is the peak attitude excursion inside that window - a value on
EVERY run including the passes, rather than a rare binary.

Usage: stopfix_score.py --csv .../stopfix_ab2.csv
       stopfix_score.py --glob '.../archive/shm_trace/*wkc_settle*.json'
"""
import json, glob, os, sys, csv, math, argparse, statistics as st

R2D = 57.2958
LIMIT_DEG = 28.65          # SafetyChecker's 0.5 rad, in degrees

ap = argparse.ArgumentParser()
ap.add_argument("--csv"); ap.add_argument("--glob")
a = ap.parse_args()

jobs = []          # (arm, rep, verdict, path)
if a.csv:
    for r in csv.DictReader(open(a.csv)):
        if r.get("snapshot") and r["snapshot"] != "NONE":
            jobs.append((r["arm"], r.get("rep", "?"), r.get("verdict", "?"), r["snapshot"]))
elif a.glob:
    for p in sorted(glob.glob(a.glob)):
        b = os.path.basename(p)
        arm = next((k for k in ("HARSH", "BOUND", "WATCH", "BLIND") if k in b), "?")
        jobs.append((arm, "?", "?", p))
else:
    sys.exit("need --csv or --glob")


def speed(x):
    return math.hypot(x["vx"], x["vy"])


rows = []
for arm, rep, verdict, path in jobs:
    try: d = json.load(open(path))
    except Exception: continue
    R = [x for x in (d.get("records") or [])
         if x.get("pitch") is not None and x.get("vx") is not None]
    if len(R) < 200: continue

    cruise = max(speed(x) for x in R)
    k = None
    if cruise >= 0.5:
        last_fast = max(i for i, x in enumerate(R) if speed(x) > 0.6 * cruise)
        k = next((i for i in range(last_fast, len(R)) if speed(R[i]) < 0.15), None)
    # An E-STOP also brings the robot to a halt, and that halt is a fall in
    # progress, not a mission stop.  Scoring it would count OPEN-26's failure
    # mode as a finish-line excursion.
    if k is not None and R[k].get("op_mode") == 2:
        k = None
    if k is None or k >= len(R) - 50:
        rows.append(dict(arm=arm, rep=rep, verdict=verdict, reached=False))
        continue

    W = R[k:]
    pk_p = max(abs(x["pitch"]) for x in W) * R2D
    pk_r = max(abs(x["roll"])  for x in W) * R2D
    z0 = st.median([x["z"] for x in R[max(0, k - 200):k]] or [R[k]["z"]])
    t_lim = next((x["t"] - R[k]["t"] for x in W
                  if max(abs(x["pitch"]), abs(x["roll"])) * R2D >= LIMIT_DEG), None)
    rows.append(dict(arm=arm, rep=rep, verdict=verdict, reached=True,
                     dwell=W[-1]["t"] - R[k]["t"],
                     pk_pitch=pk_p, pk_roll=pk_r, worst=max(pk_p, pk_r),
                     sag=z0 - min(x["z"] for x in W),
                     estop=any(x.get("op_mode") == 2 for x in W),
                     t_lim=t_lim))

scored = [r for r in rows if r.get("reached")]
print(f"\n  scored {len(rows)} snapshots, {len(scored)} came to a stop")
if not scored:
    print("  nothing to compare - no run ever came to a stop"); sys.exit(0)


def band(v):
    return f"{st.median(v):6.2f} (mean {st.mean(v):6.2f}, max {max(v):6.2f})"


print(f"\n  {'arm':<7} {'n':>3}  {'peak |pitch| after stopping':<34} "
      f"{'peak |roll|':<34} {'sag':>7}  {'over 28.65':>10}")
for arm in sorted({r["arm"] for r in scored}):
    g = [r for r in scored if r["arm"] == arm]
    over = sum(1 for r in g if r["t_lim"] is not None)
    print(f"  {arm:<7} {len(g):>3}  {band([r['pk_pitch'] for r in g]):<34} "
          f"{band([r['pk_roll'] for r in g]):<34} "
          f"{st.median([r['sag'] for r in g]):>6.3f}m  {over:>4}/{len(g):<4}")

arms = sorted({r["arm"] for r in scored})
if len(arms) == 2:
    A = [r["worst"] for r in scored if r["arm"] == arms[0]]
    B = [r["worst"] for r in scored if r["arm"] == arms[1]]
    print(f"\n  worst-axis excursion after the stop:"
          f"   {arms[0]} {st.median(A):.2f} deg   {arms[1]} {st.median(B):.2f} deg"
          f"   (safety limit {LIMIT_DEG})")
    # Mann-Whitney U, normal approximation with tie-corrected midranks (no scipy here)
    allv = sorted([(v, 0) for v in A] + [(v, 1) for v in B])
    ranks = {}
    i = 0
    while i < len(allv):
        j = i
        while j + 1 < len(allv) and allv[j + 1][0] == allv[i][0]: j += 1
        for t in range(i, j + 1): ranks[t] = (i + j) / 2 + 1
        i = j + 1
    RA = sum(ranks[i] for i, (_, s) in enumerate(allv) if s == 0)
    na, nb = len(A), len(B)
    U = RA - na * (na + 1) / 2
    sd = (na * nb * (na + nb + 1) / 12) ** 0.5
    z = (U - na * nb / 2) / sd if sd else 0.0
    p = math.erfc(abs(z) / 2 ** 0.5)
    print(f"  Mann-Whitney  U={U:.0f}  z={z:+.2f}  p={p:.3f}"
          f"   {'DIFFERENT' if p < 0.05 else 'not separable at this n'}")

nf = [r for r in rows if not r.get("reached")]
if nf:
    print(f"\n  {len(nf)} run(s) never came to a stop - they failed while moving,"
          f" a different mode, excluded:")
    for arm in sorted({r["arm"] for r in nf}):
        print(f"      {arm}: {sum(1 for r in nf if r['arm'] == arm)}")
