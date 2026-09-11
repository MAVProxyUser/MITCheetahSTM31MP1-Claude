#!/usr/bin/env python3
"""What happens in the moment a run enters OPEN-28's second mode?

Established: falls are not the tail of ordinary variation. Fitting the body of
the peak-attitude distribution and extrapolating under-predicts the crossing
rate by 5x pooled and 69x inside the largest single course family, so there is
a genuine second mode and about 13% of runs enter it. Established too: no
RUN-LEVEL property separates the runs that enter it from the runs that do not,
even scored on all 34 crossings rather than the 14 falls - ride height, roll,
foot force, sag, approach speed, geometry, duration, all flat.

That leaves something momentary. This looks for it, and the control is the
strongest kind available: **each run against its own earlier cruise.** Every
run-level confound - this course, this seed, this ride height, this day's
build - is identical between the window and its control by construction, so
anything that separates is genuinely about the moment.

For each crossing run:
  t_e   = when the worst axis first passes ENTER deg (default 12, well below
          SafetyChecker's 28.65, so this is entry to the mode and not the
          E-stop itself)
  event = the WINDOW seconds before t_e
  ctrl  = every other non-overlapping WINDOW-second slice of that same run's
          upright cruise

Channels are chosen to be things a momentary disturbance would show in:
body rates, ride height, per-leg tracking error, vertical foot force, and the
contact pattern - a trot alternates 1001/0110, so all-four-off is a stumble
and all-four-on at speed is not a trot.

Usage: open28_entry.py --csv .../open28_subcourse.csv [--enter 12] [--window 0.6]
"""
import os as _os, sys as _sys; _sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))  # noqa: E702
from snapio import load_json  # archive snapshots may be compacted to .json.zst; this resolves either
import csv, json, os, math, argparse, statistics as st

R2D = 57.2958
ap = argparse.ArgumentParser()
ap.add_argument("--csv", action="append", required=True)
ap.add_argument("--enter", type=float, default=12.0)
ap.add_argument("--window", type=float, default=0.6)
ap.add_argument("--limit", type=float, default=28.65)
a = ap.parse_args()


def rows():
    out = []
    for c in a.csv:
        seen = set()
        for r in csv.DictReader(open(c)):
            rid = r.get("run_id")
            if rid and rid in seen:
                continue
            if rid:
                seen.add(rid)
            out.append(r)
    return out


def feats(W):
    """One window -> the numbers a momentary disturbance would move."""
    if len(W) < 20:
        return None
    n0 = sum(1 for x in W if sum(1 for i in range(4) if x.get(f"c{i}", 0) > 0) == 0)
    n4 = sum(1 for x in W if sum(1 for i in range(4) if x.get(f"c{i}", 0) > 0) == 4)
    return dict(
        wy=max(abs(x["wy"]) for x in W),
        wx=max(abs(x["wx"]) for x in W),
        wz=max(abs(x["wz"]) for x in W),
        zmin=min(x["z"] for x in W),
        zdrop=max(x["z"] for x in W) - min(x["z"] for x in W),
        terr=max(max(abs(x.get(f"track_err{i}", 0)) for i in range(4)) for x in W),
        fzmin=min(sum(x.get(f"foot_fz{i}", 0) for i in range(4)) for x in W),
        fzmax=max(sum(x.get(f"foot_fz{i}", 0) for i in range(4)) for x in W),
        airborne=n0 / len(W),
        allfour=n4 / len(W),
        period=max(x.get("period_ms", 0) for x in W),
    )


ev, ct = [], []
nrun = 0
for r in rows():
    p = r.get("snapshot", "")
    if not p or p == "NONE" or not os.path.exists(p):
        continue
    try:
        pk = max(float(r.get("peak_pitch") or 0), float(r.get("peak_roll") or 0))
    except ValueError:
        continue
    if pk < a.limit:
        continue                       # only runs that entered the mode
    try:
        d = load_json(p)
    except Exception:
        continue
    R = [x for x in d.get("records", []) if x.get("pitch") is not None
         and x.get("vx") is not None and x.get("wy") is not None]
    if len(R) < 600:
        continue
    worst = lambda x: max(abs(x["pitch"]), abs(x["roll"])) * R2D
    te = next((x["t"] for x in R if x["t"] > 6.0 and worst(x) >= a.enter), None)
    if te is None:
        continue
    E = [x for x in R if 0 < te - x["t"] <= a.window]
    fe = feats(E)
    if not fe:
        continue
    # this run's OWN earlier cruise, non-overlapping, upright and moving
    cs, t = [], R[0]["t"] + 6.0
    while t + a.window < te - a.window:
        S = [x for x in R if t <= x["t"] < t + a.window]
        if (len(S) >= 20 and all(worst(x) < a.enter for x in S)
                and st.median([math.hypot(x["vx"], x["vy"]) for x in S]) > 1.5):
            f = feats(S)
            if f:
                cs.append(f)
        t += a.window
    if not cs:
        continue
    nrun += 1
    ev.append(fe)
    ct.extend(cs)

print(f"\n  {nrun} runs entered the mode and had usable control windows")
print(f"  {len(ev)} entry windows ({a.window}s before the attitude passes {a.enter} deg)")
print(f"  {len(ct)} control windows from those same runs' own earlier cruise")
if not ev or not ct:
    raise SystemExit("  not enough data")

keys = ["wy", "wx", "wz", "zmin", "zdrop", "terr", "fzmin", "fzmax",
        "airborne", "allfour", "period"]
print(f"\n  {'channel':<12} {'entry':>9} {'own cruise':>12} {'z':>7} {'p':>8}")
res = []
for k in keys:
    x = [q[k] for q in ev]
    y = [q[k] for q in ct]
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
    U = RA - na * (na + 1) / 2
    sd = (na * nb * (na + nb + 1) / 12) ** 0.5
    z = (U - na * nb / 2) / sd if sd else 0.0
    p = math.erfc(abs(z) / 2 ** 0.5)
    res.append((p, k, st.median(x), st.median(y), z))
    print(f"  {k:<12} {st.median(x):>9.3f} {st.median(y):>12.3f} {z:>+7.2f} {p:>8.4f}"
          f"{'  <--' if p < 0.05 / len(keys) else ''}")
print(f"  (Bonferroni over {len(keys)} channels: {0.05/len(keys):.4f})")
res.sort()
print(f"\n  strongest separation: {res[0][1]} "
      f"(entry {res[0][2]:.3f} vs own cruise {res[0][3]:.3f}, p={res[0][0]:.4f})")
