#!/usr/bin/env python3
"""OPEN-28's live hypothesis: the dog loses SUPPORT before it loses attitude.

Reading backward from the E-stop rather than forward from the last calm
sample, every fall trace examined so far shows the same order - ride height
sags first, and only then does the pitch run away:

    hp_gap14 rep6      z      pitch
      t-0.55s        0.271      5.1     sustained roll, height held
      t-0.50s        0.254      3.8     <- HEIGHT GOES FIRST
      t-0.45s        0.247      9.6
      t-0.35s        0.232     17.2
      t-0.20s        0.195     25.7
      t-0.05s        0.179     29.9     -> E-stop

kin_z tracks z all the way down, so the legs really are shortening: this is
loss of support, not the estimator being fooled by its own kinematics (which
is the OPEN-26 thread, and a different claim).

That was n = 4 falls with effects of 0.2 deg and 1.7 %, which is exactly where
small-sample luck bites. This scores it at whatever n the campaign has, and
reports the three things the picture predicts, each against passes from the
SAME courses:

  1. does z sag across the run before anything else happens
  2. is |roll| at cruise higher in the runs that end badly
  3. is the vertical foot-force sum lower

plus the sequencing itself: how long before the attitude leaves its band does
the height start dropping, measured per fall rather than asserted from two.

Usage: open28_support.py --csv .../open28_subcourse.csv
"""
import csv, json, os, math, argparse, statistics as st

R2D = 57.2958
LIMIT = 28.65

ap = argparse.ArgumentParser()
ap.add_argument("--csv", required=True)
ap.add_argument("--fallcol", default="fall")
ap.add_argument("--outcome", default="crossing", choices=("fall", "crossing"),
                help="THE EVENT TO STUDY. 'fall' was the wrong choice for a year "
                     "of this issue: 34 runs cross SafetyChecker's limit and only "
                     "14 of them fall, because the recovery ladder stands 59%% of "
                     "them back up. Recovery is downstream noise; the mechanism "
                     "lives at the crossing. Studying falls threw away 20 of 34 "
                     "events and is why every discriminant came back null.")
ap.add_argument("--limit", type=float, default=28.65)
a = ap.parse_args()

rows = [r for r in csv.DictReader(open(a.csv)) if r.get("course") or r.get("arm")]
seen, clean = set(), []
for r in rows:
    rid = r.get("run_id")
    if rid and rid in seen:            # stale: the run never started
        continue
    if rid:
        seen.add(rid)
    clean.append(r)
if len(clean) != len(rows):
    print(f"  dropped {len(rows)-len(clean)} stale row(s) on a repeated run id")


def speed(x):
    return math.hypot(x["vx"], x["vy"])


def profile(r):
    p = r.get("snapshot", "")
    if not p or p == "NONE" or not os.path.exists(p):
        return None
    try:
        d = json.load(open(p))
    except Exception:
        return None
    R = [x for x in d.get("records", []) if x.get("z") is not None and x.get("vx") is not None]
    if len(R) < 400:
        return None
    k = next((i for i in range(1, len(R)) if R[i]["op_mode"] == 2 and R[i-1]["op_mode"] != 2), None)
    end = R[k]["t"] if k else R[-1]["t"]

    # healthy cruise: upright and moving, and stopping well clear of the event
    cru = [x for x in R if x["t"] < end - 0.6 and speed(x) > 1.5
           and abs(x["pitch"]) * R2D < 10 and abs(x["roll"]) * R2D < 10]
    if len(cru) < 300:
        return None
    t0, t1 = cru[0]["t"], cru[-1]["t"]
    early = [x["z"] for x in cru if x["t"] - t0 <= 1.5]
    late = [x["z"] for x in cru if t1 - x["t"] <= 1.5]
    out = dict(
        z=st.median([x["z"] for x in cru]),
        sag=st.median(early) - st.median(late),
        roll=st.median([abs(x["roll"]) * R2D for x in cru]),
        fz=st.median([sum(x.get(f"foot_fz{i}", 0) for i in range(4)) for x in cru]),
        lead=None)

    # THE SEQUENCING, per fall: how long before the attitude leaves 10 deg does
    # the height first drop 3 cm below its cruise median and stay down?
    if k:
        zmed = out["z"]
        W = [x for x in R if x["t"] <= R[k]["t"]]
        t_att = next((x["t"] for x in W
                      if max(abs(x["pitch"]), abs(x["roll"])) * R2D >= 10.0
                      and x["t"] > t1 - 3.0), None)
        t_z = None
        for i, x in enumerate(W):
            if x["t"] < t1 - 3.0 or x["z"] > zmed - 0.03:
                continue
            if all(y["z"] < zmed - 0.02 for y in W[i:i+15]):
                t_z = x["t"]
                break
        if t_att is not None and t_z is not None:
            out["lead"] = t_att - t_z          # positive => height moved first
    return out


F, P = [], []
for r in clean:
    pr = profile(r)
    if pr:
        if a.outcome == "fall":
            hit = r.get(a.fallcol) not in ("none", "", None)
        else:
            try:
                hit = max(float(r.get("peak_pitch") or 0),
                          float(r.get("peak_roll") or 0)) >= a.limit
            except ValueError:
                hit = False
        (F if hit else P).append(pr)

print(f"\n  outcome = {a.outcome}: {len(F)} events, {len(P)} non-events")
if not F or not P:
    print("  need both to compare"); raise SystemExit

print(f"\n  {'quantity':<32} {'event':>9} {'non-ev':>9} {'z':>7} {'p':>7}")
for key, label in (("z", "median ride height (m)"),
                   ("sag", "height lost across the run (m)"),
                   ("roll", "median |roll| at cruise (deg)"),
                   ("fz", "median foot-force sum")):
    x = [q[key] for q in F]
    y = [q[key] for q in P]
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
    print(f"  {label:<32} {st.median(x):>9.3f} {st.median(y):>9.3f} {z:>+7.2f} {p:>7.3f}"
          f"{'  <-- ' if p < 0.0125 else ''}")
print(f"  (four tests: 0.0125 is the Bonferroni threshold, not 0.05)")

leads = [q["lead"] for q in F if q["lead"] is not None]
if leads:
    pos = sum(1 for v in leads if v > 0)
    print(f"\n  SEQUENCING, measured per fall (n={len(leads)}):")
    print(f"    height drops before the attitude leaves 10 deg in {pos}/{len(leads)} falls")
    print(f"    median lead {st.median(leads):+.3f} s   range {min(leads):+.3f} to {max(leads):+.3f}")
    print(f"    a positive lead means SUPPORT went first, which is the claim")
else:
    print("\n  no fall had both landmarks - sequencing not measurable here")
