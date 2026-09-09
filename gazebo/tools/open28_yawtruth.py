#!/usr/bin/env python3
"""Is the "uncommanded" yaw actually the MPC correcting an estimator error?

OPEN-28's second mode is entered by a yaw the FOLLOWER did not ask for: body
1.70 rad/s against a 0.30 rad/s command, 6.4x, leading the attitude excursion
by >=2 s in 29 of 29 entries. Eleven downstream hypotheses are dead, including
the commanded yaw cap and the whole MPC cost vector.

"Uncommanded" has so far meant "not commanded by the path follower". There is
a second commander. The MPC tracks a yaw REFERENCE against the state
estimator's yaw, and the estimator has a known weakness in exactly this DOF:
the KF trusts each foot as a fixed world point on the gait schedule's say-so
(ContactEstimator::run() copies contactPhase - there is no contact estimation),
and yaw is the direction most sensitive to a mis-trusted foot.

If the estimate drifts, the MPC will command a large, entirely correct yaw
moment to fix an error that is not there, and the real body will spin while the
follower believes it is going straight. From outside that is indistinguishable
from a disturbance - which is why it survived eleven tests aimed downstream.

gz pose truth carries yaw, so the estimate can be checked against it directly:

  drift(t) = wrap(yaw_est(t) - yaw_truth(t))

and the question is whether drift grows BEFORE the yaw rate does. Ordering is
the whole test - drift during a spin proves nothing, since a spinning body will
out-run any estimator.

Usage: open28_yawtruth.py --campaign open28_yawtruth
"""
import csv, json, os, math, glob, argparse, statistics as st

R2D = 57.2958
ap = argparse.ArgumentParser()
ap.add_argument("--campaign", default="open28_yawtruth")
ap.add_argument("--root", default="/Users/kfinisterre/Desktop/Cheetah/rundata/campaigns")
ap.add_argument("--yaw", type=float, default=1.0, help="rad/s that marks the yaw event")
a = ap.parse_args()

csvp = os.path.join(a.root, a.campaign + ".csv")
rows, seen = [], set()
for r in csv.DictReader(open(csvp)):
    rid = r.get("run_id")
    if rid and rid in seen:
        continue
    if rid:
        seen.add(rid)
    rows.append(r)


def wrap(x):
    while x > math.pi: x -= 2 * math.pi
    while x < -math.pi: x += 2 * math.pi
    return x


def load_truth(p):
    """pose_feed writes {"t": wall, "p": {"0": [x,y,z,yaw,...]}} per line."""
    out = []
    try:
        for line in open(p):
            try:
                d = json.loads(line)
            except Exception:
                continue
            q = (d.get("p") or {}).get("0")
            if q and len(q) >= 4:
                out.append((d["t"], q[3]))
    except Exception:
        pass
    return out


ev = []
for r in rows:
    snap = r.get("snapshot", "")
    truth = r.get("truth", "")
    if not snap or snap == "NONE" or not os.path.exists(snap):
        continue
    if not truth or not os.path.exists(truth):
        continue
    try:
        d = json.load(open(snap))
    except Exception:
        continue
    R = [x for x in d.get("records", []) if x.get("wz") is not None
         and x.get("yaw") is not None and x.get("vx") is not None]
    T = load_truth(truth)
    if len(R) < 600 or len(T) < 300:
        continue
    # the two streams have different clocks; align on the yaw RATE profile by
    # cross-correlating d(yaw)/dt, which is sharp and present in both
    tt = [t for t, _ in T]
    ty = [y for _, y in T]
    dt_t = (tt[-1] - tt[0]) / max(1, len(tt) - 1)
    trate = [abs(wrap(ty[i + 1] - ty[i])) / dt_t for i in range(len(ty) - 1)]
    rrate = [abs(x["wz"]) for x in R]
    if not trate or not rrate:
        continue
    # coarse: match the index of the single largest yaw-rate excursion in each
    it = max(range(len(trate)), key=lambda i: trate[i])
    ir = max(range(len(rrate)), key=lambda i: rrate[i])
    off = tt[it] - R[ir]["t"]

    # the yaw event, on the record clock
    te = next((x["t"] for x in R if x["t"] > 6.0 and abs(x["wz"]) >= a.yaw), None)
    if te is None:
        continue
    def truth_at(tr):
        w = tr + off
        best = min(T, key=lambda q: abs(q[0] - w))
        return best[1] if abs(best[0] - w) < 0.10 else None
    # drift in a window ENDING before the yaw event, and in one during it
    pre = [x for x in R if 2.0 < te - x["t"] <= 4.0]
    dur = [x for x in R if abs(x["t"] - te) <= 0.5]
    dp = [abs(wrap(x["yaw"] - truth_at(x["t"]))) * R2D
          for x in pre if truth_at(x["t"]) is not None]
    dd = [abs(wrap(x["yaw"] - truth_at(x["t"]))) * R2D
          for x in dur if truth_at(x["t"]) is not None]
    if len(dp) < 20 or len(dd) < 10:
        continue
    ev.append((st.median(dp), st.median(dd), max(abs(x["wz"]) for x in R)))

print(f"\n  {len(ev)} runs with a yaw event, a trace and aligned pose truth")
if not ev:
    raise SystemExit("  nothing to compare yet - let the campaign collect")
pre = [e[0] for e in ev]
dur = [e[1] for e in ev]
print(f"\n  |yaw estimate - gz truth|, degrees:")
print(f"    2-4 s BEFORE the yaw event : median {st.median(pre):6.2f}   max {max(pre):6.2f}")
print(f"    during the yaw event       : median {st.median(dur):6.2f}   max {max(dur):6.2f}")
print(f"\n  Drift BEFORE the event is the test. Large drift only DURING it means")
print(f"  the estimator lost a spinning body, which proves nothing. Large drift")
print(f"  before it means the MPC was handed a yaw error that was not real, and")
print(f"  the spin is its correct response to a wrong number.")
