#!/usr/bin/env python3
"""Commanded joint torque against the motors' limits, at the collapse.

OPEN-28's moving crossings: >=6 s of nominal trot, then z drops ~3 cm in
~0.3 s and attitude jumps 25 deg in 0.2 s. Joints, height and attitude depart
within the same ~50 ms; commanded torque follows 0.18 s LATER. Something at
the foot gives before the controller does anything.

The candidate is a silent clip. The LegController's clamp is
setMaxTorqueCheetah3(208.5) - a Cheetah 3 number; the bridge applies
tau = kp*(q_des-q) + kd*(qd_des-qd) + tau_ff with no clamp; Gazebo enforces the
SDF's 23.7 N*m (abad, hip) and 35.55 N*m (knee). The QP is allowed f_max=175 N
vertical and +-70 N in the cone per foot, whose joint torques at the Go1's
levers exceed those limits. Nothing tells the controller.

BRIDGE_DUMP=<path> records every 5th command at 100 Hz:
  t, q[12], qd_des[12], q_des[12], kp[12], kd[12], tau_ff[12]
In stance kp=0, so tau_ff is the MPC's commanded joint torque. This reads it
back and asks, per crossing and against each run's own cruise baseline:

  * how close does |tau_ff| get to the limit in ordinary cruise
  * does any joint EXCEED the limit in the 0.5 s before the crossing
  * does that exceedance precede the body sinking

Joint order is MIT's: leg-major, (abad, hip, knee) per leg, legs FR FL RR RL.

Usage: open28_torque.py --campaign open28_clip
"""
import csv, json, os, math, glob, argparse, statistics as st

R2D = 57.2958
LIM = [23.7, 23.7, 35.55] * 4
ap = argparse.ArgumentParser()
ap.add_argument("--campaign", required=True)
ap.add_argument("--root", default="/Users/kfinisterre/Desktop/Cheetah/rundata/campaigns")
ap.add_argument("--limit", type=float, default=28.65)
a = ap.parse_args()


def load_dump(p):
    out = []
    try:
        for line in open(p):
            f = line.strip().split(",")
            if len(f) < 73:
                continue
            try:
                t = float(f[0]); tau = [float(v) for v in f[61:73]]
            except ValueError:
                continue
            out.append((t, tau))
    except Exception:
        pass
    return out


rows = list(csv.DictReader(open(os.path.join(a.root, a.campaign + ".csv"))))
seen = set()
res = []
for r in rows:
    rid = r.get("run_id")
    if rid and rid in seen:
        continue
    if rid:
        seen.add(rid)
    snap = r.get("snapshot", "")
    dump = r.get("bridge_dump", "") or ""
    if not snap or snap == "NONE" or not os.path.exists(snap) or not os.path.exists(dump):
        continue
    d = json.load(open(snap))
    ALL = [x for x in d.get("records", []) if x.get("t") is not None]
    R = [x for x in ALL if x.get("vx") is not None and x.get("roll") is not None and x["t"] > 6.0]
    if len(R) < 600:
        continue
    k = next((i for i in range(1, len(R)) if R[i]["op_mode"] == 2 and R[i-1]["op_mode"] != 2), None)
    pre = R[:k] if k else R
    att = lambda x: max(abs(x["roll"]), abs(x["pitch"])) * R2D
    ic = next((i for i, x in enumerate(pre) if att(x) >= a.limit), None)
    D = load_dump(dump)
    if len(D) < 500:
        continue
    # The bridge stamps wall time; the trace stamps its own clock. Align on the
    # bridge's first command against the trace's FIRST record - not R[0], which
    # is already filtered to t > 6 s and would put the window six seconds off.
    # Then refine: the 100 Hz dump and the 500 Hz trace both carry the
    # controller's q_des-vs-q behaviour, so cross-correlate |tau_ff| sum
    # against the trace's own feed-forward diagnostic (track_err[3] =
    # mean |tau_ff|) over the whole run and take the lag that maximises it.
    off = D[0][0] - ALL[0]["t"]
    try:
        tr = [(x["t"], x.get("track_err3", 0.0)) for x in ALL if x.get("track_err3") is not None]
        if len(tr) > 2000:
            import bisect
            tt = [t for t, _ in tr]; tv = [v for _, v in tr]
            best, bl = -1e18, 0.0
            for lag in [off + k * 0.05 for k in range(-40, 41)]:
                acc = 0.0; n = 0
                for t, tau in D[::5]:
                    i = bisect.bisect_left(tt, t - lag)
                    if 0 < i < len(tt):
                        acc += tv[i] * sum(abs(v) for v in tau); n += 1
                if n and acc / n > best:
                    best, bl = acc / n, lag
            off = bl
    except Exception:
        pass
    ratio = lambda tau: max(abs(tau[j]) / LIM[j] for j in range(12))
    cruise = [ratio(tau) for t, tau in D if any(abs(t - off - x["t"]) < 0.01 and math.hypot(x["vx"], x["vy"]) > 1.5 and att(x) < 8 for x in pre[::50])]
    rec = dict(arm=r.get("arm") or r.get("course"), rep=r.get("rep"), crossed=(ic is not None and k is not None),
               cruise_p50=st.median(cruise) if cruise else float("nan"),
               cruise_p99=sorted(cruise)[int(0.99 * len(cruise))] if cruise else float("nan"))
    if rec["crossed"]:
        tc = pre[ic]["t"] + off
        W = [(t, tau) for t, tau in D if 0 <= tc - t <= 0.5]
        rec["pre_max"] = max((ratio(tau) for _, tau in W), default=float("nan"))
        over = [(tc - t, max(range(12), key=lambda j: abs(tau[j]) / LIM[j])) for t, tau in W if ratio(tau) >= 1.0]
        rec["first_over"] = over[0][0] if over else None
        rec["over_joint"] = over[0][1] if over else None
        zb = st.median(x["z"] for x in pre[:ic] if 1.0 < pre[ic]["t"] - x["t"] <= 4.0)
        tz = next((x["t"] for x in pre[:ic+1] if 0 <= pre[ic]["t"] - x["t"] <= 1.5 and x["z"] < zb - 0.02), None)
        rec["sink_at"] = (pre[ic]["t"] - tz) if tz else None
    res.append(rec)

print(f"\n  runs with a trace and a bridge dump: {len(res)}   crossed: {sum(1 for q in res if q['crossed'])}")
by = {}
for q in res:
    by.setdefault(q["arm"], []).append(q)
print(f"\n  |tau_ff| / motor limit, in ORDINARY CRUISE (all runs)")
for arm, g in sorted(by.items()):
    print(f"    {arm:<12} n={len(g):>3}   p50 {st.median(q['cruise_p50'] for q in g):.2f}   "
          f"p99 {st.median(q['cruise_p99'] for q in g):.2f}   "
          f"(1.00 = at the limit; the sim clips above it)")
X = [q for q in res if q["crossed"]]
if X:
    print(f"\n  AT THE CROSSING, last 0.5 s ({len(X)} crossings)")
    print(f"    peak |tau_ff|/limit: median {st.median(q['pre_max'] for q in X):.2f}   "
          f"over 1.0 in {sum(1 for q in X if q['first_over'] is not None)}/{len(X)}")
    fo = [q["first_over"] for q in X if q["first_over"] is not None]
    if fo:
        print(f"    first exceedance: median {st.median(fo):.3f} s before the crossing")
        both = [(q["first_over"], q["sink_at"]) for q in X if q["first_over"] is not None and q["sink_at"] is not None]
        if both:
            print(f"    exceedance BEFORE the 2 cm sink in {sum(1 for f, z in both if f > z)}/{len(both)}")
        names = ["FR", "FL", "RR", "RL"]; jn = ["abad", "hip", "knee"]
        import collections
        c = collections.Counter(f"{names[q['over_joint']//3]}-{jn[q['over_joint']%3]}" for q in X if q["over_joint"] is not None)
        print(f"    joint that exceeds first: {dict(c)}")
