#!/usr/bin/env python3
"""Score every stance exchange: is the new pair loaded before the old pair leaves?

OPEN-28's escalations begin at a diagonal exchange where the new stance pair
is loaded ~25 ms AFTER the old pair has left the ground, while the MPC is
still commanding torque on the airborne pair - a support hole, seen in 6/11
escalations and 4/313 ordinary exchanges. The contact table is behind the
feet, and CTRL_MPC_SCHED_LEAD shifts that table.

This scores the quantity the mechanism moves, at EVERY exchange in every run,
so a 20-rep arm carries hundreds of samples instead of a handful of falls:

  * new-pair loading delay (ms) after the new foot reaches the ground
  * support-hole rate: exchanges where the new pair loads after the old is off
  * MPC torque still on the old pair in the first 60 ms after exchange
  * escalations and crossings, second

Usage: open28_handoff.py --csv .../open28_sched.csv
"""
import csv, json, os, math, argparse, statistics as st, bisect, collections

R2D = 57.2958
ap = argparse.ArgumentParser()
ap.add_argument("--csv", required=True)
ap.add_argument("--limit", type=float, default=28.65)
a = ap.parse_args()
gh = lambda y, l: y["z"] + y[f"foot_z{l}"]
DIAG = {0: 3, 1: 2, 2: 1, 3: 0}


def exchanges(pre, i0, i1):
    """A touchdown is a descent through 1 cm by a foot that was genuinely in
    swing - above 2 cm at some point in the previous 50 ms. Without that, the
    foot height jittering around the 1 cm line double-counts: 36k 'exchanges'
    in 30 runs where a 3-4 Hz trot over 40 s of cruise produces ~600 real
    touchdowns per run, and every phantom scores as an instant, hole-free
    exchange that dilutes the rate."""
    out = []
    for n in range(i0 + 26, i1):
        for l in range(4):
            if gh(pre[n-1], l) >= 0.01 and gh(pre[n], l) < 0.01 \
                    and any(gh(pre[m], l) > 0.02 for m in range(n - 25, n)):
                out.append((n, l))
    return out


def score(pre, D, Dt, off, n, l):
    new = [l, DIAG[l]]; old = [m for m in range(4) if m not in new]
    t0 = pre[n]["t"]; W = pre[n:n+45]
    fz = lambda y, m: y.get(f"foot_fz{m}", 0)
    t_off = next(((y["t"] - t0) * 1000 for y in W if all(fz(y, m) < 1.0 for m in old)), None)
    t_on = next(((y["t"] - t0) * 1000 for y in W if sum(fz(y, m) for m in new) >= 4.0), None)
    a_, b_ = bisect.bisect_left(Dt, t0 + off), bisect.bisect_left(Dt, t0 + off + 0.06)
    seg = D[a_:b_]
    tau_old = st.median(max(abs(tf[3*m+2]) for m in old) for _, tf in seg) if seg else float("nan")
    return dict(delay=t_on, hole=(t_off is not None and t_on is not None and t_on > t_off), tau_old=tau_old,
                drop=pre[n]["z"] - min(y["z"] for y in W))


rows, seen = [], set()
for r in csv.DictReader(open(a.csv)):
    rid = r.get("run_id")
    if rid and rid in seen:
        continue
    if rid:
        seen.add(rid)
    rows.append(r)
by = collections.defaultdict(lambda: dict(ex=[], runs=0, cross=0, esc=0))
for r in rows:
    dp, sp_ = r.get("bridge_dump", ""), r.get("snapshot", "")
    if not (dp and os.path.exists(dp) and sp_ and sp_ != "NONE" and os.path.exists(sp_)):
        continue
    D = []
    for line in open(dp):
        f = line.strip().split(",")
        if len(f) >= 73:
            try: D.append((float(f[0]), [float(v) for v in f[61:73]]))
            except ValueError: pass
    d = json.load(open(sp_)); ALL = [x for x in d["records"] if x.get("t") is not None]
    R = [x for x in ALL if x.get("vx") is not None and x.get("roll") is not None and x.get("z") is not None and x.get("foot_z0") is not None and x["t"] > 6.0]
    if len(R) < 800 or len(D) < 500:
        continue
    k = next((i for i in range(1, len(R)) if R[i]["op_mode"] == 2 and R[i-1]["op_mode"] != 2), None)
    pre = R[:k] if k else R; off = D[0][0] - ALL[0]["t"]; Dt = [q[0] for q in D]
    sp = lambda x: math.hypot(x["vx"], x["vy"])
    g = by[r.get("course", "?")]; g["runs"] += 1
    ic = next((i for i, x in enumerate(pre) if max(abs(x["roll"]), abs(x["pitch"])) * R2D >= a.limit), None)
    if ic is not None and k and sp(pre[ic]) > 0.8:
        g["cross"] += 1
    cru = [j for j in range(600, len(pre) - 100) if sp(pre[j]) > 1.5 and max(abs(pre[j]["pitch"]), abs(pre[j]["roll"])) * R2D < 10]
    if len(cru) > 2:
        for n, l in exchanges(pre, cru[0], cru[-1]):
            g["ex"].append(score(pre, D, Dt, off, n, l))

print(f"\n  {'arm':<24} {'runs':>4} {'exchanges':>9} {'delay p50':>9} {'delay p90':>9} {'hole rate':>10} {'tau old':>8} {'drop p99':>9} {'crossed':>8}")
for arm in sorted(by):
    g = by[arm]; ex = g["ex"]
    if not ex:
        continue
    dl = sorted(q["delay"] for q in ex if q["delay"] is not None)
    holes = sum(1 for q in ex if q["hole"])
    to = [q["tau_old"] for q in ex if q["tau_old"] == q["tau_old"]]
    dr = sorted(q["drop"] for q in ex)
    print(f"  {arm:<24} {g['runs']:>4} {len(ex):>9} {st.median(dl):>8.0f}ms {dl[int(0.9*len(dl))]:>8.0f}ms "
          f"{100*holes/len(ex):>9.2f}% {st.median(to):>8.1f} {100*dr[int(0.99*len(dr))]:>7.1f}cm {g['cross']:>4}/{g['runs']}")
print("\n  the mechanism predicts hole rate and old-pair torque fall as the table is brought back into step with the feet.")
