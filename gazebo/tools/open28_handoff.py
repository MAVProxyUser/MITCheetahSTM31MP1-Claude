#!/usr/bin/env python3
"""Score every stance exchange against the SCHEDULE, on honest signals.

The first version of this scorer read the trace's `foot_fz` as a foot FORCE.
It is a foot SPEED (m/s) - `ShmTrace.h` says so in its header, and has since
2026-09-04 - so "new pair loaded (Σfz ≥ 4)" measured "new feet moving fast",
"old pair off (fz < 1)" measured "old feet planted", and the "support hole"
the lead campaign was launched to test was a misread. This version uses only
signals whose writers were checked (`robot/src/RobotRunner.cpp`):

  c0..c3      contactEstimate = the gait SCHEDULE (0 in swing, 0..1 in stance)
  foot_z0..3  FK foot height terms; z + foot_z = height above ground
  z           estimated body height (kin_z is the detector's FK height)
  bridge dump tau_ff per joint at 100 Hz - the torque the leg controller
              actually sent, on the bridge's wall clock; first dumped row =
              first trace record (checked against the E-stop edge, +-6 ms)

An EXCHANGE is a scheduled flip: the tick c_l rises for a diagonal pair. For
each one, relative to that flip:

  early touchdown   how long before the flip each NEW foot met the ground
  old-pair torque   max knee |tau_ff| of the OLD pair at fixed offsets - the
                    MPC de-loads it when the TABLE says swing, so the drop
                    time is where CTRL_MPC_SCHED_LEAD shows (0 / -22 / -44 ms
                    if the knob works: one MPC step is 22 ms here)
  unsupported       ms in [-80,+60] with NEITHER pair both on the ground and
                    above 3 N*m at the knee
  physical gap      time from the last old liftoff to the first new touchdown
                    (positive = both pairs airborne, i.e. flight)
  body drop         z lost in the 90 ms after the flip; sink = > 2 cm

Usage: open28_handoff.py --csv .../open28_sched.csv [--limit 28.65]
"""
import csv, json, os, math, argparse, statistics as st, bisect, collections

R2D = 57.2958
ap = argparse.ArgumentParser()
ap.add_argument("--csv", required=True)
ap.add_argument("--limit", type=float, default=28.65)
a = ap.parse_args()
gh = lambda y, l: y["z"] + y[f"foot_z{l}"]
DIAG = {0: 3, 1: 2, 2: 1, 3: 0}
OFFS = [-80, -60, -44, -30, -22, -14, -8, 0, 8, 14, 22, 30, 44, 60]   # ms
TAU_ON = 3.0     # N*m of knee feed-forward that counts as "commanded"
GROUND = 0.01    # m


def align(D, ALL):
    """Bridge clock -> trace clock offset.

    Coarse: the bridge's first dumped command against the trace's first
    record. That is only good to a few tens of ms - the motor task that sends
    commands starts BEFORE the RobotRunner whose clock the trace carries, and
    the gap varies per run (run 4527 read 44 ms off, which shifted its whole
    torque timeline by two segments while its trace-only vz/z timelines were
    identical to the run beside it). Fine: cross-correlate the bridge's
    Σ|tau_ff| against the trace's track_err3 (= mean |tau_ff|), mean-removed,
    but ONLY within +-80 ms of the coarse guess in 2 ms steps. The gait makes
    that correlation periodic at the 110 ms half-cycle, so a wide search locks
    onto the wrong tooth (the first attempt, +-2 s in 50 ms steps, moved
    offsets by +50..+106 ms); a window narrower than half a period cannot.
    Validated against the E-stop edge (legs zeroed in both clocks) on FAIL runs."""
    off = D[0][0] - ALL[0]["t"]
    tr = [(x["t"], x["track_err3"]) for x in ALL if x.get("track_err3") is not None and x["t"] > 6.0]
    if len(tr) < 2000:
        return off
    tt = [t for t, _ in tr]; tv = [v for _, v in tr]; mv = st.mean(tv)
    S = [(t, sum(abs(v) for v in tau)) for t, tau in D[::2]]
    ms = st.mean(s for _, s in S)

    def score(lag):
        acc = 0.0; n = 0
        for t, s_ in S:
            i = bisect.bisect_left(tt, t - lag)
            if 0 < i < len(tt):
                acc += (tv[i] - mv) * (s_ - ms); n += 1
        return acc / n if n else -1e18
    return max((off + k * 0.002 for k in range(-40, 41)), key=score)


def crossings(seq, thr, rising, hold):
    """Indices where seq crosses thr in the given direction and holds for
    `hold` samples - the debounce that kills 1 cm jitter double-counts."""
    out = []
    for i in range(1, len(seq) - hold):
        if rising:
            ok = seq[i-1] < thr and all(v >= thr for v in seq[i:i+hold])
        else:
            ok = seq[i-1] >= thr and all(v < thr for v in seq[i:i+hold])
        if ok:
            out.append(i)
    return out


def score_exchange(pre, n, new, old, D, Dt, off):
    t0 = pre[n]["t"]
    lo = max(0, n - 60); hi = min(len(pre), n + 60)   # +-120 ms at 500 Hz
    W = pre[lo:hi]
    rel = lambda i: (W[i]["t"] - t0) * 1000.0
    # physical touchdown of each new foot (last downward crossing before +80 ms)
    td = []
    for l in new:
        seq = [gh(y, l) for y in W]
        c = [rel(i) for i in crossings(seq, GROUND, False, 3) if rel(i) <= 80]
        td.append(c[-1] if c else None)
    # physical liftoff of each old foot (first upward crossing after -80 ms)
    lo_ = []
    for l in old:
        seq = [gh(y, l) for y in W]
        c = [rel(i) for i in crossings(seq, GROUND, True, 3) if rel(i) >= -80]
        lo_.append(c[0] if c else None)
    # torque timelines from the bridge, nearest sample within 6 ms
    def knee(t_ms, legs):
        i = bisect.bisect_left(Dt, t0 + off + t_ms / 1000.0)
        cands = [j for j in (i - 1, i) if 0 <= j < len(Dt) and abs(Dt[j] - (t0 + off + t_ms / 1000.0)) <= 0.006]
        if not cands:
            return None
        j = min(cands, key=lambda j: abs(Dt[j] - (t0 + off + t_ms / 1000.0)))
        return max(abs(D[j][1][3*m + 2]) for m in legs)
    tl_old = [knee(o, old) for o in OFFS]
    tl_new = [knee(o, new) for o in OFFS]
    fine = list(range(-80, 62, 2))
    ko = [(o, knee(o, old)) for o in fine]; kn = [(o, knee(o, new)) for o in fine]
    drop = next((o for o, v in ko if v is not None and v < TAU_ON), None) if any(v is not None and v >= TAU_ON for o, v in ko if o <= -60) else None
    # UNSUPPORTED time: ms in [-80,+60] where NEITHER pair is both on the
    # ground and commanded above threshold - a swinging leg's PD torque is
    # not support; an early-landed foot being pushed down by its swing PD is
    def down(o, legs):          # any foot of the pair on the ground at offset o (trace, 500 Hz)
        i = n + int(round(o / 2.0))
        return 0 <= i < len(pre) and any(gh(pre[i], m) < GROUND for m in legs)
    both_off = 2 * sum(1 for (o, vo), (_, vn) in zip(ko, kn)
                       if vo is not None and vn is not None
                       and not (vo >= TAU_ON and down(o, old)) and not (vn >= TAU_ON and down(o, new)))
    rise = next((o for o, v in kn if v is not None and v >= TAU_ON and (drop is None or o >= drop)), None)
    # body vertical velocity and height at the same offsets (trace, 500 Hz):
    # an unsupported body accelerates down at g, and that needs no bridge
    def at(o, key):
        i = n + int(round(o / 2.0))
        return pre[i][key] if 0 <= i < len(pre) else None
    tl_vz = [at(o, "vz") for o in OFFS]
    tl_z = [at(o, "z") for o in OFFS]
    z0 = pre[n]["z"]; Wz = pre[n:n+45]
    dropz = z0 - min(y["z"] for y in Wz) if Wz else 0.0
    tds = [v for v in td if v is not None]; los = [v for v in lo_ if v is not None]
    return dict(early=[-v for v in tds],                     # + = landed before the flip
                lift=los,                                     # rel. flip, + = after
                phys_gap=(min(tds) - max(los)) if (len(tds) == 2 and len(los) == 2) else None,
                drop=drop, rise=rise, both_off=both_off,
                cmd_gap=(rise - drop) if (drop is not None and rise is not None) else None,
                tl_old=tl_old, tl_new=tl_new, tl_vz=tl_vz, tl_z=tl_z, dropz=dropz)


rows, seen = [], set()
for r in csv.DictReader(open(a.csv)):
    rid = r.get("run_id")
    if rid and rid in seen:
        continue
    if rid:
        seen.add(rid)
    rows.append(r)
by = collections.defaultdict(lambda: dict(ex=[], runs=0, cross=0, cruise_s=0.0, gaps=0))
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
    R = [x for x in ALL if x.get("vx") is not None and x.get("roll") is not None and x.get("z") is not None
         and x.get("foot_z0") is not None and x.get("c0") is not None and x["t"] > 6.0]
    if len(R) < 800 or len(D) < 500:
        continue
    k = next((i for i in range(1, len(R)) if R[i]["op_mode"] == 2 and R[i-1]["op_mode"] != 2), None)
    pre = R[:k] if k else R
    off = align(D, ALL); Dt = [q[0] for q in D]
    sp = lambda x: math.hypot(x["vx"], x["vy"])
    g = by[r.get("course", "?")]; g["runs"] += 1
    # RECEIVE GAPS: >= 30 ms between consecutive dumped commands = the bridge
    # did not process packets for that long (OPEN-28's initiator). Trace clock.
    gaps = [Dt[i] - off for i in range(len(Dt) - 1) if (Dt[i+1] - Dt[i]) * 1000 >= 30]
    ic = next((i for i, x in enumerate(pre) if max(abs(x["roll"]), abs(x["pitch"])) * R2D >= a.limit), None)
    t_cross = None
    if ic is not None and k and sp(pre[ic]) > 0.8:
        g["cross"] += 1; t_cross = pre[ic]["t"]
    cru = [j for j in range(600, len(pre) - 100) if sp(pre[j]) > 1.5 and max(abs(pre[j]["pitch"]), abs(pre[j]["roll"])) * R2D < 10]
    if len(cru) <= 2:
        continue
    i0, i1 = cru[0], cru[-1]
    g["cruise_s"] += pre[i1]["t"] - pre[i0]["t"]
    g["gaps"] += sum(1 for t in gaps if pre[i0]["t"] <= t <= pre[i1]["t"])
    for lead_leg in (0, 1):                       # pair A = {0,3} flips with c0, pair B = {1,2} with c1
        new = [lead_leg, DIAG[lead_leg]]; old = [m for m in range(4) if m not in new]
        for n in range(max(i0, 61), min(i1, len(pre) - 61)):
            if pre[n-1][f"c{lead_leg}"] <= 0 and pre[n][f"c{lead_leg}"] > 0:
                q = score_exchange(pre, n, new, old, D, Dt, off)
                t0 = pre[n]["t"]; j = bisect.bisect_left(gaps, t0 - 0.025)
                q["lethal_gap"] = j < len(gaps) and gaps[j] <= t0 + 0.005   # a freeze starting in [-25,+5] ms
                q["crossed"] = t_cross is not None and 0 <= t_cross - t0 <= 0.4
                g["ex"].append(q)

med = lambda v: st.median(v) if v else float("nan")
pct = lambda v, p: sorted(v)[int(p * (len(v) - 1))] if v else float("nan")
print(f"\n  {'arm':<24} {'runs':>4} {'exch':>6} {'early td p50':>12} {'p90':>6} {'old tau drop':>12} {'unsupp p50':>11} {'p90':>6} {'phys gap p50':>12} {'p90':>6} {'sink':>6} {'dz p50':>7} {'crossed':>8}")
for arm in sorted(by):
    g = by[arm]; ex = g["ex"]
    if not ex:
        continue
    early = [v for q in ex for v in q["early"]]
    drops = [q["drop"] for q in ex if q["drop"] is not None]
    cg = [q["both_off"] for q in ex]
    pg = [q["phys_gap"] for q in ex if q["phys_gap"] is not None]
    dz = [q["dropz"] for q in ex]
    sink = sum(1 for v in dz if v > 0.02)
    print(f"  {arm:<24} {g['runs']:>4} {len(ex):>6} {med(early):>10.0f}ms {pct(early,0.9):>4.0f}ms {med(drops):>10.0f}ms "
          f"{med(cg):>9.0f}ms {pct(cg,0.9):>4.0f}ms {med(pg):>10.0f}ms {pct(pg,0.9):>4.0f}ms {100*sink/len(ex):>5.1f}% "
          f"{100*med(dz):>5.1f}cm {g['cross']:>4}/{g['runs']}")
print("\n  receive gaps (the bridge not processing commands for >= 30 ms) and what they do at an exchange:")
print(f"  {'arm':<24} {'gaps/s':>7} {'lethal exch':>11} {'sink|lethal':>11} {'cross|lethal':>12} {'no-gap exch':>11} {'sink|nogap':>10} {'cross|nogap':>11}")
for arm in sorted(by):
    g = by[arm]; ex = g["ex"]
    if not ex:
        continue
    L = [q for q in ex if q.get("lethal_gap")]; N = [q for q in ex if not q.get("lethal_gap")]
    rate = lambda lst, key: (100.0 * sum(1 for q in lst if q[key]) / len(lst)) if lst else float("nan")
    print(f"  {arm:<24} {g['gaps']/max(1e-9, g['cruise_s']):>7.3f} {len(L):>11} {rate([dict(q, s=q['dropz'] > 0.02) for q in L], 's'):>10.1f}% {rate(L, 'crossed'):>11.1f}% "
          f"{len(N):>11} {rate([dict(q, s=q['dropz'] > 0.02) for q in N], 's'):>9.2f}% {rate(N, 'crossed'):>10.2f}%")
print("\n  old-pair max knee |tau_ff| (N*m, median over exchanges) at ms relative to the scheduled flip:")
print(f"  {'arm':<24} " + " ".join(f"{o:>5}" for o in OFFS))
for arm in sorted(by):
    ex = by[arm]["ex"]
    if not ex:
        continue
    cols = []
    for i in range(len(OFFS)):
        v = [q["tl_old"][i] for q in ex if q["tl_old"][i] is not None]
        cols.append(f"{med(v):>5.1f}")
    print(f"  {arm:<24} " + " ".join(cols))
print("\n  new-pair max knee |tau_ff|, same offsets:")
for arm in sorted(by):
    ex = by[arm]["ex"]
    if not ex:
        continue
    cols = []
    for i in range(len(OFFS)):
        v = [q["tl_new"][i] for q in ex if q["tl_new"][i] is not None]
        cols.append(f"{med(v):>5.1f}")
    print(f"  {arm:<24} " + " ".join(cols))
print("\n  body vz (m/s, median) at the same offsets - free fall reads as a ramp of -0.098 per 10 ms:")
for arm in sorted(by):
    ex = by[arm]["ex"]
    if not ex:
        continue
    cols = []
    for i in range(len(OFFS)):
        v = [q["tl_vz"][i] for q in ex if q["tl_vz"][i] is not None]
        cols.append(f"{med(v):>5.2f}")
    print(f"  {arm:<24} " + " ".join(cols))
print("\n  body z (cm, median) relative to z at the flip:")
for arm in sorted(by):
    ex = by[arm]["ex"]
    if not ex:
        continue
    cols = []
    for i in range(len(OFFS)):
        v = [100 * (q["tl_z"][i] - q["tl_z"][OFFS.index(0)]) for q in ex if q["tl_z"][i] is not None and q["tl_z"][OFFS.index(0)] is not None]
        cols.append(f"{med(v):>5.2f}")
    print(f"  {arm:<24} " + " ".join(cols))
print("\n  if the knob works, 'old tau drop' sits at 0 / -22 / -44 ms for lead 0 / 1 / 2; if it does not, this campaign measured nothing.")
