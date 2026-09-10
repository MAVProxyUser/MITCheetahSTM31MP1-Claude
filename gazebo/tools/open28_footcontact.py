#!/usr/bin/env python3
"""At the instant a sink begins, does a STANCE foot lose the ground?

Everything the trace and the bridge dump can say about OPEN-28's collapse has
been said: a stance leg gives way in under 50 ms at steady cruise on flat
ground, with no precursor in body state, foot force, loop timing, control
effort, torque saturation (the clip is a feature of sinks, not escalation),
leg-to-leg contact (18-20 cm clear by forward kinematics), or the gait
schedule. The one binary none of those instruments carry is whether the foot
that was supposed to be holding the body up was still touching the ground.

Gazebo's foot contact SENSORS answer that directly. contact_feed.py streams
{"t": wall, "c": [FL, FR, RL, RR]} at 200 Hz - NOTE the order, which is not
MIT's FR, FL, RR, RL. Two clocks: the feed stamps wall time, the trace its own;
aligned by cross-correlating the feed's total-contact count against the
trace's schedule count, which share the gait rhythm.

For every sink (escalated and recovered) and matched random instants:
  * was every scheduled-stance foot in ground contact in the 40 ms before
  * did a scheduled-stance foot DROP contact in [-40, +40] ms of onset
  * was a scheduled-SWING foot touching (early touchdown / toe catch)

Usage: open28_footcontact.py --campaign open28_contact
"""
import csv, json, os, math, argparse, statistics as st, bisect, random, collections

R2D = 57.2958
MIT_FROM_FEED = {0: 1, 1: 0, 2: 3, 3: 2}   # feed index -> MIT leg index (FL,FR,RL,RR -> FR,FL,RR,RL)
ap = argparse.ArgumentParser()
ap.add_argument("--campaign", default="open28_contact")
ap.add_argument("--root", default="/Users/kfinisterre/Desktop/Cheetah/rundata/campaigns")
ap.add_argument("--limit", type=float, default=28.65)
a = ap.parse_args()
random.seed(41)


def load_contact(p):
    out = []
    for line in open(p):
        try:
            d = json.loads(line)
            c = d["c"]
            out.append((d["t"], [c[MIT_FROM_FEED[i]] if False else None for i in range(4)]))
        except Exception:
            continue
    # reorder into MIT leg order once, properly
    res = []
    for line in open(p):
        try:
            d = json.loads(line); c = d["c"]
            res.append((d["t"], [c[1], c[0], c[3], c[2]]))     # MIT FR,FL,RR,RL from feed FL,FR,RL,RR
        except Exception:
            pass
    return res


def align(trace, feed):
    """lag such that feed_t - lag ~ trace_t, from the gait rhythm shared by both."""
    tt = [x["t"] for x in trace]; tc = [sum(1 for l in range(4) if (x.get(f"c{l}", 0) or 0) > 0) for x in trace]
    ft = [t for t, _ in feed]; fc = [sum(c) for _, c in feed]
    if len(ft) < 500 or len(tt) < 500:
        return None
    lag0 = ft[0] - tt[0]
    best, bl = -1e18, lag0
    for k in range(-60, 61):
        lag = lag0 + 0.05 * k
        acc = n = 0
        for t, c in feed[::4]:
            i = bisect.bisect_left(tt, t - lag)
            if 0 < i < len(tt):
                acc += (c - 2) * (tc[i] - 2); n += 1
        if n and acc / n > best:
            best, bl = acc / n, lag
    return bl


rows = list(csv.DictReader(open(os.path.join(a.root, a.campaign + ".csv"))))
G = collections.defaultdict(list)
for r in rows:
    snap, con = r.get("snapshot", ""), r.get("contact", "")
    if not (snap and snap != "NONE" and os.path.exists(snap) and con and os.path.exists(con)):
        continue
    d = json.load(open(snap))
    R = [x for x in d.get("records", []) if x.get("vx") is not None and x.get("roll") is not None and x.get("z") is not None and x["t"] > 6.0]
    F = load_contact(con)
    if len(R) < 800 or len(F) < 500:
        continue
    lag = align(R, F)
    if lag is None:
        continue
    ft = [t for t, _ in F]
    k = next((i for i in range(1, len(R)) if R[i]["op_mode"] == 2 and R[i-1]["op_mode"] != 2), None)
    pre = R[:k] if k else R
    sp = lambda x: math.hypot(x["vx"], x["vy"])

    def look(i):
        x = pre[i]; t0 = x["t"]
        stn = [l for l in range(4) if (x.get(f"c{l}", 0) or 0) > 0]; sw = [l for l in range(4) if l not in stn]
        def feed_at(dt):
            j = bisect.bisect_left(ft, t0 + lag + dt); j = min(max(j, 0), len(F) - 1); return F[j][1]
        before = [feed_at(-0.04 + 0.005 * n) for n in range(8)]
        around = [feed_at(-0.04 + 0.005 * n) for n in range(16)]
        all_stance_on = all(all(c[l] for l in stn) for c in before)
        stance_drop = any(not c[l] for c in around for l in stn)
        swing_touch = any(c[l] for c in before for l in sw)
        return dict(all_on=all_stance_on, drop=stance_drop, swing_touch=swing_touch)

    i = 400
    while i < len(pre) - 100:
        x = pre[i]
        if sp(x) > 1.5 and max(abs(x["pitch"]), abs(x["roll"])) * R2D < 10:
            zb = st.median(pre[j]["z"] for j in range(i - 400, i - 50))
            if x["z"] < zb - 0.02:
                esc = any(max(abs(y["pitch"]), abs(y["roll"])) * R2D >= a.limit for y in pre[i:i+750])
                G["escalated" if esc else "recovered"].append(look(i)); i += 750; continue
        i += 1
    cru = [j for j in range(600, len(pre) - 100, 25) if sp(pre[j]) > 1.5 and max(abs(pre[j]["pitch"]), abs(pre[j]["roll"])) * R2D < 10]
    for j in random.sample(cru, min(6, len(cru))) if cru else []:
        G["random"].append(look(j))

print(f"\n  escalated {len(G['escalated'])}   recovered {len(G['recovered'])}   random {len(G['random'])}")
print(f"\n  {'from Gazebo foot contact sensors':<48} {'escalated':>10} {'recovered':>10} {'random':>8}")
for key, lab in (("all_on", "every scheduled-stance foot on the ground, 40 ms before"),
                 ("drop", "a scheduled-stance foot LOST contact within +-40 ms"),
                 ("swing_touch", "a scheduled-SWING foot was touching, 40 ms before")):
    out = []
    for g in ("escalated", "recovered", "random"):
        v = [q[key] for q in G[g]]
        out.append(f"{100*sum(v)/len(v):.0f}% ({sum(v)}/{len(v)})" if v else "-")
    print(f"  {lab:<48} {out[0]:>10} {out[1]:>10} {out[2]:>8}")
