#!/usr/bin/env python3
"""What the LEGS do at the lie-down's damping hand-off, from the bridge dump
(ISSUES OPEN-30, 2026-09-14 22:35). The snapshot has no joint angles, so the
abad splay that turns a hand-off into a belly-drop is only on record when the
campaign ran with DUMP=1 (open28_subcourse.sh -> BRIDGE_DUMP, 100 Hz, q per
joint in the URDF convention: abad, thigh, calf per leg FR FL RR RL).

  usage: liedown_handoff.py label=CAMPAIGN_GLOB[:arm] ...

The hand-off is the last stretch (>= 0.5 s) with kd == 8 on all twelve joints
and kp == 0 (edampCommand); the 20 ms PASSIVE hop before STAND_UP is the same
signature and is skipped by the length rule. An arm that never hands off
(WP_LIEDOWN_EDAMP=0) has no such stretch and is scored over the last 1.2 s of
the dump instead ("held"). Per run: the pose at the hand-off, the abad
excursion per leg over the hold (max |q - q0|), the knee margin to the
operational stop, and the left/right asymmetry of the splay; per label the
collapse count (mean abad excursion > 0.15 rad) and, where the campaign CSV
also carries the snapshot, the rank correlation of the splay asymmetry with
liedown_peak's stage-2 roll."""
import sys, os, csv, glob, statistics as st
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
DATA = os.environ.get("CHEETAH_DATA", os.path.expanduser("~/Desktop/Cheetah/rundata"))
KNEE_OP = -2.635   # the operational calf stop (-151 deg), mechanical -2.818
LEG = ["FR", "FL", "RR", "RL"]

def handoff(dump):
    rows = list(csv.DictReader(open(dump)))
    if len(rows) < 200: return None
    f = lambda r, k: float(r[k])
    sig = [all(abs(f(r, 'kd%d' % j) - 8.0) < 1e-6 for j in range(12)) and all(f(r, 'kp%d' % j) == 0.0 for j in range(12)) for r in rows]
    segs, s = [], None
    for i, v in enumerate(sig):
        if v and s is None: s = i
        if not v and s is not None: segs.append((s, i - 1)); s = None
    if s is not None: segs.append((s, len(rows) - 1))
    segs = [(a, b) for a, b in segs if f(rows[b], 't') - f(rows[a], 't') >= 0.5]
    if segs:
        a, b = segs[-1]; mode = "handoff"
    else:
        b = len(rows) - 1; a = max(0, b - 120); mode = "held"
    q0 = [f(rows[a], 'q%d' % j) for j in range(12)]
    exc = [max(abs(f(r, 'q%d' % (3 * l)) - q0[3 * l]) for r in rows[a:b + 1]) for l in range(4)]
    knee_margin = [q0[3 * l + 2] - KNEE_OP for l in range(4)]        # > 0: the calf has not reached the stop (less folded than -2.635)
    return dict(mode=mode, hold_s=f(rows[b], 't') - f(rows[a], 't'), abad0=[q0[3 * l] for l in range(4)], thigh0=[q0[3 * l + 1] for l in range(4)],
                calf0=[q0[3 * l + 2] for l in range(4)], exc=exc, mean_exc=st.mean(exc), asym=(exc[0] + exc[2]) / 2 - (exc[1] + exc[3]) / 2,
                knee_margin_max=max(knee_margin), tff0=[f(rows[max(0, a - 1)], 'tff%d' % j) for j in range(12)])

def rank(v):
    o = sorted(range(len(v)), key=lambda i: v[i]); rk = [0] * len(v)
    for j, i in enumerate(o): rk[i] = j
    return rk

def spearman(a, b):
    ra, rb = rank(a), rank(b); ma, mb = st.mean(ra), st.mean(rb)
    num = sum((x - ma) * (y - mb) for x, y in zip(ra, rb)); den = (sum((x - ma) ** 2 for x in ra) * sum((y - mb) ** 2 for y in rb)) ** 0.5
    return num / den if den else 0.0

def main():
    try:
        from liedown_peak import score as roll_score
    except Exception:
        roll_score = None
    for spec in sys.argv[1:]:
        label, sel = spec.split("=", 1)
        pat, _, arm = sel.partition(":")
        out = []
        for fcsv in sorted(glob.glob(os.path.join(DATA, "campaigns", pat + ".csv"))):
            for r in csv.DictReader(open(fcsv)):
                if arm and r.get("course") != arm: continue
                # only a finished run has a lie-down: a fall's dump ends at the fall
                if r.get("verdict") != "PASS" or not r.get("bridge_dump") or not os.path.exists(r["bridge_dump"]): continue
                try: h = handoff(r["bridge_dump"])
                except Exception: h = None
                if h is None: continue
                h["run"] = r.get("run_id"); h["roll"] = None
                if roll_score and r.get("snapshot"):
                    try:
                        sc = roll_score(r["snapshot"]); h["roll"] = sc["peak_roll"] if sc else None
                    except Exception: pass
                out.append(h)
        if not out:
            print("%-14s no dumps" % label); continue
        col = [h for h in out if h["mean_exc"] > 0.15]
        print("%-14s n=%3d  %s | abad excursion median %.3f rad, p90 %.3f | collapsed (mean splay > 0.15) %d | knee short of the op stop at the hand-off: %d runs | asym median %.3f"
              % (label, len(out), "held " + ("%d" % sum(h["mode"] == "held" for h in out)) if any(h["mode"] == "held" for h in out) else "hand-off",
                 st.median(h["mean_exc"] for h in out), sorted(h["mean_exc"] for h in out)[min(len(out) - 1, int(0.9 * len(out)))], len(col),
                 sum(h["knee_margin_max"] > 0 for h in out), st.median(abs(h["asym"]) for h in out)))
        both = [h for h in out if h["roll"] is not None]
        if len(both) >= 6:
            print("    rho(splay asymmetry, stage-2 roll) = %+.2f   rho(mean splay, roll) = %+.2f   (n=%d)"
                  % (spearman([abs(h["asym"]) for h in both], [h["roll"] for h in both]), spearman([h["mean_exc"] for h in both], [h["roll"] for h in both]), len(both)))
        for h in sorted(out, key=lambda h: -h["mean_exc"])[:4]:
            print("    run %s %s %.2fs abad0 %s  splay %s  knee0 %s  roll %s" % (h["run"], h["mode"], h["hold_s"], " ".join("%+.2f" % v for v in h["abad0"]),
                  " ".join("%.2f" % v for v in h["exc"]), " ".join("%.2f" % v for v in h["calf0"]), ("%.1f" % h["roll"]) if h["roll"] is not None else "-"))

if __name__ == "__main__":
    main()
