#!/usr/bin/env python3
"""Score a wkc_box campaign by what happens on the leg to wp4 - the leg where
the 0.4-budget arm dies at cruise (ISSUES OPEN-28, 2026-09-11). Per run, from
the archived ctrl log: the leg's window (reached wp03 -> reached wp04 or the
E-stop/[FALL]), the [ESTERR] truth-vs-estimate forward speed inside it
(SIM_ESTERR=1 arms only; 10 Hz), and the [SCHED] table-lead switches. Per arm:
PASS count, peak pitch median, median of the leg's max truth speed, max
estimated speed, and the estimate-minus-truth at the leg's fastest point.

  box_lead_score.py CAMPAIGN_NAME [--leg 3]
"""
import argparse, csv, glob, os, re, statistics as st, sys

RE_EST = re.compile(r"\[ESTERR\] t=([0-9.]+) .*? vT=([-0-9.]+),([-0-9.]+),([-0-9.]+) vE=([-0-9.]+),([-0-9.]+),([-0-9.]+) dvx=([-+0-9.]+)")
RE_SCHED = re.compile(r"\[SCHED\] table lead (\d) -> (\d) adopted .*?v=([0-9.]+)")


def leg_window(lines, leg):
    start = end = None
    for i, l in enumerate(lines):
        if start is None and ("reached wp%02d" % leg) in l:
            start = i
            continue
        if start is not None and (("reached wp%02d" % (leg + 1)) in l or "Orientation safety check failed" in l
                                  or "[FALL]" in l or "Unsafe locomotion" in l):
            end = i
            break
    return start, end if end is not None else len(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("campaign")
    ap.add_argument("--leg", type=int, default=3, help="leg from wpN to wpN+1 (default 3 -> 4)")
    a = ap.parse_args()
    data = os.environ.get("CHEETAH_DATA", os.path.expanduser("~/Desktop/Cheetah/rundata"))
    rows = list(csv.DictReader(open(os.path.join(data, "campaigns", a.campaign + ".csv"))))
    arch = os.path.join(data, "conductor", "archive")
    per = {}
    print("  arm     rep verdict run   legwin  vT_max  vE_max  dvx@max  sched-switches(v)   peak pitch")
    for r in rows:
        arm, rep, v, rid = r["course"], r["rep"], r["verdict"], r["run_id"]
        if v == "NONE":
            continue
        logs = glob.glob(os.path.join(arch, "*_run%s_ctrl_0.log" % rid))
        if not logs:
            print("  %-7s %3s %-7s %5s  (no archived ctrl log)" % (arm, rep, v, rid)); continue
        lines = open(logs[0], errors="replace").read().split("\n")
        s, e = leg_window(lines, a.leg)
        est = []
        if s is not None:
            for l in lines[s:e]:
                m = RE_EST.search(l)
                if m:
                    est.append((float(m.group(2)), float(m.group(5)), float(m.group(8))))
        sw = [(int(m.group(1)), int(m.group(2)), float(m.group(3))) for l in lines for m in [RE_SCHED.search(l)] if m]
        vt = max((x[0] for x in est), default=None); ve = max((x[1] for x in est), default=None)
        dv = None
        if est:
            k = max(range(len(est)), key=lambda i: est[i][0]); dv = est[k][1] - est[k][0]
        d = per.setdefault(arm, dict(n=0, p=0, pitch=[], vt=[], ve=[], dv=[], sw=[]))
        d["n"] += 1; d["p"] += (v == "PASS")
        try: d["pitch"].append(float(r["peak_pitch"]))
        except Exception: pass
        if vt is not None: d["vt"].append(vt); d["ve"].append(ve); d["dv"].append(dv)
        d["sw"].append(len(sw))
        f = lambda x: ("%.2f" % x) if x is not None else "  -  "
        print("  %-7s %3s %-7s %5s  %s  %s  %s  %s  %-18s %s" % (
            arm, rep, v, rid, ("%d-%d" % (s, e)) if s is not None else "  -   ", f(vt), f(ve), ("%+.2f" % dv) if dv is not None else "  -  ",
            " ".join("%d>%d@%.2f" % x for x in sw) or "none", r.get("peak_pitch", "")))
    print("== per arm: PASS/n | peak pitch median | leg vT_max median | vE_max median | (vE-vT)@max median | lead switches/run")
    med = lambda xs: ("%.2f" % st.median(xs)) if xs else "-"
    for arm, d in per.items():
        print("  %-7s %d/%d | %s | %s | %s | %s | %s" % (arm, d["p"], d["n"], med(d["pitch"]), med(d["vt"]), med(d["ve"]), med(d["dv"]),
                                                    med(d["sw"]) if d["sw"] else "-"))


if __name__ == "__main__":
    main()
