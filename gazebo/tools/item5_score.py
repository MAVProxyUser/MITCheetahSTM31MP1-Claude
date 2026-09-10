#!/usr/bin/env python3
"""item 5 (contact schedule vs reality in the estimator): score a campaign
whose arms ran with SIM_ESTERR=1 (estimate against Gazebo truth, logged and
never fed back) and, in one arm, SIM_CONTACT_GATE=1.

Per arm: verdicts, and from each run's archived ctrl log
  * |dvx| (forward-velocity estimate error, body frame) over CRUISE samples
    (|vT| > 1.0 m/s): mean and p90 - the quantity the Raibert foothold and
    the MPC state cost consume;
  * |dvy| the same for lateral, |dvz| for vertical (the first live run showed
    a 0.1-0.2 m/s vertical-velocity estimate error while dvx sat at 0.03);
  * the contact gate's own last "[contact-gate] vetoed X of Y" line, so an
    arm that claims to gate is shown to have fired.

Usage: item5_score.py <campaign.csv> [<campaign.csv> ...]
The ctrl log for run N is $RUN_DIR/archive/*_run<N>_ctrl_0.log, or the live
$RUN_DIR/ctrl_0.log when N is the most recent run.
"""
import csv, glob, os, re, statistics, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from paths import RUN_DIR  # noqa: E402

RX = re.compile(r"\[ESTERR\] t=([\d.]+) pT=(\S+) pE=(\S+) vT=(\S+) vE=(\S+) dvx=([+-][\d.]+)")
GATE = re.compile(r"\[contact-gate\] vetoed (\d+) of (\d+) scheduled-stance samples \(([\d.]+)%\)")

def ctrl_log(rid):
    m = sorted(glob.glob(os.path.join(RUN_DIR, "archive", "*_run%s_ctrl_0.log" % rid)))
    if m:
        return m[-1]
    live = os.path.join(RUN_DIR, "ctrl_0.log")
    try:
        if re.search(r"\[RUNID\] run=%s\b" % rid, open(live, errors="replace").read()):
            return live
    except OSError:
        pass
    return None

def score_run(rid):
    p = ctrl_log(rid)
    if not p:
        return None
    dvx, dvy, dvz, gate = [], [], [], None
    for line in open(p, errors="replace"):
        m = RX.search(line)
        if m:
            vT = [float(x) for x in m.group(4).split(",")]
            vE = [float(x) for x in m.group(5).split(",")]
            if (vT[0] ** 2 + vT[1] ** 2) ** 0.5 > 1.0:
                dvx.append(abs(vE[0] - vT[0])); dvy.append(abs(vE[1] - vT[1])); dvz.append(abs(vE[2] - vT[2]))
            continue
        g = GATE.search(line)
        if g:
            gate = (int(g.group(1)), int(g.group(2)), float(g.group(3)))
    return dict(n=len(dvx), dvx=dvx, dvy=dvy, dvz=dvz, gate=gate, log=os.path.basename(p))

def p90(v):
    if not v: return float("nan")
    s = sorted(v); return s[min(len(s) - 1, int(0.9 * len(s)))]

def main(paths):
    for path in paths:
        rows = list(csv.DictReader(open(path)))
        arms = {}
        for r in rows:
            a = arms.setdefault(r["course"], dict(n=0, passed=0, dvx=[], dvy=[], dvz=[], gate=[], runs=0, missing=0))
            a["n"] += 1; a["passed"] += r["verdict"] == "PASS"
            s = score_run(r["run_id"])
            if not s or s["n"] == 0:
                a["missing"] += 1; continue
            a["runs"] += 1
            a["dvx"].append(statistics.mean(s["dvx"])); a["dvy"].append(statistics.mean(s["dvy"])); a["dvz"].append(statistics.mean(s["dvz"]))
            a["gate"].append(s["gate"][2] if s["gate"] else None)
        print("== %s (%d rows)" % (os.path.basename(path), len(rows)))
        print("  %-10s %7s  %8s %8s  %8s %8s  %8s %8s  %s" % ("arm", "PASS", "|dvx|mean", "p90", "|dvy|mean", "p90", "|dvz|mean", "p90", "gate veto % (runs with ESTERR / missing)"))
        for name, a in sorted(arms.items()):
            gv = [g for g in a["gate"] if g is not None]
            print("  %-10s %3d/%-3d  %8.3f %8.3f  %8.3f %8.3f  %8.3f %8.3f  %s  (%d / %d)" % (
                name, a["passed"], a["n"],
                statistics.mean(a["dvx"]) if a["dvx"] else float("nan"), p90(a["dvx"]),
                statistics.mean(a["dvy"]) if a["dvy"] else float("nan"), p90(a["dvy"]),
                statistics.mean(a["dvz"]) if a["dvz"] else float("nan"), p90(a["dvz"]),
                ("%.1f%% median" % statistics.median(gv)) if gv else "no gate line",
                a["runs"], a["missing"]))

if __name__ == "__main__":
    main(sys.argv[1:] or sys.exit(__doc__))
