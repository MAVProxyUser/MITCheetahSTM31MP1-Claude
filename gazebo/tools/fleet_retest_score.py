#!/usr/bin/env python3
"""Score a 3-dog fleet re-test (dash:100 trotting 3.0, the case that fails in
a fleet and passes solo): per rep, the run id, the verdict counts, the sim's
real-time factor sampled during that run (fleet_dash_retest.log.rtf, written
by the sampler beside the chain), and per dog the ESTOP time, the speed and
the attitude at the trip from the archived shm trace.

Usage: fleet_retest_score.py [<fleet log>]   (default: $CAMPAIGN_DIR/fleet_dash_retest.log)
"""
import glob, json, os, re, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from paths import RUN_DIR, CAMPAIGN_DIR  # noqa: E402

def trip(path):
    R = [x for x in json.load(open(path))["records"] if x.get("pitch") is not None]
    k = next((i for i in range(1, len(R)) if R[i]["op_mode"] == 2 and R[i - 1]["op_mode"] != 2), None)
    if k is None:
        return None
    x = R[k]
    pre = [y for y in R if x["t"] - 1.0 <= y["t"] < x["t"]]
    vx = sum(y.get("vx", 0.0) for y in pre) / max(1, len(pre))
    return dict(t=x["t"], vx=vx, pitch=x["pitch"] * 57.3, roll=x["roll"] * 57.3, z=x.get("z"))

def main(log):
    txt = open(log, errors="replace").read()
    rtf = {}
    try:
        for line in open(log + ".rtf"):
            m = re.match(r"(\S+) run=(\d+) dogs=(\d+) n=(\d+) mean=([\d.]+) min=([\d.]+) p5=([\d.]+) p50=([\d.]+)", line)
            if m:
                rtf.setdefault(m.group(2), []).append((float(m.group(5)), float(m.group(7)), float(m.group(8))))
    except OSError:
        pass
    reps = re.findall(r"=== fleet rep (\d+) (\d+:\d+) ===(.*?)(?==== fleet rep|=== fleet re-test done|\Z)", txt, re.S)
    for rep, hhmm, body in reps:
        m = re.search(r"run (\d+) phase=\S+\s+dogs=(\d+)\s+PASS=(\d+) FAIL=(\d+) FELL=(\d+)", body)
        capped = "CAPPED" in body
        if not m:
            print("rep %s %s: no runner summary%s" % (rep, hhmm, " (CAPPED)" if capped else "")); continue
        rid, dogs, p, f, fell = m.groups()
        r = rtf.get(rid, [])
        rtf_s = ("rtf mean %.3f p5 %.3f p50 %.3f (%d samples of 20 s)" % (
            sum(a for a, _, _ in r) / len(r), min(b for _, b, _ in r), sum(c for _, _, c in r) / len(r), len(r))) if r else "rtf: not sampled"
        print("rep %s %s run %s: dogs=%s PASS=%s FAIL=%s FELL=%s%s | %s" % (rep, hhmm, rid, dogs, p, f, fell, " CAPPED" if capped else "", rtf_s))
        for d in range(3):
            for path in sorted(glob.glob(os.path.join(RUN_DIR, "archive", "shm_trace", "*_run%s_dog%d_*.json" % (rid, d)))):
                t = trip(path)
                tag = os.path.basename(path).split("_", 3)[-1]
                if t:
                    print("    dog%d %-28s ESTOP t=%.1fs at vx %.2f m/s, pitch %.1f roll %.1f z %.3f" % (d, tag, t["t"], t["vx"], t["pitch"], t["roll"], t["z"] or 0))
                else:
                    print("    dog%d %-28s no ESTOP edge" % (d, tag))

if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(CAMPAIGN_DIR, "fleet_dash_retest.log"))
