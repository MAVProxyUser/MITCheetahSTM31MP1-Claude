#!/usr/bin/env python3
"""OPEN-31 (operational joint limits): score a DUMP=1 campaign whose arms are
CTRL_JOINT_LIMITS=0 / =1.

Per arm: verdicts and median peak attitude from the CSV; the heartbeat's own
counters summed per run from the archived ctrl log ("[stm32mp1] joint
limits: clamps=N stops=M"); and, from the bridge dump (100 Hz, sim joint
angles in URDF convention, commands in the MIT abstract frame where
knee = -calf), how often the calf sits outside Unitree's operational range
(-151..-53 deg) and on the mechanical stops (-161.5 / -50.9), split into the
same three phases as the OPEN-31 filing: stand-up (first 14 s of the dump),
locomotion, finish (last 12 s). The abad stop (+-49.5 deg) is counted too.

Usage: open31_score.py <campaign.csv> [...]
"""
import csv, glob, math, os, re, statistics, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from paths import RUN_DIR  # noqa: E402

D = 180.0 / math.pi
HB = re.compile(r"joint limits: clamps=(\d+) stops=(\d+)")
CALF_LO, CALF_HI = -151.0, -53.0      # operational, URDF convention
STOP_LO, STOP_HI = -161.5, -50.9      # mechanical
ABAD_STOP = 49.5

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

def counters(rid):
    p = ctrl_log(rid)
    if not p:
        return None
    c = s = 0; seen = False
    for line in open(p, errors="replace"):
        m = HB.search(line)
        if m:
            seen = True; c += int(m.group(1)); s += int(m.group(2))
    return (c, s) if seen else None

def dump_stats(path):
    """-> {phase: dict(n, calf_below, calf_above, stop_lo, stop_hi, abad_stop,
                       cmd_below, cmd_above, calf_min, calf_max)}"""
    try:
        rows = list(csv.reader(open(path)))
    except OSError:
        return None
    if len(rows) < 3:
        return None
    hdr, rows = rows[0], rows[1:]
    t0, t1 = float(rows[0][0]), float(rows[-1][0])
    out = {}
    for r in rows:
        t = float(r[0]) - t0
        ph = "standup" if t < 14.0 else ("finish" if (t1 - t0 - t) < 12.0 else "loco")
        o = out.setdefault(ph, dict(n=0, calf_below=0, calf_above=0, stop_lo=0, stop_hi=0,
                                    abad_stop=0, cmd_below=0, cmd_above=0, calf_min=1e9, calf_max=-1e9))
        for leg in range(4):
            calf = float(r[1 + 3 * leg + 2]) * D
            abad = float(r[1 + 3 * leg + 0]) * D
            cmd_calf = -float(r[25 + 3 * leg + 2]) * D     # MIT knee -> URDF calf
            kp_calf = float(r[37 + 3 * leg + 2])             # a command with kp 0 is inert (limp boot, PASSIVE)
            o["n"] += 1
            o["calf_below"] += calf < CALF_LO; o["calf_above"] += calf > CALF_HI
            o["stop_lo"] += calf <= STOP_LO + 0.3; o["stop_hi"] += calf >= STOP_HI - 0.3
            o["abad_stop"] += abs(abad) >= ABAD_STOP - 0.2
            if kp_calf > 0.0:
                o["cmd_below"] += cmd_calf < CALF_LO; o["cmd_above"] += cmd_calf > CALF_HI
            o["calf_min"] = min(o["calf_min"], calf); o["calf_max"] = max(o["calf_max"], calf)
    return out

def pct(a, n):
    return 100.0 * a / n if n else float("nan")

def main(paths):
    for path in paths:
        rows = list(csv.DictReader(open(path)))
        arms = {}
        for r in rows:
            a = arms.setdefault(r["course"], dict(n=0, passed=0, pitch=[], roll=[], clamps=[], stops=[], hb_missing=0,
                                                 dump=dict(), dump_missing=0))
            a["n"] += 1; a["passed"] += r["verdict"] == "PASS"
            for k in ("pitch", "roll"):
                try: a[k].append(float(r["peak_" + k]))
                except (ValueError, KeyError): pass
            c = counters(r["run_id"])
            if c is None: a["hb_missing"] += 1
            else: a["clamps"].append(c[0]); a["stops"].append(c[1])
            ds = dump_stats(r.get("bridge_dump") or "")
            if not ds: a["dump_missing"] += 1; continue
            for ph, o in ds.items():
                agg = a["dump"].setdefault(ph, dict(n=0, calf_below=0, calf_above=0, stop_lo=0, stop_hi=0,
                                                   abad_stop=0, cmd_below=0, cmd_above=0, calf_min=1e9, calf_max=-1e9))
                for k in ("n", "calf_below", "calf_above", "stop_lo", "stop_hi", "abad_stop", "cmd_below", "cmd_above"):
                    agg[k] += o[k]
                agg["calf_min"] = min(agg["calf_min"], o["calf_min"]); agg["calf_max"] = max(agg["calf_max"], o["calf_max"])
        print("== %s (%d rows)" % (os.path.basename(path), len(rows)))
        for name, a in sorted(arms.items()):
            print("  %-6s %2d/%-2d PASS  median peak pitch %5.1f roll %5.1f  | clamps/run median %s  stops/run median %s  (heartbeat missing %d)" % (
                name, a["passed"], a["n"],
                statistics.median(a["pitch"]) if a["pitch"] else float("nan"),
                statistics.median(a["roll"]) if a["roll"] else float("nan"),
                ("%d" % statistics.median(a["clamps"])) if a["clamps"] else "-",
                ("%d" % statistics.median(a["stops"])) if a["stops"] else "-", a["hb_missing"]))
            print("         %-8s %8s  %-19s %-15s %-15s %-9s %-19s" % ("phase", "leg-smp", "calf range (deg)", "calf< -151 / > -53", "on stop lo / hi", "abad stop", "CMD(kp>0) calf < -151 / > -53"))
            for ph in ("standup", "loco", "finish"):
                o = a["dump"].get(ph)
                if not o: continue
                n = o["n"]
                print("         %-8s %8d  %6.1f .. %6.1f    %5.2f%% / %5.2f%%   %5.2f%% / %5.2f%%   %5.2f%%   %5.2f%% / %5.2f%%" % (
                    ph, n, o["calf_min"], o["calf_max"], pct(o["calf_below"], n), pct(o["calf_above"], n),
                    pct(o["stop_lo"], n), pct(o["stop_hi"], n), pct(o["abad_stop"], n),
                    pct(o["cmd_below"], n), pct(o["cmd_above"], n)))
            if a["dump_missing"]:
                print("         (%d run(s) without a readable dump)" % a["dump_missing"])

if __name__ == "__main__":
    main(sys.argv[1:] or sys.exit(__doc__))
