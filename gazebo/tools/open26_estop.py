#!/usr/bin/env python3
"""Is the "level collapse" actually a pitch-triggered SAFETY E-STOP?

OPEN-26 has been built on a classification taken from the LAST tick of each
trace: |roll|<10 and |pitch|<15 at the end meant "level collapse", 66-69% of
all falls, "a mode nobody has characterised".

One fully instrumented fall says that classification is measuring the wrong
instant. The robot pitched to 31.3 deg, crossed SafetyChecker's 0.5 rad limit,
the FSM went to ESTOP, leg commands were zeroed, tauFeedForward went to
exactly 0.000 - and only THEN did it sink to the deck and settle flat. The
end-of-trace pose is the corpse, not the cause.

This asks the same question of every fall that has op_mode in its trace:
  * did op_mode reach ESTOP (2), and how long before the body hit the floor?
  * what was the PEAK pitch/roll before that, versus at the end?
  * do runs that PASS ever E-stop?

Usage: open26_estop.py [--archive DIR]
"""
import json, glob, os, sys, argparse, statistics as st
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import paths

ap = argparse.ArgumentParser(); ap.add_argument("--archive", default=paths.ARCHIVE_DIR)
a = ap.parse_args()

rows=[]
for f in sorted(glob.glob(os.path.join(a.archive, "*.json"))):
    try: d=json.load(open(f))
    except Exception: continue
    R=[x for x in (d.get("records") or []) if x.get("op_mode") is not None and x.get("z") is not None]
    if len(R)<500: continue
    reason=(d.get("reason") or "")
    ez=[x["z"] for x in R]; pe=st.median(sorted(ez)[len(ez)//2:])
    # descent onset
    s=e=None
    for i in range(len(ez)-1,-1,-1):
        if ez[i]<=pe-0.15 and e is None: e=i
        if e is not None and ez[i]>=pe-0.03: s=i; break
    k=None
    for i in range(1,len(R)):
        if R[i]["op_mode"]==2 and R[i-1]["op_mode"]!=2: k=i; break
    pre = R[:k+1] if k is not None else R
    rows.append(dict(
        file=os.path.basename(f), reason=reason,
        estop=(k is not None),
        estop_lead=(R[s]["t"]-R[k]["t"]) if (k is not None and s is not None) else None,
        peak_pitch=max(abs(x["pitch"]) for x in pre)*57.2958,
        peak_roll=max(abs(x["roll"]) for x in pre)*57.2958,
        end_pitch=abs(R[-1]["pitch"])*57.2958, end_roll=abs(R[-1]["roll"])*57.2958))

falls=[r for r in rows if r["reason"].startswith("FALL") or "FAIL" in r["reason"]]
passes=[r for r in rows if "PASS" in r["reason"]]
def show(name, v):
    if not v: 
        print("  %-22s none in this archive"%name); return
    es=[r for r in v if r["estop"]]
    print("  %-22s n=%2d   E-STOPPED %d (%.0f%%)"%(name,len(v),len(es),100*len(es)/len(v)))
    if es:
        print("      peak pitch before E-stop  %.1f deg   (limit is 28.6)"%st.mean([r["peak_pitch"] for r in es]))
        print("      peak roll  before E-stop  %.1f deg"%st.mean([r["peak_roll"] for r in es]))
        print("      pitch at END of trace     %.1f deg   <- what the old classifier read"%st.mean([r["end_pitch"] for r in es]))
        ld=[r["estop_lead"] for r in es if r["estop_lead"] is not None]
        if ld: print("      E-stop leads the descent by %.2f s"%(-st.mean(ld)))
print("  archive: %s"%a.archive)
print()
show("FALLS", falls); print(); show("PASSES", passes)
