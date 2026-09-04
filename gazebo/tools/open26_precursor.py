#!/usr/bin/env python3
"""Is losing the feet a CAUSE of the fold, or a consequence of it?

Operator, 2026-09-04: "Contact IS inferable from IMU + encoders alone ... I
think you should test adding this into your theory of tests."

So contact inference is used here as an INSTRUMENT, not as a control input -
which is where it belongs, and where it also works on the real EDU dog. Two
signals are read for every run:

  INFERRED  world foot speed < 0.15 m/s  (IMU + joint encoders only)
  TRUE      gz foot contact sensors      (sim-only labels, for scoring)

and two questions asked:

  1. Do the feet leave the ground BEFORE the body starts down? Before means
     the robot lets go and then falls - a cause. After means the legs come up
     because the body is already going - a consequence.
  2. Does the same thing happen in runs that PASS? A precursor that fires just
     as often on a healthy run predicts nothing, however good the story is.

Usage: open26_precursor.py CSV [CSV...]
"""
import json, csv, sys, os, statistics as st
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

TH = 0.15          # m/s, the inferred-contact threshold

def plateau(v): return st.median(sorted(v)[len(v)//2:])

def truth_series(path):
    T=[];Z=[]
    try:
        for line in open(path):
            try: o=json.loads(line)
            except Exception: continue
            q=(o.get("p") or {}).get("0")
            if q: T.append(o["t"]); Z.append(q[2])
    except Exception: pass
    return T,Z

def contact_series(path):
    T=[];C=[]
    try:
        for line in open(path):
            try: o=json.loads(line)
            except Exception: continue
            if "c" in o: T.append(o["t"]); C.append(o["c"])
    except Exception: pass
    return T,C

rows=[]
for f in sys.argv[1:]:
    try:
        for r in csv.DictReader(open(f)): rows.append(r)
    except Exception: pass

FALL_LEAD=[];PASS_LEAD=[];FALL_TRUE=[];PASS_TRUE=[]
print("  run                    verdict | inferred all-feet-off | true all-feet-off")
print("                                 | relative to descent    | relative to descent")
for r in rows:
    if r.get("snapshot","NONE") in ("NONE",""): continue
    try: d=json.load(open(r["snapshot"]))
    except Exception: continue
    R=[x for x in d["records"] if x.get("foot_fz0") is not None]
    if len(R)<500: continue
    ez=[x["z"] for x in R]; pe=plateau(ez)
    fell = r.get("verdict")!="PASS"
    # reference instant: descent onset for a fall; for a pass, the equivalent
    # point in its own controlled lie-down, so the two are comparable
    s=None
    for i in range(len(ez)-1,-1,-1):
        if ez[i] <= pe-0.15:
            for j in range(i,-1,-1):
                if ez[j] >= pe-0.03: s=j; break
            break
    if s is None: continue
    t_on=R[s]["t"]
    # INFERRED: last moment before onset when every foot was moving
    inf=None
    for x in R[:s][::-1]:
        if t_on-x["t"] > 3.0: break
        if all(x["foot_fz%d"%i] >= TH for i in range(4)):
            inf = t_on - x["t"]
        elif inf is not None:
            break
    # TRUE: same question from the contact sensors, aligned by body height
    tru=None
    if r.get("contact"):
        pT,pZ=truth_series(r["truth"]); cT,cC=contact_series(r["contact"])
        if len(pZ)>200 and len(cC)>200:
            pz0=plateau(pZ); ez0=plateau(ez)
            eT=[x["t"] for x in R[::10]]; eZ=[x["z"] for x in R[::10]]
            best=(None,1e18); j=0
            for off_ms in range(-2000,15001,200):
                off=off_ms/1000.0; err=0.0; n=0; j=0
                for t,z in zip(eT,eZ):
                    tt=pT[0]+(t-eT[0])+off
                    while j+1<len(pT) and abs(pT[j+1]-tt)<=abs(pT[j]-tt): j+=1
                    if abs(pT[j]-tt)>0.1: continue
                    err+=((z-ez0)-(pZ[j]-pz0))**2; n+=1
                if n>100 and err/n<best[1]: best=(off,err/n)
            if best[0] is not None:
                tc_on = pT[0]+(t_on-eT[0])+best[0]
                k=0
                for idx in range(len(cT)-1,-1,-1):
                    if cT[idx] > tc_on: continue
                    if tc_on-cT[idx] > 3.0: break
                    if sum(cC[idx])==0: tru = tc_on-cT[idx]
                    elif tru is not None: break
    (FALL_LEAD if fell else PASS_LEAD).append(inf if inf is not None else 0.0)
    if tru is not None: (FALL_TRUE if fell else PASS_TRUE).append(tru)
    print("  %-22s %-7s | %19s | %s"%(
        (r.get("gait","?")+" "+r.get("terrain","?")+" r"+r.get("rep","?"))[:22],
        "FELL" if fell else "PASS",
        ("%.2f s before"%inf) if inf else "never (feet kept)",
        ("%.2f s before"%tru) if tru else "never / no label"))
print()
def rep(n,v):
    if not v: return
    nz=[x for x in v if x>0]
    print("  %-26s n=%2d   fired on %d of them   mean lead %.2f s"%(
        n,len(v),len(nz),st.mean(nz) if nz else 0.0))
rep("FELL  - inferred",FALL_LEAD); rep("PASSED- inferred",PASS_LEAD)
rep("FELL  - true sensor",FALL_TRUE); rep("PASSED- true sensor",PASS_TRUE)
