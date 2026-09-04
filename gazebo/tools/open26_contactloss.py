"""OPEN-26: does the robot LOSE ALL FOUR FEET, and does it happen BEFORE the fall?

truth_z - kin_z is the lowest foot's height above ground, and it peaks at
0.154 m mid-fold: every foot off the deck while the body is still up. Two
questions follow, and they have different consequences.

  1. What does the gait SCHEDULE believe at that moment? The KF trusts a foot
     as a fixed world point on the schedule's say-so (ContactEstimator::run()
     copies contactPhase - there is no contact estimation). If the schedule
     says "stance" for a foot that is 15 cm in the air, the filter is being
     lied to by construction, and that is a mechanism.

  2. Does the contact loss PRECEDE the descent, or follow it? Before => the
     robot lets go and then falls, and this is a cause. After => the legs come
     up because the body is already going down, and it is a symptom.

Usage: open26_contactloss.py CSV [CSV...]
"""
import json,csv,sys,statistics as st
def plateau(v): return st.median(sorted(v)[len(v)//2:])
CLEAR=0.05          # a foot this far up is not bearing load
rows=[]
for f in sys.argv[1:]:
    try:
        for r in csv.DictReader(open(f)): rows.append(r)
    except Exception: pass
LEAD=[];SCHED=[];PEAK=[];N=0
print("  run                   | all feet up at | descent at | lead   | schedule says stance")
for r in rows:
    if r.get("verdict")=="PASS" or r.get("snapshot","NONE") in ("NONE",""): continue
    try: d=json.load(open(r["snapshot"]))
    except Exception: continue
    R=[x for x in d["records"] if x.get("kin_z") is not None]
    if len(R)<300: continue
    tT=[];tZ=[]
    for line in open(r["truth"]):
        try: o=json.loads(line)
        except Exception: continue
        p=(o.get("p") or {}).get("0")
        if p: tT.append(o["t"]); tZ.append(p[2])
    if len(tZ)<50: continue
    ez=[x["z"] for x in R]; pe=plateau(ez); pt=plateau(tZ); OFF=pt-pe
    oe=ot=None
    for i in range(len(ez)-1,-1,-1):
        if ez[i]>=pe-0.03: oe=R[i]["t"]; break
    for i in range(len(tZ)-1,-1,-1):
        if tZ[i]>=pt-0.03: ot=tT[i]; break
    if oe is None or ot is None: continue
    i0=next((i for i,x in enumerate(R) if x["t"]>=oe-2.0), None)   # look 2s BEFORE
    if i0 is None: continue
    j=0; first_up=None; peak=0.0; sched_at_peak=None
    for i in range(i0, min(len(R), i0+1500)):
        dt=R[i]["t"]-oe; tt=ot+dt
        while j+1<len(tT) and abs(tT[j+1]-tt)<=abs(tT[j]-tt): j+=1
        clearance=(tZ[j]-OFF) - R[i]["kin_z"]      # lowest foot above ground
        if clearance>peak:
            peak=clearance
            sched_at_peak=max(R[i]["c%d"%k] for k in range(4))
        if clearance>CLEAR and first_up is None and R[i]["t"]>=oe-2.0:
            first_up=R[i]["t"]
    if first_up is None: continue
    lead=oe-first_up          # >0 means feet left the ground BEFORE the descent
    N+=1; LEAD.append(lead); PEAK.append(peak)
    if sched_at_peak is not None: SCHED.append(sched_at_peak)
    print("  %-21s | %+13.2fs | %9.2fs | %+5.2fs | max phase %.2f"%(
        (r.get("gait","?")+" "+r.get("terrain","?")+" r"+r.get("rep","?"))[:21],
        first_up-oe, 0.0, lead, sched_at_peak if sched_at_peak is not None else -1))
if N:
    print()
    print("  n=%d collapses"%N)
    print("  peak clearance of the LOWEST foot      : %.3f m"%st.mean(PEAK))
    print("  feet leave the ground, relative to the descent: %+.2f s"%st.mean(LEAD))
    print("     (positive = BEFORE the body starts down)")
    if SCHED:
        print("  gait schedule's max contact phase at that moment: %.2f"%st.mean(SCHED))
        print("     (the schedule runs 0..1; a foot it calls planted is trusted")
        print("      by the KF as a fixed world point)")
