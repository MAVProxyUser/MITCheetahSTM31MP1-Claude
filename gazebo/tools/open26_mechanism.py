"""OPEN-26 mechanism: is the estimator's height following the LEGS or the BODY?

The hypothesis is that the KF corrects body position from foot kinematics,
trusting each foot as a fixed world point on the gait SCHEDULE's say-so (there
is no contact estimation - ContactEstimator::run() copies contactPhase). If
the legs buckle while the feet stay planted, the leg's kinematic height
shrinks and the filter reads that as the body having dropped.

That predicts something specific and falsifiable: during a collapse the
estimator's z should track kin_z (pure leg geometry + orientation) much more
closely than it tracks the real body. Both are "descending", so a correlation
of rates is not enough - this compares the TRACKING ERROR of each pairing over
the descent, on the same samples, in the same units.

Usage: open26_mechanism.py CSV [CSV...]
"""
import json,csv,sys,statistics as st

def truth_series(path):
    ts=[];zs=[]
    try:
        for line in open(path):
            try: o=json.loads(line)
            except Exception: continue
            p=(o.get("p") or {}).get("0")
            if p: ts.append(o["t"]); zs.append(p[2])
    except Exception: pass
    return ts,zs

def plateau(v): return st.median(sorted(v)[len(v)//2:])

def resample(ts,zs,grid):
    """nearest-sample lookup onto a common time grid"""
    out=[];j=0
    for g in grid:
        while j+1 < len(ts) and abs(ts[j+1]-g) <= abs(ts[j]-g): j+=1
        out.append(zs[j])
    return out

rows=[]
for f in sys.argv[1:]:
    for r in csv.DictReader(open(f)): rows.append(r)

EK=[];ET=[];N=0
print("  run                    | RMS |est - kin_z| | RMS |est - truth| | ratio")
for r in rows:
    if r.get("verdict")=="PASS" or r.get("snapshot","NONE") in ("NONE",""): continue
    try: d=json.load(open(r["snapshot"]))
    except Exception: continue
    R=[x for x in d["records"] if x.get("kin_z") is not None]
    if len(R)<300: continue
    ets=[x["t"] for x in R]; ez=[x["z"] for x in R]; kz=[x["kin_z"] for x in R]
    tts,tzs=truth_series(r["truth"])
    if len(tzs)<50: continue
    pe,pt=plateau(ez),plateau(tzs)
    # descent window in the ESTIMATOR's own series
    s=e=None
    for i in range(len(ez)-1,-1,-1):
        if ez[i]<=pe-0.15 and e is None: e=i
        if e is not None and ez[i]>=pe-0.03: s=i; break
    if s is None or e is None or e<=s+20: continue
    grid=ets[s:e+1]
    # align truth by its own descent onset, then express both plateau-relative
    ot=None
    for i in range(len(tzs)-1,-1,-1):
        if tzs[i]>=pt-0.03: ot=tts[i]; break
    if ot is None: continue
    oe=ets[s]
    tg=[ot+(g-oe) for g in grid]
    tr=resample(tts,tzs,tg)
    ez_r=[ez[i]-pe for i in range(s,e+1)]
    kz_r=[kz[i]-pe for i in range(s,e+1)]      # kin_z shares the estimator's reference
    tr_r=[v-pt for v in tr]
    ek=(sum((a-b)**2 for a,b in zip(ez_r,kz_r))/len(ez_r))**0.5
    et=(sum((a-b)**2 for a,b in zip(ez_r,tr_r))/len(ez_r))**0.5
    EK.append(ek); ET.append(et); N+=1
    print("  %-22s | %14.4f | %14.4f | %5.1fx"%(
        r.get("gait","?")+" rep"+r.get("rep","?"), ek, et, et/ek if ek else 0))
if N:
    mk,mt=st.mean(EK),st.mean(ET)
    print()
    print("  n=%d collapses"%N)
    print("  the estimator's height sits  %.4f m  from the LEG-KINEMATIC height"%mk)
    print("  and                          %.4f m  from the REAL BODY"%mt)
    print("  -> it tracks the legs %.1fx more closely than it tracks the body"%(mt/mk if mk else 0))
