"""OPEN-26: the estimator's height DIVERGES mid-descent, and goes unphysical.

Earlier analyses sampled the bottom of the fall and found agreement (7 mm),
which read as a null. That was the wrong instant: printing one collapse's
series aligned showed the whole discrepancy lives in a ~0.3 s window in the
MIDDLE of the descent and has closed again by the deck.

    dt      est_z    kin_z   truth
    0.00s    0.258    0.260    0.260
    0.40s    0.054    0.081    0.139     <- estimator 8.5 cm below truth
    0.80s   -0.008    0.038    0.037     <- and BELOW THE FLOOR

A body height below zero is not a bias, it is the filter losing the state:
the robot's belly cannot be underground. This measures the peak divergence,
when it happens, and how often the estimate goes negative - the one symptom
that needs no reference to interpret.

Usage: open26_divergence.py CSV [CSV...]
"""
import json,csv,sys,statistics as st
def plateau(v): return st.median(sorted(v)[len(v)//2:])
rows=[]
for f in sys.argv[1:]:
    try:
        for r in csv.DictReader(open(f)): rows.append(r)
    except Exception: pass
PE=[];PK=[];TN=[];NEG=0;N=0;NEGDEPTH=[]
print("  run                   | peak est-truth | at    | peak kin-truth | est goes negative")
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
    i0=next((i for i,x in enumerate(R) if x["t"]>=oe), None)
    if i0 is None: continue
    best_e=best_k=0.0; best_t=0.0; neg=0.0
    j=0
    for i in range(i0, min(len(R), i0+1000)):        # 2 s of descent
        dt=R[i]["t"]-oe
        tt=ot+dt
        while j+1<len(tT) and abs(tT[j+1]-tt)<=abs(tT[j]-tt): j+=1
        tz=tZ[j]-OFF
        de=tz-R[i]["z"]; dk=tz-R[i]["kin_z"]
        if de>best_e: best_e=de; best_t=dt
        if dk>best_k: best_k=dk
        if R[i]["z"]<neg: neg=R[i]["z"]
    N+=1; PE.append(best_e); PK.append(best_k)
    if neg < -0.001: NEG+=1; NEGDEPTH.append(neg)
    print("  %-21s | %13.4f | %.2fs | %13.4f | %s"%(
        (r.get("gait","?")+" "+r.get("terrain","?")+" r"+r.get("rep","?"))[:21],
        best_e,best_t,best_k, ("YES %.3f m"%neg) if neg<-0.001 else "no"))
if N:
    print()
    print("  n=%d collapses"%N)
    print("  peak |estimator below truth| : %.4f m"%st.mean(PE))
    print("  peak |kin_z below truth|     : %.4f m"%st.mean(PK))
    print("  estimate went BELOW ZERO in  : %d/%d runs (%.0f%%)"%(NEG,N,100*NEG/N))
    if NEGDEPTH: print("  ...by a mean of              : %.4f m"%st.mean(NEGDEPTH))
