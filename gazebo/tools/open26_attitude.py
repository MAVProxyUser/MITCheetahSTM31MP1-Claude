"""OPEN-26: is a common-mode ATTITUDE error what makes both heights run fast?

Both the estimator's z and the detector's kin_z descend ~1.8x too fast during
a fold, yet they do not track each other (open26_mechanism.py: 1.1x, a null).
Two signals erring the same way independently point at a shared input, and
they share exactly one - the estimated orientation. kin_z is
`-min over legs of (rBody^T (hip + p))[2]`; the position block integrates in
the world frame that same attitude defines.

The prediction is specific: attitude error should GROW through the descent,
and be large enough to explain the height gap. A body that is really level
but estimated as pitched by theta makes a foot at horizontal distance d from
the body origin appear lower by about d*sin(theta). With d ~ 0.2 m (hip
offset), 10 deg buys ~0.035 m and 20 deg ~0.068 m - the observed gap is
0.05-0.08 m, so this is a quantitative test, not a hand-wave.

Needs truth files carrying roll/pitch (pose_feed >= 2026-09-04, 6 fields).

Usage: open26_attitude.py CSV [CSV...]
"""
import json,csv,sys,math,statistics as st

def truth_series(path):
    """returns t, z, roll, pitch  (roll/pitch only if the feed emits them)"""
    T=[];Z=[];R=[];P=[]
    try:
        for line in open(path):
            try: o=json.loads(line)
            except Exception: continue
            p=(o.get("p") or {}).get("0")
            if not p: continue
            T.append(o["t"]); Z.append(p[2])
            if len(p) >= 6: R.append(p[4]); P.append(p[5])
    except Exception: pass
    return T,Z,R,P

def plateau(v): return st.median(sorted(v)[len(v)//2:])

rows=[]
for f in sys.argv[1:]:
    try:
        for r in csv.DictReader(open(f)): rows.append(r)
    except Exception: pass

have=0; skipped=0
DR=[];DP=[];GAP=[];PRED=[]
print("  run                  | attitude ERROR at bottom (deg) | height gap | predicted by tilt")
print("                       |   d_roll      d_pitch          |  est-truth |  d*sin(tilt)")
for r in rows:
    if r.get("verdict")=="PASS" or r.get("snapshot","NONE") in ("NONE",""): continue
    try: d=json.load(open(r["snapshot"]))
    except Exception: continue
    R=[x for x in d["records"] if x.get("kin_z") is not None]
    if len(R)<300: continue
    tT,tZ,tR,tP = truth_series(r["truth"])
    if len(tZ)<50: continue
    if not tR: skipped+=1; continue          # truth predates the roll/pitch append
    have+=1
    ez=[x["z"] for x in R]; pe=plateau(ez); pt=plateau(tZ)
    # bottom of the descent in each series
    ei=max(range(len(ez)), key=lambda i: -ez[i] if ez[i]<=pe-0.03 else -9e9)
    ei=None
    for i in range(len(ez)-1,-1,-1):
        if ez[i]<=pe-0.15: ei=i; break
    if ei is None: continue
    ti=None
    for i in range(len(tZ)-1,-1,-1):
        if tZ[i]<=pt-0.15: ti=i; break
    if ti is None: ti=len(tZ)-1
    d_roll = (R[ei]["roll"] - tR[ti])*57.2958
    d_pitch= (R[ei]["pitch"]- tP[ti])*57.2958
    gap = (pt - tZ[ti]) - (pe - ez[ei])       # how much MORE the estimator fell
    tilt = math.radians(math.hypot(d_roll, d_pitch))
    pred = 0.2*math.sin(tilt)
    DR.append(abs(d_roll)); DP.append(abs(d_pitch)); GAP.append(gap); PRED.append(pred)
    print("  %-20s | %+8.2f    %+8.2f          | %+9.4f  | %10.4f"%(
        (r.get("gait","?")+" "+r.get("terrain","?")+" rep"+r.get("rep","?"))[:20],
        d_roll,d_pitch,gap,pred))
print()
if have:
    print("  n=%d collapses with attitude truth  (%d skipped: truth predates the append)"%(have,skipped))
    print("  |roll error|  mean %.2f deg   |pitch error| mean %.2f deg"%(st.mean(DR),st.mean(DP)))
    print("  height the estimator over-fell : %.4f m"%st.mean([abs(g) for g in GAP]))
    print("  height a tilt that size explains: %.4f m"%st.mean(PRED))
    m=st.mean([abs(g) for g in GAP]); p=st.mean(PRED)
    print("  -> tilt accounts for %.0f%% of the gap"%(100*p/m if m else 0))
else:
    print("  NO collapses yet carry attitude truth (%d skipped)."%skipped)
    print("  pose_feed gained roll/pitch at 2026-09-04 01:2x; runs after that will.")
