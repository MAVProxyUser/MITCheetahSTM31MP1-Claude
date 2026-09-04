"""Score contact PREDICTORS against gz ground truth.

The operator's point, 2026-09-04: "I don't understand how with an IMU and
your own known gait patterns how you can't infer that a foot landed from its
position, and the force of the body on the joints... It is seemingly obvious
when a leg has made contact."

This measures exactly that, against sim contact sensors used only as labels
(the EDU dog has none, so nothing here may become a control input):

  1. SCHEDULE       - contactPhase > 0, which is what the Kalman filter
                      currently trusts as "this foot is planted". There is no
                      contact estimation in this controller at all;
                      ContactEstimator::run() copies the gait schedule.
  2. FOOT SPEED     - world-frame foot speed below a threshold. Needs only
                      joint encoders and the IMU. v_world = R^T(vBody +
                      omega x r_foot + v_rel).
  3. SCHEDULE+SPEED - believe the schedule only when the foot is also slow.

Reported per predictor: accuracy, and separately the two error types, because
they are not equally harmful. Believing a foot is planted when it is in the
air (false stance) is what feeds the estimator a fixed world point that is
actually moving - the failure OPEN-26 is chasing.

Usage: open26_contact_score.py CSV [CSV...]
"""
import json, csv, sys, statistics as st

def load_contact(path):
    T=[];C=[]
    try:
        for line in open(path):
            try: o=json.loads(line)
            except Exception: continue
            if "c" in o: T.append(o["t"]); C.append(o["c"])
    except Exception: pass
    return T,C

SPEED_TH = float(__import__("os").environ.get("SPEED_TH", "0.30"))
rows=[]
for f in sys.argv[1:]:
    try:
        for r in csv.DictReader(open(f)): rows.append(r)
    except Exception: pass

def score(pred, truth):
    tp=sum(1 for p,t in zip(pred,truth) if p and t)
    tn=sum(1 for p,t in zip(pred,truth) if not p and not t)
    fp=sum(1 for p,t in zip(pred,truth) if p and not t)      # false STANCE
    fn=sum(1 for p,t in zip(pred,truth) if not p and t)
    n=len(pred) or 1
    return dict(acc=(tp+tn)/n, false_stance=fp/n, false_swing=fn/n, n=n)

AGG={}
runs=0
for r in rows:
    if r.get("snapshot","NONE") in ("NONE","") or not r.get("contact"): continue
    try: d=json.load(open(r["snapshot"]))
    except Exception: continue
    R=[x for x in d["records"] if x.get("foot_fz0") is not None]
    if len(R)<500: continue
    cT,cC=load_contact(r["contact"])
    if len(cC)<200: continue
    # ALIGNMENT BY CROSS-CORRELATING BODY HEIGHT. Two earlier attempts failed:
    # anchoring both series at their first sample assumed they start together
    # (contact_feed launches ~7 s after the controller, once gz sim is up), and
    # a motion-onset anchor was too soft an event to place precisely. Both left
    # every predictor at chance, which is the signature of a misaligned clock,
    # not of a bad predictor.
    #
    # Height is the right key: the trace's z and pose_feed's z measure the SAME
    # physical quantity, they move over the whole run, and NEITHER is a contact
    # predictor - so the offset cannot be tilted toward the schedule or toward
    # foot speed. pose_feed shares contact_feed's wall clock, so the offset
    # transfers to the contact log unchanged.
    pT=[];pZ=[]
    try:
        for line in open(r["truth"]):
            try: o=json.loads(line)
            except Exception: continue
            q=(o.get("p") or {}).get("0")
            if q: pT.append(o["t"]); pZ.append(q[2])
    except Exception: pass
    if len(pZ)<200: continue
    eT=[x["t"] for x in R[::10]]; eZ=[x["z"] for x in R[::10]]
    pz0=st.median(pZ); ez0=st.median(eZ)
    best=(None,1e18)
    for off_ms in range(-2000, 15001, 100):        # trace t=0 is BEFORE contact
        off=off_ms/1000.0
        err=0.0; n=0; j=0
        for t,z in zip(eT,eZ):
            tt=pT[0]+(t-eT[0])+off
            while j+1<len(pT) and abs(pT[j+1]-tt)<=abs(pT[j]-tt): j+=1
            if abs(pT[j]-tt)>0.1: continue
            err+=((z-ez0)-(pZ[j]-pz0))**2; n+=1
        if n>100 and err/n<best[1]: best=(off,err/n)
    if best[0] is None: continue
    t0e=eT[0]; t0c=pT[0]+best[0]
    j=0; P_s=[];P_v=[];P_b=[];T_=[]
    for x in R[::5]:                       # 100 Hz is plenty
        tc=t0c+(x["t"]-t0e)
        while j+1<len(cT) and abs(cT[j+1]-tc)<=abs(cT[j]-tc): j+=1
        if abs(cT[j]-tc)>0.05: continue
        for leg in range(4):
            truth = bool(cC[j][leg])
            # PHASE > 0, NOT > 0.5. contactPhase is progress THROUGH stance,
            # not a stance flag: PositionVelocityEstimator.cpp:165 reads it as
            # `phase` and ramps each foot's measurement trust up over [0,0.2]
            # and down over [0.8,1]. Scoring `> 0.5` first made the schedule
            # look 37% accurate with 58% false-swing, which was measuring my
            # predicate, not the schedule.
            sched = x["c%d"%leg] > 0.0
            slow  = x["foot_fz%d"%leg] < SPEED_TH
            P_s.append(sched); P_v.append(slow); P_b.append(sched and slow); T_.append(truth)
    if len(T_)<200: continue
    runs+=1
    for name,p in (("schedule",P_s),("foot speed",P_v),("schedule AND slow",P_b)):
        s=score(p,T_)
        a=AGG.setdefault(name,{"acc":[],"fs":[],"fw":[]})
        a["acc"].append(s["acc"]); a["fs"].append(s["false_stance"]); a["fw"].append(s["false_swing"])

print("  contact predictors vs gz ground truth, %d runs" % runs)
print()
print("  predictor            accuracy   FALSE STANCE   false swing")
print("                                  (says planted, (says airborne,")
print("                                   really in air)  really down)")
for name in ("schedule","foot speed","schedule AND slow"):
    if name not in AGG: continue
    a=AGG[name]
    print("  %-18s   %6.1f%%      %6.1f%%        %6.1f%%"%(
        name,100*st.mean(a["acc"]),100*st.mean(a["fs"]),100*st.mean(a["fw"])))
print()
print("  FALSE STANCE is the one that matters: the KF pins a foot it believes")
print("  planted as a fixed world point. If that foot is actually moving, the")
print("  filter is being lied to - which is the OPEN-26 mechanism.")
