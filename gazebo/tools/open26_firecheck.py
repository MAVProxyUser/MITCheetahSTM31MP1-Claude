"""At the instant the detector's kin_z crosses 0.10, where is the body REALLY?

kin_z sits up to 0.154 m below the true body height mid-descent (its failure
mode is all four feet reading off the deck at once, which makes the body look
low). If that dip carries kin_z under SIM_FALL_Z while the robot is still up,
the detector can declare a fall on a robot that is standing - and the 0.5 s
hold is the only thing between that and a false abort.

This reads the true body height at the exact tick kin_z first crosses the
threshold, and again 0.5 s later when the hold would expire. A fall is only
correctly declared if the body is genuinely near the deck at that second time.

Usage: open26_firecheck.py CSV [CSV...]
"""
import json,csv,sys,statistics as st
def plateau(v): return st.median(sorted(v)[len(v)//2:])
FALL_Z=0.10; HOLD=0.5
rows=[]
for f in sys.argv[1:]:
    try:
        for r in csv.DictReader(open(f)): rows.append(r)
    except Exception: pass
AT=[];AFTER=[];BAD=0;N=0
print("  run                   | true body z when kin_z hits 0.10 | ...and 0.5s later")
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
    tk=None
    for x in R:
        if x["t"]>=oe and x["kin_z"]<=FALL_Z: tk=x["t"]; break
    if tk is None: continue
    def truth_at(dt):
        tt=ot+dt
        j=min(range(len(tT)), key=lambda q: abs(tT[q]-tt))
        return tZ[j]-OFF
    z_at=truth_at(tk-oe); z_after=truth_at(tk-oe+HOLD)
    N+=1; AT.append(z_at); AFTER.append(z_after)
    bad = z_after > 0.20
    if bad: BAD+=1
    print("  %-21s | %26.3f | %14.3f %s"%(
        (r.get("gait","?")+" "+r.get("terrain","?")+" r"+r.get("rep","?"))[:21],
        z_at, z_after, "  <-- STILL UP" if bad else ""))
if N:
    print()
    print("  n=%d collapses"%N)
    print("  true body height when kin_z crosses 0.10   : %.3f m"%st.mean(AT))
    print("  true body height 0.5 s later (hold expires): %.3f m"%st.mean(AFTER))
    print("  runs where the robot was STILL UP (>0.20 m) at declaration: %d/%d"%(BAD,N))
    print()
    print("  (healthy standing height is ~0.29 m; the belly is on the deck at ~0.04-0.09)")
