"""OPEN-26: where does the filter think the GROUND is?

Two mechanisms are dead. Foot-kinematics: the estimator does not track kin_z
(1.1x, a null). Orientation: attitude error is 0.02 deg in pitch, 1.45 deg in
roll, explaining 20% of the gap and 0% on the runs that show the gap most
clearly. What is left is the assumption underneath both signals - that a foot
the SCHEDULE calls "in stance" is resting on the ground at z=0.

kin_z is the body's height above its lowest foot, by forward kinematics. If
that foot is truly on the ground, kin_z IS the true body height. So

    truth_z - kin_z  =  how far the lowest foot is off the ground

is a direct measurement of the contact assumption, needing nothing new. It
should sit near zero (minus the constant 0.02 m frame offset between gz's
model origin and the estimator's reference) whenever a foot is really planted,
and depart from zero exactly when the geometry the filter trusts stops being
true.

Usage: open26_ground.py CSV [CSV...]
"""
import json,csv,sys,statistics as st

def truth_series(path):
    T=[];Z=[]
    try:
        for line in open(path):
            try: o=json.loads(line)
            except Exception: continue
            p=(o.get("p") or {}).get("0")
            if p: T.append(o["t"]); Z.append(p[2])
    except Exception: pass
    return T,Z
def plateau(v): return st.median(sorted(v)[len(v)//2:])
def at(T,Z,t):
    j=min(range(len(T)), key=lambda i: abs(T[i]-t)); return Z[j]

rows=[]
for f in sys.argv[1:]:
    try:
        for r in csv.DictReader(open(f)): rows.append(r)
    except Exception: pass

WALK=[];DESC=[];N=0
print("  run                   | lowest foot off the ground (m) | spread of the 4 feet")
print("                        |  walking      at the bottom    |  walking   bottom")
for r in rows:
    if r.get("snapshot","NONE") in ("NONE",""): continue
    try: d=json.load(open(r["snapshot"]))
    except Exception: continue
    R=[x for x in d["records"] if x.get("kin_z") is not None]
    if len(R)<300: continue
    tT,tZ=truth_series(r["truth"])
    if len(tZ)<50: continue
    ez=[x["z"] for x in R]; pe=plateau(ez); pt=plateau(tZ)
    OFF = pt - pe                      # constant frame offset, removed below
    # a mid-run walking sample, and the bottom of any descent
    mid=R[len(R)//2]
    # TIME alignment, not value alignment. The first version of this looked
    # up truth "where it was at the same depth as the estimator", which is
    # circular - it matches the two heights by construction and ends up
    # measuring kin_z - z, not foot-to-ground. Anchor both series on their
    # OWN descent onset instead (the alignment used for the rate work, which
    # a 17-run control validated at lead 0.00 s), then read truth at the
    # matched instant.
    oe = None
    for i in range(len(ez)-1,-1,-1):
        if ez[i] >= pe-0.03: oe = R[i]["t"]; break
    ot = None
    for i in range(len(tZ)-1,-1,-1):
        if tZ[i] >= pt-0.03: ot = tT[i]; break
    if oe is None or ot is None: continue
    def gap(rec):
        tz = at(tT, tZ, ot + (rec["t"] - oe))
        return (tz - OFF) - rec["kin_z"]
    gw=gap(mid)
    ei=None
    for i in range(len(ez)-1,-1,-1):
        if ez[i]<=pe-0.15: ei=i; break
    gb=gap(R[ei]) if ei is not None else None
    def spread(rec): 
        v=[rec["foot_z%d"%i] for i in range(4)]; return max(v)-min(v)
    if gw is None: continue
    N+=1; WALK.append(gw)
    if gb is not None: DESC.append(gb)
    print("  %-21s | %+8.4f   %s        |  %6.4f   %s"%(
        (r.get("gait","?")+" "+r.get("terrain","?")+" r"+r.get("rep","?"))[:21],
        gw, ("%+8.4f"%gb) if gb is not None else "    --  ",
        spread(mid), ("%6.4f"%spread(R[ei])) if ei is not None else "  --  "))
print()
if N:
    print("  n=%d runs"%N)
    print("  lowest foot off the ground, WALKING NORMALLY : %+.4f m"%st.mean(WALK))
    if DESC:
        print("  lowest foot off the ground, AT THE BOTTOM    : %+.4f m"%st.mean(DESC))
        print("  -> the contact assumption moves by %.4f m"%abs(st.mean(DESC)-st.mean(WALK)))
