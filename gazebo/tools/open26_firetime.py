"""How early does the DECLARATION actually land? Measured at the real threshold.

The lead above is over a plateau-relative band. The detector fires on
kin_z < SIM_FALL_Z (0.10) HELD 0.5 s, so the declaration is later than the
crossing. This computes the crossing of the actual threshold in kin_z, and
the crossing of the SAME PHYSICAL HEIGHT in gz truth (matched by each
series' own plateau, since gz's model origin sits ~0.02 m above the
estimator's reference), then subtracts the 0.5 s hold.
"""
import json,csv,statistics as st
FALL_Z=0.10; HOLD=0.5
rows=list(csv.DictReader(open("/tmp/c27.csv")))
def plat(v): return st.median(sorted(v)[len(v)//2:])
def cross(ts,zs,thr,after):
    """First crossing below thr AFTER `after`. The `after` is not optional:
    the robot BEGINS belly-down on the deck by design, so a search from the
    start of the series returns t~0 every time - which is what produced a
    set of negative leads on the first attempt."""
    for i in range(len(zs)):
        if ts[i] >= after and zs[i] <= thr: return ts[i]
    return None
out=[]
for r in rows:
    if r["verdict"]=="PASS" or r["snapshot"] in ("NONE",""): continue
    R=[x for x in json.load(open(r["snapshot"]))["records"] if x.get("kin_z") is not None]
    if len(R)<200: continue
    ts=[x["t"] for x in R]; kz=[x["kin_z"] for x in R]
    tts=[];tzs=[]
    for line in open(r["truth"]):
        try: o=json.loads(line)
        except Exception: continue
        p=(o.get("p") or {}).get("0")
        if p: tts.append(o["t"]); tzs.append(p[2])
    if len(tzs)<50: continue
    pk,pt=plat(kz),plat(tzs)
    # align on each series' own descent onset (last time it was at
    # plateau-0.03) so the two independent clocks are comparable
    def onset(ts_,zs_,p):
        for i in range(len(zs_)-1,-1,-1):
            if zs_[i]>=p-0.03: return ts_[i]
        return ts_[0]
    ok=onset(ts,kz,pk); ot=onset(tts,tzs,pt)
    tk=cross(ts,kz,FALL_Z,ok)                       # detector's own crossing
    tt=cross(tts,tzs,FALL_Z+(pt-pk),ot)             # same height, in truth
    if tk is None or tt is None: continue
    lead=(tt-ot)-(tk-ok)
    out.append(lead)
    print("  %-9s rep%s  kin_z hits %.2f at +%.2fs after onset; body at +%.2fs -> lead %.2fs"%(
        r["gait"],r["rep"],FALL_Z,tk-ok,tt-ot,lead))
if out:
    m=st.mean(out)
    print()
    print("  n=%d collapses"%len(out))
    print("  the DETECTOR's height reaches its threshold %.2f s before the body does"%m)
    print("  minus the %.1f s hold -> the [FALL] declaration precedes the real"%HOLD)
    print("  arrival by %.2f s%s"%(m-HOLD, "" if m-HOLD>0 else " (i.e. NOT early once the hold is counted)"))
