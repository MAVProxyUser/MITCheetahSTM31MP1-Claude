import json,csv,statistics as st
def plateau(zs): return st.median(sorted(zs)[len(zs)//2:])
def band(ts,zs,p,top=0.03,bot=0.15):
    hi,lo=p-top,p-bot; s=e=None
    for i in range(len(zs)-1,-1,-1):
        if zs[i]<=lo and e is None: e=i
        if e is not None and zs[i]>=hi: s=i; break
    if s is None or e is None or e<=s: return None
    dt=ts[e]-ts[s]
    return dict(dt=dt,rate=(zs[s]-zs[e])/dt if dt>0 else 0,i1=e)
rows=list(csv.DictReader(open("/tmp/c27.csv")))
print("gait      rep verd  kind  |  est_rate  kin_rate  TRUE_rate |  est_lead  kin_lead")
E=[];K=[];T=[];LE=[];LK=[];G={}
for r in rows:
    if r["snapshot"] in ("NONE",""): continue
    d=json.load(open(r["snapshot"]))
    R=[x for x in d["records"] if x.get("z") is not None and x.get("kin_z") is not None]
    if len(R)<200: continue
    ts=[x["t"] for x in R]
    ez=[x["z"] for x in R]; kz=[x["kin_z"] for x in R]
    e=band(ts,ez,plateau(ez)); k=band(ts,kz,plateau(kz))
    tts=[];tzs=[]
    for line in open(r["truth"]):
        try: o=json.loads(line)
        except Exception: continue
        p=(o.get("p") or {}).get("0")
        if p: tts.append(o["t"]); tzs.append(p[2])
    t=band(tts,tzs,plateau(tzs)) if len(tzs)>50 else None
    if not (e and k and t): continue
    last=R[e["i1"]]; ro=abs(last["roll"])*57.2958; pi=abs(last["pitch"])*57.2958
    kind="LEVEL" if (ro<10 and pi<15) else ("TIP" if (ro>25 or pi>25) else "MIXED")
    print("%-9s %3s %-5s %-5s | %9.3f %9.3f %10.3f | %+8.2fs %+9.2fs"%(
        r["gait"],r["rep"],r["verdict"],kind,e["rate"],k["rate"],t["rate"],
        t["dt"]-e["dt"], t["dt"]-k["dt"]))
    if r["verdict"]!="PASS":
        E.append(e["rate"]);K.append(k["rate"]);T.append(t["rate"])
        LE.append(t["dt"]-e["dt"]);LK.append(t["dt"]-k["dt"])
        G.setdefault(r["gait"],[]).append((e["rate"],k["rate"],t["rate"],t["dt"]-e["dt"],t["dt"]-k["dt"]))
if E:
    print()
    print("  COLLAPSES n=%d"%len(E))
    print("    gz TRUE descent rate        : %.3f m/s"%st.mean(T))
    print("    estimator z  descent rate   : %.3f  (%.2fx truth, leads %+.2fs)"%(st.mean(E),st.mean(E)/st.mean(T),st.mean(LE)))
    print("    detector kin_z descent rate : %.3f  (%.2fx truth, leads %+.2fs)"%(st.mean(K),st.mean(K)/st.mean(T),st.mean(LK)))
    for g,v in G.items():
        print("    -- %-8s n=%d  est %.2fx  kin %.2fx  est_lead %+.2fs  kin_lead %+.2fs"%(
            g,len(v),st.mean([x[0] for x in v])/st.mean([x[2] for x in v]),
            st.mean([x[1] for x in v])/st.mean([x[2] for x in v]),
            st.mean([x[3] for x in v]),st.mean([x[4] for x in v])))
