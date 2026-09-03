import json,csv,statistics as st
rows=list(csv.DictReader(open("/tmp/c22.csv")))
def plateau(zs): return st.median(sorted(zs)[len(zs)//2:])
def band(ts,zs,p,top=0.03,bot=0.19):
    hi,lo=p-top,p-bot; s=e=None
    for i in range(len(zs)-1,-1,-1):
        if zs[i]<=lo and e is None: e=i
        if e is not None and zs[i]>=hi: s=i; break
    if s is None or e is None or e<=s: return None
    dt=ts[e]-ts[s]
    return dict(dt=dt,dz=zs[s]-zs[e],rate=(zs[s]-zs[e])/dt if dt>0 else 0,i0=s,i1=e)
print("rep verd  end_roll end_pitch  kind   | est_rate truth_rate  ratio | est_vz@bottom")
F=[];T=[];VZ=[]
for r in rows:
    if r["snapshot"] in ("NONE",""): continue
    d=json.load(open(r["snapshot"])); R=[x for x in d["records"] if x.get("z") is not None]
    ets=[x["t"] for x in R]; ezs=[x["z"] for x in R]
    ep=plateau(ezs); e=band(ets,ezs,ep)
    tts=[];tzs=[]
    for line in open(r["truth"]):
        try: o=json.loads(line)
        except Exception: continue
        p=(o.get("p") or {}).get("0")
        if p: tts.append(o["t"]); tzs.append(p[2])
    t=band(tts,tzs,plateau(tzs)) if len(tzs)>50 else None
    last=R[e["i1"]] if e else R[-1]
    ro=last["roll"]*57.2958; pi=last["pitch"]*57.2958
    kind="LEVEL" if (abs(ro)<10 and abs(pi)<15) else ("TIP" if (abs(ro)>25 or abs(pi)>25) else "MIXED")
    vz=last["vz"]
    print("%3s %-5s %8.1f %9.1f  %-6s | %8.3f %10.3f  %5.2f | %6.2f"%(
        r["rep"],r["verdict"],ro,pi,kind,
        e["rate"] if e else 0, t["rate"] if t else 0,
        (t["rate"]/e["rate"]) if (e and t and e["rate"]) else 0, vz))
    if r["verdict"]!="PASS" and e and t:
        F.append(e["rate"]); T.append(t["rate"]); VZ.append(abs(vz))
print()
print("  COLLAPSES ONLY  n=%d"%len(F))
print("    estimator z rate : %.3f m/s"%st.mean(F))
print("    gz TRUE rate     : %.3f m/s"%st.mean(T))
print("    estimator |vz|   : %.3f m/s"%st.mean(VZ))
print("    the estimator reports the body falling %.2fx faster than it does"%(st.mean(F)/st.mean(T)))
