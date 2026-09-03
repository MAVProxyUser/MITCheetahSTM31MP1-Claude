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
def truth(path):
    tts=[];tzs=[]
    try:
        for line in open(path):
            try: o=json.loads(line)
            except Exception: continue
            p=(o.get("p") or {}).get("0")
            if p: tts.append(o["t"]); tzs.append(p[2])
    except Exception: pass
    return tts,tzs
out=[]
for src,camp in (("/tmp/c22.csv","c22"),("/tmp/c23.csv","c23")):
    for r in csv.DictReader(open(src)):
        if r["snapshot"] in ("NONE",""): continue
        try: d=json.load(open(r["snapshot"]))
        except Exception: continue
        R=[x for x in d["records"] if x.get("z") is not None]
        ets=[x["t"] for x in R]; ezs=[x["z"] for x in R]
        e=band(ets,ezs,plateau(ezs)); tts,tzs=truth(r["truth"])
        t=band(tts,tzs,plateau(tzs)) if len(tzs)>50 else None
        if not e or not t: continue
        last=R[e["i1"]]; ro=last["roll"]*57.2958; pi=last["pitch"]*57.2958
        kind="LEVEL" if (abs(ro)<10 and abs(pi)<15) else ("TIP" if (abs(ro)>25 or abs(pi)>25) else "MIXED")
        out.append(dict(c=camp,terr=r.get("terrain","flat"),v=r["verdict"],kind=kind,
                        er=e["rate"],tr=t["rate"],lead=t["dt"]-e["dt"],vz=abs(last["vz"])))
def show(lbl,v):
    if not v: return
    er=st.mean([o["er"] for o in v]); tr=st.mean([o["tr"] for o in v])
    ld=[o["lead"] for o in v]
    print("  %-34s n=%2d  est %.3f  true %.3f  ratio %.2f  lead %+.2fs (sd %.2f)  |vz| %.3f"%(
        lbl,len(v),er,tr,er/tr,st.mean(ld),st.pstdev(ld) if len(ld)>1 else 0,
        st.mean([o["vz"] for o in v])))
print("POOLED c22 + c23")
show("CONTROL: PASS lie-down",[o for o in out if o["v"]=="PASS"])
show("  of those, on flat",[o for o in out if o["v"]=="PASS" and o["terr"]=="flat"])
show("  of those, on rough",[o for o in out if o["v"]=="PASS" and o["terr"]=="rough"])
print()
show("COLLAPSE: all",[o for o in out if o["v"]!="PASS"])
show("  LEVEL collapses",[o for o in out if o["v"]!="PASS" and o["kind"]=="LEVEL"])
show("  TIP / MIXED",[o for o in out if o["v"]!="PASS" and o["kind"]!="LEVEL"])
print()
show("  c22 alone (first look)",[o for o in out if o["v"]!="PASS" and o["c"]=="c22"])
show("  c23 alone (confirmation)",[o for o in out if o["v"]!="PASS" and o["c"]=="c23"])
