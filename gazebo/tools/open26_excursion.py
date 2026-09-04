#!/usr/bin/env python3
"""Of the attitude excursions that WOULD have been E-stopped at 28.65 deg,
how many did the robot actually survive?

Pass/fail is a blunt endpoint: at the observed rates (52% vs 68%) even 100
runs per arm reaches p<0.05 only 58% of the time. But the mechanism gives a
much sharper one. With the limit raised, every time the robot crosses 28.65
deg is a natural experiment the old limit would have killed - and one run
contains many of them.

An excursion is scored RECOVERED if the body is still up (z > 0.20 m) and the
attitude is back under the limit 2 s later, and FATAL otherwise.

Needs traces from runs that PASSED too, so the campaign must dump every run,
not only falls.

Usage: open26_excursion.py [--archive DIR] [--limit 28.65]
"""
import json, glob, os, sys, argparse, statistics as st
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import paths

ap=argparse.ArgumentParser()
ap.add_argument("--archive", default=paths.ARCHIVE_DIR)
ap.add_argument("--limit", type=float, default=28.65)
ap.add_argument("--hold-ms", type=float, default=60.0)
ap.add_argument("--settle", type=float, default=2.0)
a=ap.parse_args()

tot=rec=fatal=0
peaks=[]; files=0
for f in sorted(glob.glob(os.path.join(a.archive,"*.json"))):
    try: d=json.load(open(f))
    except Exception: continue
    R=[x for x in (d.get("records") or []) if x.get("z") is not None]
    if len(R)<500: continue
    files+=1
    i=0; n=len(R)
    while i<n:
        worst=max(abs(R[i]["roll"]),abs(R[i]["pitch"]))*57.2958
        if worst < a.limit: i+=1; continue
        # an excursion begins; find how long it stays over
        j=i; pk=worst
        while j<n and max(abs(R[j]["roll"]),abs(R[j]["pitch"]))*57.2958 >= a.limit:
            pk=max(pk,max(abs(R[j]["roll"]),abs(R[j]["pitch"]))*57.2958); j+=1
        dur_ms=(R[min(j,n-1)]["t"]-R[i]["t"])*1000.0
        if dur_ms >= a.hold_ms:          # only ones the old limit would kill
            tot+=1; peaks.append(pk)
            t_end=R[min(j,n-1)]["t"]+a.settle
            k=j
            while k<n and R[k]["t"]<t_end: k+=1
            k=min(k,n-1)
            ok = (R[k]["z"]>0.20 and
                  max(abs(R[k]["roll"]),abs(R[k]["pitch"]))*57.2958 < a.limit)
            if ok: rec+=1
            else: fatal+=1
        i=j+1
print("  traces scanned: %d"%files)
print("  excursions past %.1f deg held >= %.0f ms (i.e. ones the shipped limit"%(a.limit,a.hold_ms))
print("  would have E-STOPPED): %d"%tot)
if tot:
    print("     RECOVERED (still up %.0fs later): %d  (%.0f%%)"%(a.settle,rec,100*rec/tot))
    print("     FATAL                            : %d  (%.0f%%)"%(fatal,100*fatal/tot))
    print("     peak attitude in those excursions: mean %.1f deg, max %.1f"%(st.mean(peaks),max(peaks)))
