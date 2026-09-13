#!/usr/bin/env python3
"""Peak attitude and body speed inside ONE feature of a course, per campaign arm.

The verdict count is the wrong instrument for a feature that fails 2 % of the
time: it needs N = 40 per arm to see a change. The peak pitch the feature
produces in EVERY run is a continuous quantity, and it is what the fall is a
tail of - so measure that (feedback-effect-size-does-not-transfer: measure the
quantity the mechanism consumes, at the value it triggers on). The window is
bounded by the nav's own `reached wpA` .. `reached wpB` lines in the shm_trace
snapshot's text_log (heartbeat-resolution timestamps), +-1 s.

usage:
  window_peak.py WPA WPB LABEL=CSVGLOB[:ARM] [LABEL=CSVGLOB[:ARM] ...]
  window_peak.py 11 13 served=wkc26_recipe_* alat15=wkc26_singlelap_alat:alat15

Found 2026-09-13 on wkc_finals's wp11->wp13 S-bend (-50 then +40 deg at
cruise): 2.4 -> 8 deg median, 2.6 -> 20 deg with the body at 2.84 m/s in the
window, 2.8 -> 38 deg and every run down; the lateral budget does not move it
at 2.6 (19.2 at 1.5) although it does at 2.8 (37.6 -> 24.8) - the body's
overshoot above the stick is the variable there, not the yaw ceiling.
"""
import csv, glob, math, os, statistics as st, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from snapio import load_json
D = os.environ.get("CHEETAH_DATA", "/Users/kfinisterre/Desktop/Cheetah/rundata")
DEG = 180 / math.pi

def window_peak(snap, wpa, wpb):
    d = load_json(snap); tl = d.get("text_log", [])
    R = [x for x in d["records"] if "pitch" in x and x.get("t", 0) > 0]
    hb = None; ta = None; tb = None
    for e in tl:
        m = str(e.get("msg", "")); t = e.get("t", 0.0)
        if "ctrl loop:" in m and t and t > 0: hb = float(t)
        if m.startswith(f"[nav] reached wp{wpa}") and ta is None: ta = hb
        if m.startswith(f"[nav] reached wp{wpb}") and tb is None: tb = hb
    if ta is None: return None
    if tb is None: tb = ta + 12.0          # fell inside the window: score to the end
    W = [x for x in R if ta - 1.0 <= x["t"] <= tb + 1.0]
    if not W: return None
    return (max(abs(x["pitch"]) * DEG for x in W), max(abs(x["roll"]) * DEG for x in W),
            max(x["vx"] for x in W))

def rows(pattern, arm=None):
    out = []
    for f in sorted(glob.glob(f"{D}/campaigns/{pattern}.csv")):
        for r in csv.DictReader(open(f)):
            if arm and r.get("course") != arm: continue
            if r.get("snapshot") in (None, "", "NONE"): continue
            out.append(r)
    return out

def main():
    if len(sys.argv) < 4: print(__doc__); sys.exit(2)
    wpa, wpb = sys.argv[1].zfill(2), sys.argv[2].zfill(2)
    for spec in sys.argv[3:]:
        label, rest = spec.split("=", 1)
        pattern, _, arm = rest.partition(":")
        pk = []; fell = 0; t = []
        for r in rows(pattern, arm or None):
            try: w = window_peak(r["snapshot"], wpa, wpb)
            except Exception: w = None
            if w is None: continue
            pk.append(w)
            if r["verdict"] != "PASS" and r["waypoints"] in (str(int(wpa) + 1), wpb.lstrip("0")): fell += 1
            if r.get("mission_t_s"): t.append(float(r["mission_t_s"]))
        if not pk: print(f"{label:28s} no data"); continue
        p = sorted(x[0] for x in pk); ro = [x[1] for x in pk]; v = [x[2] for x in pk]
        print(f"{label:28s} n={len(p):3d}  peak pitch median {st.median(p):5.1f}  p90 {p[int(0.9 * (len(p) - 1))]:5.1f}  "
              f"max {max(p):5.1f} | roll median {st.median(ro):5.1f} | body vx max median {st.median(v):4.2f} | "
              f"fell in window {fell} | lap median {st.median(t):6.1f}s" if t else
              f"{label:28s} n={len(p):3d}  peak pitch median {st.median(p):5.1f}  p90 {p[int(0.9 * (len(p) - 1))]:5.1f}  "
              f"max {max(p):5.1f} | roll median {st.median(ro):5.1f} | body vx max median {st.median(v):4.2f} | fell in window {fell}")

if __name__ == "__main__":
    main()
