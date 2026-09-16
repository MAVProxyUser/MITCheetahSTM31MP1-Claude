#!/usr/bin/env python3
"""Reduce a per-tick shm_trace to a slim CSV, cheaply.

WHY THIS EXISTS (2026-09-16). The archive keeps a per-tick trace for EVERY run,
pass or fail, and comparing a failing run against its own arm's passing runs is
the only way this project has found to separate "the course is marginal" from
"the plan is wrong" - it is what localised the weave's pitch runaway to the
re-acceleration out of its reversal. But the traces are 17-35 MB of ONE-LINE
JSON (19k-40k records x ~40 fields), `json.load` on four of them blew a 120 s
budget while a run was in flight, and they are packed to `.json.zst` at the
NEXT campaign start - within minutes. So: rescue promptly, reduce ONCE with this,
and do every later comparison over the CSV.

The fast path is a single regex pass over the whole text, which avoids building
tens of thousands of dicts. Field ORDER is assumed (t, seq, tag, roll, pitch,
yaw, wx, wy, wz, vx, vy, vz, z) and verified against ShmTrace.h's writer; if the
regex matches nothing the code falls back to `json.load` rather than lying.

Units, from gazebo/ShmTrace.h - getting these wrong once cost a wrong conclusion:
  roll/pitch/yaw  RADIANS   (emitted here as DEGREES)
  wx/wy/wz        rad/s     (emitted here as deg/s)
  vx/vy/vz        vBody m/s (emitted here also as `speed` = hypot(vx,vy))
  z               m
  tag             tick | ESTOP | FALL | recover_*   -- FILTER TO tick BEFORE
                  taking any peak, or the post-E-stop flop contaminates it.

usage: trace_slim.py OUT_DIR TRACE.json [TRACE.json ...]
       (reads .json; for .json.zst pipe through `zstd -dc` first)
"""
import sys, os, re, json, math, csv

REC = re.compile(
    r'"t":\s*([-\d.eE+]+),\s*"seq":\s*\d+,\s*"tag":\s*"([a-z_]+)",'
    r'\s*"roll":\s*([-\d.eE+]+),\s*"pitch":\s*([-\d.eE+]+),\s*"yaw":\s*[-\d.eE+]+,'
    r'\s*"wx":\s*[-\d.eE+]+,\s*"wy":\s*([-\d.eE+]+),\s*"wz":\s*[-\d.eE+]+,'
    r'\s*"vx":\s*([-\d.eE+]+),\s*"vy":\s*([-\d.eE+]+),\s*"vz":\s*[-\d.eE+]+,'
    r'\s*"z":\s*([-\d.eE+]+)')
D = 180.0 / math.pi
HDR = ["t", "tag", "speed", "pitch_deg", "roll_deg", "wy_dps", "z"]

def rows_fast(text):
    for m in REC.finditer(text):
        t, tag, roll, pitch, wy, vx, vy, z = m.groups()
        yield [t, tag, "%.4f" % math.hypot(float(vx), float(vy)),
               "%.3f" % (abs(float(pitch)) * D), "%.3f" % (abs(float(roll)) * D),
               "%.2f" % (float(wy) * D), z]

def rows_slow(text):
    for r in json.loads(text).get("records", []):
        if "vx" not in r:
            continue
        yield ["%.4f" % r["t"], r.get("tag", ""),
               "%.4f" % math.hypot(r["vx"], r["vy"]),
               "%.3f" % (abs(r["pitch"]) * D), "%.3f" % (abs(r["roll"]) * D),
               "%.2f" % (r.get("wy", 0.0) * D), "%.4f" % r.get("z", 0.0)]

def main(argv):
    if len(argv) < 3:
        print(__doc__); return 2
    out = argv[1]; os.makedirs(out, exist_ok=True)
    for path in argv[2:]:
        text = open(path, errors="replace").read()
        rows = list(rows_fast(text))
        how = "regex"
        if not rows:                      # never guess - say so and do it properly
            rows = list(rows_slow(text)); how = "json.load fallback"
        dst = os.path.join(out, os.path.basename(path).replace(".json", "") + ".slim.csv")
        with open(dst, "w", newline="") as fh:
            w = csv.writer(fh); w.writerow(HDR); w.writerows(rows)
        print("  %-62s %6d ticks  %5.1f MB -> %4.0f KB  (%s)"
              % (os.path.basename(dst), len(rows), len(text) / 1e6,
                 os.path.getsize(dst) / 1024.0, how))
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
