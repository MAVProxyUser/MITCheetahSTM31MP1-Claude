#!/usr/bin/env python3
"""Sample the DEM along a mission's planned path, at waypoint-generation time.

OPEN-7, the half of the planner integration that was never built. The
friction axis ships as a scalar (`WP_TERRAIN_MU` -> a lateral-acceleration
cap), which is right for grip because grip is the same everywhere on a
surface. GROUND SHAPE is not: a plan crossing a ridge and a plan running
along a valley floor are different problems on the same terrain kind, and a
single per-kind number cannot express either.

So sample the heightmap the conductor just generated, along the path the
robot is actually going to fly, and hand the planner a PROFILE.

The quantity that matters is NOT average grade. Measured on this stack:
`rolling` has the larger peak grade (41.5% local, 0.365 m of relief over
30 m) and costs walking NOTHING, while `rough` has a gentler 5.4% mean
grade and takes walking's ceiling from 2.5 down to ~2.0-2.25. What
separates them is relief AT STRIDE SCALE - whether the four feet can be on
a common plane:

    rough    per-stride (0.35 m) height mismatch  mean 21 mm, max 69 mm
    rolling  per-stride height mismatch           mean 11 mm, max 162 mm

A long-wavelength hill is something the body walks over with every foot
agreeing; short-wavelength relief is what perturbs foot placement. So the
profile reports, per sample: ground height, local grade, and the stride
mismatch |z(s+stride) - z(s)|.

Output CSV: s_m,z_m,grade,stride_mismatch_m
"""
import argparse
import math
import os
import sys

SIZE_M = 400.0          # matches terrain.py's map extent


def load_png_gray(path):
    """Grayscale heightmap as a list of rows. PIL if available, else a
    minimal PNG reader - this runs inside the launch path, so it must not
    hard-depend on a package the venv might not carry."""
    try:
        from PIL import Image
        import numpy as np
        return np.asarray(Image.open(path).convert("L")).astype(float)
    except Exception:  # noqa: BLE001
        import zlib
        import struct
        with open(path, "rb") as f:
            data = f.read()
        assert data[:8] == b"\x89PNG\r\n\x1a\n", "not a PNG"
        pos, idat, w, h, bitdepth, color = 8, b"", 0, 0, 8, 0
        while pos < len(data):
            ln = struct.unpack(">I", data[pos:pos + 4])[0]
            typ = data[pos + 4:pos + 8]
            body = data[pos + 8:pos + 8 + ln]
            if typ == b"IHDR":
                w, h, bitdepth, color = struct.unpack(">IIBB", body[:10])
            elif typ == b"IDAT":
                idat += body
            pos += 12 + ln
        raw = zlib.decompress(idat)
        nch = {0: 1, 2: 3, 4: 2, 6: 4}[color]
        stride = w * nch
        rows, prev, p = [], bytearray(stride), 0
        for _ in range(h):
            ft = raw[p]; p += 1
            line = bytearray(raw[p:p + stride]); p += stride
            for i in range(stride):
                a = line[i - nch] if i >= nch else 0
                b = prev[i]
                c = prev[i - nch] if i >= nch else 0
                if ft == 1: line[i] = (line[i] + a) & 0xFF
                elif ft == 2: line[i] = (line[i] + b) & 0xFF
                elif ft == 3: line[i] = (line[i] + (a + b) // 2) & 0xFF
                elif ft == 4:
                    pp = a + b - c
                    pa, pb, pc = abs(pp - a), abs(pp - b), abs(pp - c)
                    pr = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
                    line[i] = (line[i] + pr) & 0xFF
            prev = line
            rows.append([line[i * nch] for i in range(w)])
        return rows


def sample(png, zscale, pts, ds=0.10, stride=0.35):
    """pts = [(north, east), ...] in metres about the local origin."""
    grid = load_png_gray(png)
    G = len(grid)
    ppm = (G - 1) / SIZE_M
    c = (G - 1) / 2.0

    def z_at(n, e):
        # heightmap rows are +north, columns +east about the centre
        r = int(round(c + n * ppm)); q = int(round(c + e * ppm))
        r = min(max(r, 0), G - 1); q = min(max(q, 0), G - 1)
        return float(grid[r][q]) / 255.0 * zscale

    # resample the polyline at ds
    out = []
    s = 0.0
    for k in range(len(pts) - 1):
        (n0, e0), (n1, e1) = pts[k], pts[k + 1]
        seg = math.hypot(n1 - n0, e1 - e0)
        steps = max(1, int(seg / ds))
        for i in range(steps):
            t = i / steps
            n, e = n0 + t * (n1 - n0), e0 + t * (e1 - e0)
            out.append((s + t * seg, n, e, z_at(n, e)))
        s += seg
    if pts:
        out.append((s, pts[-1][0], pts[-1][1], z_at(*pts[-1])))

    rows = []
    nstride = max(1, int(round(stride / ds)))
    for i, (si, n, e, z) in enumerate(out):
        j = min(i + 1, len(out) - 1)
        grade = (out[j][3] - z) / ds if j != i else 0.0
        k = min(i + nstride, len(out) - 1)
        mismatch = abs(out[k][3] - z)
        rows.append((si, z, grade, mismatch))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--png", required=True)
    ap.add_argument("--zscale", type=float, required=True)
    ap.add_argument("--waypoints", required=True,
                    help="n,e;n,e;... in metres about the local origin")
    ap.add_argument("--out", required=True)
    ap.add_argument("--ds", type=float, default=0.10)
    ap.add_argument("--stride", type=float, default=0.35)
    args = ap.parse_args()
    pts = [tuple(float(x) for x in p.split(",")) for p in
           args.waypoints.split(";") if p.strip()]
    rows = sample(args.png, args.zscale, pts, args.ds, args.stride)
    with open(args.out, "w") as f:
        f.write("s_m,z_m,grade,stride_mismatch_m\n")
        for r in rows:
            f.write("%.3f,%.4f,%.4f,%.4f\n" % r)
    mm = [r[3] for r in rows]
    gr = [abs(r[2]) for r in rows]
    print("[terrain_profile] %d samples over %.1f m: stride mismatch "
          "mean %.3f m max %.3f m, |grade| mean %.1f%% max %.1f%%"
          % (len(rows), rows[-1][0] if rows else 0.0,
             sum(mm) / max(1, len(mm)), max(mm) if mm else 0.0,
             100 * sum(gr) / max(1, len(gr)), 100 * max(gr) if gr else 0.0))


if __name__ == "__main__":
    sys.exit(main())
