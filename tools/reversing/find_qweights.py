#!/usr/bin/env python3
"""Find the convex-MPC state-cost weight vector inside Legged_sport.

MIT ships Q = {0.25,0.25,10, 2,2,50, 0,0,0.3, 0.2,0.2,0.1} - orientation,
position, angular rate, linear rate. Whether Unitree retuned it matters a lot:
this port's whole gait matrix was measured against MIT's weights, and the
"short horizon under-supports" behaviour is a direct consequence of them.

Scans .rodata for any 12-float run that looks like a plausible weight vector.
"""
import struct

BIN = ("/Users/kfinisterre/Desktop/Cheetah/pi/Unitree_latest/"
       "autostart/sportMode/bin/Legged_sport")
MIT_Q = [0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1]

data = open(BIN, 'rb').read()
hits = []

for off in range(0, len(data) - 48, 4):
    vals = struct.unpack_from('<12f', data, off)
    # all finite, non-negative, bounded - a cost vector never has huge or tiny junk
    if not all(0.0 <= v <= 200.0 for v in vals):
        continue
    if sum(1 for v in vals if v != 0.0) < 6:
        continue
    exact = all(abs(a - b) < 1e-6 for a, b in zip(vals, MIT_Q))
    # a retuned vector keeps MIT's SHAPE: big weight on z and on yaw-ish terms
    shapey = vals[5] >= 10.0 and vals[2] >= 5.0
    if exact or shapey:
        hits.append((off, vals, exact))

print(f"{len(hits)} candidate weight vectors\n")
for off, vals, exact in hits[:12]:
    tag = "  <== EXACT MATCH to MIT's Q" if exact else ""
    print(f"0x{off:06x}: " + " ".join(f"{v:g}" for v in vals) + tag)

if not any(e for _, _, e in hits):
    print("\nNo exact copy of MIT's Q vector found - Unitree retuned it, "
          "or it is built at runtime from yaml.")
