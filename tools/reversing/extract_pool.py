#!/usr/bin/env python3
"""Recover constant-pool floats referenced by a function.

aarch64 loads a large literal as `adrp xN, PAGE` + `ldr qM, [xN, #OFF]`. The
link lengths and inertia tensors of the robot live there, not in the mov/movk
immediates, so recovering the physical model means resolving those pairs and
reading the .rodata behind them.
"""
import re
import struct
import subprocess
import sys

BIN = ("/Users/kfinisterre/Desktop/Cheetah/pi/Unitree_latest/"
       "autostart/sportMode/bin/Legged_sport")
addr = sys.argv[1] if len(sys.argv) > 1 else "0xd5080"

dis = subprocess.run(
    ["r2", "-q", "-e", "scr.color=0", "-e", "bin.cache=true",
     "-c", f"s {addr}; af; pdf", BIN],
    capture_output=True, text=True, timeout=900).stdout

page = {}
targets = set()
for line in dis.splitlines():
    m = re.search(r'\badrp\s+(x\d+),\s*0x([0-9a-f]+)', line)
    if m:
        page[m.group(1)] = int(m.group(2), 16)
        continue
    m = re.search(r'\bldr\s+[qds]\d+,\s*\[(x\d+)(?:,\s*0x([0-9a-f]+))?\]', line)
    if m:
        reg, off = m.group(1), int(m.group(2), 16) if m.group(2) else 0
        if reg in page:
            targets.add(page[reg] + off)

targets = sorted(targets)
print(f"{len(targets)} constant-pool sites referenced by {addr}\n")

with open(BIN, 'rb') as fh:
    data = fh.read()

seen = []
for t in targets:
    # PIE: virtual == file offset for these ELF load segments here
    chunk = data[t:t + 16]
    if len(chunk) < 16:
        continue
    vals = struct.unpack('<4f', chunk)
    pretty = '  '.join(f'{v:12.6f}' for v in vals)
    print(f'  0x{t:06x}: {pretty}')
    seen.extend(vals)

interesting = sorted(set(round(v, 6) for v in seen
                         if 1e-4 < abs(v) < 1e4))
print(f'\ndistinct plausible physical values ({len(interesting)}):')
print('  ' + ', '.join(f'{v:g}' for v in interesting))
