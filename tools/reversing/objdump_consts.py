#!/usr/bin/env python3
"""Recover float constants from a function using objdump only.

Same job as extract_consts.py but without radare2's analysis pass, so it is
cheap enough to run while a SITL measurement is in flight (raw disassembly is
I/O, not CPU). Resolves:
  * mov wN,#lo16 + movk wN,#hi16,lsl 16   -> IEEE-754 float
  * fmov sN,#imm                          -> immediate
  * adrp xN,PAGE + ldr {q,d,s}M,[xN,#OFF] -> constant pool, read from the file

  objdump_consts.py <start_hex> <end_hex>
"""
import re
import struct
import subprocess
import sys

BIN = ("/Users/kfinisterre/Desktop/Cheetah/pi/Unitree_latest/"
       "autostart/sportMode/bin/Legged_sport")

start, end = sys.argv[1], sys.argv[2]
dis = subprocess.run(
    ["objdump", "-d", f"--start-address={start}", f"--stop-address={end}", BIN],
    capture_output=True, text=True).stdout


def f32(bits):
    return struct.unpack("<f", struct.pack("<I", bits & 0xFFFFFFFF))[0]


pending, imms, page, pool = {}, [], {}, set()
for line in dis.splitlines():
    m = re.search(r'\bmov\s+(w\d+), #(0x[0-9a-f]+|\d+)', line)
    if m:
        pending[m.group(1)] = int(m.group(2), 0)
        continue
    m = re.search(r'\bmovk\s+(w\d+), #(0x[0-9a-f]+|\d+), lsl #16', line)
    if m:
        v = f32((int(m.group(2), 0) << 16) | (pending.get(m.group(1), 0) & 0xFFFF))
        if v == 0 or 1e-6 < abs(v) < 1e6:
            imms.append(v)
        continue
    m = re.search(r'\bfmov\s+[sd]\d+, #(-?[\d.]+)', line)
    if m:
        imms.append(float(m.group(1)))
        continue
    m = re.search(r'\badrp\s+(x\d+), (0x[0-9a-f]+)', line)
    if m:
        page[m.group(1)] = int(m.group(2), 0)
        continue
    m = re.search(r'\bldr\s+[qds]\d+, \[(x\d+)(?:, #(0x[0-9a-f]+|\d+))?\]', line)
    if m and m.group(1) in page:
        pool.add(page[m.group(1)] + (int(m.group(2), 0) if m.group(2) else 0))

data = open(BIN, 'rb').read()

print(f"immediates ({len(set(round(v,6) for v in imms))}):")
for v in sorted(set(round(v, 6) for v in imms)):
    print(f"   {v:>14.6f}")

print(f"\nconstant pool ({len(pool)} sites):")
for t in sorted(pool):
    if t + 16 <= len(data):
        vals = struct.unpack_from('<4f', data, t)
        if any(v != 0 and (abs(v) < 1e-8 or abs(v) > 1e8) for v in vals):
            continue        # mid-array garbage
        print(f"   0x{t:06x}: " + "  ".join(f"{v:11.6f}" for v in vals))
