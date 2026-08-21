#!/usr/bin/env python3
"""Recover the float constants compiled into a function of Legged_sport.

aarch64 materialises a 32-bit float either as an immediate pair
    mov  wN, #lo16 ; movk wN, #hi16, lsl 16
or as an `fmov sN, #imm`, or by loading from the constant pool. This pulls all
three out of an r2 disassembly and reinterprets the bit patterns as IEEE-754
floats, which is how the robot's real physical parameters are recovered.
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
    capture_output=True, text=True, timeout=600).stdout

def f32(bits):
    return struct.unpack("<f", struct.pack("<I", bits & 0xFFFFFFFF))[0]

# --- mov/movk immediate pairs -------------------------------------------------
pending = {}
found = []
for line in dis.splitlines():
    m = re.search(r'\bmov\s+(w\d+),\s*(?:0x([0-9a-f]+)|(\d+))\b', line)
    if m:
        reg = m.group(1)
        val = int(m.group(2), 16) if m.group(2) else int(m.group(3))
        pending[reg] = val
        continue
    m = re.search(r'\bmovk\s+(w\d+),\s*0x([0-9a-f]+),\s*lsl\s*16', line)
    if m:
        reg, hi = m.group(1), int(m.group(2), 16)
        lo = pending.get(reg, 0)
        bits = (hi << 16) | (lo & 0xFFFF)
        v = f32(bits)
        if abs(v) < 1e6 and (v == 0 or abs(v) > 1e-6):
            found.append(v)

# --- fmov immediates ----------------------------------------------------------
for m in re.finditer(r'\bfmov\s+s\d+,\s*(-?[\d.]+)', dis):
    found.append(float(m.group(1)))

uniq = sorted(set(round(v, 6) for v in found))
print(f"function @ {addr}: {len(uniq)} distinct float constants")
for v in uniq:
    print(f"   {v:>14.6f}")
