#!/bin/bash
# Build the bring-up tools NATIVELY for this Mac, for bench work with a
# self-directing USB-RS485 adapter (ROBOTIS U2D2 or similar).
#
# WHY THIS EXISTS. rt_unitree targets the MP1's own UARTs and was Linux-only:
# termios2/BOTHER for the custom baud, TIOCSRS485 for hardware direction
# control. Neither exists on Darwin, so unitree_probe could only ever run on
# the board - and the board is not needed to characterise ONE motor on a
# bench. The Darwin path uses IOSSIOSPEED for the 5 Mbaud (it is not in the
# Bxxx table) and treats direction control as the adapter's job, which is what
# a U2D2 does anyway.
#
# This does NOT replace build_tools.sh - that still cross-builds for the board,
# and the board remains the deployment target.
set -e
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="$ROOT/stm32mp1/tools/bin_host"
mkdir -p "$OUT"
INC="-I$ROOT/robot/include -I$ROOT/lcm-types/cpp -I$ROOT/third-party"
clang++ -O2 -std=gnu++14 $INC \
  "$ROOT/stm32mp1/tools/unitree_probe.cpp" "$ROOT/robot/src/rt/rt_unitree.cpp" \
  -o "$OUT/unitree_probe"
echo "[tools-host] $OUT/unitree_probe"
echo
echo "  Run:  $OUT/unitree_probe /dev/tty.usbserial-XXXX 5000000 0"
echo "  Find the port with:  ls /dev/tty.usb*"
echo "  Zero-torque probe only - it commands nothing. Clamp the motor anyway."
