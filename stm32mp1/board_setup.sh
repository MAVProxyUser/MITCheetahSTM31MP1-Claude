#!/bin/bash
# One-time / per-boot board setup for the STM32MP1 Cheetah port. Run as root on the board.
#
#   sudo ./board_setup.sh
#
# Brings up the CAN bus (IMU + sensors) and reports the serial ports available for
# the Unitree RS485 buses. Adjust CAN_BITRATE / the tty list to your wiring.
set -e

CAN_IF="${CAN_IF:-can0}"
CAN_BITRATE="${CAN_BITRATE:-1000000}"   # match the DroneCAN bus (commonly 1 Mbit)

echo "== CAN =="
if ip link show "$CAN_IF" >/dev/null 2>&1; then
  ip link set "$CAN_IF" down 2>/dev/null || true
  ip link set "$CAN_IF" up type can bitrate "$CAN_BITRATE"
  echo "  $CAN_IF up @ ${CAN_BITRATE} bps"
  ip -details -statistics link show "$CAN_IF" | sed 's/^/    /' | head -6
else
  echo "  WARNING: $CAN_IF not present. Check the CAN controller/overlay is enabled."
fi
echo "  NOTE: the DroneCAN node allocator must be running for nodes to publish"
echo "        (see OP Revo Redux allocatord.service). No IMU frames until it is."

echo "== RS485 serial ports =="
ls -1 /dev/ttySTM* 2>/dev/null | sed 's/^/  /' || echo "  no /dev/ttySTM* found"
echo "  The Unitree driver expects a UART per bus (default ttySTM1..4) with a fast"
echo "  RS485 transceiver + hardware DE. ttySTM0 is the console -- do not use it."
echo "  Enabling extra UARTs and their RTS/DE line is a device-tree/overlay change."

echo "== real-time =="
echo "  Run jpos_ctrl as root so SCHED_FIFO + mlockall take effect."
echo "done."
