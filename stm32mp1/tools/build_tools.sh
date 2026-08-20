#!/bin/bash
# Cross-build the standalone STM32MP1 bring-up tools (no cmake, no LCM, no Cheetah core).
# Each is a self-contained armv7 binary you scp to the board and run to validate one
# subsystem before wiring it into the control loop.
#
#   stm32mp1/tools/build_tools.sh          # build all tools into stm32mp1/tools/bin/
set -e
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="$ROOT/stm32mp1/tools/bin"
mkdir -p "$OUT"

CXX=arm-unknown-linux-gnueabihf-g++
command -v "$CXX" >/dev/null || { echo "ERROR: $CXX not in PATH (brew install arm-unknown-linux-gnueabihf)"; exit 1; }

ARCH="-mcpu=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard"
# Match the main build: disable Eigen's aligned-SIMD assumption (ARMv7 NEON alignment traps).
EIGEN_FLAGS="-DEIGEN_MAX_ALIGN_BYTES=0 -DEIGEN_MAX_STATIC_ALIGN_BYTES=0"
COMMON="-O2 -std=gnu++14 $ARCH $EIGEN_FLAGS -Wall -Wextra -Wno-unused-parameter -static-libstdc++ -static-libgcc"
INC="-I$ROOT/robot/include -I$ROOT/lcm-types/cpp -I$ROOT/third-party"
# VectorNavData pulls in Eigen via common's cppTypes.h
INC_IMU="$INC -I$ROOT/common/include -I$ROOT/third-party/eigen"

echo "[tools] unitree_probe"
$CXX $COMMON $INC \
  "$ROOT/robot/src/rt/rt_unitree.cpp" \
  "$ROOT/stm32mp1/tools/unitree_probe.cpp" \
  -o "$OUT/unitree_probe"

echo "[tools] imu_probe"
$CXX $COMMON $INC_IMU \
  "$ROOT/robot/src/rt/rt_can_imu.cpp" \
  "$ROOT/stm32mp1/tools/imu_probe.cpp" \
  -o "$OUT/imu_probe" -lpthread -lm

echo "=== built ==="; ls -lh "$OUT" | sed 's/^/  /'
file "$OUT"/* 2>/dev/null | sed 's/^/  /'
