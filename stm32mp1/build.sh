#!/bin/bash
# Cross-build Cheetah-Software for the Octavo OSD32MP1 (Cortex-A7, armv7l, OpenSTLinux).
# Runs on macOS with the messense arm-unknown-linux-gnueabihf toolchain:
#     brew tap messense/macos-cross-toolchains && brew install arm-unknown-linux-gnueabihf
#
# Usage:
#   stm32mp1/build.sh                 # configure + build everything enabled
#   stm32mp1/build.sh --target biomimetics
#   stm32mp1/build.sh clean           # wipe the build dir first
set -e

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/mp1-build"

if [ "${1:-}" = "clean" ]; then rm -rf "$BUILD"; shift; fi
mkdir -p "$BUILD"
cd "$BUILD"

if [ ! -f "$BUILD/CMakeCache.txt" ]; then
  # CMAKE_POLICY_VERSION_MINIMUM: vendored third-party (osqp, etc.) declare
  # cmake_minimum_required < 3.5, which CMake 4.x refuses without this.
  cmake -DCMAKE_TOOLCHAIN_FILE="$ROOT/stm32mp1/toolchain.cmake" \
        -DSTM32MP1_BUILD=ON -DNO_SIM=ON \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
        "$ROOT"
fi

JOBS="$(sysctl -n hw.ncpu 2>/dev/null || echo 4)"
cmake --build . -j"$JOBS" "$@"
echo "=== artifacts ==="
find "$BUILD" -name '*.a' -o -name 'robot' -o -name 'jpos_ctrl' 2>/dev/null | sed "s#$BUILD/#  #"
