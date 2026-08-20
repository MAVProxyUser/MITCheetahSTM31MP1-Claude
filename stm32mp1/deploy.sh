#!/bin/bash
# Build + package (+ optionally push) the STM32MP1 port.
#
#   stm32mp1/deploy.sh            # build + stage into stm32mp1/deploy_pkg/ (no board access)
#   stm32mp1/deploy.sh push       # also scp the package to the board
#   stm32mp1/deploy.sh push HOST DIR   # override ssh host / remote dir
#
# The package is self-contained: jpos_ctrl finds its .so's via an $ORIGIN rpath,
# so on the board just:  cd <dir> && sudo ./run.sh
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PKG="$ROOT/stm32mp1/deploy_pkg"
BUILD="$ROOT/mp1-build"
HOST="${2:-osd32mp1}"
RDIR="${3:-/usr/local/cheetah-mp1}"

export PATH="/opt/homebrew/bin:$PATH"

echo "== building =="
"$ROOT/stm32mp1/build.sh"
"$ROOT/stm32mp1/tools/build_tools.sh"

echo "== staging into $PKG =="
rm -rf "$PKG"; mkdir -p "$PKG"

# main executables (hardware + Gazebo SITL)
cp "$(find "$BUILD" -name jpos_ctrl -type f | head -1)" "$PKG/"
for b in jpos_ctrl_sim stand_sim mit_ctrl_sim; do
  f="$(find "$BUILD" -name "$b" -type f | head -1)"; [ -n "$f" ] && cp "$f" "$PKG/"
done
# every shared lib it needs (co-located; resolved via $ORIGIN rpath)
find "$BUILD" -name '*.so' -o -name '*.so.*' | while read -r so; do cp -a "$so" "$PKG/"; done
# bring-up tools
cp "$ROOT/stm32mp1/tools/bin/unitree_probe" "$PKG/" 2>/dev/null || true
cp "$ROOT/stm32mp1/tools/bin/imu_probe"     "$PKG/" 2>/dev/null || true
# config
cp "$ROOT/stm32mp1/config/stm32mp1-defaults.yaml"      "$PKG/"
cp "$ROOT/stm32mp1/config/jpos-user-parameters.yaml"   "$PKG/"
# JPosInitializer loads config/initial_jpos_ctrl.yaml (target_jpos/mid_jpos) relative
# to the run dir; without it _target_jpos is empty -> OOB read -> crash.
mkdir -p "$PKG/config"
cp "$ROOT/config/initial_jpos_ctrl.yaml" "$PKG/config/"
# runtime helpers
cp "$ROOT/stm32mp1/run.sh"       "$PKG/" 2>/dev/null || true
cp "$ROOT/stm32mp1/board_setup.sh" "$PKG/" 2>/dev/null || true

# strip to shrink transfer (keep debug builds locally)
arm-unknown-linux-gnueabihf-strip "$PKG"/jpos_ctrl "$PKG"/jpos_ctrl_sim "$PKG"/stand_sim "$PKG"/*.so* 2>/dev/null || true

echo "== package contents =="
ls -lh "$PKG" | sed 's/^/  /'

if [ "${1:-}" = "push" ]; then
  echo "== pushing to $HOST:$RDIR =="
  ssh "$HOST" "mkdir -p $RDIR"
  scp -qr "$PKG"/* "$HOST:$RDIR/"
  echo "pushed. on the board:  cd $RDIR && sudo ./board_setup.sh && sudo ./run.sh"
else
  echo "staged only (no board access). Re-run with 'push' to scp to the board."
fi
