#!/bin/bash
#
# Deploy the freshly built host binary into host-run/ SAFELY.
#
# THE BUG THIS EXISTS TO PREVENT
# ------------------------------
# Overwriting a Mach-O IN PLACE invalidates its code signature: macOS caches the
# signature against the inode, and the loader then SIGKILLs the replaced binary
# at exec with EXIT 137 AND ZERO OUTPUT.
#
# That failure is indistinguishable from a catastrophic controller bug. Every
# mission reports 0/5 with an empty log, so it reads as "the robot dies
# instantly". It cost a full verification sweep, a single-run bisect, and a
# "regression" that was diagnosed and REVERTED on the strength of a 0/5 that had
# never actually run.
#
# Three defences, all required:
#   1. rm the target first  - a fresh inode cannot inherit a stale cached signature
#   2. re-sign ad-hoc       - codesign -f -s -
#   3. PROVE it loads       - run it briefly and require non-empty output
#
# Never cp into host-run/ by hand. Call this.
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."
# DEPLOY_SRC lets a bisect deploy a binary built in a worktree (its own
# host-build/ is not this repo's) while keeping every step below - the fresh
# inode, the re-sign, and the proof that it loads. The alternative, a bare
# `cp` into host-run/, is the thing CLAUDE.md forbids because it once shipped
# a binary that was killed at exec and produced a false 0/5 sweep.
SRC=${DEPLOY_SRC:-host-build/user/MIT_Controller/mit_ctrl_sim}
DST=host-run/mit_ctrl_sim
[ -f "$SRC" ] || { echo "deploy: $SRC not built"; exit 1; }
rm -f "$DST"                       # 1. new inode
cp "$SRC" "$DST"
codesign -f -s - "$DST" 2>/dev/null # 2. re-sign
( cd host-run && env DYLD_LIBRARY_PATH=. timeout 3 ./mit_ctrl_sim 127.0.0.1 \
    stm32mp1-defaults.yaml mc-mit-ctrl-user-parameters.yaml \
    > /tmp/_deploy_check.log 2>&1 ) || true
if [ ! -s /tmp/_deploy_check.log ]; then   # 3. prove it loads
  echo "DEPLOY FAILED: binary produced NO output - it is being killed at exec."
  echo "  Do not run sweeps against it; every result would be a false 0/5."
  exit 1
fi
# 4. prove it actually STARTS: past parameter load, into the periodic task.
# "printed something" passed a binary on 2026-09-03 that died two lines in
# with an uncaught exception - a Release build with the yaml reads compiled
# out - and the next campaign wrote NONE for every rep.
if grep -q "terminating due to uncaught exception" /tmp/_deploy_check.log || \
   ! grep -q "PeriodicTask\|Start " /tmp/_deploy_check.log; then
  echo "DEPLOY FAILED: binary starts but dies before the control loop:"
  grep -iE "exception|error|abort" /tmp/_deploy_check.log | head -3 | sed 's/^/    /'
  echo "  Rolling back is up to you; nothing was rolled back automatically."
  exit 1
fi
echo "deploy ok: $(date -r "$DST" '+%H:%M:%S'), loads and prints $(wc -l < /tmp/_deploy_check.log) lines"
