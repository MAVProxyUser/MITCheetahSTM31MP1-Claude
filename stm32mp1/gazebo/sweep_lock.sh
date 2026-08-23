#!/bin/bash
# MUTEX FOR SWEEPS. Every harness here opens with `pkill -9 -f "gz sim"`, so two
# running at once do not merely contend for CPU - they murder each other's
# simulator mid-run and report the wreckage as data. That has now happened
# twice: once destroying a corner sweep, and once destroying BOTH an atom
# ladder (three runs reported 0/143 waypoints) and the fall-signature
# collection that killed it (runs cut off at 25 samples).
#
# Discipline did not work. So: source this at the top of every sweep script.
# It takes an exclusive lock and refuses to start if another sweep holds it.
#
#   source stm32mp1/gazebo/sweep_lock.sh   # exits 1 if a sweep is running
#
LOCKDIR="${TMPDIR:-/tmp}/cheetah_sweep.lock"
if ! mkdir "$LOCKDIR" 2>/dev/null; then
  echo "[lock] REFUSING TO START - another sweep holds $LOCKDIR" >&2
  echo "[lock] holder: $(cat "$LOCKDIR/owner" 2>/dev/null || echo unknown)" >&2
  echo "[lock] if that sweep is dead: rm -rf $LOCKDIR" >&2
  exit 1
fi
echo "$0 pid=$$ started=$(date '+%H:%M:%S')" > "$LOCKDIR/owner"
trap 'rm -rf "$LOCKDIR"' EXIT INT TERM
echo "[lock] held by $0 (pid $$)"
