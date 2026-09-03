#!/bin/bash
# Phase 2: the stages that still need clean data, run on an IDLE machine.
#
# Nothing here compiles. On the board, building and running happen on different
# machines; on the Mac they compete - a `make -j8` during a sweep stole enough
# CPU from Gazebo + bridge + controller to turn a known-good 21.23 m run into
# 3.54 m, with the worst control-loop time jumping 1.0 -> 4.5 ms as the tell.
# Never build while a measurement is in flight.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
G=gazebo

echo "=== PHASE2 STAGE: max-speed refinement (bash-3.2 safe) ==="
bash $G/refine_maxspeed.sh /tmp/gaitmax2.out 48 2>&1 | tee /tmp/refine.out

echo "=== PHASE2 STAGE: async vs inline, clean ==="
bash $G/host_sweep.sh /tmp/async.cfg 48 2>&1 | tee /tmp/async.out

echo "=== PHASE2 STAGE: star retry, yaw-error clamp lowered ==="
bash $G/host_sweep.sh /tmp/star_retry.cfg 200 2>&1 | tee /tmp/star_retry.out

echo "=== PHASE2 COMPLETE ==="
