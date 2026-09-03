#!/bin/bash
# Run the remaining sweep stages back to back in ONE process.
#
# This replaces a set of independent "wait for the previous stage then start"
# chains. Those were fragile for a reason worth remembering: a chain that waits
# with `pgrep -f "<pattern>"` MATCHES ITS OWN COMMAND LINE, because the pattern
# is a literal substring of the shell command doing the waiting. So a chain can
# wait on itself forever, and `pgrep`-based status checks report stages as
# "already running" when nothing is. One sequential script has no such problem.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
G=gazebo

echo "=== STAGE: star missions ==="
bash $G/host_sweep.sh /tmp/star_runs.cfg 200 2>&1 | tee /tmp/starruns.out

echo "=== STAGE: max-speed refinement ==="
bash $G/refine_maxspeed.sh /tmp/gaitmax2.out 48 2>&1 | tee /tmp/refine.out

echo "=== STAGE: async vs inline ==="
bash $G/host_sweep.sh /tmp/async.cfg 48 2>&1 | tee /tmp/async.out

echo "=== STAGE: dynamic gaits at low speed (collapse detector deployed) ==="
cp host-build/user/MIT_Controller/mit_ctrl_sim host-run/
bash $G/host_sweep.sh /tmp/lowspeed.cfg 55 2>&1 | tee /tmp/lowspeed.out

echo "=== ALL STAGES COMPLETE ==="
