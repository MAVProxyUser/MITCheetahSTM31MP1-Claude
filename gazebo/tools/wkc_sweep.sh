#!/bin/bash
# Find the course's own limit: the Standard Course Time a handler would set is
# whatever the fastest RELIABLE speed gives. Interleaved by speed within each
# rep so a drifting host cannot order the arms.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
. gazebo/tools/campaign_lib.sh
OUT="$CAMPAIGN_DIR/wkc_sweep.csv"; echo "wall,speed,rep,verdict,waypoints,fall,secs" > "$OUT"
for r in 1 2 3; do
  for v in 1.5 1.7 1.9 2.1; do
    t0=$SECONDS
    timeout 420 python3 gazebo/conductor/mission_runner.py --terrain flat \
      --slot "course:wkc_finals" --gait trotting --speed $v --dash 0 \
      --wait-for-gate 1800 > "$CAMPAIGN_DIR/wkc_run.log" 2>&1
    V=$(grep -oE "VERDICT: [A-Z]+" "$CAMPAIGN_DIR/wkc_run.log" | head -1 | awk '{print $2}')
    W=$(grep -c 'reached wp' "$RUN_DIR/ctrl_0.log" 2>/dev/null || echo 0)
    F=$(grep -oE '\[FALL\] [a-z]+' "$RUN_DIR/ctrl_0.log" 2>/dev/null | tail -1 | awk '{print $2}')
    echo "  v=$v rep$r ${V:-NONE} wp=$W ${F:-}"
    echo "$(date +%H:%M:%S),$v,$r,${V:-NONE},$W,${F:-none},$((SECONDS-t0))" >> "$OUT"
  done
done
echo "  --- summary ---"
for v in 1.5 1.7 1.9 2.1; do
  p=$(awk -F, -v s=$v '$2==s && $4=="PASS"' "$OUT" | wc -l | tr -d ' ')
  n=$(awk -F, -v s=$v '$2==s' "$OUT" | wc -l | tr -d ' ')
  echo "  $v m/s : $p/$n PASS"
done
campaign_done wkc_sweep "sweep done"
