#!/bin/bash
# Worst IMU-stream gap per run from the archived per-run bridge logs - the
# sim-side stall class (OPEN-35): gz-transport's loopback holding the IMU
# topic under host load. Campaign CSVs carry this per run since 18:04; this
# is the same number for runs that did not go through the harness (the suite).
#   usage: stream_gap_report.sh [DATE_PREFIX] [MIN_MS]    e.g. 20260911_19 15
set -u
. "$(dirname "${BASH_SOURCE[0]}")/paths.sh"
PFX="${1:-$(date +%Y%m%d)}"; MIN="${2:-15}"
cd "$RUN_DIR/archive" || exit 1
n=0; over=0
for f in ${PFX}*_bridge_*.log; do
  [ -f "$f" ] || continue
  g=$(grep -v "peer=None" "$f" | grep -oE 'imu_gap_max=[0-9.]+' | cut -d= -f2 | sort -n | tail -1)
  [ -z "$g" ] && continue
  n=$((n+1))
  if [ "$(echo "$g > $MIN" | bc)" = 1 ]; then over=$((over+1)); echo "$(echo "$f" | cut -c1-15) $(echo "$f" | grep -o 'run[0-9]*') $(echo "$f" | grep -o 'bridge_[0-9]') imu_gap_max=${g}ms"; fi
done
echo "== $n runs matching ${PFX}*: $over with a worst IMU gap over ${MIN} ms"
