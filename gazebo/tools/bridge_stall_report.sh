#!/bin/bash
# Mid-run bridge stalls from the per-run bridge logs the conductor archives:
# the bridge's own `stalls>5ms=N/worst=X ms rx_backlog_max=B` line, one per
# second, with the pre-connect startup stall (peer=None, ~100 ms) excluded.
# This is OPEN-35's yardstick: before the time-constraint band, 2026-09-11
# 00:00-11:00 had 24 stalls over 20 ms and 5 over 100 ms in 305 runs.
#   usage: bridge_stall_report.sh [DATE_PREFIX] [MIN_MS]   e.g. 20260911_12 20
set -u
. "$(dirname "${BASH_SOURCE[0]}")/paths.sh"
PFX="${1:-$(date +%Y%m%d)}"; MIN="${2:-20}"
cd "$RUN_DIR/archive" || exit 1
n=0; for f in ${PFX}*_bridge_0.log; do [ -f "$f" ] && n=$((n+1)); done
echo "runs matching ${PFX}*: $n   (stalls >= ${MIN} ms, mid-run only; backlog ~ stall*500Hz = bridge alone stalled, backlog ~2 = machine-wide)"
for f in ${PFX}*_bridge_0.log; do
  [ -f "$f" ] || continue
  awk -v f="$f" -v min="$MIN" '/stalls>5ms=[1-9]/ && !/peer=None/ {
      match($0,/worst=[0-9.]+ms/); w=substr($0,RSTART+6,RLENGTH-8);
      if (w+0>=min) { match($0,/rx_backlog_max=[0-9]+/); b=substr($0,RSTART+15,RLENGTH-15);
        split(f,a,"_"); print a[1]"_"a[2], a[3], "second", NR, w" ms", "backlog="b } }' "$f"
done | sort | tee /dev/stderr | awk -v min="$MIN" '{c++; split($5,w," "); if (w[1]+0>=100) big++} END{printf "== %d stalls >= %s ms, %d >= 100 ms\n", c+0, min, big+0}'
