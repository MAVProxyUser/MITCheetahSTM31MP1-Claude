#!/bin/bash
# THE FINISH-LINE FALL, tested against its actual mechanism.
#
# The Westminster course at 1.9 m/s completed all 16 waypoints and then went
# over. The stop ramp was blamed and twice failed to reproduce anything (see
# stopfix_ab2.sh's header). Reading run3326's trace from the STOP rather than
# from the descent:
#
#   brake completes, robot stands still on four feet at z=0.305, and 1.3 s
#   LATER the body pitch swings +7.7 -> level -> -8.8 deg at a constant
#   ~20 deg/s, flattens for 0.2 s, then diverges to 167 deg roll. op_mode
#   never leaves 0, because SafetyChecker's orientation trip is suspended
#   across stop windows (CLOSED-20).
#
# That 1.3 s is a blind sleep_for(1500) inside K_BALANCE_STAND - a controller
# whose own onEnter comment names "the marginal impedance regime where the WBC
# stand slowly rolls over". settleOnFeet() in mit_sim_main.cpp now watches that
# dwell: leave early when actually settled, bail to the damped lie-down at
# 8 deg (a full second of margin; a healthy stop peaks near 3).
#
#   WATCH  WP_SETTLE_WATCH=1  the watched settle
#   BLIND  WP_SETTLE_WATCH=0  the old blind 1.5 s sleep
#
# Interleaved every rep, same binary, only the env differing. The endpoint is
# the peak attitude AFTER the robot stops (stopfix_score.py) - every run gives
# one, so this does not depend on catching a rare fall.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
. gazebo/tools/campaign_lib.sh
NAME=wkc_settle_ab
N="${1:-18}"; V="${2:-1.9}"
DIR="$CAMPAIGN_DIR/$NAME"; mkdir -p "$DIR"; OUT="$CAMPAIGN_DIR/$NAME.csv"
[ -s "$OUT" ] || echo "wall,arm,rep,verdict,waypoints,fall,settle,snapshot" > "$OUT"
FAILS=0

dump_with_retry(){
  local tag="$1" p
  for try in 1 2 3; do
    p=$(python3 -c "
import sys; sys.path.insert(0,'gazebo')
import shm_reaper
print(shm_reaper.dump_snapshot(0,'$tag') or 'NONE')" 2>/dev/null | tail -1)
    [ "${p:-NONE}" != "NONE" ] && { echo "$p"; return 0; }
    sleep 3
  done
  echo NONE; return 1
}

one(){ local arm="$1" rep="$2" watch="$3"
  timeout 420 python3 gazebo/conductor/mission_runner.py --terrain flat \
    --slot "course:wkc_finals" --gait trotting --speed "$V" --dash 0 \
    --wait-for-gate 1800 --extra "WP_SETTLE_WATCH=$watch" \
    > "$DIR/run.log" 2>&1
  local L="$RUN_DIR/ctrl_0.log" V_ W F S SNAP
  V_=$(grep -oE "VERDICT: [A-Z]+" "$DIR/run.log" | head -1 | awk '{print $2}')
  W=$(grep -c 'reached wp' "$L" 2>/dev/null || echo 0)
  F=$(grep -oE '\[FALL\] [a-z]+' "$L" 2>/dev/null | tail -1 | awk '{print $2}')
  S=$(grep -oE '\[settle\] (BAILING|settled|full)' "$L" 2>/dev/null | tail -1 | awk '{print $2}')
  SNAP=$(dump_with_retry "${arm}${rep}_${NAME}_${V_:-NONE}")
  echo "  $arm rep$rep ${V_:-NONE} wp=$W ${F:-nofall} settle=${S:-none} snap=$([ "$SNAP" = NONE ] && echo NONE || echo ok)"
  echo "$(date +%H:%M:%S),$arm,$rep,${V_:-NONE},$W,${F:-none},${S:-none},$SNAP" >> "$OUT"
  if [ "$SNAP" = NONE ]; then
    FAILS=$((FAILS+1))
    [ "$FAILS" -ge 3 ] && { echo "  ABORT: 3 failed dumps"; campaign_failed "$NAME" "3 failed dumps"; exit 1; }
  else FAILS=0; fi
}

for r in $(seq 1 "$N"); do
  one WATCH "$r" 1
  one BLIND "$r" 0
  python3 gazebo/tools/stopfix_score.py --csv "$OUT" 2>/dev/null | tail -8
done
echo "  --- final ---"
python3 gazebo/tools/stopfix_score.py --csv "$OUT" || true
WP=$(awk -F, '$2=="WATCH"&&$4=="PASS"' "$OUT"|wc -l|tr -d ' '); WT=$(awk -F, '$2=="WATCH"' "$OUT"|wc -l|tr -d ' ')
BP=$(awk -F, '$2=="BLIND"&&$4=="PASS"' "$OUT"|wc -l|tr -d ' '); BT=$(awk -F, '$2=="BLIND"' "$OUT"|wc -l|tr -d ' ')
echo "  RESULT  watched $WP/$WT PASS   blind $BP/$BT PASS"
campaign_done "$NAME" "watched $WP/$WT blind $BP/$BT"
