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
# Arms are given as NAME:ENV pairs on the command line and interleaved every
# rep - same binary, only the env differing. Round 1 ran two:
#
#   WATCH  WP_SETTLE_WATCH=1                       2/14 fell at the finish
#   BLIND  WP_SETTLE_WATCH=0  (the old blind sleep) 8/17
#   Fisher exact two-tailed p=0.068 - the right direction, not separable.
#
# Suggestive-but-not-significant is where this project adds an ARM, not N: a
# third arm at a lower bail threshold tests the same mechanism by dose, and
# pools with the first against the control at the same time. Round 2:
#
#   ./wkc_settle_ab.sh 14 1.9 BLIND:WP_SETTLE_WATCH=0 \
#       BAIL8:WP_SETTLE_BAIL_DEG=8 BAIL5:WP_SETTLE_BAIL_DEG=5
#
# The endpoint that matters is "fell AFTER reaching the last waypoint" -
# whole-run PASS/FAIL also counts mid-course falls, which this change cannot
# touch. stopfix_score.py adds the continuous one (peak attitude after the
# stop), which every run yields including the passes.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
. gazebo/tools/campaign_lib.sh
NAME=wkc_settle_ab
N="${1:-18}"; V="${2:-1.9}"; shift 2 2>/dev/null || true
ARMS=("$@"); [ ${#ARMS[@]} -gt 0 ] || ARMS=(WATCH:WP_SETTLE_WATCH=1 BLIND:WP_SETTLE_WATCH=0)
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

one(){ local arm="$1" rep="$2" env="$3"
  timeout 420 python3 gazebo/conductor/mission_runner.py --terrain flat \
    --slot "course:wkc_finals" --gait trotting --speed "$V" --dash 0 \
    --wait-for-gate 1800 --extra "$env" \
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
  for a in "${ARMS[@]}"; do
    one "${a%%:*}" "$r" "${a#*:}"
  done
  python3 gazebo/tools/stopfix_score.py --csv "$OUT" 2>/dev/null | tail -8
done
echo "  --- final ---"
python3 gazebo/tools/stopfix_score.py --csv "$OUT" || true
# THE endpoint: fell after reaching the last waypoint. Whole-run PASS/FAIL
# also counts mid-course falls, which a settle change cannot touch.
SUM=""
for a in "${ARMS[@]}"; do
  an="${a%%:*}"
  F=$(awk -F, -v A="$an" '$2==A && $5=="16" && $6!="none"' "$OUT"|wc -l|tr -d ' ')
  R=$(awk -F, -v A="$an" '$2==A && $5=="16"' "$OUT"|wc -l|tr -d ' ')
  echo "  $an: fell at the finish $F/$R (of the runs that got there)"
  SUM="$SUM $an $F/$R"
done
campaign_done "$NAME" "finish-line falls:$SUM"
