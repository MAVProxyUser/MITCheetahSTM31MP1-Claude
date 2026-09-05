#!/bin/bash
# Does the bounded-deceleration stop reduce the attitude excursion at the
# finish line?  Second attempt.  The first one (rundata/campaigns/stopfix_ab)
# answered nothing, and the reason is worth writing down:
#
#   dash:30 @ 2.5 m/s only reaches the finish ~58% of the time.  All 10 falls
#   in that campaign happened MID-DASH, between 6.5 m and 23.8 m - not one run
#   that fell ever printed "[stop] shedding", so WP_ADEC, the only thing that
#   differed between the arms, was never exercised on a single failing run.
#   "harsh 8/12 vs bounded 6/12" was pure dash variance wearing the label of a
#   stop-ramp result.  The campaign also archived nothing on a PASS, so the 14
#   runs that DID reach the finish left no trace to score.
#
# Three changes:
#   1. dash:12 @ 2.0 m/s - short and slow enough that nearly every run reaches
#      the finish, so the treatment actually applies to the population scored.
#      WP_END_BRAKE=0 still forces the ramp to shed the full cruise speed.
#   2. Every run is snapshotted, PASS or FALL, tagged with arm+rep.  A pass
#      that leaves no trace cannot be scored, and the interesting runs here are
#      the passes.
#   3. The endpoint is the peak attitude excursion inside the stop window, not
#      pass/fail.  Falls at the finish are rare; excursions are on every run.
#      (Same reason as the OPEN-26 dose-response: score the quantity the
#      mechanism consumes, not the rare binary it occasionally produces.)
#
# Arms, interleaved every rep, same binary, only the env differing:
#   HARSH  WP_ADEC=10  -> steps hit the floor of 12 (0.6 s): the old behaviour
#   BOUND  WP_ADEC=1.0 -> 40 steps (2.0 s) at 2.0 m/s: the fix
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
. gazebo/tools/campaign_lib.sh
NAME=stopfix_ab2
N="${1:-25}"; V="${2:-2.0}"; DASH="${3:-12}"
DIR="$CAMPAIGN_DIR/$NAME"; mkdir -p "$DIR"; OUT="$CAMPAIGN_DIR/$NAME.csv"
echo "wall,arm,adec,rep,verdict,fall,reached_finish,shed_steps,shed_s,snapshot" > "$OUT"
FAILS=0

dump_with_retry(){   # $1 = tag -> echoes path or NONE
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

one(){ local arm="$1" rep="$2" adec="$3"
  timeout 300 python3 gazebo/conductor/mission_runner.py --terrain flat \
    --slot "dash:$DASH" --gait trotting --speed "$V" --dash 0 --wait-for-gate 1800 \
    --extra "WP_CLOSE_LEG=0 WP_END_BRAKE=0 WP_ADEC=$adec" \
    > "$DIR/run.log" 2>&1
  local L="$RUN_DIR/ctrl_0.log" V_ F FIN ST SS SNAP
  V_=$(grep -oE "VERDICT: [A-Z]+" "$DIR/run.log" | head -1 | awk '{print $2}')
  F=$(grep -oE '\[FALL\] [a-z]+' "$L" 2>/dev/null | tail -1 | awk '{print $2}')
  FIN=$(grep -c 'MISSION COMPLETE' "$L" 2>/dev/null || echo 0)
  ST=$(grep -oE 'over [0-9]+ steps' "$L" 2>/dev/null | tail -1 | awk '{print $2}')
  SS=$(grep -oE '\([0-9.]+ s,' "$L" 2>/dev/null | tail -1 | tr -d '(,s ')
  # tag carries the arm and rep so the snapshot is attributable to THIS run.
  # (v1 read the newest *FALL.json by mtime, so every PASS row in its csv
  #  reported the PREVIOUS run's peak pitch.  Never key a measurement on mtime.)
  SNAP=$(dump_with_retry "${arm}${rep}_${NAME}_${V_:-NONE}")
  echo "  $arm(adec=$adec) rep$rep ${V_:-NONE} ${F:-nofall} finish=${FIN:-0} shed=${ST:-?}steps/${SS:-?}s snap=$([ "$SNAP" = NONE ] && echo NONE || echo ok)"
  echo "$(date +%H:%M:%S),$arm,$adec,$rep,${V_:-NONE},${F:-none},${FIN:-0},${ST:-},${SS:-},$SNAP" >> "$OUT"
  if [ "$SNAP" = NONE ]; then
    FAILS=$((FAILS+1))
    [ "$FAILS" -ge 3 ] && { echo "  ABORT: 3 failed dumps - the instrument is not working"; campaign_failed "$NAME" "3 failed dumps"; exit 1; }
  else FAILS=0; fi
}

for r in $(seq 1 "$N"); do one HARSH "$r" 10 ; one BOUND "$r" 1.0 ; done
HR=$(awk -F, '$2=="HARSH"&&$7>0' "$OUT"|wc -l|tr -d ' '); HT=$(awk -F, '$2=="HARSH"' "$OUT"|wc -l|tr -d ' ')
BR=$(awk -F, '$2=="BOUND"&&$7>0' "$OUT"|wc -l|tr -d ' '); BT=$(awk -F, '$2=="BOUND"' "$OUT"|wc -l|tr -d ' ')
echo "  reached the finish line: harsh $HR/$HT   bounded $BR/$BT   (the population the ramp acts on)"
python3 gazebo/tools/stopfix_score.py --csv "$OUT" || true
campaign_done "$NAME" "harsh reach $HR/$HT bound reach $BR/$BT"
