#!/bin/bash
# Chain CV (2026-09-16): cruise 2.5 vs the 2.4 RE-ACCELERATION CAP, head to head, in ONE block.
#
# Decision #9 has two candidate levers and they were measured in DIFFERENT CHAINS hours apart -
# cruise 2.5 by CS (15:06-17:01) and the cap by CT (from 19:19). Each beat its OWN interleaved
# control and those results stand:
#   cruise 2.5   17/18  vs 2.6's 15/18   passing pitch 19.7 sd 1.34 worst 22.7   lap 55.8
#   cap 2.4      12/12  vs 2.6's 10/12   passing pitch 18.3 sd 2.74 worst 25.8   lap 55.8
# But RANKING them against each other from those two chains is exactly what this tree says cannot
# be done - "blocks run at different times are not comparable on this rig" - which is why every arm
# inside a chain is interleaved run by run. The 3 deg worst-case gap between them is the size of
# effect inter-block drift has faked here before.
#
# So this chain puts them in the SAME block, alternating run by run, on the same course. Both knobs
# are explicit on both arms so neither inherits a recipe default: the cruise arm runs 2.5 with the
# cap OFF, the cap arm runs 2.6 with the cap at 2.4.
#
# THE QUANTITY THAT DECIDES is the worst PASSING peak pitch and the spread, not the mean - the
# E-stop at 28.65 deg is a THRESHOLD, so a lever that lowers the average while leaving a fat tail is
# worth less than one that pulls the tail in. Pitch is reported three ways (all runs, passing only,
# spread) because a fall's peak is high by definition and pooling it is circular; the lap time sits
# beside it because both levers cost about +0.7 s and a tie on margin is decided on time.
#
# NOTHING SHIPS. The recipe still serves 2.6 and decision #9 is the operator's; CS already answered
# "is 2.6 safe" (no - a PASSING run reached 29.1 deg) and this only ranks the two fixes.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
DATA="${CHEETAH_DATA:-$HOME/Desktop/Cheetah/rundata}"
export DUMP=0
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "gazebo/conductor/mission_[r]unner" >/dev/null && ! pgrep -f "gazebo/tools/open28_subcourse[.]sh" >/dev/null && ! pgrep -f "unittests/test_validated_[m]issions" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
tm_wait(){ local n=0; while tmutil status 2>/dev/null | grep -q "Running = 1"; do [ $n -eq 0 ] && say "Time Machine backup in progress - holding (not a run)"; n=$((n+1)); sleep 30; done; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps at $v (COURSES=${COURSES:-} ARMS=${ARMS:-} DUMP=$DUMP)"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
tier(){ local t="$1"; tm_wait; say "$t: fast suite tier"
  python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260916_cv$t.log" 2>&1
  say "$t suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260916_cv$t.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260916_cv$t.log") FAIL"
  sleep 20; wait_idle
}
# Worst PASSING peak and the spread decide; the mean and the lap sit beside them. A fall's peak is
# high by definition, so the all-runs column is reported but must never be the comparison.
score(){ awk -F, '$1!="wall" && $7!=""{
    n[$2]++; s[$2]+=$7; ss[$2]+=$7*$7; if($7+0>mx[$2])mx[$2]=$7+0
    if($4=="PASS"){ p[$2]++; sp[$2]+=$7; ssp[$2]+=$7*$7; if($7+0>mxp[$2])mxp[$2]=$7+0
                    if($17!=""){lt[$2]+=$17; ln[$2]++} } }
  END{ for(a in n){ m=s[a]/n[a]; sd=(n[a]>1)?sqrt((ss[a]-n[a]*m*m)/(n[a]-1)):0
      mp=(p[a]?sp[a]/p[a]:0); sdp=(p[a]>1)?sqrt((ssp[a]-p[a]*mp*mp)/(p[a]-1)):0
      printf "    %-9s %2d/%-2d PASS | PASSING worst %.1f -> margin %+.1f | sd %.2f | mean %.1f | lap %.1f s | all-runs mean %.1f worst %.1f\n",
        a, p[a]+0, n[a], mxp[a], 28.65-mxp[a], sdp, mp, (ln[a]?lt[a]/ln[a]:0), m, mx[a] } }' "$@" 2>/dev/null | sort; }
PREV="${PREV_CHAIN_PID:-34879}"
PREV_PAT="${PREV_CHAIN_PAT:-campaign_chain_20260912[a-z][a-z][.]sh}"
LOG="$CAMPAIGN_DIR/open28_chaincv.log"
say "chain CV pid $$: waiting for the previous chain (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -qE "$PREV_PAT"; do sleep 30; done
say "predecessor $PREV gone - CV has the rig"
rm -f "$CAMPAIGN_DIR/STOP_CV"
wait_idle; sleep 30; wait_idle; tm_wait
say "no deploy needed - CV varies planner env only; binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
A="cruise25:SPEED=2.5,WP_REACCEL_VMAX=0 cap24:SPEED=2.6,WP_REACCEL_VMAX=2.4"
say "arms (interleaved run by run in ONE block - the whole point of this chain): $A"
say "manipulation: the cap arm's logs must carry '[plan] re-acceleration cap' and the cruise arm's must not;"
say "  and the two arms' laps must differ from each other's nominal, or a SPEED token did not take"
tier 0
i=0
while [ ! -f "$CAMPAIGN_DIR/STOP_CV" ]; do
  i=$((i+1)); wait_idle; sleep 30; wait_idle; tm_wait; say "block $i: rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
  COURSES="wkc_weave" ARMS="$A" run "headtohead_cv$i" 6 2.6
  say "decision #9 head to head, pooled over CV. The E-stop is a THRESHOLD, so read the worst"
  say "  PASSING peak and the spread first; the mean and the lap decide a tie:"
  score "$CAMPAIGN_DIR"/headtohead_cv*.csv | tee -a "$LOG"
  tier "$i"
done
say "chain CV done (STOP_CV seen after block $i)"
