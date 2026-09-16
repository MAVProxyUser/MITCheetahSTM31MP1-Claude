#!/bin/bash
# Chain CS (2026-09-16): the WEAVE ENVELOPE, 2.5 vs 2.6 - decision #9's lever.
#
# Chain CQ's margin map (n = 9 a course, 3 interleaved blocks) found the weak link in the shipped
# envelope, and it is not where I had been looking. `wkc_weave` at the served 2.6 goes 8/9 with
# pitch mean 23.0 deg and worst 31.5 - i.e. 2.9 deg PAST the 28.65 E-stop - while wkc_box keeps
# +4.0 of margin, hp_gap20 +5.4 and wkc_finals +8.0, and all three go 9/9. The weave's failing run
# (8967) is a BARE ENVELOPE fall: pitch 31.3 with a 31.5 peak held 62 ms, zero unsafe-locomotion
# lines, zero RecoveryStand entries, no folds, leg_y_max 221 mm inside the 240 limit, a clean
# 500 samples/s stream, six waypoints reached. No mechanism - the robot pitched past the limit.
#
# (I had recorded the weave's deficit as LATERAL, from run 8892's 34.6 deg of roll and 243 mm foot.
# That run was the OPEN-40 limit cycle, a mechanism failure the envelope did not cause. The weave
# fails two different ways and a single fall says which one it WAS, not which one BINDS.)
#
# THE MEASUREMENT, copying chain CO's proven design rather than inventing one: CO answered the same
# question on the hairpin at n = 18 an arm over three interleaved blocks and got 2.6 -> 22.2 deg
# against 2.5 -> 19.0, exact rank-sum p < 1e-5, for +0.2 s on a 49.5 s lap. So the POWERED QUANTITY
# is PEAK PITCH PER RUN - a number every run provides, pass or fail - not the fall rate, which on
# this course would need hundreds of runs. Arms interleaved run by run via the harness' own SPEED=
# arm token, explicit on BOTH so neither inherits the recipe default. Verdicts are recorded beside
# the pitch, since at 2.6 they can differ.
#
# NOTHING SHIPS FROM THIS CHAIN. The recipe keeps serving 2.6 and decision #9 is the operator's;
# this chain only measures, and it reports the LAP TIME cost beside the margin gained so the trade
# is visible rather than asserted. Runs in the gap behind CR, tier first, then blocks of the A/B
# plus the tier, until STOP_CS (RULE ONE).
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
  python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260916_cs$t.log" 2>&1
  say "$t suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260916_cs$t.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260916_cs$t.log") FAIL"
  sleep 20; wait_idle
}
# Peak pitch per arm with the margin to the E-stop, the verdict split, and the LAP COST beside it -
# a margin gain that costs seconds is a trade and must be reported as one. Also prints the pitch
# distribution's own spread, because CQ's own history says a rising SD is the advance warning of a
# cliff, not the mean.
score(){ awk -F, '$1!="wall" && $7!=""{
    n[$2]++; if($4=="PASS")p[$2]++; s[$2]+=$7; ss[$2]+=$7*$7; if($7+0>mx[$2])mx[$2]=$7+0;
    if($17!=""){lt[$2]+=$17; ln[$2]++} }
  END{ for(a in n){ m=s[a]/n[a]; sd=(n[a]>1)?sqrt((ss[a]-n[a]*m*m)/(n[a]-1)):0;
    printf "    %-6s %2d/%-2d PASS | pitch mean %.1f sd %.2f worst %.1f -> margin %+.1f | lap %.1f s\n",
      a, p[a]+0, n[a], m, sd, mx[a], 28.65-mx[a], (ln[a]?lt[a]/ln[a]:0) } }' "$@" 2>/dev/null | sort; }
PREV="${PREV_CHAIN_PID:-98529}"
LOG="$CAMPAIGN_DIR/open28_chaincs.log"
say "chain CS pid $$: waiting for chain CR (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912cr[.]sh"; do sleep 30; done
rm -f "$CAMPAIGN_DIR/STOP_CS"
wait_idle; sleep 30; wait_idle; tm_wait
say "no deploy needed - CR already deployed the binary carrying every OPEN-40 knob; CS varies SPEED only"
say "binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
A="w25:SPEED=2.5 w26:SPEED=2.6"
say "arms (interleaved run by run, SPEED explicit on BOTH so neither inherits the recipe): $A"
tier 0
i=0
while [ ! -f "$CAMPAIGN_DIR/STOP_CS" ]; do
  i=$((i+1)); wait_idle; sleep 30; wait_idle; tm_wait; say "block $i: rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
  COURSES="wkc_weave" ARMS="$A" run "weave_cs$i" 6 2.6
  say "wkc_weave envelope, peak pitch per run vs the 28.65 deg E-stop, pooled over CS:"
  score "$CAMPAIGN_DIR"/weave_cs*.csv | tee -a "$LOG"
  tier "$i"
done
say "chain CS done (STOP_CS seen after block $i)"
