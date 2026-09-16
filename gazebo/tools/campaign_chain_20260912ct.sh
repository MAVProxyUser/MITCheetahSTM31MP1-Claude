#!/bin/bash
# Chain CT (2026-09-16): the RE-ACCELERATION CAP on the weave - the cheaper alternative to 2.5.
#
# CQ's margin map found wkc_weave has NO margin at the served 2.6 (8/9, pitch worst 31.5, i.e.
# 2.9 deg past the 28.65 E-stop, against +4.0/+5.4/+8.0 on the other three courses). The per-tick
# traces then localised it: two passing weave runs and the failing one track each other to within
# 0.1 m/s and half a degree from the reversal pivot at t~48 to t=54, ALL THREE hit the same pitch
# event at t=55 as the re-acceleration completes near 2.6 m/s (17.6 / 22.7 / 19.1 deg), and the
# passing pair hold 20-23 and are back to 8 by t=57 while the third runs to 33.2. So it is one
# reproducible event a lap that usually recovers and occasionally does not - a TAIL.
#
# CS is measuring the obvious lever, cruise 2.5 vs 2.6. THIS chain measures the TARGETED one.
# BodyLimits::v_reaccel_max (WP_REACCEL_VMAX, 0 = off, reaccel_dist 12 m) caps the forward pass for
# 12 m after ANY braking minimum - which is exactly the phase that fails. Integrating the trace's
# speeds, the event sits ~10 m past the pivot, inside that 12 m window, so the cap is active where
# it matters. And it should cost about HALF what 2.5 costs: 12 m at 2.4 instead of 2.6 is ~0.38 s,
# against CS's measured +0.75 s for slowing the whole 115 m lap.
#
# WHY RE-MEASURE A RETIRED NULL: the cap was measured null on the HAIRPIN (peak pitch 22.5 vs 22.7,
# n=12 an arm, the lap +0.3 s proving the manipulation took) and on wkc 2.8. But the hairpin had
# 4-6 deg of margin to work with and its effect would have been a shift; the weave has NO margin and
# its effect is a tail. An effect does not transfer across the band - that rule is in this tree's own
# memory - so a null measured where there was room says little about a course that overshoots by 2.9.
#
# THE SCORE, corrected by what CS's block 1 taught: a fall's peak pitch is high BY DEFINITION, so
# pooling falls into a peak-pitch mean is partly circular. This chain reports the pitch THREE ways -
# all runs, PASSING runs only, and the spread - and treats the VERDICT as the quantity that decides,
# because for a tail the continuous proxy over-states on all runs and under-states on survivors.
# n=18 an arm is the target (3 blocks of 6), which at the observed rates is ~6/18 vs 0/18, p ~ 0.02.
#
# NOTHING SHIPS FROM THIS CHAIN. Decision #9 is the operator's; this only measures, and it measures
# the cheaper option beside the one CS is testing so the two can be compared on the same course.
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
  python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260916_ct$t.log" 2>&1
  say "$t suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260916_ct$t.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260916_ct$t.log") FAIL"
  sleep 20; wait_idle
}
# THE VERDICT decides (the effect is a tail); the pitch is reported three ways so neither the
# circular pooling nor the survivor-only understatement can be mistaken for the answer.
score(){ awk -F, '$1!="wall" && $7!=""{
    n[$2]++; s[$2]+=$7; ss[$2]+=$7*$7; if($7+0>mx[$2])mx[$2]=$7+0
    if($4=="PASS"){ p[$2]++; sp[$2]+=$7; ssp[$2]+=$7*$7; if($7+0>mxp[$2])mxp[$2]=$7+0
                    if($17!=""){lt[$2]+=$17; ln[$2]++} } }
  END{ for(a in n){ m=s[a]/n[a]; sd=(n[a]>1)?sqrt((ss[a]-n[a]*m*m)/(n[a]-1)):0
      mp=(p[a]?sp[a]/p[a]:0); sdp=(p[a]>1)?sqrt((ssp[a]-p[a]*mp*mp)/(p[a]-1)):0
      printf "    %-8s VERDICT %2d/%-2d | all: mean %.1f sd %.2f worst %.1f | PASSING: mean %.1f sd %.2f worst %.1f -> margin %+.1f | lap %.1f s\n",
        a, p[a]+0, n[a], m, sd, mx[a], mp, sdp, mxp[a], 28.65-mxp[a], (ln[a]?lt[a]/ln[a]:0) } }' "$@" 2>/dev/null | sort; }
PREV="${PREV_CHAIN_PID:-45467}"
PREV_PAT="${PREV_CHAIN_PAT:-campaign_chain_20260912[a-z][a-z][.]sh}"
LOG="$CAMPAIGN_DIR/open28_chainct.log"
say "chain CT pid $$: waiting for the previous chain (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -qE "$PREV_PAT"; do sleep 30; done
say "predecessor $PREV gone - CT has the rig"
rm -f "$CAMPAIGN_DIR/STOP_CT"
wait_idle; sleep 30; wait_idle; tm_wait
say "no deploy needed - CT varies planner env only; binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
A="nocap:WP_REACCEL_VMAX=0 cap24:WP_REACCEL_VMAX=2.4"
say "arms (interleaved run by run, the knob explicit on BOTH so neither inherits a default): $A"
say "cruise stays 2.6 on both arms - this tests the TARGETED cap, not the envelope CS is testing"
tier 0
i=0
while [ ! -f "$CAMPAIGN_DIR/STOP_CT" ]; do
  i=$((i+1)); wait_idle; sleep 30; wait_idle; tm_wait; say "block $i: rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
  COURSES="wkc_weave" ARMS="$A" run "reaccel_ct$i" 6 2.6
  say "wkc_weave 2.6, re-acceleration cap off vs 2.4, pooled over CT (the VERDICT decides; a fall's"
  say "  peak pitch is high by definition, so the all-runs mean is partly circular - read all three):"
  score "$CAMPAIGN_DIR"/reaccel_ct*.csv | tee -a "$LOG"
  tier "$i"
done
say "chain CT done (STOP_CT seen after block $i)"
