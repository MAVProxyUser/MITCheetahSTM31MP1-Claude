#!/bin/bash
# Chain CC (2026-09-14 16:20): the lie-down's second stage, measured (ISSUES OPEN-30/27).
# Every finish tip on record - wkc 7190, the galloping/dash lie-down BADs, both star
# interlude roll-overs - begins at the damping hand-off: STAND_UP holds the body at
# 0.10 m for 2.5 s, then setEdamp(8.0) hands the legs to a pure damper and the body
# drops onto its folded shanks; the landing bounces (roll rate +-3-5 rad/s within
# 100 ms) in a subset of runs, rocking 10-20 deg, and occasionally past 90. The
# controller now reads WP_LIEDOWN_EDAMP (8.0 stock), WP_LIEDOWN_EDAMP0 (an initial
# gain the hold ramps DOWN from) and WP_LIEDOWN_RAMP_MS (gazebo/mit_sim_main.cpp
# dampingHold()). Deployed in the idle gap after chain CB (pid 83434; STOP_CB ends its
# loop after the current campaign; the running binary is kept as .pre_liedown), then
# on the served recipe: wkc 2.6, 12 reps x recipe / kd24 / ramp40 interleaved, scored
# by gazebo/tools/liedown_peak.py on the stage-2 peak roll and the bounce fraction
# (peak roll rate > 1 rad/s), then the hairpin x6 the same, then the fast tier, then
# a standing loop until STOP_CC (RULE ONE). Strictly sequential, one dog.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "gazebo/conductor/mission_[r]unner" >/dev/null && ! pgrep -f "gazebo/tools/open28_subcourse[.]sh" >/dev/null && ! pgrep -f "unittests/test_validated_[m]issions" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
tm_wait(){ local n=0; while tmutil status 2>/dev/null | grep -q "Running = 1"; do [ $n -eq 0 ] && say "Time Machine backup in progress - holding (not a run)"; n=$((n+1)); sleep 30; done; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps at $v (COURSES=${COURSES:-} ARMS=${ARMS:-})"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
lscore(){ local n="$1"; say "$n stage-2 lie-down score:"; python3 gazebo/tools/liedown_peak.py "recipe=$n:recipe" "kd24=$n:kd24" "ramp40=$n:ramp40" | tee -a "$LOG"; }
PREV="${PREV_CHAIN_PID:-83434}"
LOG="$CAMPAIGN_DIR/open30_chaincc.log"
say "chain CC pid $$: waiting for chain CB (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912cb[.]sh"; do sleep 30; done
rm -f "$CAMPAIGN_DIR/STOP_CC"
wait_idle; sleep 30; wait_idle; tm_wait
say "rig idle (phase $(phase)) - deploying the lie-down knob (running binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8) kept as .pre_liedown)"
cp -p host-run/mit_ctrl_sim host-run/mit_ctrl_sim.pre_liedown
if bash gazebo/deploy_host.sh >> "$LOG" 2>&1; then say "deployed $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"; else say "DEPLOY FAILED - see $LOG; holding the rig on the old binary"; fi
A="recipe: kd24:WP_LIEDOWN_EDAMP=24 ramp40:WP_LIEDOWN_EDAMP0=40,WP_LIEDOWN_RAMP_MS=600"
COURSES=wkc_finals ARMS="$A" run wkc26_liedown 12 2.6
lscore wkc26_liedown
COURSES=hp_gap20 ARMS="$A" run hp26_liedown 6 2.6
lscore hp26_liedown
tm_wait; say "fast suite tier on $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260914_liedown.log" 2>&1
say "suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260914_liedown.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260914_liedown.log") FAIL"
i=0
while [ ! -f "$CAMPAIGN_DIR/STOP_CC" ]; do
  i=$((i+1)); wait_idle; sleep 30; wait_idle; tm_wait; say "block $i: rig idle (phase $(phase))"
  COURSES=wkc_finals ARMS="recipe:" run "wkc26_recipe_cc$i" 6 2.6
  [ -f "$CAMPAIGN_DIR/STOP_CC" ] && break
  COURSES=hp_gap20 ARMS="recipe:" run "hp26_recipe_cc$i" 6 2.6
  [ -f "$CAMPAIGN_DIR/STOP_CC" ] && break
  tm_wait; say "block $i: fast suite tier"
  python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260914_cc$i.log" 2>&1
  say "block $i suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260914_cc$i.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260914_cc$i.log") FAIL"
  sleep 20
done
say "chain CC done (STOP_CC seen after block $i)"
