#!/bin/bash
# Chain CB (2026-09-13 21:05): the 2.7 rung on the shipped recipe, then the 2.8 tail
# claim settled. Chain CA (ISSUES OPEN-38 20:58) showed no controller lever moves the
# 2.8 feature's median (26 deg on the wp14->wp15 rise, body 3.0 = within 5 % of the
# trot's declared 3.1 ceiling); QVX 0.25 and F_MAX 220 each read 4/5 against the
# recipe's 2/5, which N = 5 cannot separate, and body height 0.27 was harmful (0/6 at
# wp0). So: (1) the cruise itself is the honest lever - wkc 2.7 x6, hp 2.7 x6 (5/6
# before the cap), box 2.7 x4 on the served recipe; (2) then recipe / QVX 0.25 /
# F_MAX 220 at 2.8, 12 reps interleaved, scored on the wp13->15 window; (3) then a
# standing loop of the served recipe at 2.6 until STOP_CB (RULE ONE). Runs after chain
# CA (pid 41248; `touch $CAMPAIGN_DIR/STOP_CA` ends it after its current campaign).
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
PREV="${PREV_CHAIN_PID:-41248}"
LOG="$CAMPAIGN_DIR/open38_chaincb.log"
say "chain CB pid $$: waiting for chain CA (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912ca[.]sh"; do sleep 30; done
rm -f "$CAMPAIGN_DIR/STOP_CB"
wait_idle; sleep 30; wait_idle; tm_wait
EX=$(curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin)["recipes"]["course"]["extra"])' 2>/dev/null)
say "rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8); course recipe serves: $EX"
COURSES=wkc_finals ARMS="recipe:" run wkc27_recipe_cb 6 2.7
for W in "13 15" "11 13"; do say "wkc27 window $W:"; python3 gazebo/tools/window_peak.py $W "v27=wkc27_recipe_cb:recipe" "v26=wkc26_recipe_bx1:recipe" "v28=wkc28_recipe_by1:recipe" | tee -a "$LOG"; done
COURSES=hp_gap20 ARMS="recipe:" run hp27_recipe_cb 6 2.7
COURSES=wkc_box ARMS="recipe:" run box27_recipe_cb 4 2.7
A="recipe: qvx025:CTRL_MPC_QVX=0.25 fmax220:CTRL_F_MAX=220"
COURSES=wkc_finals ARMS="$A" run wkc28_ctrl2 12 2.8
say "wkc28 levers, CA + CB pooled, window 13->15:"; python3 gazebo/tools/window_peak.py 13 15 "recipe=wkc28_ctrl*:recipe" "qvx025=wkc28_ctrl*:qvx025" "fmax220=wkc28_ctrl*:fmax220" | tee -a "$LOG"
i=0
while [ ! -f "$CAMPAIGN_DIR/STOP_CB" ]; do
  i=$((i+1)); wait_idle; sleep 30; wait_idle; tm_wait; say "block $i: rig idle (phase $(phase))"
  COURSES=wkc_finals ARMS="recipe:" run "wkc26_recipe_cb$i" 6 2.6
  [ -f "$CAMPAIGN_DIR/STOP_CB" ] && break
  COURSES=hp_gap20 ARMS="recipe:" run "hp26_recipe_cb$i" 6 2.6
  [ -f "$CAMPAIGN_DIR/STOP_CB" ] && break
  tm_wait; say "block $i: fast suite tier"
  python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260913_cb$i.log" 2>&1
  say "block $i suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260913_cb$i.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260913_cb$i.log") FAIL"
  sleep 20
done
say "chain CB done (STOP_CB seen after block $i)"
