#!/bin/bash
# Chain BZ (2026-09-13 18:30): the 2.8 rung's remaining feature, measured. With the turn
# cap shipped, wkc_finals at 2.8 is 10/12 and both misses are the same instant: the
# re-acceleration out of the wp14 near-stop pivot, the orientation E-stop at pitch
# 30-33 deg with the body level in roll the moment the command reaches 2.8 on the
# straight (runs 6664/6666, ISSUES OPEN-38 18:25) - the OPEN-28 rise runaway, which
# the 1.0 m/s^2 slew was measured to soften at 2.2/2.4. A/B on the shipped recipe at
# 2.8: WP_VSLEW 1.0 (recipe) vs 0.7 vs 0.5, 6 reps interleaved, scored on the wp13->15
# window peak and the lap. Then a standing loop of the served recipe at 2.6 (wkc x6,
# hp x6, fast tier) until STOP_BZ, so the rig never idles (RULE ONE). Runs after chain
# BY (pid 70353; `touch $CAMPAIGN_DIR/STOP_BY` ends it after its current campaign).
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
PREV="${PREV_CHAIN_PID:-70353}"
LOG="$CAMPAIGN_DIR/open38_chainbz.log"
say "chain BZ pid $$: waiting for chain BY (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912by[.]sh"; do sleep 30; done
rm -f "$CAMPAIGN_DIR/STOP_BZ"
wait_idle; sleep 30; wait_idle; tm_wait
EX=$(curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin)["recipes"]["course"]["extra"])' 2>/dev/null)
say "rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8); course recipe serves: $EX"
COURSES=wkc_finals ARMS="recipe: slew07:WP_VSLEW=0.7 slew05:WP_VSLEW=0.5" run wkc28_slew 6 2.8
say "wkc28_slew wp13->15 window:"; python3 gazebo/tools/window_peak.py 13 15 "recipe=wkc28_slew:recipe" "slew07=wkc28_slew:slew07" "slew05=wkc28_slew:slew05" | tee -a "$LOG"
say "wkc28_slew S-bend 11->13:"; python3 gazebo/tools/window_peak.py 11 13 "recipe=wkc28_slew:recipe" "slew07=wkc28_slew:slew07" "slew05=wkc28_slew:slew05" | tee -a "$LOG"
i=0
while [ ! -f "$CAMPAIGN_DIR/STOP_BZ" ]; do
  i=$((i+1)); wait_idle; sleep 30; wait_idle; tm_wait; say "block $i: rig idle (phase $(phase))"
  COURSES=wkc_finals ARMS="recipe:" run "wkc26_recipe_bz$i" 6 2.6
  [ -f "$CAMPAIGN_DIR/STOP_BZ" ] && break
  COURSES=hp_gap20 ARMS="recipe:" run "hp26_recipe_bz$i" 6 2.6
  [ -f "$CAMPAIGN_DIR/STOP_BZ" ] && break
  tm_wait; say "block $i: fast suite tier"
  python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260913_bz$i.log" 2>&1
  say "block $i suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260913_bz$i.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260913_bz$i.log") FAIL"
  sleep 20
done
say "chain BZ done (STOP_BZ seen after block $i)"
