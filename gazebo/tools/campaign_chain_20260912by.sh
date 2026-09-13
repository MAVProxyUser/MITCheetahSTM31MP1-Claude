#!/bin/bash
# Chain BY (2026-09-13 16:20): the 2.8 rung on the SHIPPED recipe, at depth. With the
# turn cap in the course recipe (chain BX ships WP_VTURN=2.4 behind BW), wkc_finals at
# 2.8 went 0/6 -> 6/6 in chain BV (S-bend 37.8 -> 8.7 deg, laps 95.2-96.0 s, the next
# feature the wp13->wp15 window at 24 deg). One block is not a rung: this holds the
# rig behind chain BX (pid 61142; `touch $CAMPAIGN_DIR/STOP_BX` ends BX's standing
# loop after its current campaign) with wkc 2.8 x6, hp_gap20 2.8 x6 (2.7 was 5/6 before
# the cap), wkc_box 2.8 x4 and the fast tier per block, looped until STOP_BY. All arms
# are the served recipe at SPEED (ARMS="recipe:"), so a served-recipe drift shows up
# as a verdict change, not a hidden variable. Strictly sequential, one dog.
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
PREV="${PREV_CHAIN_PID:-61142}"
LOG="$CAMPAIGN_DIR/open38_chainby.log"
say "chain BY pid $$: waiting for chain BX (pid $PREV) to exit (touch STOP_BX to end its standing loop)"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912bx[.]sh"; do sleep 30; done
rm -f "$CAMPAIGN_DIR/STOP_BY"
wait_idle; sleep 30; wait_idle; tm_wait
EX=$(curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin)["recipes"]["course"]["extra"])' 2>/dev/null)
say "rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8); course recipe serves: $EX"
echo "$EX" | grep -q "WP_VTURN=2.4" || say "WARNING: the served recipe has no WP_VTURN=2.4 - this block measures the OLD recipe at 2.8"
i=0
while [ ! -f "$CAMPAIGN_DIR/STOP_BY" ]; do
  i=$((i+1)); wait_idle; sleep 30; wait_idle; tm_wait; say "block $i: rig idle (phase $(phase))"
  COURSES=wkc_finals ARMS="recipe:" run "wkc28_recipe_by$i" 6 2.8
  say "wkc28 on the shipped recipe, all blocks - S-bend (11->13) and the next feature (13->15):"
  python3 gazebo/tools/window_peak.py 11 13 "recipe=wkc28_recipe_by*:recipe" | tee -a "$LOG"; python3 gazebo/tools/window_peak.py 13 15 "recipe=wkc28_recipe_by*:recipe" | tee -a "$LOG"
  [ -f "$CAMPAIGN_DIR/STOP_BY" ] && break
  COURSES=hp_gap20 ARMS="recipe:" run "hp28_recipe_by$i" 6 2.8
  [ -f "$CAMPAIGN_DIR/STOP_BY" ] && break
  COURSES=wkc_box ARMS="recipe:" run "box28_recipe_by$i" 4 2.8
  [ -f "$CAMPAIGN_DIR/STOP_BY" ] && break
  tm_wait; say "block $i: fast suite tier"
  python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260913_by$i.log" 2>&1
  say "block $i suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260913_by$i.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260913_by$i.log") FAIL"
  sleep 20
done
say "chain BY done (STOP_BY seen after block $i)"
