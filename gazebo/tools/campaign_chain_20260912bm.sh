#!/bin/bash
# Chain BM (2026-09-12 23:10): the other sub-courses on the served recipe (budget
# 2.0): wkc_box 2.6/2.8 and wkc_weave 2.6/2.8, 6 reps interleaved - the envelope
# table's remaining rows on the honest planner. After chain BL (pid 5560).
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
PREV="${PREV_CHAIN_PID:-5560}"
LOG="$CAMPAIGN_DIR/open38_chainbm.log"
say "chain BM pid $$: waiting for chain BL (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912bl[.]sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; tm_wait; say "rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
COURSES=wkc_box ARMS="v26:SPEED=2.6 v28:SPEED=2.8" run box_recipe_ladder 6 2.6
COURSES=wkc_weave ARMS="v26:SPEED=2.6 v28:SPEED=2.8" run weave_recipe_ladder 6 2.6
say "chain BM done"
