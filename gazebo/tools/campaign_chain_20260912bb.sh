#!/bin/bash
# Chain BB (rewritten 2026-09-12 18:50): the top rungs under BOTH lateral
# budgets, after chain BD (pid 53378). Chain BC found the budget is the lever on
# the honest wkc_finals at 2.6 (2.5 -> 1/6, 2.0 -> 6/6, p = 0.015, no time
# cost; ISSUES OPEN-38). Before the course recipe changes, the next rung up
# and the hairpin - which shares the recipe - have to be measured under both
# budgets in one interleaved block:
#   wkc_finals 2.6 and 2.8 x {2.5, 2.0}, 5 reps (the 2.8/2.5 arm is the control
#   that contains the feature);
#   hp_gap20 2.6 @2.0 and 2.7 x {2.5, 2.0}, 5 reps (2.6 @2.5 was 6/6 in AZ).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-53378}"
LOG="$CAMPAIGN_DIR/open38_chainbb.log"
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
say "chain BB pid $$: waiting for chain BD (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912bd[.]sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; tm_wait; say "rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
COURSES=wkc_finals ARMS="s26a25:SPEED=2.6,WP_ALAT=2.5 s26a20:SPEED=2.6,WP_ALAT=2.0 s28a25:SPEED=2.8,WP_ALAT=2.5 s28a20:SPEED=2.8,WP_ALAT=2.0" run wkc_top_alat 5 2.6
COURSES=hp_gap20 ARMS="v26a20:SPEED=2.6,WP_ALAT=2.0 v27a25:SPEED=2.7,WP_ALAT=2.5 v27a20:SPEED=2.7,WP_ALAT=2.0" run hp_top_alat 5 2.7
say "chain BB done"
