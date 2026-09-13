#!/bin/bash
# Chain BH (2026-09-12 21:10): the rung above the new ceiling. With WP_ALAT=2.0
# shipped in the course recipe (ISSUES OPEN-38/OPEN-28: wkc_finals 2.6 went
# 20/21 against 3/21), 2.8 still fell 0/5 - every time at wp13, the +40 deg
# corner at wp12 into the 16 m leg, which a 2.0 budget lets the planner take
# at cruise (the graded fillet there is ~18 m, so no braking). Does a smaller
# budget move that rung too, or is 2.8 the trot's own limit on a 40 deg
# corner? Interleaved, 5 reps: wkc_finals 2.8 x {2.0, 1.5, 1.2}, then
# hp_gap20 2.8 x {2.0, 1.5} (2.7 @2.0 was 5/5). After chain BG (pid 51149).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-51149}"
LOG="$CAMPAIGN_DIR/open38_chainbh.log"
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
say "chain BH pid $$: waiting for chain BG (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912bg[.]sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; tm_wait; say "rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
COURSES=wkc_finals ARMS="a20:WP_ALAT=2.0 a15:WP_ALAT=1.5 a12:WP_ALAT=1.2" run wkc28_alat 5 2.8
COURSES=hp_gap20 ARMS="a20:WP_ALAT=2.0 a15:WP_ALAT=1.5" run hp28_alat 5 2.8
say "chain BH done"
