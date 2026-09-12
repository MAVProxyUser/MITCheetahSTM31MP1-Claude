#!/bin/bash
# Chain AP (2026-09-12 09:20): after chain AO (pid 92867) - a third N=10
# block on the 2.6 box (the cell the trot lead fixed) and the fast tier.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-92867}"
LOG="$CAMPAIGN_DIR/open31_chainap.log"
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "^python3 gazebo/conductor/mission_runner.py" >/dev/null && ! pgrep -f "^bash gazebo/tools/open28_subcourse.sh" >/dev/null && ! pgrep -f "test_validated_missions.py" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
tm_wait(){ local n=0; while tmutil status 2>/dev/null | grep -q "Running = 1"; do [ $n -eq 0 ] && say "Time Machine backup in progress - holding (not a run)"; n=$((n+1)); sleep 30; done; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps at $v (COURSES=${COURSES:-} ARMS=${ARMS:-})"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
say "chain AP pid $$: waiting for chain AO (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912ao.sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; say "rig idle (phase $(phase))"
COURSES=wkc_box ARMS="v26:SPEED=2.6 v24:SPEED=2.4" run box_ship_n10c 10 2.6
tm_wait
say "running the fast suite on $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260912e.log" 2>&1
say "suite exit $?: $(grep -E "SUMMARY|passed|failed" "$CAMPAIGN_DIR/suite_fast_20260912e.log" | tail -3 | tr '\n' ' ')"
say "chain AP done"
