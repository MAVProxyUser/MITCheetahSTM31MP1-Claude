#!/bin/bash
# Chain AJ (2026-09-12 00:00): the rest of the night on the shipping
# configuration, after chain AI (pid 12121). Repeatability, not novelty:
#   1. wkc_finals 2.4 vs 2.2 at N=10 again (a second night-time measurement
#      of the new rung's 9/10)
#   2. the 3-dog fleet dash:100 at 3.0, 3 reps
#   3. the fast suite tier
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-12121}"
LOG="$CAMPAIGN_DIR/open31_chainaj.log"
FLOG="$CAMPAIGN_DIR/fleet_dash_night.log"
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
say "chain AJ pid $$: waiting for chain AI (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260911ai.sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; say "rig idle (phase $(phase))"
COURSES=wkc_finals ARMS="v24:SPEED=2.4 v22:SPEED=2.2" run wkc_ship_n10b 10 2.4
tm_wait
say "3-dog fleet dash:100 trotting 3.0, 3 reps"
curl -s -m 5 -X DELETE http://127.0.0.1:8420/api/fleet_cap >/dev/null
( until grep -q "fleet re-test done" "$FLOG" 2>/dev/null; do c=$(curl -s -m 3 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("fleet_cap"))' 2>/dev/null); if [ -n "$c" ] && [ "$c" != 3 ] && [ "$c" != None ]; then curl -s -m 5 -X DELETE http://127.0.0.1:8420/api/fleet_cap >/dev/null; echo "$(date +%H:%M:%S) cap was $c - reset" >> "$FLOG.capreset"; fi; sleep 2; done ) &
RESET_PID=$!
echo "3-dog dash:100 trotting 3.0, night, 3 reps ($(date +%H:%M))" > "$FLOG"
for i in 1 2 3; do
  echo "=== fleet rep $i $(date +%H:%M) ===" >> "$FLOG"
  timeout 420 python3 gazebo/conductor/mission_runner.py --terrain flat --slot dash:100 --slot dash:100 --slot dash:100 --gait trotting --gait trotting --gait trotting --speed 3.0 --speed 3.0 --speed 3.0 --dash 0 --dash 0 --dash 0 --wait-for-gate 1800 --timeout 300 2>&1 | grep -E "mission result|FELL|VERDICT|PASS=|CAPPED|refused|ERROR" >> "$FLOG"
  sleep 5
done
echo "=== fleet re-test done $(date +%H:%M) ===" >> "$FLOG"
kill "$RESET_PID" 2>/dev/null
say "fleet: $(grep -c 'VERDICT: PASS' "$FLOG") of 3 reps all-pass; per-dog PASS $(grep -c 'mission result: PASS' "$FLOG")/9"
sleep 20; wait_idle; tm_wait
say "running the fast suite on $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260912a.log" 2>&1
say "suite exit $?: $(grep -E "SUMMARY|passed|failed" "$CAMPAIGN_DIR/suite_fast_20260912a.log" | tail -3 | tr '\n' ' ')"
say "chain AJ done"
