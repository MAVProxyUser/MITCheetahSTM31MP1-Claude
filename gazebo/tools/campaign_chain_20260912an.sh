#!/bin/bash
# Chain AN (2026-09-12 05:35): the morning, after chain AM (pid 6955). Third
# full tier on the shipping configuration, then the two new rungs' third
# and second N=10 blocks (wkc_finals 2.4 vs 2.2, hp_gap20 2.5 vs 2.3) - the
# day-to-day record of the rungs the trot lead opened.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-6955}"
LOG="$CAMPAIGN_DIR/open31_chainan.log"
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
say "chain AN pid $$: waiting for chain AM (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912am.sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; tm_wait
say "running the FULL suite (third pass on the shipping configuration) on $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
python3 unittests/test_validated_missions.py > "$CAMPAIGN_DIR/suite_full_20260912a.log" 2>&1
say "suite exit $?: $(grep -E "SUMMARY|passed|failed" "$CAMPAIGN_DIR/suite_full_20260912a.log" | tail -3 | tr '\n' ' ')"
bash gazebo/tools/stream_gap_report.sh "$(date +%Y%m%d)_$(date +%H | awk '{printf "%02d", $1-1}')" 15 2>/dev/null | tail -3 | sed 's/^/    /' | tee -a "$LOG"
bash gazebo/tools/stream_gap_report.sh "$(date +%Y%m%d_%H)" 15 2>/dev/null | tail -3 | sed 's/^/    /' | tee -a "$LOG"
sleep 20; wait_idle
COURSES=wkc_finals ARMS="v24:SPEED=2.4 v22:SPEED=2.2" run wkc_ship_n10c 10 2.4
COURSES=hp_gap20 ARMS="v25:SPEED=2.5 v23:SPEED=2.3" run hp_ship_n10b 10 2.5
say "chain AN done"
