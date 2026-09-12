#!/bin/bash
# Chain AI (2026-09-11 22:10): the overnight sequence on the shipping
# configuration (trot-only lead 1, RT bands, four-criteria judge), after
# chain AH (pid 97387). Repeats and depth where the day's numbers are thin:
#   1. hp_gap20 2.7 vs 2.5, 10 reps - 2.7 was 2/5 on lead 1 (coin flip or
#      marginal?), 2.5 5/5.
#   2. the 2.6 box vs 2.4, 10 reps - the cell the lead fixed (5/5 at N=5).
#   3. the FULL suite again - a second 20/20 on the record, or the case that
#      moves, with the stream-gap report beside it.
# Every campaign CSV carries imu_gap_max_ms; the suite gets the report.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-97387}"
LOG="$CAMPAIGN_DIR/open31_chainai.log"
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
say "chain AI pid $$: waiting for chain AH (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260911ah.sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; say "rig idle (phase $(phase)) - yaml trot lead: $(grep -m1 '^CTRL_MPC_LEAD_SLOW' host-run/ctrl_tuning.yaml | cut -c1-24)"
COURSES=hp_gap20 ARMS="v27:SPEED=2.7 v25:SPEED=2.5" run hp_ship27_n10 10 2.7
COURSES=wkc_box ARMS="v26:SPEED=2.6 v24:SPEED=2.4" run box_ship_n10 10 2.6
tm_wait
say "running the FULL suite (second pass on the shipping configuration) on $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
python3 unittests/test_validated_missions.py > "$CAMPAIGN_DIR/suite_full_20260911d.log" 2>&1
say "suite exit $?: $(grep -E "SUMMARY|passed|failed" "$CAMPAIGN_DIR/suite_full_20260911d.log" | tail -3 | tr '\n' ' ')"
say "worst IMU gap per run this hour and last (over 15 ms):"
bash gazebo/tools/stream_gap_report.sh "$(date +%Y%m%d)_$(date +%H | awk '{printf "%02d", $1-1}')" 15 2>/dev/null | tail -8 | sed 's/^/    /' | tee -a "$LOG"
bash gazebo/tools/stream_gap_report.sh "$(date +%Y%m%d_%H)" 15 2>/dev/null | tail -8 | sed 's/^/    /' | tee -a "$LOG"
say "chain AI done"
