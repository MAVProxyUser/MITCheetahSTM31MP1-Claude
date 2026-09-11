#!/bin/bash
# Chain X (2026-09-11 14:45): the fast suite on the binary as it now ships -
# the controller's periodic tasks on the mach time-constraint band (deployed
# 14:26, OPEN-35) on top of the four-criteria fall judge. The previous suite
# run (12:23) predates the controller change. Runs after chain W (pid 54579).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-54579}"
LOG="$CAMPAIGN_DIR/open31_chainx.log"
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "^python3 gazebo/conductor/mission_runner.py" >/dev/null && ! pgrep -f "^bash gazebo/tools/open28_subcourse.sh" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
tm_wait(){ local n=0; while tmutil status 2>/dev/null | grep -q "Running = 1"; do [ $n -eq 0 ] && say "Time Machine backup in progress - holding (not a run)"; n=$((n+1)); sleep 30; done; }
say "chain X pid $$: waiting for chain W (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260911w.sh"; do sleep 30; done
wait_idle; sleep 20; wait_idle; tm_wait
say "rig idle (phase $(phase)) - running the fast suite on $(md5 -q host-run/mit_ctrl_sim | cut -c1-8) (controller on the RT band, four-criteria judge)"
python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260911c.log" 2>&1
say "suite exit $?: $(grep -E "SUMMARY|passed|failed" "$CAMPAIGN_DIR/suite_fast_20260911c.log" | tail -3 | tr '\n' ' ')"
say "chain X done"
