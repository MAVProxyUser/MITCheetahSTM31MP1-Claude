#!/bin/bash
# Chain U (2026-09-11 11:50): the fast suite tier on the binary that ships
# after today's harness work - the fall judge's third criterion ("down at
# an angle", OPEN-36) and the bridge on the mach time-constraint band
# (OPEN-35). Neither touches the control law; the suite is the check that a
# judge criterion does not misfire on a valid run and that the RT bridge
# does not change a verdict. Runs after chain T (pid 17085) releases the rig.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-17085}"
LOG="$CAMPAIGN_DIR/open31_chainu.log"
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "^python3 gazebo/conductor/mission_runner.py" >/dev/null && ! pgrep -f "^bash gazebo/tools/open28_subcourse.sh" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
tm_wait(){ local n=0; while tmutil status 2>/dev/null | grep -q "Running = 1"; do [ $n -eq 0 ] && say "Time Machine backup in progress - holding (not a run)"; n=$((n+1)); sleep 30; done; }
say "chain U pid $$: waiting for chain T (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260911t.sh"; do sleep 30; done
wait_idle; sleep 20; wait_idle; tm_wait
say "rig idle (phase $(phase)) - running the fast suite on $(md5 -q host-run/mit_ctrl_sim | cut -c1-8) (down-judge binary, RT bridge)"
python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260911b.log" 2>&1
say "suite exit $?: $(grep -E "SUMMARY|passed|failed" "$CAMPAIGN_DIR/suite_fast_20260911b.log" | tail -3 | tr '\n' ' ')"
say "chain U done"
