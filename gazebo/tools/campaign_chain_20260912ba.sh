#!/bin/bash
# Chain BA (2026-09-12 16:05): the validation tiers on the continuity-window
# follower AND the vertex-stop planner (ISSUES OPEN-38), after chain AZ (pid 44075). AZ runs no suite;
# chain AW ran the fast tier on the follower alone (5/5 before it was stopped); the FULL tier
# (SAR recipes, lissajous x3, spiro) has to pass before the binary is called
# shipped. The default invocation runs both tiers; a second fast pass is
# more evidence, not waste. Patterns are bracketed so no monitor's argv can
# hold this chain in wait_idle (2026-09-12 15:32 - it happened to chain AW).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-44075}"
LOG="$CAMPAIGN_DIR/open38_chainba.log"
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "^python3 gazebo/conductor/mission_[r]unner" >/dev/null && ! pgrep -f "^bash gazebo/tools/open28_subcourse[.]sh" >/dev/null && ! pgrep -f "^python3 unittests/test_validated_[m]issions" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
tm_wait(){ local n=0; while tmutil status 2>/dev/null | grep -q "Running = 1"; do [ $n -eq 0 ] && say "Time Machine backup in progress - holding (not a run)"; n=$((n+1)); sleep 30; done; }
say "chain BA pid $$: waiting for chain AZ (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912az[.]sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; tm_wait
say "rig idle (phase $(phase)) - running BOTH suite tiers on $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
python3 unittests/test_validated_missions.py > "$CAMPAIGN_DIR/suite_full_20260912_vertexstop.log" 2>&1
say "suite exit $?: $(grep -E "SUMMARY|passed|failed" "$CAMPAIGN_DIR/suite_full_20260912_vertexstop.log" | tail -3 | tr '\n' ' ')"
say "chain BA done"
