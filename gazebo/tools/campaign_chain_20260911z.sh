#!/bin/bash
# Chain Z (2026-09-11 15:45): the FULL suite tier on the binary and yaml as
# they ship tonight - lead 1 pinned (15:22), the fall judge's four criteria,
# the controller's periodic tasks and the bridge on the mach time-constraint
# band. The fast tier passed 13/13 at 15:42; the full tier is the record's
# headline validation. Runs after chain Y (pid 92785).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-92785}"
LOG="$CAMPAIGN_DIR/open31_chainz.log"
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "^python3 gazebo/conductor/mission_runner.py" >/dev/null && ! pgrep -f "^bash gazebo/tools/open28_subcourse.sh" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
tm_wait(){ local n=0; while tmutil status 2>/dev/null | grep -q "Running = 1"; do [ $n -eq 0 ] && say "Time Machine backup in progress - holding (not a run)"; n=$((n+1)); sleep 30; done; }
say "chain Z pid $$: waiting for chain Y (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260911y.sh"; do sleep 30; done
wait_idle; sleep 20; wait_idle; tm_wait
say "rig idle (phase $(phase)) - running the FULL suite on $(md5 -q host-run/mit_ctrl_sim | cut -c1-8), yaml lead: $(grep -m1 '^CTRL_MPC_SCHED_LEAD' host-run/ctrl_tuning.yaml | cut -c1-24)"
python3 unittests/test_validated_missions.py > "$CAMPAIGN_DIR/suite_full_20260911b.log" 2>&1
say "suite exit $?: $(grep -E "SUMMARY|passed|failed" "$CAMPAIGN_DIR/suite_full_20260911b.log" | tail -3 | tr '\n' ' ')"
say "chain Z done"
