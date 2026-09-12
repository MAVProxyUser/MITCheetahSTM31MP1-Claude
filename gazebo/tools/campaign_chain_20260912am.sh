#!/bin/bash
# Chain AM (2026-09-12 03:10): the fleet regression, separated. The 3-dog
# dash at 3.0 read 4/9 dog-runs tonight against 18/18 at 05:40 yesterday;
# every fall sat on a 21-23 ms stream gap (three bridges on the loopback),
# but the shipped trot lead also moved the ramp from lead 2 to lead 1. Six
# fleet runs, alternating: shipped (LEAD_SLOW 1) / old schedule (LEAD_SLOW 2
# by env, --extra on every slot), after chain AL (pid 4296).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-4296}"
LOG="$CAMPAIGN_DIR/open31_chainam.log"
FLOG="$CAMPAIGN_DIR/fleet_dash_leadab.log"
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "^python3 gazebo/conductor/mission_runner.py" >/dev/null && ! pgrep -f "^bash gazebo/tools/open28_subcourse.sh" >/dev/null && ! pgrep -f "test_validated_missions.py" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
tm_wait(){ local n=0; while tmutil status 2>/dev/null | grep -q "Running = 1"; do [ $n -eq 0 ] && say "Time Machine backup in progress - holding (not a run)"; n=$((n+1)); sleep 30; done; }
say "chain AM pid $$: waiting for chain AL (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912al.sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; tm_wait; say "rig idle (phase $(phase))"
curl -s -m 5 -X DELETE http://127.0.0.1:8420/api/fleet_cap >/dev/null
( until grep -q "fleet A/B done" "$FLOG" 2>/dev/null; do c=$(curl -s -m 3 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("fleet_cap"))' 2>/dev/null); if [ -n "$c" ] && [ "$c" != 3 ] && [ "$c" != None ]; then curl -s -m 5 -X DELETE http://127.0.0.1:8420/api/fleet_cap >/dev/null; echo "$(date +%H:%M:%S) cap was $c - reset" >> "$FLOG.capreset"; fi; sleep 2; done ) &
RESET_PID=$!
echo "3-dog dash:100 trotting 3.0, shipped lead vs old schedule (LEAD_SLOW=2), 3 pairs ($(date +%H:%M))" > "$FLOG"
for i in 1 2 3; do
  for arm in ship sched; do
    X=(); [ "$arm" = sched ] && X=(--extra CTRL_MPC_LEAD_SLOW=2 --extra CTRL_MPC_LEAD_SLOW=2 --extra CTRL_MPC_LEAD_SLOW=2)
    echo "=== fleet rep $i arm $arm $(date +%H:%M) ===" >> "$FLOG"
    timeout 420 python3 gazebo/conductor/mission_runner.py --terrain flat --slot dash:100 --slot dash:100 --slot dash:100 --gait trotting --gait trotting --gait trotting --speed 3.0 --speed 3.0 --speed 3.0 --dash 0 --dash 0 --dash 0 ${X[@]+"${X[@]}"} --wait-for-gate 1800 --timeout 300 2>&1 | grep -E "mission result|FELL|VERDICT|PASS=|CAPPED|refused|ERROR" >> "$FLOG"
    sleep 5
  done
done
echo "=== fleet A/B done $(date +%H:%M) ===" >> "$FLOG"
kill "$RESET_PID" 2>/dev/null
say "fleet A/B: shipped $(awk '/arm ship/{a=1} /arm sched/{a=0} a && /mission result: PASS/{n++} END{print n+0}' "$FLOG")/9 dog-runs, old schedule $(awk '/arm sched/{a=1} /arm ship/{a=0} a && /mission result: PASS/{n++} END{print n+0}' "$FLOG")/9"
say "chain AM done"
