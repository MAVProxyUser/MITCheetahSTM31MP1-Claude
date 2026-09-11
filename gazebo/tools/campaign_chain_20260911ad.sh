#!/bin/bash
# Chain AD (2026-09-11 18:05): the last lead question. The walking recipes'
# afternoon falls were the sim's IMU stream holding 21-45 ms (OPEN-35's third
# class), not the lead (chain AC: bands on/off identical, all passing once the
# stream was clean). spiro (trotting 1.8, continuous curvature, 'leg
# y-position bad' at 16:57) and lissajous 11:9 have not been A/B'd on the
# lead with a clean stream. Recipe gaits, lead 1 vs 2 by env, interleaved;
# the CSV's imu_gap_max_ms column says whether each run's stream was clean.
# Runs after chain AC (pid 33763).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-33763}"
LOG="$CAMPAIGN_DIR/open31_chainad.log"
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "^python3 gazebo/conductor/mission_runner.py" >/dev/null && ! pgrep -f "^bash gazebo/tools/open28_subcourse.sh" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps (COURSES=${COURSES:-} ARMS=${ARMS:-} RECIPE_GAIT=${RECIPE_GAIT:-0})"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
say "chain AD pid $$: waiting for chain AC (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260911ac.sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; say "rig idle (phase $(phase))"
export RECIPE_GAIT=1
COURSES="spiro:9.0:8" ARMS="l1:CTRL_MPC_SCHED_LEAD=1 l2:CTRL_MPC_SCHED_LEAD=2" run spiro_lead2 3 1.8
COURSES="lissajous:15:11:9" ARMS="l1:CTRL_MPC_SCHED_LEAD=1 l2:CTRL_MPC_SCHED_LEAD=2" run lissajous_lead2 2 1.5
say "chain AD done"
