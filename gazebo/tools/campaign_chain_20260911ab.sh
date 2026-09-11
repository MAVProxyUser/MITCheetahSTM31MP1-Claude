#!/bin/bash
# Chain AB (2026-09-11 17:10): the three full-tier cases that failed on the
# lead-1 pin (ISSUES OPEN-37) - expsquare and lissajous 11:9 (walking, gait
# 20) and spiro (trotting 1.8, continuous curvature) - on their RECIPE gaits
# and speeds, lead 1 vs lead 2 pinned by env, 3 reps interleaved, on the
# deployed binary (controller and bridge on the RT band). Runs after chain
# AA (pid 7760).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-7760}"
LOG="$CAMPAIGN_DIR/open31_chainab.log"
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
say "chain AB pid $$: waiting for chain AA (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260911aa.sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; say "rig idle (phase $(phase)) - yaml lead line: $(grep -m1 'CTRL_MPC_SCHED_LEAD: 1' host-run/ctrl_tuning.yaml | cut -c1-28)"
export RECIPE_GAIT=1
COURSES="expsquare:5:12" ARMS="l1:CTRL_MPC_SCHED_LEAD=1 l2:CTRL_MPC_SCHED_LEAD=2" run expsquare_lead 3 1.5
COURSES="spiro:9.0:8" ARMS="l1:CTRL_MPC_SCHED_LEAD=1 l2:CTRL_MPC_SCHED_LEAD=2" run spiro_lead 3 1.8
COURSES="lissajous:15:11:9" ARMS="l1:CTRL_MPC_SCHED_LEAD=1 l2:CTRL_MPC_SCHED_LEAD=2" run lissajous_lead 3 1.5
say "chain AB done"
