#!/bin/bash
# Chain C (2026-09-10 14:35): the solo 100 m dash at 3.0 (trotting) is marginal
# on BOTH today's binaries (2/12 on the OPEN-31 build, 2/6 on the 07:04
# build), against a late-August table that had it crossing 3/3 at 3.1. The
# default-behaviour audit since that table finds three controller-side
# changes shipped ON: the contact-table lead implementation (alias removed,
# knob 2), the unix-socket transport with the bridge draining commands from
# its main loop, and the joint clamp (already exonerated: 1/6 vs 1/6). This
# chain waits for chain B (which brackets lead 1/3 and UDP) and adds the arms
# it lacks: the EXACT shipped lead behaviour (knob 1 + alias), the bridge's
# receive thread, and velocity aiding off (the one estimator default that
# could change how hard the MPC drives toward the commanded speed).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-44359}"
LOG="$CAMPAIGN_DIR/open31_chainc.log"
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "[m]ission_runner.py" >/dev/null && ! pgrep -f "[o]pen28_subcourse.sh" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps at $v (COURSES=${COURSES:-} ARMS=${ARMS:-})"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
say "chain C pid $$: waiting for chain B (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null; do sleep 30; done
wait_idle; sleep 20; wait_idle
say "rig idle (phase $(phase))"
COURSES="dash:100" ARMS="base:CTRL_JOINT_LIMITS=0 ship:CTRL_JOINT_LIMITS=0,CTRL_MPC_SCHED_LEAD=1,CTRL_MPC_TABLE_ALIAS=1 rxthread:CTRL_JOINT_LIMITS=0,BRIDGE_RX_THREAD=1 noaid:CTRL_JOINT_LIMITS=0,SIM_VEL_AIDING=0" run dash30_bisect2 5 3.0
say "chain C done"
