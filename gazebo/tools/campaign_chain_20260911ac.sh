#!/bin/bash
# Chain AC (2026-09-11 17:45): expsquare (walking, gait 20) fails on lead 2 as
# well as lead 1 (chain AB, 17:41: l1 0/1, l2 0/1 - 'leg 3 moving too quickly
# 10.3 m/s'), and it was 20/20 this morning with zero safety trips. The lead
# is not the cause. What else changed for a walking recipe since 05:49: the
# bridge on the real-time band (11:50) and the controller's periodic tasks on
# it (14:26) - the fast tier that passed after each has no walking case. So:
# the three full-tier cases, recipe gaits, lead 2 (the schedule's value
# there), bands ON vs OFF (CTRL_RT=0 BRIDGE_RT=0), interleaved. Takes the rig
# from chain AB once its expsquare campaign has finished (kills AB by argv;
# an orphaned harness finishes its own campaign first, wait_idle covers it).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
LOG="$CAMPAIGN_DIR/open31_chainac.log"
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
say "chain AC pid $$: waiting for chain AB's expsquare campaign to finish"
until [ -f "$CAMPAIGN_DIR/expsquare_lead.done" ] || [ -f "$CAMPAIGN_DIR/expsquare_lead.failed" ]; do sleep 5; done
P=$(pgrep -f "^bash gazebo/tools/campaign_chain_20260911ab.sh" | head -1)
if [ -n "$P" ] && ps -p "$P" -o command= | grep -q "campaign_chain_20260911ab.sh"; then kill "$P"; say "chain AB (pid $P) stopped after its expsquare campaign; its spiro/lissajous lead arms are superseded by this chain"; fi
sleep 5; wait_idle; sleep 30; wait_idle; say "rig idle (phase $(phase))"
export RECIPE_GAIT=1
B="CTRL_RT=0,BRIDGE_RT=0"
COURSES="expsquare:5:12" ARMS="rt:CTRL_MPC_SCHED_LEAD=2 nort:CTRL_MPC_SCHED_LEAD=2,$B" run expsquare_rt 3 1.5
COURSES="spiro:9.0:8" ARMS="rt:CTRL_MPC_SCHED_LEAD=2 nort:CTRL_MPC_SCHED_LEAD=2,$B" run spiro_rt 3 1.8
COURSES="lissajous:15:11:9" ARMS="rt:CTRL_MPC_SCHED_LEAD=2 nort:CTRL_MPC_SCHED_LEAD=2,$B" run lissajous_rt 2 1.5
say "chain AC done"
