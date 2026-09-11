#!/bin/bash
# Chain T (2026-09-11 11:30): the box at 2.6 is a CRUISE failure, not a corner
# one. Both genuine alon04 falls at N=8 (runs 5082, 5084) build pitch stride
# over stride on the 21 m leg to wp4 with the estimate at 2.65-2.85 m/s under a
# 2.6 command - the lead-2 trot above ~2.7 m/s, the sprint regression's regime,
# below the lead schedule's 2.7 COMMANDED threshold. WP_ALON 0.2 "fixes" it by
# never letting the leg reach cruise (9/9 vs 4/10). So: same box, same 0.4
# budget, three lead policies, interleaved:
#   ctrl   - as shipped (V_HI 2.7 on the command; lead 2 on every leg)
#   vhi25  - schedule threshold 2.5 (the 2.6 legs run on lead 1 from the ramp)
#   pin1   - lead 1 pinned everywhere (corners too)
# Runs after chain S (pid 8297) releases the rig.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
LOG="$CAMPAIGN_DIR/open31_chaint.log"
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "mission_runner.py" >/dev/null && ! pgrep -f "^bash gazebo/tools/open28_subcourse.sh" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps at $v (COURSES=${COURSES:-} ARMS=${ARMS:-})"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
while kill -0 8297 2>/dev/null && ps -p 8297 -o command= | grep -q "campaign_chain_20260911s.sh"; do sleep 20; done
wait_idle; sleep 30; wait_idle; say "chain T pid $$: rig idle (phase $(phase))"
# SIM_ESTERR=1 on every arm: ground-truth-vs-estimate lines ([ESTERR], instrumentation only,
# nothing reaches the control loop) so the 2.65-2.85 m/s the estimate reports on the wp4 leg
# under a 2.60 command can be read against truth - is the body over-speed, or the estimate?
COURSES=wkc_box ARMS="ctrl:WP_ALON=0.4,SIM_ESTERR=1 vhi25:WP_ALON=0.4,CTRL_MPC_LEAD_V_HI=2.5,SIM_ESTERR=1 pin1:WP_ALON=0.4,CTRL_MPC_SCHED_LEAD=1,SIM_ESTERR=1" run box26_lead 5 2.6
say "chain T done"
