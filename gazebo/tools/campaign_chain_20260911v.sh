#!/bin/bash
# Chain V (2026-09-11 12:30): the early lead switch as a shipping candidate.
# Chain T: on the 2.6 box the shipped schedule (lead 2 on the legs) is 2/5,
# lead 1 pinned 5/5, lead 1 adopted mid-ramp at 2.5 in between (3/5). The
# 09-10 F2 sweep found lead 1 everywhere costs wkc_finals at 2.2. Candidate:
# switch at 2.0 (hysteresis 1.8) - hp_gap20 1.9 stays on lead 2, wkc's 2.2
# legs and the box's 2.6 legs run lead 1 from the bottom of the ramp, corners
# and the reversal stay on lead 2. Measured where it matters, interleaved:
#   1. wkc_finals full course at 2.2 and 2.4: shipped / early switch / pinned 1
#   2. the box at 2.6: shipped / early switch
# Runs after chain U (pid 19614, the fast suite) releases the rig.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-19614}"
LOG="$CAMPAIGN_DIR/open31_chainv.log"
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "^python3 gazebo/conductor/mission_runner.py" >/dev/null && ! pgrep -f "^bash gazebo/tools/open28_subcourse.sh" >/dev/null && ! pgrep -f "test_validated_missions.py" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps at $v (COURSES=${COURSES:-} ARMS=${ARMS:-})"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
say "chain V pid $$: waiting for chain U (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260911u.sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; say "rig idle (phase $(phase))"
E="CTRL_MPC_LEAD_V_HI=2.0,CTRL_MPC_LEAD_V_LO=1.8"
COURSES=wkc_finals ARMS="ship22:SPEED=2.2 early22:SPEED=2.2,$E pin22:SPEED=2.2,CTRL_MPC_SCHED_LEAD=1 ship24:SPEED=2.4 early24:SPEED=2.4,$E pin24:SPEED=2.4,CTRL_MPC_SCHED_LEAD=1" run wkc_lead_early 5 2.2
COURSES=wkc_box ARMS="ship:WP_ALON=0.4 early:WP_ALON=0.4,$E" run box26_lead_early 5 2.6
say "chain V done"
