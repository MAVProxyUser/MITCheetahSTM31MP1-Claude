#!/bin/bash
# Chain F (2026-09-10 22:45): chain B's sprint bisect found the contact-table
# lead: knob 1 (physical lead 2) passes the solo 100 m dash at 3.0 5/5 with
# clean attitude while the default knob 2 (physical 3) passes 2/5, knob 3
# 0/5, and the transport arm sits at the base rate. The default was chosen on
# hp_gap20 at 1.9, where the record says the SHORTER physical leads collapsed
# more (measured on the old UDP transport). Before the default moves, two
# questions: (1) does knob 1 cost anything on hp_gap20 at 1.9 on the unix
# transport, and (2) does it move the wkc_finals speed limit, whose 2.2 runs
# died 7/8 on the reversal's pitch trip this morning.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-51307}"
LOG="$CAMPAIGN_DIR/open31_chainf.log"
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
# idle = no runner, no harness (by EXACT basename, never an argv pattern: a
# wrapper shell carrying a script in its argv cost seven idle hours today).
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -x -f "python3 gazebo/conductor/mission_runner.py.*" >/dev/null 2>&1 && ! pgrep -f "^bash gazebo/tools/open28_subcourse.sh" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps at $v (COURSES=${COURSES:-} ARMS=${ARMS:-})"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
say "chain F pid $$: waiting for chain E (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null; do sleep 30; done
wait_idle; sleep 20; wait_idle
say "rig idle (phase $(phase))"
COURSES=hp_gap20 ARMS="lead2:CTRL_MPC_SCHED_LEAD=2 lead1:CTRL_MPC_SCHED_LEAD=1" run hp19_lead 8 1.9
COURSES=wkc_finals ARMS="lead2:CTRL_MPC_SCHED_LEAD=2 lead1:CTRL_MPC_SCHED_LEAD=1" run wkc22_lead 6 2.2
say "chain F done"
