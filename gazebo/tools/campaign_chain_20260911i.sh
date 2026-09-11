#!/bin/bash
# Chain I (2026-09-11 03:30): chain G is showing the OLD alias path (knob 1 +
# CTRL_MPC_TABLE_ALIAS=1) beating the fixed knob 2 at wkc 2.2 inside one hour,
# which the "alias = one extra segment, bit-for-bit knob 2" accounting says
# cannot happen. Measure the two tables at the solver's input instead of
# arguing: STM32MP1_MPC_IN=2 prints every solve's step-0/step-1 table against
# the gait's own segment ([MPCIN2]); open28_mpcin.py reads the lead off it.
# Two runs each, after chain H.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-69763}"
LOG="$CAMPAIGN_DIR/open31_chaini.log"
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "^python3 gazebo/conductor/mission_runner.py" >/dev/null && ! pgrep -f "^bash gazebo/tools/open28_subcourse.sh" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps at $v (COURSES=${COURSES:-} ARMS=${ARMS:-})"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
say "chain I pid $$: waiting for chain H (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null; do sleep 30; done
wait_idle; sleep 20; wait_idle
say "rig idle (phase $(phase))"
COURSES=wkc_finals ARMS="ship:CTRL_MPC_SCHED_LEAD=1,CTRL_MPC_TABLE_ALIAS=1,STM32MP1_MPC_IN=2 fixed2:CTRL_MPC_SCHED_LEAD=2,STM32MP1_MPC_IN=2" run wkc22_leadprobe 2 2.2
say "chain I done"
