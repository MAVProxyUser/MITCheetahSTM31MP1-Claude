#!/bin/bash
# Chain W (2026-09-11 13:05): can lead 1 simply be the default? Chain T: lead 1
# pinned carried the 2.6 box 5/5; chain V (in flight): pinned 1 is holding
# wkc_finals at 2.2 and 2.4 at least as well as the shipped schedule, and the
# early switch has failed the lie-down judge twice after switching back to
# lead 2 during the arrival. The one cell where the 09-10 F2 sweep charged
# lead 1 a cost was the hairpin apex - measured then on the pre-slew recipe.
# So: hp_gap20 at 1.9 and 2.3, shipped schedule (lead 2 there) vs lead 1
# pinned, 5 reps interleaved. If lead 1 holds the hairpin, it ships as the
# default and the schedule retires. Runs after chain V (pid 31476).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-31476}"
LOG="$CAMPAIGN_DIR/open31_chainw.log"
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
say "chain W pid $$: waiting for chain V (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260911v.sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; say "rig idle (phase $(phase))"
COURSES=hp_gap20 ARMS="ship19:SPEED=1.9 pin19:SPEED=1.9,CTRL_MPC_SCHED_LEAD=1 ship23:SPEED=2.3 pin23:SPEED=2.3,CTRL_MPC_SCHED_LEAD=1" run hp_lead_pin 5 1.9
say "chain W done"
