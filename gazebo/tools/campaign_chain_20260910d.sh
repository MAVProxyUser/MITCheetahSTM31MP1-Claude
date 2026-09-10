#!/bin/bash
# Chain D (2026-09-10 14:40): the OPEN-32 done-criteria named three courses
# for the contact-gate A/B. hp_gap20 is measured (null to three decimals),
# the dash arm was aborted because the sprint itself is marginal (OPEN-34,
# so it carries no gate information), and the wkc_finals arm was dropped
# when chain A was stopped. This runs it, after chain C.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-50373}"
LOG="$CAMPAIGN_DIR/open31_chaind.log"
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
say "chain D pid $$: waiting for chain C (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null; do sleep 30; done
wait_idle; sleep 20; wait_idle
say "rig idle (phase $(phase))"
COURSES=wkc_finals ARMS="cg0:SIM_CONTACT_GATE=0,SIM_ESTERR=1 cg1:SIM_CONTACT_GATE=1,SIM_ESTERR=1" run item5_contactgate_wkc 6 1.9
say "chain D done"
