#!/bin/bash
# Chain M (2026-09-11 06:20): chain L's first campaign (the WP_VSLEW dose at
# wkc 2.2) was refused by campaign_claim ("a mission is already running") in
# the 20 s after chain K's suite exited - a transient match on its unanchored
# mission_runner pgrep. Re-run it after chain L.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-28017}"
LOG="$CAMPAIGN_DIR/open31_chainm.log"
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
say "chain M pid $$: waiting for chain L (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null; do sleep 30; done
wait_idle; sleep 30; wait_idle
say "rig idle (phase $(phase))"
COURSES=wkc_finals ARMS="vs0:WP_VSLEW=0 vs1:WP_VSLEW=1.0 vs2:WP_VSLEW=2.0" run wkc22_vslew 5 2.2
say "chain M done"
