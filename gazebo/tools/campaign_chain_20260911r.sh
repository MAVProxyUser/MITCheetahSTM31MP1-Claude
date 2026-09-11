#!/bin/bash
# Chain R (2026-09-11 10:45): two follow-ups on the shipped slew recipe.
#   1. hp_gap20's own envelope on it (the ladder had 2.0 as its last reliable
#      rung on the old recipe; the slew moved wkc's by a rung): 2.1 / 2.3,
#      5 reps interleaved, recipe as served.
#   2. the box at 2.6, WP_ALON 0.4 vs 0.2 at N = 8 - the 4/5 vs 2/5 above is
#      suggestive, not a result.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
LOG="$CAMPAIGN_DIR/open31_chainr.log"
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
wait_idle; sleep 30; wait_idle; say "chain R pid $$: rig idle (phase $(phase))"
COURSES=hp_gap20 ARMS="v21:SPEED=2.1 v23:SPEED=2.3" run hp_ladder_slew 5 2.1
COURSES=wkc_box ARMS="alon04:WP_ALON=0.4 alon02:WP_ALON=0.2" run box26_alon8 8 2.6
say "chain R done"
