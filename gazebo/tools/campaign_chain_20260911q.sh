#!/bin/bash
# Chain Q (2026-09-11 10:20): with the reversal slewed, wkc_finals' next
# bounding feature is the box - three 90 deg corners 6-7 m apart - whose
# third corner pitches the dog over at 2.6 (0/5, pitch 30-38 deg at wp7).
# Braking into a corner is WP_ALON's territory (0.4 in the recipe, the
# atom lesson). Dose it on the wkc_box sub-course at 2.6, 5 reps each.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
LOG="$CAMPAIGN_DIR/open31_chainq.log"
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
wait_idle; sleep 30; wait_idle; say "chain Q pid $$: rig idle (phase $(phase))"
COURSES=wkc_box ARMS="alon04:WP_ALON=0.4 alon03:WP_ALON=0.3 alon02:WP_ALON=0.2" run box26_alon 5 2.6
say "chain Q done"
