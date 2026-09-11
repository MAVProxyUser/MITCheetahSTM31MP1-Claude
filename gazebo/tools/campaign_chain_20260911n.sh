#!/bin/bash
# Chain N (2026-09-11 08:10): WP_VSLEW=1.0 took 11 deg off the reversal-exit
# pitch at wkc 2.2 (5/5 vs 4/5, max 17.4 vs 32.3). Does the same margin move
# the rung the ladder called dead (2.4: 0/8), and does it restore the
# concrete hairpin's 2.1 (3/5 with the falls at the first corner)?
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
LOG="$CAMPAIGN_DIR/open31_chainn.log"
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
wait_idle; sleep 30; wait_idle; say "chain N pid $$: rig idle (phase $(phase))"
COURSES=wkc_finals ARMS="vs0:WP_VSLEW=0 vs1:WP_VSLEW=1.0" run wkc24_vslew 5 2.4
COURSES=hp_gap20 ARMS="c21:TERRAIN=concrete,SPEED=2.1,WP_VSLEW=0 c21vs1:TERRAIN=concrete,SPEED=2.1,WP_VSLEW=1.0" run hp_concrete21_vslew 5 2.1
say "chain N done"
