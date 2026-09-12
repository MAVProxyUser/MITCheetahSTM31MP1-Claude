#!/bin/bash
# Chain AQ (2026-09-12 11:00): the mission's corner budget as the next lever,
# after chain AP (pid 26735). On the shipping configuration the full course
# dies at 2.6 on the first 75-degree entry off the opening straight and on the
# reversal approach (0/15 across the longitudinal budgets), and the hairpin's
# 2.5 loses one-in-ten at its first corner at 30 deg of pitch. Both are
# corner ENTRIES at cruise: try the lateral budget WP_ALAT (recipe 2.5) at
# 2.0 and 1.5, which lowers the speed the mission carries into a turn.
#   wkc_finals at 2.6: ALAT 2.5 / 2.0 / 1.5, 5 reps interleaved
#   hp_gap20 at 2.5:   ALAT 2.5 / 2.0 / 1.5, 5 reps interleaved
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-26735}"
LOG="$CAMPAIGN_DIR/open31_chainaq.log"
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
say "chain AQ pid $$: waiting for chain AP (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912ap.sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; say "rig idle (phase $(phase))"
COURSES=wkc_finals ARMS="alat25:WP_ALAT=2.5 alat20:WP_ALAT=2.0 alat15:WP_ALAT=1.5" run wkc26_alat 5 2.6
COURSES=hp_gap20 ARMS="alat25:WP_ALAT=2.5 alat20:WP_ALAT=2.0 alat15:WP_ALAT=1.5" run hp25_alat 5 2.5
say "chain AQ done"
