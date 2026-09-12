#!/bin/bash
# Chain AS (2026-09-12 12:10): the planner-side rule, measured. The trot's
# falls above its course envelope are a pitch runaway while the profile
# re-accelerates to cruise after a braking feature (OPEN-28 deep dive). The
# planner now has WP_REACCEL_VMAX (hold the profile at or under it for
# WP_REACCEL_DIST m after every braking minimum; off by default). Arms,
# interleaved, on the full course at 2.6 (0/15 today) and the hairpin at 2.6
# (4/5): off / cap 2.3 / cap 2.2, 5 reps. Runs after chain AR (pid 65786).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-65786}"
LOG="$CAMPAIGN_DIR/open31_chainas.log"
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
say "chain AS pid $$: waiting for chain AR (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912ar.sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; say "rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
A="off:WP_ALON=0.4 cap23:WP_REACCEL_VMAX=2.3 cap22:WP_REACCEL_VMAX=2.2"
COURSES=wkc_finals ARMS="$A" run wkc26_reaccel 5 2.6
COURSES=hp_gap20 ARMS="$A" run hp26_reaccel 5 2.6
say "chain AS done"
