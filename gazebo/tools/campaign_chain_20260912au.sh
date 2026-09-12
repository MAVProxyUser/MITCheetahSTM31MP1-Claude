#!/bin/bash
# Chain AU (2026-09-12 15:20): the rung between. wkc_finals is 5/5 and
# 10/10-class at 2.4 on the shipped recipe and 0/15 at 2.6 on every lever
# tried (chains AQ/AR/AS/AT); 2.5 has never been run on the full course. A
# lever swept past the cliff looks inert whatever it does (CLAUDE.md, "tune
# at the MARGIN, not past the cliff"), so this chain finds the margin and
# puts the planner's re-acceleration cap on it in the same interleaved block:
#   ship24  - the shipped rung, the block's own base rate (stream control)
#   ship25  - the unknown rung, shipped recipe
#   cap25   - 2.5 with WP_REACCEL_VMAX=2.3 (hold the profile <= 2.3 for 12 m
#             after every braking minimum, then release to cruise)
# 8 reps, interleaved, after chain AT (pid 7106).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-7106}"
LOG="$CAMPAIGN_DIR/open31_chainau.log"
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
say "chain AU pid $$: waiting for chain AT (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912at.sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; say "rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
COURSES=wkc_finals ARMS="ship24:SPEED=2.4 ship25:SPEED=2.5 cap25:SPEED=2.5,WP_REACCEL_VMAX=2.3" run wkc25_margin 8 2.5
say "chain AU done"
