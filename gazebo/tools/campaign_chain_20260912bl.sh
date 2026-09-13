#!/bin/bash
# Chain BL (2026-09-12 23:10): the 2.8 rule, overnight. Under budgets 1.5/1.2 the
# 2.8 falls on wkc_finals moved to the closing corner - a PLANNED near-stop whose
# exit re-accelerates to 2.8 (ISSUES OPEN-38/28, chain BH). That is the one place
# the planner re-acceleration cap (WP_REACCEL_VMAX, chain AS, off by default)
# actually acts, so: budget 1.5 alone vs 1.5 + cap 2.4 vs 1.5 + cap 2.2, wkc_finals
# at 2.8, 6 reps interleaved. After chain BK (pid 76678).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "gazebo/conductor/mission_[r]unner" >/dev/null && ! pgrep -f "gazebo/tools/open28_subcourse[.]sh" >/dev/null && ! pgrep -f "unittests/test_validated_[m]issions" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
tm_wait(){ local n=0; while tmutil status 2>/dev/null | grep -q "Running = 1"; do [ $n -eq 0 ] && say "Time Machine backup in progress - holding (not a run)"; n=$((n+1)); sleep 30; done; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps at $v (COURSES=${COURSES:-} ARMS=${ARMS:-})"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
PREV="${PREV_CHAIN_PID:-76678}"
LOG="$CAMPAIGN_DIR/open38_chainbl.log"
say "chain BL pid $$: waiting for chain BK (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912bk[.]sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; tm_wait; say "rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
COURSES=wkc_finals ARMS="a15:WP_ALAT=1.5 a15cap24:WP_ALAT=1.5,WP_REACCEL_VMAX=2.4 a15cap22:WP_ALAT=1.5,WP_REACCEL_VMAX=2.2" run wkc28_alat15_cap 6 2.8
say "chain BL done"
