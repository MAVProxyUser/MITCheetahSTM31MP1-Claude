#!/bin/bash
# Chain AT (2026-09-12 14:00): after chain AS (pid 74726). The MPC's pitch
# and pitch-rate weights did not move the runaway on the course at 2.6 (chain
# AR, 0/8 in the first two rounds); the lateral budget did not either. The
# next controller-side candidates for the pitch-up while re-accelerating:
#   qvx025  - CTRL_MPC_QVX=0.25   (half the velocity-tracking weight: push
#                                  less hard for speed, less nose-up moment)
#   bh027   - CTRL_BODY_H=0.27    (3 cm lower stance: shorter moment arm)
#   fmax220 - CTRL_F_MAX=220      (more per-foot force to hold the nose down)
# against the shipped values, on the full course at 2.6, 5 reps interleaved.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-74726}"
LOG="$CAMPAIGN_DIR/open31_chainat.log"
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
say "chain AT pid $$: waiting for chain AS (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912as.sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; say "rig idle (phase $(phase))"
COURSES=wkc_finals ARMS="ship:WP_ALON=0.4 qvx025:CTRL_MPC_QVX=0.25 bh027:CTRL_BODY_H=0.27 fmax220:CTRL_F_MAX=220" run wkc26_ctrl 5 2.6
say "chain AT done"
