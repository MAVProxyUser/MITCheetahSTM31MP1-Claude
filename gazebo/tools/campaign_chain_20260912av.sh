#!/bin/bash
# Chain AV (2026-09-12 15:40): the re-acceleration SLEW at 2.6. Tracing the
# wp08 exit (a 55 deg corner the planner fillets at R=5.8 m and plans at full
# cruise) in every 2.6 fall and in the 2.4 passes: the follower cuts the
# command to ~1.7 at the vertex, the body drops to ~1.4, then re-accelerates
# at the WP_VSLEW=1.0 rise and pitches up as vx passes 2.2-2.5 - peak 19-23
# deg in the 2.4 passes, 30-32 deg (E-stop) in the 2.6 falls, 19 deg in the
# one 2.6 pass. The planner's re-acceleration cap (chain AS) cannot act on a
# cut the follower makes at runtime; the slew on the command's RISE is the
# lever that does. Arms: the shipped 1.0 m/s^2 / 0.6 / 0.4, wkc_finals 2.6,
# 6 reps interleaved. Runs after chain AT (pid 7106).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-7106}"
LOG="$CAMPAIGN_DIR/open31_chainav.log"
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
say "chain AV pid $$: waiting for chain AT (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912at.sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; say "rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
COURSES=wkc_finals ARMS="ship:WP_VSLEW=1.0 vs06:WP_VSLEW=0.6 vs04:WP_VSLEW=0.4" run wkc26_vslew 6 2.6
say "chain AV done"
