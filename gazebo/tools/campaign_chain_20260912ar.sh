#!/bin/bash
# Chain AR (2026-09-12 11:25): the controller's pitch authority at speed,
# after chain AQ (pid 59992). The deep dive says every 2.6 course fall and
# 2.5/2.6 hairpin fall is a 0.8 s pitch-up runaway at 2.4-2.8 m/s under a
# re-acceleration or a turn at cruise (roll and yaw small). The MPC weights
# on pitch and pitch rate ship at 0.5 and 0.1 (MIT's). Four arms, recipe as
# served (trot lead 1), interleaved:
#   ship  - as shipped
#   qp2   - CTRL_MPC_QP=1.0   (2x pitch weight)
#   qwy4  - CTRL_MPC_QWY=0.4  (4x pitch-rate weight)
#   both  - the two together
# on the full course at 2.6 (0/15 today: room to rescue) and the hairpin at
# 2.6 (4/5: room to move either way), 5 reps each.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-59992}"
LOG="$CAMPAIGN_DIR/open31_chainar.log"
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
say "chain AR pid $$: waiting for chain AQ (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912aq.sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; say "rig idle (phase $(phase))"
A="ship:CTRL_MPC_QP=0.5 qp2:CTRL_MPC_QP=1.0 qwy4:CTRL_MPC_QWY=0.4 both:CTRL_MPC_QP=1.0,CTRL_MPC_QWY=0.4"
COURSES=wkc_finals ARMS="$A" run wkc26_pitchw 5 2.6
COURSES=hp_gap20 ARMS="$A" run hp26_pitchw 5 2.6
say "chain AR done"
