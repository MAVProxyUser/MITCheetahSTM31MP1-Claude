#!/bin/bash
# Chain BD (2026-09-12 16:30): the second lever on the HONEST course. On the
# single lap with the vertex stop (ISSUES OPEN-38), wkc_finals 2.6 falls at the
# wp03->wp04 stretch: the -75 deg corner taken at cruise with the yaw rate on
# the lateral-budget cap (w = a_lat / v), then the 90 deg wp04 braking - the
# feature that peaks 21 deg at 2.4 and 38 deg (E-stop) at 2.6. Traced: the body
# leaves wp02 at 1.4 m/s, re-accelerates at the WP_VSLEW=1.0 rise to 2.3-2.5 in
# the 6 m leg (pitch +13 deg at 2.46) and enters wp03 still nose-up; the slew
# on the command rise is the lever that paces that. Chain AV was to measure it
# on the double lap (stopped); this is the honest measurement: 1.0 / 0.6 / 0.4,
# interleaved, wkc_finals at 2.6, 6 reps, after chain BC (pid 51525).

set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-51525}"
LOG="$CAMPAIGN_DIR/open38_chainbd.log"
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
say "chain BD pid $$: waiting for chain BC (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912bc[.]sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; tm_wait; say "rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
COURSES=wkc_finals ARMS="vs10:WP_VSLEW=1.0 vs06:WP_VSLEW=0.6 vs04:WP_VSLEW=0.4" run wkc26_singlelap_vslew 6 2.6
say "chain BD done"
