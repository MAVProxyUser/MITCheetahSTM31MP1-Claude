#!/bin/bash
# Chain BW (2026-09-13 14:40): the standing block behind chain BV (pid 23925), so the
# rig never idles while the turn-cap decision is made. Each block: wkc_finals 2.6
# off vs WP_VTURN=2.4, 6 reps interleaved (the tail comparison for the ship decision
# needs N ~ 40 per arm - a 2 % tail cannot be counted at 8; every block adds 6+6 and
# the window scorer reads them all through the glob wkc26_vturn*), hp_gap20 2.6 off
# vs 2.4 x4 (the no-op check accumulates the same way), then the fast suite tier on
# the deployed binary. `touch $CAMPAIGN_DIR/STOP_BW` ends it after the current
# campaign. Strictly sequential, one dog, served recipe, binary as deployed.
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
PREV="${PREV_CHAIN_PID:-23925}"
LOG="$CAMPAIGN_DIR/open38_chainbw.log"
say "chain BW pid $$: waiting for chain BV (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912bv[.]sh"; do sleep 30; done
rm -f "$CAMPAIGN_DIR/STOP_BW"
A="off: vt24:WP_VTURN=2.4"
i=0
while [ ! -f "$CAMPAIGN_DIR/STOP_BW" ]; do
  i=$((i+1)); wait_idle; sleep 30; wait_idle; tm_wait; say "block $i: rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
  COURSES=wkc_finals ARMS="$A" run "wkc26_vturn_loop$i" 6 2.6
  say "wkc26_vturn* S-bend (11->13), all blocks so far:"; python3 gazebo/tools/window_peak.py 11 13 "off=wkc26_vturn*:off" "vt24=wkc26_vturn*:vt24" | tee -a "$LOG"
  [ -f "$CAMPAIGN_DIR/STOP_BW" ] && break
  COURSES=hp_gap20 ARMS="$A" run "hp26_vturn_loop$i" 4 2.6
  [ -f "$CAMPAIGN_DIR/STOP_BW" ] && break
  tm_wait; say "block $i: fast suite tier"
  python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260913_bw$i.log" 2>&1
  say "block $i suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260913_bw$i.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260913_bw$i.log") FAIL"
  sleep 20
done
say "chain BW done (STOP_BW seen after block $i)"
