#!/bin/bash
# Chain BU (2026-09-13 13:10): the shallow-corner rule (BodyLimits::v_turn_cap,
# WP_VTURN / WP_VTURN_DEG, off by default). The wp11->wp13 S-bend costs 20 deg at
# 2.6 in every run and holds the shipped rung's 2 % tail; no budget or speed cap
# moves it because a 40-50 deg fillet is 8-18 m and never brakes. This caps every
# corner of >= 30 deg at the cap speed, so the plan brakes for them like it does for
# the sharp ones. Deployed in the idle gap after chain BT (pid 93408; running binary
# kept as .pre_vturn), then A/B at 2.6 on the served recipe: off / 2.4 / 2.2, 8 reps
# interleaved, scored on the S-bend peak (window 11->13), the wp03 corner (2->4) and
# the lap time; then the fast tier.
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
PREV="${PREV_CHAIN_PID:-93408}"
LOG="$CAMPAIGN_DIR/open38_chainbu.log"
say "chain BU pid $$: waiting for chain BT (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912bt[.]sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; tm_wait
say "rig idle (phase $(phase)) - deploying the turn-cap planner (running binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8) kept as .pre_vturn)"
cp -p host-run/mit_ctrl_sim host-run/mit_ctrl_sim.pre_vturn
if bash gazebo/deploy_host.sh >> "$LOG" 2>&1; then say "deployed $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"; else say "DEPLOY FAILED - see $LOG"; exit 1; fi
COURSES=wkc_finals ARMS="off: vt24:WP_VTURN=2.4 vt22:WP_VTURN=2.2" run wkc26_vturn 8 2.6
say "S-bend (11->13):"; python3 gazebo/tools/window_peak.py 11 13 "off=wkc26_vturn:off" "vt24=wkc26_vturn:vt24" "vt22=wkc26_vturn:vt22" | tee -a "$LOG"
say "wp03 corner (2->4):"; python3 gazebo/tools/window_peak.py 2 4 "off=wkc26_vturn:off" "vt24=wkc26_vturn:vt24" "vt22=wkc26_vturn:vt22" | tee -a "$LOG"
sleep 20; wait_idle; tm_wait; say "fast suite tier on $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260913_vturn.log" 2>&1
say "suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260913_vturn.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260913_vturn.log") FAIL"
say "chain BU done"
