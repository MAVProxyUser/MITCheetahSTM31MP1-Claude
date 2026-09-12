#!/bin/bash
# Chain AZ (2026-09-12 16:00): the second planner defect on wkc_finals, fixed
# before the single-lap envelope is measured (ISSUES OPEN-38). The reversal
# stop was registered by coordinates and BodyPathPlanner braked every path
# pass within 2 m of them; wp03 sits one metre from the reversal vertex wp07,
# so the wp03 fillet was braked to v_min and the box section ran at 1.3-1.7
# under a 2.4-2.6 cruise in every run on record. Reversal stops now resolve to
# their own vertex (addStopAtVertex). This chain:
#   1. waits for the rig to go idle (chain AW's suite is left to finish),
#      keeps the running binary as .pre_vertexstop, deploys the new one;
#   2. probes wkc_finals 2.4 x2: single lap (wp07 before any visit home),
#      `[plan] vertex stop at wp07` in the log, no creep at wp03 (mission time
#      under the 103 s the wp03 crawl gave);
#   3. the single-lap ladders: wkc_finals 2.2/2.4/2.6 and hp_gap20 2.5/2.6/2.7,
#      6 reps each, interleaved.
# Chains AW (its ladders), AX and AY were stopped and re-queued behind it.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
LOG="$CAMPAIGN_DIR/open38_chainaz.log"
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
lap_check(){   # $1 = run id
  local rid="$1" L r7 home tm vs
  L=$(ls "$RUN_DIR/archive/"*run${rid}_ctrl_0.log 2>/dev/null | head -1)
  [ -z "$L" ] && L="$RUN_DIR/ctrl_0.log"
  r7=$(grep -n "reached wp07" "$L" | head -1 | cut -d: -f1)
  home=$(awk -F'[ =]' '/^\[nav\] wp7\/16 N=/{ if ($4+0 > -1.5 && $4+0 < 1.5 && $6+0 > -1.5 && $6+0 < 1.5) {print NR; exit} }' "$L")
  tm=$(grep -o "MISSION COMPLETE t=[0-9.]*s" "$L" | tail -1)
  vs=$(grep -o "\[plan\] vertex stop at wp[0-9]* -> path s=[0-9.]* m ([0-9.]* m from the vertex)" "$L" | head -1)
  if [ -n "$home" ] && { [ -z "$r7" ] || [ "$home" -lt "$r7" ]; }; then say "  run $rid: DOUBLE LAP STILL PRESENT (home at line $home before wp07 at line ${r7:-none}) $tm"; return 1; fi
  say "  run $rid: single lap (wp07 reached at line ${r7:-none}) $tm | ${vs:-NO vertex-stop line}"; return 0
}
say "chain AZ pid $$: waiting for the rig to go idle (chain AW's suite finishes first)"
wait_idle; sleep 30; wait_idle; tm_wait
say "rig idle (phase $(phase)) - deploying the vertex-stop planner (running binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8) kept as .pre_vertexstop)"
cp -p host-run/mit_ctrl_sim host-run/mit_ctrl_sim.pre_vertexstop
if bash gazebo/deploy_host.sh >> "$LOG" 2>&1; then say "deployed $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"; else say "DEPLOY FAILED - see $LOG"; exit 1; fi
COURSES=wkc_finals ARMS="ship:WP_ALON=0.4" run wkc24_vertexstop_probe 2 2.4
for rid in $(awk -F, 'NR>1{print $11}' "$CAMPAIGN_DIR/wkc24_vertexstop_probe.csv"); do lap_check "$rid"; done
COURSES=wkc_finals ARMS="v22:SPEED=2.2 v24:SPEED=2.4 v26:SPEED=2.6" run wkc_singlelap_ladder 6 2.4
COURSES=hp_gap20 ARMS="v25:SPEED=2.5 v26:SPEED=2.6 v27:SPEED=2.7" run hp_singlelap_ladder 6 2.6
say "chain AZ done"
