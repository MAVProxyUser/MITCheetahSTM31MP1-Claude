#!/bin/bash
# Chain AW (2026-09-12 16:10): the harness before the robot, again (ISSUES
# OPEN-38). Every wkc_finals and hp_gap20 run on record was a DOUBLE LAP: the
# follower's nearest-index tie-break on the collinear reversal U-turned the dog
# 4.2-4.4 m before the vertex, the waypoint layer froze on that waypoint, the
# follower drove the rest of the course and the closing leg home, and the
# legacy pure-pursuit nav then drove a second lap from home. Every 2.6 fall
# traced at "wp9" was in that legacy lap; the planner never drove where the
# deep dive was looking. BodyPathPlanner::nearestIndex now advances at most
# 1 m of path per tick (kTrackWindow). This chain:
#   1. waits for the rig to go idle, keeps the running binary as
#      host-run/mit_ctrl_sim.pre_trackwin, deploys the new one (deploy_host.sh
#      proves it loads);
#   2. probes: wkc_finals 2.4 x2 on the shipped recipe - "reached wp07" must
#      come BEFORE any visit home, and the mission time is the single-lap
#      time (171 s was the double lap);
#   3. the fast suite tier on the new follower (it touches every mission);
#   4. re-measures the envelope on the single lap: wkc_finals 2.2/2.4/2.6 and
#      hp_gap20 2.5/2.6/2.7, 6 reps each, interleaved.
# Chains AS/AT/AV/AU (all on the double-lap mission) were stopped for it.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
LOG="$CAMPAIGN_DIR/open38_chainaw.log"
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
lap_check(){   # $1 = run id: single lap iff "reached wp07" precedes any home visit while stuck on wp7
  local rid="$1" L r7 home tm
  L=$(ls "$RUN_DIR/archive/"*run${rid}_ctrl_0.log 2>/dev/null | head -1)
  [ -z "$L" ] && L="$RUN_DIR/ctrl_0.log"
  r7=$(grep -n "reached wp07" "$L" | head -1 | cut -d: -f1)
  home=$(awk -F'[ =]' '/^\[nav\] wp7\/16 N=/{ if ($4+0 > -1.5 && $4+0 < 1.5 && $6+0 > -1.5 && $6+0 < 1.5) {print NR; exit} }' "$L")
  tm=$(grep -o "MISSION COMPLETE t=[0-9.]*s" "$L" | tail -1)
  if [ -n "$home" ] && { [ -z "$r7" ] || [ "$home" -lt "$r7" ]; }; then say "  run $rid: DOUBLE LAP STILL PRESENT (home at line $home before wp07 at line ${r7:-none}) $tm"; return 1; fi
  say "  run $rid: single lap (wp07 reached at line ${r7:-none}, no home visit while stuck) $tm"; return 0
}
say "chain AW pid $$: waiting for the rig to go idle"
wait_idle; sleep 30; wait_idle
say "rig idle (phase $(phase)) - deploying the continuity-window follower (running binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8) kept as .pre_trackwin)"
cp -p host-run/mit_ctrl_sim host-run/mit_ctrl_sim.pre_trackwin
if bash gazebo/deploy_host.sh >> "$LOG" 2>&1; then say "deployed $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"; else say "DEPLOY FAILED - see $LOG"; exit 1; fi
COURSES=wkc_finals ARMS="ship:WP_ALON=0.4" run wkc24_singlelap_probe 2 2.4
for rid in $(awk -F, 'NR>1{print $11}' "$CAMPAIGN_DIR/wkc24_singlelap_probe.csv"); do lap_check "$rid"; done
say "fast suite tier on $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260912_trackwin.log" 2>&1
say "suite exit $?: $(grep -E "SUMMARY|passed|failed" "$CAMPAIGN_DIR/suite_fast_20260912_trackwin.log" | tail -3 | tr '\n' ' ')"
sleep 20; wait_idle
COURSES=wkc_finals ARMS="v22:SPEED=2.2 v24:SPEED=2.4 v26:SPEED=2.6" run wkc_singlelap_ladder 6 2.4
COURSES=hp_gap20 ARMS="v25:SPEED=2.5 v26:SPEED=2.6 v27:SPEED=2.7" run hp_singlelap_ladder 6 2.6
say "chain AW done"
