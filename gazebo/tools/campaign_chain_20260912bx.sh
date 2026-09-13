#!/bin/bash
# Chain BX (2026-09-13): SHIP the shallow-corner cap in the course recipe (server.py
# course extra gains WP_VTURN=2.4, ISSUES OPEN-38). Runs after chain BW (pid 24919)
# exits - BW is the standing loop; `touch $CAMPAIGN_DIR/STOP_BW` ends it after its
# current campaign, and that is the idle gap this chain restarts the conductor in.
# Evidence on the served recipe, binary 5ed4a3ae: wkc 2.6 off 15/15 clean vs 2.4
# 16/16 (chains BU+BV, interleaved), S-bend peak pitch 20.0 -> 8.0 deg at N=16 an
# arm with no overlap, straights and the wp03 corner unchanged, +0.7 s of a 96 s
# lap; hairpin and box at 2.6 measured unaffected (BV). Gate: the hairpin and box
# arms must not have cost a pass (one harness fall tolerated), else NOT shipped and
# the recipe edit must be reverted by hand. Then: conductor restarted the sanctioned
# way, the served extra checked, both suite tiers, wkc 2.6 and hp 2.6 recipe blocks,
# then a standing loop of the same until STOP_BX (RULE ONE).
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
passes(){ awk -F, -v A="$2" 'NR>1 && $2==A && $4=="PASS"' "$CAMPAIGN_DIR/$1.csv" | wc -l | tr -d ' '; }
suite(){ local tag="$1"; shift; tm_wait; say "suite $tag on the new recipe, binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
  python3 unittests/test_validated_missions.py "$@" > "$CAMPAIGN_DIR/suite_${tag}.log" 2>&1
  say "suite $tag exit $?: $(grep -cE '^  PASS' "$CAMPAIGN_DIR/suite_${tag}.log") PASS, $(grep -cE '^  FAIL' "$CAMPAIGN_DIR/suite_${tag}.log") FAIL"
  sleep 20; wait_idle
}
standing_loop(){ local i=0
  while [ ! -f "$CAMPAIGN_DIR/STOP_BX" ]; do
    i=$((i+1)); wait_idle; sleep 30; wait_idle; tm_wait; say "block $i: rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
    COURSES=wkc_finals ARMS="recipe:" run "wkc26_recipe_bx$i" 6 2.6
    [ -f "$CAMPAIGN_DIR/STOP_BX" ] && break
    COURSES=hp_gap20 ARMS="recipe:" run "hp26_recipe_bx$i" 6 2.6
    [ -f "$CAMPAIGN_DIR/STOP_BX" ] && break
    suite "fast_20260913_bx$i" --fast
  done
  say "chain BX done (STOP_BX seen after block $i)"
}
PREV="${PREV_CHAIN_PID:-24919}"
LOG="$CAMPAIGN_DIR/open38_chainbx.log"
say "chain BX pid $$: waiting for chain BW (pid $PREV) to exit (touch STOP_BW to end it after its current campaign)"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912bw[.]sh"; do sleep 30; done
rm -f "$CAMPAIGN_DIR/STOP_BX"
wait_idle; sleep 30; wait_idle; tm_wait
HO=$(passes hp26_vturn off); HV=$(passes hp26_vturn vt24); BO=$(passes box26_vturn off); BVV=$(passes box26_vturn vt24)
say "gate: hairpin 2.6 off $HO/6 vs 2.4 $HV/6; box 2.6 off $BO/6 vs 2.4 $BVV/6 (one fall tolerated)"
if [ $((HV+1)) -lt "${HO:-0}" ] || [ $((BVV+1)) -lt "${BO:-0}" ]; then
  say "RECIPE NOT SHIPPED: the cap cost a shared course - conductor NOT restarted; server.py carries WP_VTURN=2.4 and must be reverted by hand"
  standing_loop; exit 0
fi
say "restarting the conductor the sanctioned way to load WP_VTURN=2.4 into the course recipe"
python3 -c "import sys; sys.path.insert(0,'gazebo/conductor'); import conductor_ctl; ok=conductor_ctl.restart_server('course recipe + WP_VTURN=2.4'); sys.exit(0 if ok else 1)" >> "$LOG" 2>&1 || say "restart_server returned non-zero - checking the served recipe anyway"
sleep 5
EX=$(curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin)["recipes"]["course"]["extra"])' 2>/dev/null)
say "course recipe now serves: $EX"
if ! echo "$EX" | grep -q "WP_VTURN=2.4"; then say "RECIPE MISMATCH after restart - holding the rig on the served recipe instead"; standing_loop; exit 1; fi
suite "both_20260913_vturn_ship"   # no flag = BOTH tiers (fast + full, ~40 min)
COURSES=wkc_finals ARMS="recipe:" run wkc26_recipe_vturn 6 2.6
COURSES=hp_gap20 ARMS="recipe:" run hp26_recipe_vturn 6 2.6
say "wkc26 S-bend on the shipped recipe vs the pooled A/B:"; python3 gazebo/tools/window_peak.py 11 13 "recipe=wkc26_recipe_vturn:recipe" "off=wkc26_vturn*:off" "vt24=wkc26_vturn*:vt24" | tee -a "$LOG"
standing_loop
