#!/bin/bash
# Chain BG (2026-09-12 20:45): ship the lateral budget in the course recipe and
# validate it (ISSUES OPEN-38/OPEN-28). server.py now carries WP_ALAT=2.0 in
# the course recipe (commit d503577); it lands only when the conductor
# restarts. After chain BF (pid 5658):
#   1. gate: BF's box at 2.6 under the 2.0 budget must not be worse than under
#      2.5 (the recipe is shared by every course: mission). If it is, the
#      restart is SKIPPED and the operator decides.
#   2. restart the conductor the sanctioned way (conductor_ctl.restart_server:
#      /api/stop first, port confirmed free, relaunch, wait for idle) in the
#      idle gap, and confirm /api/state serves WP_ALAT=2.0 for course:.
#   3. validate on the recipe alone (no --extra): the fast suite tier, then
#      wkc_finals 2.6 x6 and hp_gap20 2.6 x6 - the numbers the panel will get.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-5658}"
LOG="$CAMPAIGN_DIR/open38_chainbg.log"
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
say "chain BG pid $$: waiting for chain BF (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912bf[.]sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; tm_wait
B25=$(passes box26_alat_n5 alat25); B20=$(passes box26_alat_n5 alat20); H25=$(passes hp26_alat_n5 alat25); H20=$(passes hp26_alat_n5 alat20)
say "gate: box 2.6 budget 2.5 -> $B25/5, budget 2.0 -> $B20/5; hairpin 2.6 budget 2.5 -> $H25/5, budget 2.0 -> $H20/5"
if [ "${B20:-0}" -lt "${B25:-0}" ] || [ "${H20:-0}" -lt "${H25:-0}" ]; then
  say "RECIPE NOT SHIPPED: the 2.0 budget cost a shared course (box $B20 vs $B25, hairpin $H20 vs $H25) - conductor NOT restarted, server.py still carries WP_ALAT=2.0 (revert or ship by hand)"
  say "fallback: the honest shipping rungs on the LIVE recipe (budget 2.5)"
  COURSES=hp_gap20 ARMS="v26:SPEED=2.6" run hp26_live_n6 6 2.6
  say "chain BG done (recipe held back)"; exit 0
fi
say "restarting the conductor the sanctioned way to load WP_ALAT=2.0 (was 2.5) into the course recipe"
python3 -c "import sys; sys.path.insert(0,'gazebo/conductor'); import conductor_ctl; ok=conductor_ctl.restart_server('course recipe WP_ALAT 2.5 -> 2.0'); sys.exit(0 if ok else 1)" >> "$LOG" 2>&1 || { say "RESTART FAILED - see $LOG"; exit 1; }
sleep 5
EX=$(curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin)["recipes"]["course"]["extra"])' 2>/dev/null)
say "course recipe now serves: $EX"
echo "$EX" | grep -q "WP_ALAT=2.0" || { say "RECIPE MISMATCH after restart - stopping"; exit 1; }
tm_wait
say "fast suite tier on the new recipe, binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260912_alat20.log" 2>&1
say "suite exit $?: $(grep -cE '^  PASS' "$CAMPAIGN_DIR/suite_fast_20260912_alat20.log") PASS, $(grep -cE '^  FAIL' "$CAMPAIGN_DIR/suite_fast_20260912_alat20.log") FAIL"
sleep 20; wait_idle
COURSES=wkc_finals ARMS="recipe:" run wkc26_recipe_n6 6 2.6
COURSES=hp_gap20 ARMS="recipe:" run hp26_recipe_n6 6 2.6
say "chain BG done"
