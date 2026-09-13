#!/bin/bash
# Chain CA (2026-09-13 19:40): the controller levers on the 2.8 feature, measured on
# the HONEST course. With the turn cap shipped, wkc_finals at 2.8 pools to 13/18 and
# every miss is the same instant: the E-stop at pitch 30-33 deg as the command reaches
# 2.8 on the wp14->wp15 straight with the body at 3.0 (ISSUES OPEN-38). The slew is
# null there (1.0/0.7/0.5: 3/6, 4/6, 4/6, peak 27 deg in every arm), so the feature
# is the trot's own pitch at that body speed after a stop - a controller property.
# Chain AT (2026-09-12 14:00) tried three controller levers at 2.6 and was measured on
# the DOUBLE LAP, so its verdicts do not stand. Arms on the shipped recipe at 2.8, 6
# reps interleaved, scored on the wp13->wp15 peak, the S-bend and the lap:
#   recipe   - the served recipe as is
#   qvx025   - CTRL_MPC_QVX=0.25  (half the velocity-tracking weight: less push)
#   bh027    - CTRL_BODY_H=0.27   (3 cm lower stance: shorter moment arm)
#   fmax220  - CTRL_F_MAX=220     (more per-foot force to hold the nose)
# Then a standing loop of the served recipe at 2.6 until STOP_CA (RULE ONE). Runs after
# chain BZ (pid 16387; `touch $CAMPAIGN_DIR/STOP_BZ` ends it after its current campaign).
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
PREV="${PREV_CHAIN_PID:-16387}"
LOG="$CAMPAIGN_DIR/open38_chainca.log"
say "chain CA pid $$: waiting for chain BZ (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912bz[.]sh"; do sleep 30; done
rm -f "$CAMPAIGN_DIR/STOP_CA"
wait_idle; sleep 30; wait_idle; tm_wait
EX=$(curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin)["recipes"]["course"]["extra"])' 2>/dev/null)
say "rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8); course recipe serves: $EX"
A="recipe: qvx025:CTRL_MPC_QVX=0.25 bh027:CTRL_BODY_H=0.27 fmax220:CTRL_F_MAX=220"
COURSES=wkc_finals ARMS="$A" run wkc28_ctrl 6 2.8
for W in "13 15" "11 13"; do say "wkc28_ctrl window $W:"; python3 gazebo/tools/window_peak.py $W "recipe=wkc28_ctrl:recipe" "qvx025=wkc28_ctrl:qvx025" "bh027=wkc28_ctrl:bh027" "fmax220=wkc28_ctrl:fmax220" | tee -a "$LOG"; done
i=0
while [ ! -f "$CAMPAIGN_DIR/STOP_CA" ]; do
  i=$((i+1)); wait_idle; sleep 30; wait_idle; tm_wait; say "block $i: rig idle (phase $(phase))"
  COURSES=wkc_finals ARMS="recipe:" run "wkc26_recipe_ca$i" 6 2.6
  [ -f "$CAMPAIGN_DIR/STOP_CA" ] && break
  COURSES=hp_gap20 ARMS="recipe:" run "hp26_recipe_ca$i" 6 2.6
  [ -f "$CAMPAIGN_DIR/STOP_CA" ] && break
  tm_wait; say "block $i: fast suite tier"
  python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260913_ca$i.log" 2>&1
  say "block $i suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260913_ca$i.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260913_ca$i.log") FAIL"
  sleep 20
done
say "chain CA done (STOP_CA seen after block $i)"
