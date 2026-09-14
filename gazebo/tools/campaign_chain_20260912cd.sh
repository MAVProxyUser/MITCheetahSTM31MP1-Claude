#!/bin/bash
# Chain CD (2026-09-14 17:35): the lie-down damping arm at the depth a 1-3 % tail needs.
# Chain CC's A/B (ISSUES OPEN-30 17:27) moved the stage-2 roll distribution - stock
# hold p90 13.5 deg vs kd 24 max 6.1 at N = 10 - but the finish tips are a tail, and
# a tail needs N ~ 50+ an arm. This holds the rig behind chain CC (pid 15912; STOP_CC
# ends its loop after the current campaign) with blocks of wkc 2.6 x6 recipe / kd24
# interleaved, hp 2.6 x6 the same, and the fast tier, scored by liedown_peak.py over
# the pooled globs wkc26_ld_cd* / hp26_ld_cd* each block, until STOP_CD (RULE ONE).
# Course verdicts stay the served-recipe record; the lie-down score is the quantity.
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
PREV="${PREV_CHAIN_PID:-15912}"
LOG="$CAMPAIGN_DIR/open30_chaincd.log"
say "chain CD pid $$: waiting for chain CC (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912cc[.]sh"; do sleep 30; done
rm -f "$CAMPAIGN_DIR/STOP_CD"
A="recipe: kd24:WP_LIEDOWN_EDAMP=24"
i=0
while [ ! -f "$CAMPAIGN_DIR/STOP_CD" ]; do
  i=$((i+1)); wait_idle; sleep 30; wait_idle; tm_wait; say "block $i: rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
  COURSES=wkc_finals ARMS="$A" run "wkc26_ld_cd$i" 6 2.6
  say "wkc lie-down, all CD blocks + CC:"; python3 gazebo/tools/liedown_peak.py "recipe=wkc26_l*:recipe" "kd24=wkc26_l*:kd24" | tee -a "$LOG"
  [ -f "$CAMPAIGN_DIR/STOP_CD" ] && break
  COURSES=hp_gap20 ARMS="$A" run "hp26_ld_cd$i" 6 2.6
  say "hairpin lie-down, all CD blocks + CC:"; python3 gazebo/tools/liedown_peak.py "recipe=hp26_l*:recipe" "kd24=hp26_l*:kd24" | tee -a "$LOG"
  [ -f "$CAMPAIGN_DIR/STOP_CD" ] && break
  tm_wait; say "block $i: fast suite tier"
  python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260914_cd$i.log" 2>&1
  say "block $i suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260914_cd$i.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260914_cd$i.log") FAIL"
  sleep 20
done
say "chain CD done (STOP_CD seen after block $i)"
