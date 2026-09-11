#!/bin/bash
# Chain K (2026-09-11 05:15): two things the closed OPEN-34 left unmeasured
# on the shipping binary, run while the operator sleeps rather than leaving
# the rig idle: (1) the 3-dog dash at 3.0 that opened the item (six reps,
# fleet cap held at 3 by a resetter, no RTF sampler - it was host load), and
# (2) the FULL suite tier (the SAR / lissajous / spirograph cases the fast
# tier omits).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
LOG="$CAMPAIGN_DIR/open31_chaink.log"
FLOG="$CAMPAIGN_DIR/fleet_dash_sched.log"
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "^python3 gazebo/conductor/mission_runner.py" >/dev/null && ! pgrep -f "^bash gazebo/tools/open28_subcourse.sh" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
wait_idle; say "chain K pid $$: rig idle (phase $(phase))"
curl -s -m 5 -X DELETE http://127.0.0.1:8420/api/fleet_cap >/dev/null
( until grep -q "fleet re-test done" "$FLOG" 2>/dev/null; do c=$(curl -s -m 3 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("fleet_cap"))' 2>/dev/null); if [ -n "$c" ] && [ "$c" != 3 ] && [ "$c" != None ]; then curl -s -m 5 -X DELETE http://127.0.0.1:8420/api/fleet_cap >/dev/null; echo "$(date +%H:%M:%S) cap was $c - reset" >> "$FLOG.capreset"; fi; sleep 2; done ) &
RESET_PID=$!
echo "3-dog dash:100 trotting 3.0 on the scheduled lead, 6 reps ($(date +%H:%M))" > "$FLOG"
for i in 1 2 3 4 5 6; do
  echo "=== fleet rep $i $(date +%H:%M) ===" >> "$FLOG"
  timeout 420 python3 gazebo/conductor/mission_runner.py --terrain flat --slot dash:100 --slot dash:100 --slot dash:100 --gait trotting --gait trotting --gait trotting --speed 3.0 --speed 3.0 --speed 3.0 --dash 0 --dash 0 --dash 0 --wait-for-gate 1800 --timeout 300 2>&1 | grep -E "mission result|FELL|VERDICT|PASS=|CAPPED|refused|ERROR" >> "$FLOG"
  sleep 5
done
echo "=== fleet re-test done $(date +%H:%M) ===" >> "$FLOG"
kill "$RESET_PID" 2>/dev/null
say "fleet re-test: $(grep -c 'VERDICT: PASS' "$FLOG") of 6 reps all-pass; per-dog PASS $(grep -c 'mission result: PASS' "$FLOG")/18"
sleep 20; wait_idle
say "running the FULL suite tier"
python3 unittests/test_validated_missions.py > "$CAMPAIGN_DIR/suite_full_20260911.log" 2>&1
say "full suite exit $?: $(grep -cE '^  PASS' "$CAMPAIGN_DIR/suite_full_20260911.log") PASS / $(grep -cE '^  FAIL' "$CAMPAIGN_DIR/suite_full_20260911.log") FAIL"
say "chain K done"
