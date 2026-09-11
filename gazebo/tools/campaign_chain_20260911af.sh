#!/bin/bash
# Chain AF (2026-09-11 19:50): ship lead 1 FOR TROTTING ONLY and prove it on the full tier.
# The 17:00 full tier's 17/20 on the pin was the sim's sensor stream (OPEN-35's
# third class), not the lead: with the stream clean, expsquare (walking) is
# 3/3 on both leads, spiro 3/3 on both, and chains T/V/W's trotting gains stand
# (box 2.6 5/5 vs 10/22, wkc 2.4 5/5 vs 2/5). This chain uncomments the pin in
# the tracked yaml, runs the FULL suite, and prints every run's worst IMU gap
# beside the verdict so a contaminated case can be told from a real one.
# Runs after chain AE (pid 43693).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-43693}"
LOG="$CAMPAIGN_DIR/open31_chainaf.log"
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "^python3 gazebo/conductor/mission_runner.py" >/dev/null && ! pgrep -f "^bash gazebo/tools/open28_subcourse.sh" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
tm_wait(){ local n=0; while tmutil status 2>/dev/null | grep -q "Running = 1"; do [ $n -eq 0 ] && say "Time Machine backup in progress - holding (not a run)"; n=$((n+1)); sleep 30; done; }
say "chain AF pid $$: waiting for chain AE (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260911ae.sh"; do sleep 30; done
wait_idle; sleep 20; wait_idle; tm_wait
# TROT-ONLY lead 1, per gait: the schedule's slow-band knob for trotting
# becomes 1 (CTRL_MPC_LEAD_SLOW), so gait 9 runs lead 1 in both bands while
# walking and every other gait keep the table lead they were validated at
# (knob 2). No pin, no code change - the schedule path already exists.
Y=host-run/ctrl_tuning.yaml
if ! grep -q '^CTRL_MPC_LEAD_SLOW: 1' "$Y"; then
  sed -i.bak 's/^# CTRL_MPC_LEAD_SLOW: 2      # knob below it/CTRL_MPC_LEAD_SLOW: 1        # SHIPPED 2026-09-11 (chain AF): trotting on lead 1 in BOTH bands - the trot gains of chains T\/V\/W (box 2.6 5\/5 vs 10\/22, wkc 2.4 5\/5 vs 2\/5) without touching walking (OPEN-37). Was: 2 (knob below the sprint band)/' "$Y" && rm -f "$Y.bak"
fi
say "yaml trot lead line: $(grep -m1 '^CTRL_MPC_LEAD_SLOW' "$Y" | cut -c1-40)"
say "rig idle (phase $(phase)) - running the FULL suite on $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
python3 unittests/test_validated_missions.py > "$CAMPAIGN_DIR/suite_full_20260911c.log" 2>&1
say "suite exit $?: $(grep -E "SUMMARY|passed|failed" "$CAMPAIGN_DIR/suite_full_20260911c.log" | tail -3 | tr '\n' ' ')"
say "worst IMU gap per suite run (over 15 ms):"
bash gazebo/tools/stream_gap_report.sh "$(date +%Y%m%d)_$(date +%H | awk '{printf "%02d", $1-1}')" 15 2>/dev/null | tail -12 | sed 's/^/    /' | tee -a "$LOG"
bash gazebo/tools/stream_gap_report.sh "$(date +%Y%m%d_%H)" 15 2>/dev/null | tail -12 | sed 's/^/    /' | tee -a "$LOG"
say "chain AF done"
