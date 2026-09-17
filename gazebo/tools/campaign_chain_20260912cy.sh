#!/bin/bash
# Chain CY (2026-09-17): (d) vs (f) vs stock with the TRIGGER PINNED, because CX's
# design was wrong for the question CX was asking.
#
# WHY CY EXISTS, stated plainly. Chain CX put stock, advisory and cap5 at the
# SHIPPED CTRL_MAX_PLEG_Y (0.240). That was the right trigger for CW's question
# ("does the advisory matter in production?") and it is the WRONG trigger for
# CX's ("does BOUNDING the cycle buy what ABOLISHING it does?"). CX block 1
# proved it: cap5 logged ZERO trips in six runs, so the arm that the chain exists
# to measure produced no data at all. At a ~7 % per-run trip rate, six runs expect
# 0.4 trips and P(zero) is about 0.65 - an empty cap arm is the most likely single
# outcome, and three arms split the eligible runs three ways.
#
# This tree already records the rule I failed to apply: "reps x base_rate before
# launching; when the rate is too low, pin the TRIGGER frequent on both arms and
# vary only the response." The remaining question is purely about the RESPONSE to
# a trip, so the trigger should be pinned and only the response knobs varied -
# exactly as chain CU did when it measured (d) at 78 runs an arm.
#
# CTRL_MAX_PLEG_Y=0.21 ON ALL THREE ARMS. 0.21 is chosen because the margin map
# puts the natural lateral excursion at 219-224 mm, so 210 mm trips on nearly
# every run without being absurd; CU used the same value and got 5213 trips from
# 78 runs. Every arm sets it explicitly so none inherits a default.
#
# WHAT EACH ARM MUST SHOW, written before the data:
#   stock    - no [locoadv], no [lococap], cycles UNBOUNDED (CU saw 5332 for 5308)
#   advisory - [locoadv] > 0, cycles EXACTLY 0
#   cap5     - [lococap] > 0, cycles a small MULTIPLE OF 5 (5, 10, 15 ...): the
#              latch engages on the 5th transition and only re-arms after the leg
#              is clean for CTRL_LOCO_UNSAFE_CYCLE_GAP_TICKS = 250 ticks (0.5 s).
#              Cycles of 0 would mean the cap is over-suppressing and is really an
#              advisory; cycles of 80+ would mean it never engaged.
#
# NOTHING SHIPS. Both response knobs stay default-off in the tree.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
DATA="${CHEETAH_DATA:-$HOME/Desktop/Cheetah/rundata}"
export DUMP=0
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "gazebo/conductor/mission_[r]unner" >/dev/null && ! pgrep -f "gazebo/tools/open28_subcourse[.]sh" >/dev/null && ! pgrep -f "unittests/test_validated_[m]issions" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
tm_wait(){ local n=0; while tmutil status 2>/dev/null | grep -q "Running = 1"; do [ $n -eq 0 ] && say "Time Machine backup in progress - holding (not a run)"; n=$((n+1)); sleep 30; done; }
FLOOR_GB="${FLOOR_GB:-12}"
disk_ok(){ local f; f=$(df -g /System/Volumes/Data | awk 'NR==2{print $4}'); if [ "${f:-0}" -lt "$FLOOR_GB" ]; then say "DISK FLOOR: ${f} GB free is below ${FLOOR_GB} GB - CY stops here"; return 1; fi; say "disk ${f} GB free (floor ${FLOOR_GB})"; return 0; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps at $v (COURSES=${COURSES:-} ARMS=${ARMS:-} DUMP=$DUMP)"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
tier(){ local t="$1"; tm_wait; say "$t: fast suite tier"
  python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260917_cy$t.log" 2>&1
  say "$t suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260917_cy$t.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260917_cy$t.log") FAIL"
  sleep 20; wait_idle
}
PREV="${PREV_CHAIN_PID:-26083}"
PREV_PAT="${PREV_CHAIN_PAT:-campaign_chain_20260912[a-z][a-z][.]sh}"
LOG="$CAMPAIGN_DIR/open28_chaincy.log"
say "chain CY pid $$: waiting for the previous chain (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -qE "$PREV_PAT"; do sleep 30; done
say "CY has the rig (predecessor $PREV gone)"
rm -f "$CAMPAIGN_DIR/STOP_CY"
wait_idle; sleep 30; wait_idle; tm_wait

# No build: CX already deployed option (f). Verify it is still there rather than
# assuming - a chain that silently ran a relabelled stock arm would be worse than
# no chain at all.
BIN=$(md5 -q host-run/mit_ctrl_sim | cut -c1-8)
KNOB=$(strings host-run/mit_ctrl_sim | grep -c 'CTRL_LOCO_UNSAFE_CYCLE_CAP')
TAG=$(strings host-run/mit_ctrl_sim | grep -c 'lococap')
ADV=$(strings host-run/mit_ctrl_sim | grep -c 'CTRL_LOCO_UNSAFE_ADVISORY_VMAX')
say "no build needed - binary $BIN; knob check: CYCLE_CAP x$KNOB, [lococap] x$TAG, ADVISORY_VMAX x$ADV"
if [ "$KNOB" -lt 1 ] || [ "$TAG" -lt 1 ] || [ "$ADV" -lt 1 ]; then
  say "ABORTING CY: a response knob is missing from the deployed binary, so an arm would be a relabelled stock arm."
  exit 1
fi

A="stock:CTRL_MAX_PLEG_Y=0.21,CTRL_LOCO_UNSAFE_ADVISORY_VMAX=-1,CTRL_LOCO_UNSAFE_CYCLE_CAP=0 advisory:CTRL_MAX_PLEG_Y=0.21,CTRL_LOCO_UNSAFE_ADVISORY_VMAX=1.0,CTRL_LOCO_UNSAFE_CYCLE_CAP=0 cap5:CTRL_MAX_PLEG_Y=0.21,CTRL_LOCO_UNSAFE_ADVISORY_VMAX=-1,CTRL_LOCO_UNSAFE_CYCLE_CAP=5"
say "arms (interleaved run by run; trigger PINNED at 0.21 on all three so every run trips): $A"
say "SCOPE: 0.21 is an INDUCED trigger. This chain measures the RESPONSE, not the envelope -"
say "  the production-rate question is already answered by CW at the shipped 0.240 (p = 0.0097)."
say "expected per arm: stock cycles UNBOUNDED | advisory cycles EXACTLY 0 | cap5 cycles a MULTIPLE OF 5"
tier 0
i=0
while [ ! -f "$CAMPAIGN_DIR/STOP_CY" ]; do
  disk_ok || break
  i=$((i+1)); wait_idle; sleep 30; wait_idle; tm_wait; say "block $i: rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
  COURSES="wkc_weave" ARMS="$A" run "captrip_cy$i" 3 2.6
  say "OPEN-40 (d) vs (f) vs stock with the trigger pinned. READ THE CAP ARM'S CYCLE"
  say "  DISTRIBUTION FIRST - it is the manipulation check and it has three outcomes:"
  python3 gazebo/tools/open40_conditioned.py 'captrip_cy*.csv' 2>&1 | tee -a "$LOG"
  tier "$i"
done
say "chain CY done (STOP_CY or disk floor, after block $i)"
