#!/bin/bash
# Chain CZ (2026-09-17): the same three arms on a DIFFERENT COURSE, because the
# cap's evidence base is one course and one trigger.
#
# THE COVERAGE GAP, written out:
#   advisory  wkc_finals  0.21 induced   CU  78/78
#   advisory  wkc_weave   0.240 shipped  CW   6/6
#   advisory  wkc_weave   0.21 induced   CY  14/14
#   cap5      wkc_weave   0.21 induced   CY  13/14
#   cap5      wkc_finals  -              NOT TESTED
# So option (d) is validated on two courses and two triggers, while option (f) -
# the one I am recommending as the fallback for a reviewer who will not accept a
# suppressed guard - rests entirely on one course at one induced trigger. That is
# too thin a base for a recommendation, and this chain fixes it.
#
# IT ALSO TESTS A PREDICTION, which is the better reason to run it. Today's fall
# taxonomy split the bleed route three ways by which axis crossed 28.65 deg, and
# the roll and pitch groups did not overlap: roll deaths came at 47-64 cycles and
# 111 mm of bleed, pitch deaths at 68-198 cycles and 232 mm (Fisher p = 8.6e-05).
# I recorded that as a PATTERN and not a mechanism, because a roll excursion is
# exactly what `wkc_weave` induces and course geometry is an obvious confound.
# The prediction that follows: on `wkc_finals`, whose geometry is not a weave, the
# ROLL group should shrink or vanish and the pitch group should dominate. If roll
# deaths persist at the same share, it is a robot mode rather than a course
# artefact - and that would be the more interesting answer.
#
# CAVEAT TO CHECK IN THE DATA, not to assume away: OPEN-38 records that every
# `wkc_finals` run on record was a DOUBLE LAP. CU's numbers came off this course
# and stand, but the mission shape must be read before the verdicts - compare
# mission_t_s against the nominal lap and watch the waypoint count.
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
disk_ok(){ local f; f=$(df -g /System/Volumes/Data | awk 'NR==2{print $4}'); if [ "${f:-0}" -lt "$FLOOR_GB" ]; then say "DISK FLOOR: ${f} GB free is below ${FLOOR_GB} GB - CZ stops here"; return 1; fi; say "disk ${f} GB free (floor ${FLOOR_GB})"; return 0; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps at $v (COURSES=${COURSES:-} ARMS=${ARMS:-} DUMP=$DUMP)"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
tier(){ local t="$1"; tm_wait; say "$t: fast suite tier"
  python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260917_cz$t.log" 2>&1
  say "$t suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260917_cz$t.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260917_cz$t.log") FAIL"
  sleep 20; wait_idle
}
# The inherited default here was a DEAD pid (CX's 26083), which would make the
# wait below a no-op and launch CZ straight into a live rig. Refuse instead:
# this chain is always launched with PREV_CHAIN_PID set explicitly.
PREV="${PREV_CHAIN_PID:-}"
if [ -z "$PREV" ]; then echo "CZ: refusing to start - PREV_CHAIN_PID is unset, and a wrong default would launch into a live rig" >&2; exit 2; fi
PREV_PAT="${PREV_CHAIN_PAT:-campaign_chain_20260912[a-z][a-z][.]sh}"
LOG="$CAMPAIGN_DIR/open28_chaincz.log"
say "chain CZ pid $$: waiting for the previous chain (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -qE "$PREV_PAT"; do sleep 30; done
say "CZ has the rig (predecessor $PREV gone)"
rm -f "$CAMPAIGN_DIR/STOP_CZ"
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
  say "ABORTING CZ: a response knob is missing from the deployed binary, so an arm would be a relabelled stock arm."
  exit 1
fi

A="stock:CTRL_MAX_PLEG_Y=0.21,CTRL_LOCO_UNSAFE_ADVISORY_VMAX=-1,CTRL_LOCO_UNSAFE_CYCLE_CAP=0 advisory:CTRL_MAX_PLEG_Y=0.21,CTRL_LOCO_UNSAFE_ADVISORY_VMAX=1.0,CTRL_LOCO_UNSAFE_CYCLE_CAP=0 cap5:CTRL_MAX_PLEG_Y=0.21,CTRL_LOCO_UNSAFE_ADVISORY_VMAX=-1,CTRL_LOCO_UNSAFE_CYCLE_CAP=5"
say "arms (interleaved run by run; trigger PINNED at 0.21 on all three so every run trips): $A"
say "SCOPE: 0.21 is an INDUCED trigger. This chain measures the RESPONSE, not the envelope -"
say "  the production-rate question is already answered by CW at the shipped 0.240 (p = 0.0097)."
say "expected per arm: stock cycles UNBOUNDED | advisory cycles EXACTLY 0 | cap5 cycles a MULTIPLE OF 5"
tier 0
i=0
while [ ! -f "$CAMPAIGN_DIR/STOP_CZ" ]; do
  disk_ok || break
  i=$((i+1)); wait_idle; sleep 30; wait_idle; tm_wait; say "block $i: rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
  COURSES="wkc_finals" ARMS="$A" run "genzz_cz$i" 3 2.6
  say "OPEN-40 (d) vs (f) vs stock with the trigger pinned. READ THE CAP ARM'S CYCLE"
  say "  DISTRIBUTION FIRST - it is the manipulation check and it has three outcomes:"
  python3 gazebo/tools/open40_conditioned.py 'genzz_cz*.csv' 2>&1 | tee -a "$LOG"
  tier "$i"
done
say "chain CZ done (STOP_CZ or disk floor, after block $i)"
