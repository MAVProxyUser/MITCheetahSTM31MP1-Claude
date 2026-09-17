#!/bin/bash
# Chain CX (2026-09-17): rank OPEN-40's two candidate fixes against stock, in one block.
#
# Decision #4 now has two implementations and only one of them has ever run:
#   (d) ADVISORY - never transition above a speed. Measured twice: chain CU at an
#       induced 0.21 (78/78 vs 31/78, p = 1.1e-13) and chain CW at the SHIPPED
#       0.240 (conditioned 5/5 vs 2/8, p = 0.0210). Safer on every axis both
#       times. Its cost is that it gives up the guard's ACTION entirely above the
#       speed gate.
#   (f) CYCLE CAP - keep the action for the first N trips, go advisory only once
#       the pattern proves itself a cycle. Strictly more conservative. NEVER RUN.
#
# The cap value is a calculation rather than a guess: the height bleed is ~2.29 mm
# per RecoveryStand cycle (fitted on 11 historical runs, then validated on 5 CW
# runs it never saw - new-sample median 2.19), and a normal entry carries ~95 mm
# above the 0.20 m fold line. So cap 5 spends 11.5 mm, about an eighth of the
# budget, while the observed PASS/FAIL boundary sits near 43 cycles.
#
# THREE ARMS, INTERLEAVED RUN BY RUN, and BOTH knobs explicit on ALL THREE so no
# arm inherits a default - that is the CU lesson. CTRL_MAX_PLEG_Y stays UNSET, so
# the trigger is the shipped 0.240 coming from the code.
#
# THE QUESTION CX ANSWERS: does bounding the cycle buy the same outcome as
# abolishing it? If cap5 matches advisory, ship the cap and keep the guard for
# one-off trips. If cap5 tracks stock, the cycle does its damage inside 5 cycles
# and only (d) works. Either answer decides the implementation.
#
# NOTHING SHIPS. Both knobs stay default-off in the tree.
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
disk_ok(){ local f; f=$(df -g /System/Volumes/Data | awk 'NR==2{print $4}'); if [ "${f:-0}" -lt "$FLOOR_GB" ]; then say "DISK FLOOR: ${f} GB free is below ${FLOOR_GB} GB - CX stops here"; return 1; fi; say "disk ${f} GB free (floor ${FLOOR_GB})"; return 0; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps at $v (COURSES=${COURSES:-} ARMS=${ARMS:-} DUMP=$DUMP)"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
tier(){ local t="$1"; tm_wait; say "$t: fast suite tier"
  python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260917_cx$t.log" 2>&1
  say "$t suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260917_cx$t.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260917_cx$t.log") FAIL"
  sleep 20; wait_idle
}
PREV="${PREV_CHAIN_PID:-64880}"
PREV_PAT="${PREV_CHAIN_PAT:-campaign_chain_20260912[a-z][a-z][.]sh}"
LOG="$CAMPAIGN_DIR/open28_chaincx.log"
say "chain CX pid $$: waiting for the previous chain (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -qE "$PREV_PAT"; do sleep 30; done
say "predecessor $PREV gone - CX has the rig"
rm -f "$CAMPAIGN_DIR/STOP_CX"
wait_idle; sleep 30; wait_idle; tm_wait

# BUILD AND DEPLOY. Option (f) has only ever been syntax-checked, so this is the
# first time it becomes a real binary. If either step fails the OLD binary stays,
# and the old binary has no cap knob - which would make cap5 physically identical
# to stock and the whole chain a null. Say so loudly rather than producing data.
OLD=$(md5 -q host-run/mit_ctrl_sim | cut -c1-8); DEPLOYED=0
if cmake --build host-build -j2 --target mit_ctrl_sim > "$CAMPAIGN_DIR/build_cx.log" 2>&1; then
  say "build ok ($(grep -c 'warning' "$CAMPAIGN_DIR/build_cx.log") warnings); deploying through deploy_host.sh"
  if bash gazebo/deploy_host.sh >> "$CAMPAIGN_DIR/build_cx.log" 2>&1; then
    DEPLOYED=1; say "deploy ok: binary $OLD -> $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
  else say "DEPLOY FAILED - old binary stays, it has NO cap knob, cap5 == stock, CHAIN IS A NULL"; fi
else say "BUILD FAILED (see build_cx.log) - old binary stays, cap5 == stock, CHAIN IS A NULL"; fi

# The knob must be IN the binary before we spend rig time on it (probe-a-default).
KNOB=$(strings host-run/mit_ctrl_sim | grep -c 'CTRL_LOCO_UNSAFE_CYCLE_CAP')
TAG=$(strings host-run/mit_ctrl_sim | grep -c 'lococap')
say "knob check: CTRL_LOCO_UNSAFE_CYCLE_CAP x$KNOB, [lococap] tag x$TAG (both must be >= 1)"
if [ "$DEPLOYED" != 1 ] || [ "$KNOB" -lt 1 ] || [ "$TAG" -lt 1 ]; then
  say "ABORTING CX: the cap knob is not in the deployed binary, so cap5 would be a relabelled stock arm."
  exit 1
fi

A="stock:CTRL_LOCO_UNSAFE_ADVISORY_VMAX=-1,CTRL_LOCO_UNSAFE_CYCLE_CAP=0 advisory:CTRL_LOCO_UNSAFE_ADVISORY_VMAX=1.0,CTRL_LOCO_UNSAFE_CYCLE_CAP=0 cap5:CTRL_LOCO_UNSAFE_ADVISORY_VMAX=-1,CTRL_LOCO_UNSAFE_CYCLE_CAP=5"
say "arms (interleaved run by run, BOTH knobs explicit on ALL THREE): $A"
say "CTRL_MAX_PLEG_Y deliberately UNSET: the trigger is the shipped 0.240 from the code"
say "manipulation to read FIRST, per arm: stock -> no [locoadv] and no [lococap];"
say "  advisory -> [locoadv] > 0 and ZERO RecoveryStand cycles;"
say "  cap5 -> [lococap] > 0 and cycles CAPPED near 5, not zero and not 80."
tier 0
i=0
while [ ! -f "$CAMPAIGN_DIR/STOP_CX" ]; do
  disk_ok || break
  i=$((i+1)); wait_idle; sleep 30; wait_idle; tm_wait; say "block $i: rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
  COURSES="wkc_weave" ARMS="$A" run "capab_cx$i" 6 2.6
  say "OPEN-40 (d) vs (f) vs stock, pooled over CX. Conditioned scoring - the marginal"
  say "  rate is diluted because only ~7 % of runs trip at the shipped trigger:"
  python3 gazebo/tools/open40_conditioned.py 'capab_cx*.csv' 2>&1 | tee -a "$LOG"
  tier "$i"
done
say "chain CX done (STOP_CX or disk floor, after block $i)"
