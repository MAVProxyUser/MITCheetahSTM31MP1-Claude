#!/bin/bash
# Chain CF (2026-09-14 23:15) - chain CE with its probe gate fixed. CE's probe (run 7651) PASSED and did
# exactly what the mechanism predicts (held through, stage-2 roll 0.68 deg, zero abad excursion on all
# four legs, knees at -2.65), but CE looked for its ctrl log in the ARCHIVE, where the conductor only
# moves it at the NEXT launch, read "no kd 0.0 line" and fell back to kd24 for its block 1. This chain
# waits for CE (pid 84972; STOP_CE ends it after that campaign), skips the probe - run 7651 is it -
# and runs the recipe / nodamp blocks as designed. Original header follows.
# Chain CE (2026-09-14 22:50): the lie-down's second stage, mechanism arm.
# Pooling 60 stock lie-downs by the body's DROP after the damping hand-off (ISSUES
# OPEN-30 22:35): ~75 % stay PROPPED on the folded legs (stage-2 roll ~1 deg), ~20 %
# drop 6-7 cm onto the belly, and those are the runs that rock (4-13 deg) and, rarely,
# tip. The bridge dumps of finished runs say what yields: the knees hold at their stop,
# the ABAD joints splay outward under the pure damper (edampCommand = zeroCommand +
# kdJoint; a damper carries no static load, and the abad has no stop within 0.8 rad).
# The cheapest test needs no build: WP_LIEDOWN_EDAMP=0 skips the damper and STAND_UP's
# own Cartesian PD holds through the second stage. This chain follows CD (pid 54900;
# STOP_CD ends it after its current campaign), probes that arm once, then interleaves
# recipe / nodamp x6 on wkc 2.6 and the hairpin with the fast tier per block - DUMP=1
# so every hand-off's joint motion is on record (liedown_handoff.py) - and prints the
# pooled stage-2 scores (liedown_peak.py; a held-through run is scored over the hold's
# last 1.5 s) each block, until STOP_CF (RULE ONE). If the probe does not PASS with the
# `damping hold: kd 0.0` line in its ctrl log, the arm falls back to CD's kd24.
# Scoring runs in the idle gap between campaigns: reading the archive while a run is on
# the rig costs the sim's sensor stream samples (OPEN-39, 22:26-22:32: three tier falls,
# each with the run's only deficit second as its event second, during such a scan).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
DATA="${CHEETAH_DATA:-$HOME/Desktop/Cheetah/rundata}"
export DUMP=1
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "gazebo/conductor/mission_[r]unner" >/dev/null && ! pgrep -f "gazebo/tools/open28_subcourse[.]sh" >/dev/null && ! pgrep -f "unittests/test_validated_[m]issions" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
tm_wait(){ local n=0; while tmutil status 2>/dev/null | grep -q "Running = 1"; do [ $n -eq 0 ] && say "Time Machine backup in progress - holding (not a run)"; n=$((n+1)); sleep 30; done; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps at $v (COURSES=${COURSES:-} ARMS=${ARMS:-} DUMP=$DUMP)"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
ctrl_log(){ local f; f=$(ls "$DATA"/conductor/archive/*run"$1"_ctrl_0.log* 2>/dev/null | head -1); case "$f" in "") ;; *.zst) zstd -dc "$f";; *) cat "$f";; esac; }
PREV="${PREV_CHAIN_PID:-84972}"
LOG="$CAMPAIGN_DIR/open30_chaincf.log"
say "chain CF pid $$: waiting for chain CE (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912ce[.]sh"; do sleep 30; done
rm -f "$CAMPAIGN_DIR/STOP_CF"
wait_idle; sleep 30; wait_idle; tm_wait
A="recipe: nodamp:WP_LIEDOWN_EDAMP=0"; B=nodamp
say "arms: $A (probe = CE's run 7651: PASS, held through, stage-2 roll 0.68 deg, abad excursion 0.000 on all four legs)"
i=0
while [ ! -f "$CAMPAIGN_DIR/STOP_CF" ]; do
  i=$((i+1)); wait_idle; sleep 30; wait_idle; tm_wait; say "block $i: rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
  COURSES=wkc_finals ARMS="$A" run "wkc26_ld_cf$i" 6 2.6
  say "wkc lie-down, pooled (all stock blocks / all $B blocks):"; python3 gazebo/tools/liedown_peak.py "recipe=wkc26_l*:recipe" "$B=wkc26_l*:$B" | tee -a "$LOG"
  say "wkc hand-off joints (CE dumps):"; python3 gazebo/tools/liedown_handoff.py "recipe=wkc26_ld_c[ef]*:recipe" "$B=wkc26_ld_c[ef]*:$B" | tee -a "$LOG"
  [ -f "$CAMPAIGN_DIR/STOP_CF" ] && break
  COURSES=hp_gap20 ARMS="$A" run "hp26_ld_cf$i" 6 2.6
  say "hairpin lie-down, pooled:"; python3 gazebo/tools/liedown_peak.py "recipe=hp26_l*:recipe" "$B=hp26_l*:$B" | tee -a "$LOG"
  say "hairpin hand-off joints (CE dumps):"; python3 gazebo/tools/liedown_handoff.py "recipe=hp26_ld_c[ef]*:recipe" "$B=hp26_ld_c[ef]*:$B" | tee -a "$LOG"
  [ -f "$CAMPAIGN_DIR/STOP_CF" ] && break
  tm_wait; say "block $i: fast suite tier"
  python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260914_cf$i.log" 2>&1
  say "block $i suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260914_cf$i.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260914_cf$i.log") FAIL"
  sleep 20
done
say "chain CF done (STOP_CF seen after block $i)"
