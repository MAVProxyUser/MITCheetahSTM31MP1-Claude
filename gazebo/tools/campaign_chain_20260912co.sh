#!/bin/bash
# Chain CO (2026-09-16 05:25): what 0.1 m/s of cruise BUYS in pitch margin on the hairpin. CN block 1
# retired the re-acceleration cap (null on the pitch body, 22.9 vs 23.5 deg, and the lap +0.3 s says
# the manipulation took), so the remaining lever for run 7994's runaway is the cruise itself. The
# question is not the direction - slower is obviously calmer - it is the SIZE: how many degrees of
# pitch does 2.5 buy over 2.6, and at what lap cost. That is the same shape as the turn-cap
# experiment that shipped in September. Interleaved per run via the harness' own SPEED= arm token.
# Original CN header follows.
# Chain CN (2026-09-16 04:25): the hairpin RE-ACCELERATION cap. That fall
# was a pitch runaway on the straight out of the reversal: the profile re-accelerated, the body
# overshot its 2.6 cruise to 2.83 m/s, the rear-most stance knee was asked for 43 N.m against a
# 35.55 N.m joint limit at full reach, and the shortfall arrived as nose-up pitch, stride after
# stride, to the 33 deg E-stop. `v_reaccel_max` (WP_REACCEL_VMAX, off by default, reaccel_dist 12 m)
# caps the forward pass after any braking minimum - exactly that phase. Measured on PEAK PITCH per
# run, a number every run provides, not a 1-in-300 fall: the last two chains taught me to pick the
# quantity the mechanism consumes. Arms: served recipe vs a 2.4 cap. Original CM header follows.
# Chain CM (2026-09-16 03:20): the leg_y_max instrument. CL block 1
# came back 12/12 and "0 with any y-position line, 0 trips" in BOTH arms - the trip is 2 runs in
# ~600, so a 12-run block measures nothing. The controller now reports the quantity the limit acts
# on directly: max |p(1)| per second in its own frame, per-run max in the new leg_y_max_mm column.
# The FK derivation predicts ~247 mm against the 240 mm limit; one run now tests that instead of
# one in three hundred. Deploys it in the gap behind CL, probes, tier first, then wkc 2.6 x6 per
# arm (0.24 vs 0.30) + the tier each block, reporting the excursion distribution per arm.
# Original CL header follows.
# Chain CL (2026-09-16 01:50): MEASURE the lateral foot limit, 0.24 vs 0.30 (ISSUES OPEN-39 01:40).
# The derivation says 0.24 is a stance-envelope policy at 63 % of the 0.379 m mechanical bound,
# inherited from mini-cheetah's ratio, and the shipped trot exceeds it by 0-7 mm - which is why
# locomotionSafe's y-position branch fires at all (run 8315 tripped it at cruise and fell, run 8507
# tripped it 54 times while standing and survived). WHICH VALUE SHIPS IS THE OPERATOR'S CALL; this
# chain only measures, with CTRL_MAX_PLEG_Y explicit on BOTH arms (default stays 0.24). The powered
# quantity is not the fall rate - the fall is 1 run in ~600 - it is the MANIPULATION: runs carrying
# any y-position line at all, and the trip count per run, which 8507 shows reaching 54. Deploys the
# knob in the gap behind CK, probes, tier first, then wkc 2.6 x6 per arm + the tier each block.
# Original CK header follows.
# Chain CK (2026-09-15 23:15): the CORRECTED held-sample rule (ISSUES OPEN-39 22:57). CJ's own
# first runs corrected its design: every run logs 50 [leghold] lines (the throttle cap) at ordinary
# cruise values, so held samples are ROUTINE - two control ticks on one sensor packet at matched
# rates - and the first cut, which SKIPPED unchanged ticks, would have slowed every genuine trip in
# proportion to how often the stream holds. The rule now: count every over-limit tick as before, but
# require two DISTINCT values while over the limit before tripping (a frozen sample reports one value
# however long it lasts; a real transient reports a new one every tick). The 50-line log becomes
# heartbeat counters ([stm32mp1] held samples: held=N/s maxrun=N). CTRL_LEG_HELD_GATE=0 restores the
# plain count. Built and compile-verified 23:14. Same shape as CJ below.
# CI tier 0 lost the star in a 404-samples/s second: eight consecutive ticks each read leg 0 at
# EXACTLY 9.749 m/s and the foot at exactly 0.000 m above hip - a frozen sample, which persists
# perfectly and so defeats a tick count (run 8315 shows the contrast: a real transient reads a
# different value every tick, 10.06 -> 12.39, and was absorbed correctly). A tick whose leg state
# is bit-for-bit identical to the previous tick now advances no counter (logged [leghold];
# CTRL_LEG_HELD_GATE=0 disables). Built and compile-verified at 22:24. Same shape as CI below.
# Chain CI (2026-09-15 21:55): the debounce on the OTHER TWO locomotionSafe branches
# (ISSUES OPEN-39 21:50). Run 8315 forced it: the leg-speed debounce absorbed a four-tick
# 12.4 m/s spike inside a 464-samples/s second exactly as designed - and the run fell anyway
# through `leg 2 y-position is bad (-0.240 m, max 0.240)`, the same one-tick-trip-into-
# RECOVERY_STAND defect in a sibling branch, grazing the limit by 0 mm. CTRL_LEGY_TRIP_TICKS
# and CTRL_HIP_TRIP_TICKS now default to 5 ticks too (1 = upstream). Absorbed grazes log as
# [legkin]. Same shape as chain CH (header kept below).
# Chain CH (2026-09-15 03:55): the DEBOUNCED leg-speed trip (ISSUES OPEN-39 03:50:
# 8 of the 189 runs archived since 22:00 logged "leg N is moving too quickly" and all 8 fell -
# RECOVERY_STAND re-commands a stand pose mid-stride; the one-tick spike comes from a touchdown
# impact as readily as from a sample gap). CTRL_LEGV_TRIP_TICKS=5 by default now; 1 restores stock.
# Same shape as chain CG (below, kept verbatim): waits for CG (pid 19680; STOP_CG ends it after its
# current campaign), builds + deploys in the gap, probes the served recipe, tier first, then the
# standing nodamp / stock lie-down A/B with the tier per block - and per block counts the [legv]
# absorbed-spike lines and the trips in the block's archived ctrl logs, until STOP_CO.
# Chain CG (2026-09-15 00:20): ship the lie-down fix as the default, in the gap.
# ISSUES OPEN-30: the lie-down's second stage handed the legs to a pure joint damper
# and in 8 of 11 stock hand-offs on record (bridge dumps, chains CE/CF) the abad joints
# splayed 0.45-0.74 rad, the body dropped onto its belly and rocked 2-13 deg - every
# finish tip and star interlude roll-over started there. WP_LIEDOWN_EDAMP=0 (no damper,
# STAND_UP's own PD holds through) gave abad excursion 0.000 rad in 6 of 6 and a stage-2
# roll of 0.3 deg median / 0.7 max, course verdicts unchanged. mit_sim_main.cpp's default
# is now 0.0 (8 restores stock). This chain waits for CF (pid 94457; STOP_CF ends it after
# its current campaign), then in the idle gap: builds mit_ctrl_sim, deploys it through
# deploy_host.sh (never a bare cp), probes the served recipe once (qualified on the
# snapshot + dump: held through, no damper segment), runs the fast tier FIRST (the star
# case carries the dash interlude, i.e. the stand-back-up from the held lie-down), then
# holds the rig on blocks of wkc 2.6 x6 nodamp / stock:WP_LIEDOWN_EDAMP=8, hairpin x6 the
# same, and the tier, DUMP=1, pooled scores each block, until STOP_CO (RULE ONE). If the
# build or deploy fails, or the probe does not qualify, the blocks run the same physical
# A/B as explicit env arms on whatever binary is deployed.
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
tier(){ local t="$1"; tm_wait; say "$t: fast suite tier"
  python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260916_co$t.log" 2>&1
  say "$t suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260916_co$t.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260916_co$t.log") FAIL"
  sleep 20; wait_idle
}
ycount(){ shift 0
  awk -F, 'FNR>1 && $7!="" { n[$2]++; p[$2]=p[$2]" "$7; if($7+0>mx[$2]) mx[$2]=$7+0; s[$2]+=$7; if($17!="") {t[$2]+=$17; tn[$2]++} }
    END { for (a in n) printf "    %s: n=%d  peak pitch mean %.1f  MAX %.1f deg  |  lap %.1f s\n", a, n[a], s[a]/n[a], mx[a], (tn[a]?t[a]/tn[a]:0) }' "$@" 2>/dev/null; }
legv_count(){ local runs f trips=0 spikes=0 n=0
  runs=$(awk -F, 'NR>1{print $11}' "$@" 2>/dev/null)
  for r in $runs; do f=$(ls "$DATA"/conductor/archive/*run"$r"_ctrl_0.log 2>/dev/null | head -1); [ -z "$f" ] && continue; n=$((n+1))
    grep -qE 'moving too quickly|y-position is bad|is above hip' "$f" && trips=$((trips+1)); s=$(grep -cE '^\[(legv|legkin|leghold)\]' "$f"); [ "$s" -gt 0 ] && spikes=$((spikes+1)); done
  echo "$n runs, $trips tripped, $spikes with an absorbed spike"; }
PREV="${PREV_CHAIN_PID:-26959}"
LOG="$CAMPAIGN_DIR/open28_chainco.log"
say "chain CO pid $$: waiting for chain CN (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912cn[.]sh"; do sleep 30; done
rm -f "$CAMPAIGN_DIR/STOP_CO"
wait_idle; sleep 30; wait_idle; tm_wait
DEPLOYED=1; say "no deploy needed - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8) already carries the knobs"
# probe the served recipe once and qualify on the snapshot + dump (archived at the run's end)
wait_idle; tm_wait
COURSES=hp_gap20 ARMS="recipe:" run hp26_vcruise_probe 1 2.6
Q=$(awk -F, 'NR>1{print ($4=="PASS") ? "QUALIFIED: run "$11" verdict PASS" : "NOT QUALIFIED: run "$11" verdict "$4}' "$CAMPAIGN_DIR/hp26_vcruise_probe.csv" 2>/dev/null | tail -1)
say "probe $Q"
# the blocks use explicit env on both arms whatever the probe said (identical physics
# to the served default on the new binary, and the same A/B on the old one); the served
# default itself is what the probe and every tier case exercise
A="v26:SPEED=2.6 v25:SPEED=2.5"
case "$Q" in QUALIFIED*) say "the served recipe lies down without a damper (deploy $DEPLOYED) - arms: $A";;
  *) say "served recipe NOT held through (deploy $DEPLOYED) - the tier will say whether the old binary is still deployed; arms: $A";; esac
tier 0
i=0
while [ ! -f "$CAMPAIGN_DIR/STOP_CO" ]; do
  i=$((i+1)); wait_idle; sleep 30; wait_idle; tm_wait; say "block $i: rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
  COURSES=hp_gap20 ARMS="$A" run "hp26_vcruise_co$i" 6 2.6
  say "hairpin peak pitch per arm, pooled over CO (degrees of pitch per 0.1 m/s, and the lap cost):"
  ycount "$CAMPAIGN_DIR"/hp26_vcruise_co*.csv | tee -a "$LOG"
  tier "$i"
done
say "chain CO done (STOP_CO seen after block $i)"
