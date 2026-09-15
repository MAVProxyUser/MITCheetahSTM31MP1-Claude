#!/bin/bash
# Chain CH (2026-09-15 03:55): ship the DEBOUNCED leg-speed trip, in the gap (ISSUES OPEN-39 03:50:
# 8 of the 189 runs archived since 22:00 logged "leg N is moving too quickly" and all 8 fell -
# RECOVERY_STAND re-commands a stand pose mid-stride; the one-tick spike comes from a touchdown
# impact as readily as from a sample gap). CTRL_LEGV_TRIP_TICKS=5 by default now; 1 restores stock.
# Same shape as chain CG (below, kept verbatim): waits for CG (pid 19680; STOP_CH ends it after its
# current campaign), builds + deploys in the gap, probes the served recipe, tier first, then the
# standing nodamp / stock lie-down A/B with the tier per block - and per block counts the [legv]
# absorbed-spike lines and the trips in the block's archived ctrl logs, until STOP_CH.
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
# same, and the tier, DUMP=1, pooled scores each block, until STOP_CH (RULE ONE). If the
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
  python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260915_ch$t.log" 2>&1
  say "$t suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260915_ch$t.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260915_ch$t.log") FAIL"
  sleep 20; wait_idle
}
legv_count(){ local runs f trips=0 spikes=0 n=0
  runs=$(awk -F, 'NR>1{print $11}' "$@" 2>/dev/null)
  for r in $runs; do f=$(ls "$DATA"/conductor/archive/*run"$r"_ctrl_0.log 2>/dev/null | head -1); [ -z "$f" ] && continue; n=$((n+1))
    grep -q 'moving too quickly' "$f" && trips=$((trips+1)); s=$(grep -c '^\[legv\]' "$f"); [ "$s" -gt 0 ] && spikes=$((spikes+1)); done
  echo "$n runs, $trips tripped, $spikes with an absorbed spike"; }
PREV="${PREV_CHAIN_PID:-19680}"
LOG="$CAMPAIGN_DIR/open39_chainch.log"
say "chain CH pid $$: waiting for chain CG (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912cg[.]sh"; do sleep 30; done
rm -f "$CAMPAIGN_DIR/STOP_CH"
wait_idle; sleep 30; wait_idle; tm_wait
OLD=$(md5 -q host-run/mit_ctrl_sim | cut -c1-8)
say "deploy: building mit_ctrl_sim (debounced leg-speed trip, CTRL_LEGV_TRIP_TICKS=5) - deployed binary now $OLD"
DEPLOYED=0
if cmake --build host-build -j2 --target mit_ctrl_sim > "$CAMPAIGN_DIR/build_ch.log" 2>&1; then
  say "build ok ($(grep -c 'warning' "$CAMPAIGN_DIR/build_ch.log") warnings); deploying through deploy_host.sh"
  if bash gazebo/deploy_host.sh >> "$CAMPAIGN_DIR/build_ch.log" 2>&1; then DEPLOYED=1; say "deploy ok: binary $OLD -> $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"; else say "DEPLOY FAILED (see build_ch.log) - the old binary stays"; fi
else
  say "BUILD FAILED (see build_ch.log) - the old binary stays"
fi
# probe the served recipe once and qualify on the snapshot + dump (archived at the run's end)
wait_idle; tm_wait
COURSES=wkc_finals ARMS="nodamp:" run wkc26_ld_ch_probe 1 2.6
Q=$(python3 - <<'PY'
import csv, sys
sys.path.insert(0, "gazebo/tools")
from liedown_peak import score
from liedown_handoff import handoff
import os
rows = list(csv.DictReader(open(os.path.join(os.environ.get("CHEETAH_DATA", os.path.expanduser("~/Desktop/Cheetah/rundata")), "campaigns", "wkc26_ld_ch_probe.csv"))))
r = rows[-1] if rows else {}
try: s = score(r["snapshot"]); h = handoff(r["bridge_dump"])
except Exception as e: print("probe: unscorable (%s) verdict %s" % (e, r.get("verdict"))); raise SystemExit
ok = r.get("verdict") == "PASS" and s and s["mode"] == "held" and s["peak_roll"] < 3.0 and h and h["mode"] == "held" and h["mean_exc"] < 0.05
print("%s: run %s verdict %s | snapshot %s peak roll %.2f | dump %s abad excursion %.3f" % ("QUALIFIED" if ok else "NOT QUALIFIED", r.get("run_id"), r.get("verdict"), s and s["mode"], s["peak_roll"] if s else -1, h and h["mode"], h["mean_exc"] if h else -1))
PY
)
say "probe $Q"
# the blocks use explicit env on both arms whatever the probe said (identical physics
# to the served default on the new binary, and the same A/B on the old one); the served
# default itself is what the probe and every tier case exercise
A="nodamp:WP_LIEDOWN_EDAMP=0 stock:WP_LIEDOWN_EDAMP=8"
case "$Q" in QUALIFIED*) say "the served recipe lies down without a damper (deploy $DEPLOYED) - arms: $A";;
  *) say "served recipe NOT held through (deploy $DEPLOYED) - the tier will say whether the old binary is still deployed; arms: $A";; esac
tier 0
i=0
while [ ! -f "$CAMPAIGN_DIR/STOP_CH" ]; do
  i=$((i+1)); wait_idle; sleep 30; wait_idle; tm_wait; say "block $i: rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
  COURSES=wkc_finals ARMS="$A" run "wkc26_ld_ch$i" 6 2.6
  say "wkc lie-down, pooled:"; python3 gazebo/tools/liedown_peak.py "nodamp=wkc26_ld_c[fgh]*:nodamp" "stock_new=wkc26_ld_c[gh]*:stock" "stock_all=wkc26_l*:recipe" | tee -a "$LOG"
  say "wkc hand-off joints (dumps):"; python3 gazebo/tools/liedown_handoff.py "nodamp=wkc26_ld_c[fgh]*:nodamp" "stock=wkc26_ld_c[gh]*:stock" | tee -a "$LOG"
  [ -f "$CAMPAIGN_DIR/STOP_CH" ] && break
  COURSES=hp_gap20 ARMS="$A" run "hp26_ld_ch$i" 6 2.6
  say "hairpin lie-down, pooled:"; python3 gazebo/tools/liedown_peak.py "nodamp=hp26_ld_c[fgh]*:nodamp" "stock_new=hp26_ld_c[gh]*:stock" "stock_all=hp26_l*:recipe" | tee -a "$LOG"
  say "hairpin hand-off joints (dumps):"; python3 gazebo/tools/liedown_handoff.py "nodamp=hp26_ld_c[fgh]*:nodamp" "stock=hp26_ld_c[gh]*:stock" | tee -a "$LOG"
  [ -f "$CAMPAIGN_DIR/STOP_CH" ] && break
  say "block $i leg-speed check over its runs: $(legv_count "$CAMPAIGN_DIR/wkc26_ld_ch$i.csv" "$CAMPAIGN_DIR/hp26_ld_ch$i.csv")"
  tier "$i"
done
say "chain CH done (STOP_CH seen after block $i)"
