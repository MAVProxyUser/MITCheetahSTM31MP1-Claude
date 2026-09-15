#!/bin/bash
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
# same, and the tier, DUMP=1, pooled scores each block, until STOP_CG (RULE ONE). If the
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
  python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260915_cg$t.log" 2>&1
  say "$t suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260915_cg$t.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260915_cg$t.log") FAIL"
  sleep 20; wait_idle
}
PREV="${PREV_CHAIN_PID:-94457}"
LOG="$CAMPAIGN_DIR/open30_chaincg.log"
say "chain CG pid $$: waiting for chain CF (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912cf[.]sh"; do sleep 30; done
rm -f "$CAMPAIGN_DIR/STOP_CG"
wait_idle; sleep 30; wait_idle; tm_wait
OLD=$(md5 -q host-run/mit_ctrl_sim | cut -c1-8)
say "deploy: building mit_ctrl_sim (WP_LIEDOWN_EDAMP default 0.0) - deployed binary now $OLD"
DEPLOYED=0
if cmake --build host-build -j2 --target mit_ctrl_sim > "$CAMPAIGN_DIR/build_cg.log" 2>&1; then
  say "build ok ($(grep -c 'warning' "$CAMPAIGN_DIR/build_cg.log") warnings); deploying through deploy_host.sh"
  if bash gazebo/deploy_host.sh >> "$CAMPAIGN_DIR/build_cg.log" 2>&1; then DEPLOYED=1; say "deploy ok: binary $OLD -> $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"; else say "DEPLOY FAILED (see build_cg.log) - the old binary stays"; fi
else
  say "BUILD FAILED (see build_cg.log) - the old binary stays"
fi
# probe the served recipe once and qualify on the snapshot + dump (archived at the run's end)
wait_idle; tm_wait
COURSES=wkc_finals ARMS="nodamp:" run wkc26_ld_cg_probe 1 2.6
Q=$(python3 - <<'PY'
import csv, sys
sys.path.insert(0, "gazebo/tools")
from liedown_peak import score
from liedown_handoff import handoff
import os
rows = list(csv.DictReader(open(os.path.join(os.environ.get("CHEETAH_DATA", os.path.expanduser("~/Desktop/Cheetah/rundata")), "campaigns", "wkc26_ld_cg_probe.csv"))))
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
while [ ! -f "$CAMPAIGN_DIR/STOP_CG" ]; do
  i=$((i+1)); wait_idle; sleep 30; wait_idle; tm_wait; say "block $i: rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
  COURSES=wkc_finals ARMS="$A" run "wkc26_ld_cg$i" 6 2.6
  say "wkc lie-down, pooled:"; python3 gazebo/tools/liedown_peak.py "nodamp=wkc26_ld_c[fg]*:nodamp" "stock_cg=wkc26_ld_cg*:stock" "stock_all=wkc26_l*:recipe" | tee -a "$LOG"
  say "wkc hand-off joints (dumps):"; python3 gazebo/tools/liedown_handoff.py "nodamp=wkc26_ld_c[fg]*:nodamp" "stock=wkc26_ld_cg*:stock" | tee -a "$LOG"
  [ -f "$CAMPAIGN_DIR/STOP_CG" ] && break
  COURSES=hp_gap20 ARMS="$A" run "hp26_ld_cg$i" 6 2.6
  say "hairpin lie-down, pooled:"; python3 gazebo/tools/liedown_peak.py "nodamp=hp26_ld_c[fg]*:nodamp" "stock_cg=hp26_ld_cg*:stock" "stock_all=hp26_l*:recipe" | tee -a "$LOG"
  say "hairpin hand-off joints (dumps):"; python3 gazebo/tools/liedown_handoff.py "nodamp=hp26_ld_c[fg]*:nodamp" "stock=hp26_ld_cg*:stock" | tee -a "$LOG"
  [ -f "$CAMPAIGN_DIR/STOP_CG" ] && break
  tier "$i"
done
say "chain CG done (STOP_CG seen after block $i)"
