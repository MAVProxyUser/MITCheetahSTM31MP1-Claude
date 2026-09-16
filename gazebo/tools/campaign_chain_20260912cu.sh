#!/bin/bash
# Chain CU (2026-09-16): OPEN-40 option (d) - is the ADVISORY trip better than any transition?
#
# Chain CR closed off every other option on the deployed binary, with the trigger pinned so each
# arm actually trips (CTRL_MAX_PLEG_Y=0.21 on wkc_finals at the served 2.6):
#   stock     1/3   296 per-leg trips, 296 RecoveryStand entries (1:1 - the limit cycle), 0 tipped
#   dwell     0/3     3 trips,   5 entries (the cycle collapses 60x, exactly as designed)
#                     ... and 3 of 3 TIPPED COMPLETELY OVER, roll 131/180/136 - 4 of 4 with the probe
#   foldgate  1/3   209 trips, 209 entries, 57 folds SUPPRESSED, outcome identical to stock
# So: the transition itself makes the cycle; holding it there tips the robot, because
# RecoveryStand::_StandUp interpolates from a captured pose and assumes a STATIONARY body (and the
# bailout it hits had never executed in 7251 archived runs); and suppressing the fold is a measured
# null, which also refutes the historical fold-as-killer association as the depth confound it was.
#
# What is left is not routing a MOVING robot into a state built for a fallen one.
# CTRL_LOCO_UNSAFE_ADVISORY_VMAX counts and logs the trip above the knob and does NOT transition;
# below it stock behaviour is untouched, so a genuinely fallen or slow robot still recovers.
#
# THIS DISABLES A SAFETY TRANSITION IN THE REGIME WHERE IT FIRES. The argument that there is nothing
# to guard against there is that every y-position line in the archive grazes the limit by 0-11 mm of
# 240 - an argument, not a proof. NOTHING SHIPS FROM THIS CHAIN; the default stays -1 and decision #4
# is the operator's. The chain only measures, and the trigger is INDUCED (0.21), so its verdicts are
# not an envelope number for anything.
#
# THE MANIPULATION CHECK IS THE FIRST THING TO READ: the advisory arm must show many [locoadv]
# suppressions and ~ZERO RecoveryStand entries. If it shows entries, the knob did not take and the
# block measured nothing - and if it shows no suppressions, the trigger did not fire and the block
# measured nothing either. Both failure modes print.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
DATA="${CHEETAH_DATA:-$HOME/Desktop/Cheetah/rundata}"
export DUMP=0
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
  python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260916_cu$t.log" 2>&1
  say "$t suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260916_cu$t.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260916_cu$t.log") FAIL"
  sleep 20; wait_idle
}
mech(){ python3 - "$@" <<'PY'
import sys, glob, os, csv, re
data=os.environ.get("CHEETAH_DATA", os.path.expanduser("~/Desktop/Cheetah/rundata"))
arms={}
for path in sys.argv[1:]:
    try: rows=list(csv.DictReader(open(path)))
    except Exception: continue
    for r in rows:
        arm=(r.get("course") or "?").strip(); rid=(r.get("run_id") or "").strip()
        a=arms.setdefault(arm, dict(n=0,p=0,logs=0,trip=0,ent=0,adv=0,fold=0,tip=0))
        a["n"]+=1
        if r.get("verdict")=="PASS": a["p"]+=1
        if r.get("fall")=="tipped": a["tip"]+=1
        if not rid: continue
        g=glob.glob(os.path.join(data,"conductor","archive","*run%s_ctrl_0.log"%rid))
        if not g: continue
        a["logs"]+=1
        try: t=open(g[0],errors="replace").read()
        except Exception: continue
        a["trip"]+=len(re.findall(r"Unsafe locomotion: leg",t))
        a["ent"] +=len(re.findall(r"Recovery Balance\] body height",t))
        a["fold"]+=len(re.findall(r"Folding legs",t))
        m=re.findall(r"\[locoadv\][^\n]*\((\d+) so far\)",t)
        if m: a["adv"]+=int(m[-1])
for arm in sorted(arms):
    a=arms[arm]
    note=""
    if a["logs"] and a["trip"]==0: note="  <- NO TRIPS: trigger did not fire, block measured nothing"
    if arm!="stock" and a["logs"] and a["adv"]==0 and a["trip"]>0: note="  <- NO SUPPRESSIONS: knob did not take"
    print("    %-9s %2d/%-2d PASS | logs %2d | trips %4d | RecovStand entries %4d | folds %4d | [locoadv] suppressed %4d | TIPPED %2d%s"
          % (arm, a["p"], a["n"], a["logs"], a["trip"], a["ent"], a["fold"], a["adv"], a["tip"], note))
PY
}
PREV="${PREV_CHAIN_PID:-83966}"
PREV_PAT="${PREV_CHAIN_PAT:-campaign_chain_20260912[a-z][a-z][.]sh}"
LOG="$CAMPAIGN_DIR/open28_chaincu.log"
say "chain CU pid $$: waiting for the previous chain (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -qE "$PREV_PAT"; do sleep 30; done
say "predecessor $PREV gone - CU has the rig"
rm -f "$CAMPAIGN_DIR/STOP_CU"
wait_idle; sleep 30; wait_idle; tm_wait
OLD=$(md5 -q host-run/mit_ctrl_sim | cut -c1-8); DEPLOYED=0
if cmake --build host-build -j2 --target mit_ctrl_sim > "$CAMPAIGN_DIR/build_cu.log" 2>&1; then
  say "build ok ($(grep -c 'warning' "$CAMPAIGN_DIR/build_cu.log") warnings); deploying through deploy_host.sh"
  if bash gazebo/deploy_host.sh >> "$CAMPAIGN_DIR/build_cu.log" 2>&1; then DEPLOYED=1; say "deploy ok: binary $OLD -> $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"; else say "DEPLOY FAILED - the old binary stays, and it has no advisory knob, so the block is a NULL"; fi
else say "BUILD FAILED (see build_cu.log) - the old binary stays, and the block is a NULL"; fi
A="stock:CTRL_MAX_PLEG_Y=0.21,CTRL_LOCO_UNSAFE_ADVISORY_VMAX=-1 advisory:CTRL_MAX_PLEG_Y=0.21,CTRL_LOCO_UNSAFE_ADVISORY_VMAX=1.0"
say "arms (interleaved run by run, both knobs explicit on BOTH so neither inherits a default): $A"
say "deploy=$DEPLOYED - if 0 the advisory arm is physically identical to stock"
tier 0
i=0
while [ ! -f "$CAMPAIGN_DIR/STOP_CU" ]; do
  i=$((i+1)); wait_idle; sleep 30; wait_idle; tm_wait; say "block $i: rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
  COURSES="wkc_finals" ARMS="$A" run "adv_cu$i" 3 2.6
  say "OPEN-40 option (d), pooled over CU. READ THE MANIPULATION FIRST: the advisory arm must show"
  say "  many [locoadv] suppressions and ~ZERO RecoveryStand entries, or the block measured nothing."
  mech "$CAMPAIGN_DIR"/adv_cu*.csv | tee -a "$LOG"
  tier "$i"
done
say "chain CU done (STOP_CU seen after block $i)"
