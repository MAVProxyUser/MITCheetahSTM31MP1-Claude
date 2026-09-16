#!/bin/bash
# Chain CR (2026-09-16): the OPEN-40 A/B - does holding RecoveryStand actually save the robot?
#
# CQ's margin probe produced the reproducer and the mechanism. `wkc_weave` at the served 2.6 drove
# leg 2's foot to 243 mm (3 mm past the 240 limit) and HELD it there for 49 ticks, so OPEN-39's
# debounce correctly called it genuine and tripped - and the trip landed in a 500 Hz limit cycle:
# FSM_State_RecoveryStand::checkTransition() reads only control_mode, which nav pins at
# K_LOCOMOTION for the whole mission, so the recovery state is handed back after ONE tick, forever.
# It cannot stand a robot on one tick per cycle, and onEnter() re-decides fold-vs-stand on EVERY
# entry against 0.2 < z < 0.45, so the decision is re-made every 2 ms against a height the cycle
# bleeds. Trips and recovery entries are EQUAL to the unit in both runs that enter it (54/54 and
# 45/45) - the cycle measured rather than argued, one onEnter() per trip. Both entered at ~0.30 m,
# so headroom at entry does NOT separate them (this header first said 8892 entered at 0.219 with
# 19 mm of margin; that was a MID-cycle value read from a tail window, and is withdrawn - see the
# correction in ISSUES OPEN-40). What separates them is bleed per CYCLE: 1.06 mm on wkc_finals
# (0.057 m over 54 passes, bottoming at 0.249, escapes) vs 2.49 mm on the weave (0.112 m over 45),
# which crosses 0.20 - and the weave's last four entries, 0.197/0.192/0.188/0.183, produced EXACTLY
# four `Folding legs` lines, so crossing and fold correspond one for one. The recovery state is what
# folded four legs under a body at cruise, starved of the ticks it needed to stand.
#
# CTRL_LOCO_UNSAFE_HOLD_MS arms a dwell (g_locoUnsafeHold) that RecoveryStand honours before
# handing back, so the stand-up ramp (standup_ramp_iter 250 = 500 ms) actually runs. 0 = stock.
# 600 ms is chosen to clear that ramp with margin, not tuned. PASSIVE and every other mode request
# are still honoured immediately, so the dwell can never trap the robot away from an E-stop.
#
# THE DESIGN CHANGED BEFORE LAUNCH, and the reason is a power calculation I should have done
# first. Measuring the archive (20 KB tail of all 7077 ctrl logs) says the cycle occurs in 353
# runs, 5.0 %, and 307 of those fell - 24 % of every fall on record - but the rate tracks the
# TRIGGER fixes: 24.9 % on 09-05, 0.9 % on 09-15 after the debounces, 0.2 % on 09-16. Post-debounce
# it is ~0.6 % of runs (8 cycles in ~1240 runs over two days), so a 6-rep block expects 0.04
# cycles and a natural-trigger A/B on the weave would have measured NOTHING for days - CL's exact
# failure, which I nearly repeated after writing its lesson down.
#
# So the trigger is HELD FIXED AND FREQUENT on EVERY arm, and only the RESPONSE varies.
# CTRL_MAX_PLEG_Y=0.21 is the value, taken from this chain's own margin data rather than invented:
# wkc_finals' per-run lateral maximum is 219-224 mm across 6 runs at the served 2.6 (chain CQ's
# leg_y_max column), so a 210 mm limit is crossed by EVERY run - a ~100 % trigger - and crossed
# WHERE THE LATERAL EXCURSION PEAKS, which is cornering at cruise.
# The first draft used MIT's own upstream 0.18. That is a documented ~100 % trigger too ("the rear
# legs cross 0.18 m within ~1 s of gait entry"), but it fires DURING THE VELOCITY RAMP while the
# robot is still slow - which would have made the fold-gate arm a NULL BY CONSTRUCTION, because
# that fix only acts on a body that is travelling. A trigger has to fire in the REGIME the fix
# addresses, not merely often.
# The course is wkc_finals at the served 2.6 because it normally passes ~100/100 clean, so any fall
# in any arm is attributable to the induced trip and its response - a clean baseline, unlike the
# weave, which already falls on its own.
#
# THREE ARMS, every knob explicit on every one so nothing is inherited: stock, the DWELL
# (CTRL_LOCO_UNSAFE_HOLD_MS=600), and the FOLD GATE (CTRL_RECOVER_FOLD_VMAX=1.0).
#
# WHAT THIS DOES AND DOES NOT MEASURE: it measures whether either response changes the OUTCOME of
# a trip. It says nothing about the natural fall rate, because the trip is induced - and it must
# not be quoted as an envelope or reliability number for 0.21, a limit the shipped gait crosses on
# every run by design. The manipulation check sits beside every result (runs that tripped, worst
# cycle length, folds, dwells per arm); if an arm does not trip, the block measured nothing and
# says so. THE COLUMN THAT DECIDES IT IS `folded`: across the 79 historical entries from a healthy
# body, 42 of the 61 that fell issued a fold against 3 of the 18 that survived, while cycle
# DURATION did not discriminate at all - so judge the arms on whether the fold happened, with the
# verdict beside it, never on cycle length.
#
# NOTHING SHIPS FROM THIS CHAIN. The default stays 0 and decision #4 is the operator's; this
# chain only measures. Builds and deploys in the gap behind CQ, probes, tier first, then blocks of
# the weave A/B + the tier, until STOP_CR (RULE ONE).
# TWO BUGS IN THIS CHAIN'S OWN FIRST DRAFT, fixed before launch and recorded because both
# come from trusting a notation instead of the writer (open28_subcourse.sh):
#   1. ARMS are separated by a SPACE, not a comma - `for a in $ARMS` word-splits, and an arm's
#      OWN env uses commas (converted by `tr ',' ' '` at the --extra call). A comma between arms
#      makes ONE arm whose env carries a bogus `name:VAR=x` token.
#   2. COURSES="" does NOT mean "no course": line 25 is `${COURSES:-wkc_weave wkc_box
#      wkc_hairpin}`, so empty takes the default, and with ARMS set line 215 passes the WHOLE
#      string as a single slot spec. Name the one course explicitly.
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
  python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260916_cr$t.log" 2>&1
  say "$t suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260916_cr$t.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260916_cr$t.log") FAIL"
  sleep 20; wait_idle
}
# The report that decides this chain. Per ARM: runs, verdicts, how many TRIPPED at all (the
# manipulation check - without trips the arms are the same controller), the worst cycle length
# seen (trips in one run), how many runs the recovery FOLDED in, and how many dwells fired.
# A run's ctrl log is archived at the NEXT launch, so the newest run lags one launch; pooling
# over every CR block each time makes that lag self-correcting.
cyc(){ python3 - "$@" <<'PY'
import sys, glob, os, csv, re
data=os.environ.get("CHEETAH_DATA", os.path.expanduser("~/Desktop/Cheetah/rundata"))
arms={}
for path in sys.argv[1:]:
    try: rows=list(csv.DictReader(open(path)))
    except Exception: continue
    for r in rows:
        arm=(r.get("course") or "?").strip(); rid=(r.get("run_id") or "").strip()
        if not rid: continue
        a=arms.setdefault(arm, dict(n=0,p=0,trip=0,worst=0,fold=0,dwell=0,seen=0,zent=[],zmin=[]))
        a["n"]+=1
        if r.get("verdict")=="PASS": a["p"]+=1
        g=glob.glob(os.path.join(data,"conductor","archive","*run%s_ctrl_0.log"%rid))
        if not g: continue
        a["seen"]+=1
        try: t=open(g[0],errors="replace").read()
        except Exception: continue
        trips=len(re.findall(r"Unsafe locomotion", t))
        if trips: a["trip"]+=1
        a["worst"]=max(a["worst"],trips)
        if "Folding legs" in t: a["fold"]+=1
        a["dwell"]+=len(re.findall(r"unsafe dwell expired", t))
        # z is what the fold decision reads, so collect it only for runs that
        # actually tripped and keep BOTH the entry height and the lowest the
        # cycle drove it to - the bleed (first - min) is the discriminator,
        # not the entry height, which was the same ~0.30 m in both runs.
        if trips:
            zs=[float(x) for x in re.findall(r"Recovery Balance\] body height is ([0-9.]+)", t)]
            if zs: a["zent"].append(zs[0]); a["zmin"].append(min(zs))
for arm in sorted(arms):
    a=arms[arm]
    z=("z %.3f->%.3f (worst %.3f, fold line 0.20)" % (
          sum(a["zent"])/len(a["zent"]), sum(a["zmin"])/len(a["zmin"]), min(a["zmin"]))
       ) if a["zent"] else "z (no trip, nothing measured)"
    print("    %-26s %2d/%-2d PASS | logs %2d | tripped %2d | worst cycle %3d trips | folded %2d | dwells %3d | %s"
          % (arm, a["p"], a["n"], a["seen"], a["trip"], a["worst"], a["fold"], a["dwell"], z))
PY
}
PREV="${PREV_CHAIN_PID:-78002}"
LOG="$CAMPAIGN_DIR/open28_chaincr.log"
say "chain CR pid $$: waiting for chain CQ (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912cq[.]sh"; do sleep 30; done
rm -f "$CAMPAIGN_DIR/STOP_CR"
wait_idle; sleep 30; wait_idle; tm_wait
OLD=$(md5 -q host-run/mit_ctrl_sim | cut -c1-8); DEPLOYED=0
if cmake --build host-build -j2 --target mit_ctrl_sim > "$CAMPAIGN_DIR/build_cr.log" 2>&1; then
  say "build ok ($(grep -c 'warning' "$CAMPAIGN_DIR/build_cr.log") warnings); deploying through deploy_host.sh"
  if bash gazebo/deploy_host.sh >> "$CAMPAIGN_DIR/build_cr.log" 2>&1; then DEPLOYED=1; say "deploy ok: binary $OLD -> $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"; else say "DEPLOY FAILED (see build_cr.log) - the old binary stays"; fi
else say "BUILD FAILED (see build_cr.log) - the old binary stays"; fi
# Probe: one weave run with the dwell armed. It qualifies if the binary carries the knob at all,
# which the [recover] dwell line proves - but the weave only trips on some runs, so a probe that
# does not trip is INCONCLUSIVE, not a failure, and the blocks run either way.
wait_idle; tm_wait
COURSES="wkc_finals" ARMS="probe:CTRL_MAX_PLEG_Y=0.21,CTRL_LOCO_UNSAFE_HOLD_MS=600,CTRL_RECOVER_FOLD_VMAX=1.0" run dwell_probe 1 2.6
say "probe rows:"; cyc "$CAMPAIGN_DIR/dwell_probe.csv" | tee -a "$LOG"
A="stock:CTRL_MAX_PLEG_Y=0.21,CTRL_LOCO_UNSAFE_HOLD_MS=0,CTRL_RECOVER_FOLD_VMAX=-1 dwell:CTRL_MAX_PLEG_Y=0.21,CTRL_LOCO_UNSAFE_HOLD_MS=600,CTRL_RECOVER_FOLD_VMAX=-1 foldgate:CTRL_MAX_PLEG_Y=0.21,CTRL_LOCO_UNSAFE_HOLD_MS=0,CTRL_RECOVER_FOLD_VMAX=1.0"
say "arms (interleaved run by run, explicit on BOTH so neither inherits a default): $A"
say "deploy=$DEPLOYED - if 0, both arms ran on a binary without the knob and the block is a NULL by construction"
tier 0
i=0
while [ ! -f "$CAMPAIGN_DIR/STOP_CR" ]; do
  i=$((i+1)); wait_idle; sleep 30; wait_idle; tm_wait; say "block $i: rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
  COURSES="wkc_finals" ARMS="$A" run "dwell_cr$i" 3 2.6
  say "OPEN-40, three arms on wkc_finals 2.6: trigger PINNED at CTRL_MAX_PLEG_Y=0.21 on every arm"
  say "  (INDUCED - not an envelope number for 0.21). stock / dwell / foldgate; judge on FOLDED."
  say "  Pooled over CR; an arm with no trips measured nothing:"
  cyc "$CAMPAIGN_DIR"/dwell_cr*.csv | tee -a "$LOG"
  tier "$i"
done
say "chain CR done (STOP_CR seen after block $i)"
