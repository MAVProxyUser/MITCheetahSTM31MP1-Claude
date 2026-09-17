#!/bin/bash
# Chain CW (2026-09-17): the OPEN-40 advisory at the SHIPPED trigger, in production conditions.
#
# WHY THIS AND NOT THE TIER I CANCELLED. Earlier today I cancelled a planned tier
# that would have run the advisory at the shipped CTRL_MAX_PLEG_Y, because I
# thought the trip rate was ~1 in 531 and 36 runs would see nothing. That number
# was a FALL count misread as a trip rate. Measured properly over 241 shipped-
# trigger runs by reading the ctrl logs:
#     22/241 = 9.1 %  graze 240 mm on >= 1 sample
#     17/241 = 7.1 %  actually TRIP the 5-tick debounce
#     11/241 = 4.6 %  go into a SUSTAINED cycle (>= 31 trip/recover pairs)
#      7/241 = 2.9 %  fall
# And chain CV, running right now on wkc_weave at 2.5-2.6, has produced THREE
# falls in 24 runs (12.5 %) - runs 9771, 9793, 9797 - and I classified all three
# from their own ctrl logs as OPEN-40, in BOTH arms. So on this course the natural
# trigger is frequent enough to measure without inducing anything.
#
# WHAT THIS CHAIN ADDS THAT CU DID NOT. CU pinned CTRL_MAX_PLEG_Y=0.21 so every
# run tripped, and got 78/78 against stock's 31/78. That is the mechanism proven,
# but the POPULATION is different: at 0.21 the guard fires early and often, in the
# body of the distribution; at the shipped 0.24 it fires only on the genuinely
# marginal excursions, i.e. the tail. The advisory's premise - "at cruise, staying
# in LOCOMOTION beats handing to RECOVERY_STAND" - is exactly the claim that could
# behave differently in the tail. This chain tests it there, on the trigger that
# will actually ship.
#
# CRITICAL: CTRL_MAX_PLEG_Y IS NOT SET ON EITHER ARM. The shipped default (0.24)
# must come from the code, not from this script - the whole point is the shipped
# value. Only the advisory knob differs between arms.
#
# NOTHING SHIPS. CTRL_LOCO_UNSAFE_ADVISORY_VMAX stays default -1; this measures it.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
DATA="${CHEETAH_DATA:-$HOME/Desktop/Cheetah/rundata}"
export DUMP=0
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "gazebo/conductor/mission_[r]unner" >/dev/null && ! pgrep -f "gazebo/tools/open28_subcourse[.]sh" >/dev/null && ! pgrep -f "unittests/test_validated_[m]issions" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
tm_wait(){ local n=0; while tmutil status 2>/dev/null | grep -q "Running = 1"; do [ $n -eq 0 ] && say "Time Machine backup in progress - holding (not a run)"; n=$((n+1)); sleep 30; done; }
# HARD DISK FLOOR, above the harness's own DISK_STOP_GB=8. Free space was 15 GB
# when this was written and the operator's restart (#6) is the lever that frees
# any, so this chain stops itself with room to spare rather than racing the guard.
FLOOR_GB="${FLOOR_GB:-12}"
disk_ok(){ local f; f=$(df -g /System/Volumes/Data | awk 'NR==2{print $4}'); if [ "${f:-0}" -lt "$FLOOR_GB" ]; then say "DISK FLOOR: ${f} GB free is below ${FLOOR_GB} GB - CW stops here, deliberately, without touching the harness guard"; return 1; fi; say "disk ${f} GB free (floor ${FLOOR_GB})"; return 0; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps at $v (COURSES=${COURSES:-} ARMS=${ARMS:-} DUMP=$DUMP)"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
tier(){ local t="$1"; tm_wait; say "$t: fast suite tier"
  python3 unittests/test_validated_missions.py --fast > "$CAMPAIGN_DIR/suite_fast_20260917_cw$t.log" 2>&1
  say "$t suite exit $?: $(grep -cE "^  PASS" "$CAMPAIGN_DIR/suite_fast_20260917_cw$t.log") PASS, $(grep -cE "^  FAIL" "$CAMPAIGN_DIR/suite_fast_20260917_cw$t.log") FAIL"
  sleep 20; wait_idle
}
# The endpoint that matters here is not only PASS/FAIL. OPEN-40's hazard is the
# SUSTAINED cycle - falls ran 31-107 trip/recover pairs and bled 155 mm of height,
# survivors ran 1-42 and bled 1 mm - so this also counts cycles per run straight
# out of each run's ctrl log, which is a continuous measure rather than a verdict.
cycles(){ python3 - "$CAMPAIGN_DIR" <<'PY' 2>/dev/null | tee -a "$LOG"
import csv,glob,os,re,sys,statistics as st
CD=sys.argv[1]; A=os.path.join(os.path.dirname(CD.rstrip('/')),"conductor","archive")
trip=re.compile(r"y-position is bad \((-?[\d.]+) m, max ([\d.]+), (\d+) ticks\)")
rb=re.compile(r"\[Recovery Balance\] body height is ([\d.]+)")
adv=re.compile(r"\[locoadv\]")
rows=[]
for f in glob.glob(os.path.join(CD,"advship_cw*.csv")):
    rows += list(csv.DictReader(open(f)))
by={}
for r in rows:
    run=r.get("run_id");
    if not run: continue
    fs=glob.glob(os.path.join(A,"*_run%s_ctrl_0.log"%run))
    if not fs: continue
    t=open(fs[0],errors="replace").read()
    tr=[abs(float(m.group(1)))*1000 for m in trip.finditer(t)]
    hs=[float(m.group(1)) for m in rb.finditer(t)]
    d=by.setdefault(r["course"],{"n":0,"trip":0,"cyc":[],"bled":[],"adv":0,"fall":0})
    d["n"]+=1
    if tr: d["trip"]+=1
    d["cyc"].append(len(hs))
    if hs: d["bled"].append((hs[0]-min(hs))*1000)
    if adv.search(t): d["adv"]+=1
    if r["verdict"]!="PASS": d["fall"]+=1
print("    OPEN-40 MECHANISM COUNTS from the ctrl logs (the continuous endpoint):")
print("    arm        n   tripped  [locoadv]  RecovStand cycles med/max   bled med mm   falls")
for a in sorted(by):
    d=by[a]
    print("    %-9s %3d  %5d    %7d    %11.0f /%5d   %9.0f    %4d" % (
        a,d["n"],d["trip"],d["adv"],st.median(d["cyc"]) if d["cyc"] else 0,
        max(d["cyc"]) if d["cyc"] else 0, st.median(d["bled"]) if d["bled"] else 0, d["fall"]))
print("    MANIPULATION: the advisory arm must show [locoadv] > 0 AND cycles ~0;")
print("    the stock arm must show [locoadv] = 0. If both are 0, no run tripped and the block is a null.")
PY
}
score(){ awk -F, '$1!="wall" && $7!=""{
    n[$2]++; s[$2]+=$7; ss[$2]+=$7*$7; if($7+0>mx[$2])mx[$2]=$7+0
    if($4=="PASS"){ p[$2]++; sp[$2]+=$7; ssp[$2]+=$7*$7; if($7+0>mxp[$2])mxp[$2]=$7+0
                    if($17!=""){lt[$2]+=$17; ln[$2]++} }
    if($21!=""&&$21+0>ly[$2])ly[$2]=$21+0 }
  END{ for(a in n){ m=s[a]/n[a]
      mp=(p[a]?sp[a]/p[a]:0); sdp=(p[a]>1)?sqrt((ssp[a]-p[a]*mp*mp)/(p[a]-1)):0
      printf "    %-9s %2d/%-2d PASS | PASSING worst %.1f -> margin %+.1f | sd %.2f | leg_y max %d mm | lap %.1f s\n",
        a, p[a]+0, n[a], mxp[a], 28.65-mxp[a], sdp, ly[a], (ln[a]?lt[a]/ln[a]:0) } }' "$@" 2>/dev/null | sort; }
PREV="${PREV_CHAIN_PID:-91216}"
PREV_PAT="${PREV_CHAIN_PAT:-campaign_chain_20260912[a-z][a-z][.]sh}"
LOG="$CAMPAIGN_DIR/open28_chaincw.log"
say "chain CW pid $$: waiting for the previous chain (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -qE "$PREV_PAT"; do sleep 30; done
say "predecessor $PREV gone - CW has the rig"
rm -f "$CAMPAIGN_DIR/STOP_CW"
wait_idle; sleep 30; wait_idle; tm_wait
say "no deploy needed - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8) already carries the knob, default off"
say "CTRL_MAX_PLEG_Y is deliberately UNSET on both arms: the shipped 0.24 must come from the code"
A="stock:CTRL_LOCO_UNSAFE_ADVISORY_VMAX=-1 advisory:CTRL_LOCO_UNSAFE_ADVISORY_VMAX=1.0"
say "arms (interleaved run by run): $A"
tier 0
i=0
while [ ! -f "$CAMPAIGN_DIR/STOP_CW" ]; do
  disk_ok || break
  i=$((i+1)); wait_idle; sleep 30; wait_idle; tm_wait; say "block $i: rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
  COURSES="wkc_weave" ARMS="$A" run "advship_cw$i" 6 2.6
  say "the advisory at the SHIPPED trigger, pooled over CW:"
  score "$CAMPAIGN_DIR"/advship_cw*.csv | tee -a "$LOG"
  cycles
  tier "$i"
done
say "chain CW done (STOP_CW or disk floor, after block $i)"
