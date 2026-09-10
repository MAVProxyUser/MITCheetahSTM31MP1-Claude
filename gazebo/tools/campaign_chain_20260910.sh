#!/bin/bash
# The 2026-09-10 afternoon chain: enforce OPEN-31 (operational joint limits),
# A/B the contact gate (schedule-vs-reality in the estimator), and A/B two
# sim-fidelity gaps (foot friction, orientation noise) - all on hp_gap20 at
# 1.9 first, then wkc_finals and the dash. Written as a file rather than an
# inline bash -c so the sequence that produced the numbers is in git.
#
# Stages, each gated on the rig actually being free (RULE ONE: never idle,
# never overlap):
#   0. wait for the speed-ladder harness (pid $LADDER_PID) to EXIT, then patch
#      open28_subcourse.sh (TERRAIN= arm token, raw slot specs) - a running
#      bash script must never be edited in place;
#   1. wait for the fleet re-test marker (or its deadline) and an idle conductor;
#   2. deploy the OPEN-31 binary through deploy_host.sh (backup + restore on
#      failure) - deploy binds the sensor port, so only on an idle rig;
#   3. the campaigns, interleaved arms, ~4.5 h of rig time.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
LADDER_PID="${LADDER_PID:-82632}"
FLOG="$CAMPAIGN_DIR/fleet_dash_retest.log"
LOG="$CAMPAIGN_DIR/open31_chain.log"
FLEET_DEADLINE=$(date -j -f %H:%M 15:00 +%s 2>/dev/null || echo 0)
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){
  local ph
  while :; do
    ph=$(phase)
    if ! pgrep -f "[m]ission_runner.py" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi
    sleep 20
  done
}

# ---- stage 0: the ladder harness must be gone before its script is edited ----
say "chain pid $$: waiting for the ladder harness pid $LADDER_PID to exit"
while kill -0 "$LADDER_PID" 2>/dev/null; do sleep 30; done
say "ladder harness exited; patching open28_subcourse.sh (TERRAIN= token, raw slot specs)"
python3 - <<'PY'
p='gazebo/tools/open28_subcourse.sh'; s=open(p).read()
if 'TERRAIN=*' in s:
    print("harness already patched")
else:
    a='''  local v="$V" tok rest=""
  for tok in $env; do case "$tok" in SPEED=*) v="${tok#SPEED=}";; *) rest="${rest:+$rest }$tok";; esac; done'''
    b='''  local v="$V" terr="${TERRAIN:-flat}" tok rest=""
  # TERRAIN=<kind> inside an arm's env sets that arm's surface kind (flat,
  # concrete, grass, ...) so a friction A/B interleaves like a speed ladder.
  # Consumed here, not passed to the controller.
  for tok in $env; do case "$tok" in SPEED=*) v="${tok#SPEED=}";; TERRAIN=*) terr="${tok#TERRAIN=}";; *) rest="${rest:+$rest }$tok";; esac; done'''
    assert s.count(a)==1, "arm-token anchor"
    s=s.replace(a,b)
    a='''  timeout 300 python3 gazebo/conductor/mission_runner.py --terrain flat \\
    --slot "course:$crs" --gait trotting --speed "$v" --dash 0 \\'''
    b='''  # a COURSES entry with a colon is a raw slot spec (dash:100, star:10.514:5);
  # a bare name is a course file under gazebo/courses/.
  local slot="course:$crs"; case "$crs" in *:*) slot="$crs";; esac
  timeout 300 python3 gazebo/conductor/mission_runner.py --terrain "$terr" \\
    --slot "$slot" --gait trotting --speed "$v" --dash 0 \\'''
    assert s.count(a)==1, "launch-line anchor"
    s=s.replace(a,b)
    a='SNAP=$(dump_with_retry "${crs}r${rep}_${NAME}_${V_:-NONE}"'
    b='SNAP=$(dump_with_retry "${crs//:/_}r${rep}_${NAME}_${V_:-NONE}"'
    assert s.count(a)==1, "snapshot-tag anchor"
    s=s.replace(a,b)
    open(p,'w').write(s); print("harness patched")
PY
bash -n gazebo/tools/open28_subcourse.sh && say "harness syntax ok" || say "HARNESS SYNTAX ERROR - fix by hand"
( git add gazebo/tools/open28_subcourse.sh && git commit -q -m "Harness: per-arm TERRAIN= token and raw slot specs (dash:100) in COURSES; snapshot tags sanitised" && git push -q mp1repo HEAD:master && git push -q mp1repo HEAD:main && say "harness patch committed" ) || say "harness commit did not go through - commit by hand"

# ---- stage 1: the fleet re-test owns the rig until its marker appears ----
say "waiting for the fleet re-test marker (deadline 15:00)"
until grep -q "fleet re-test done" "$FLOG" 2>/dev/null; do
  [ "$FLEET_DEADLINE" -gt 0 ] && [ "$(date +%s)" -gt "$FLEET_DEADLINE" ] && { say "fleet deadline passed without a marker - proceeding when the rig is idle"; break; }
  sleep 60
done
wait_idle; sleep 30; wait_idle
say "rig idle (phase $(phase))"

# ---- stage 2: deploy the OPEN-31 binary ----
cp host-run/mit_ctrl_sim "$CAMPAIGN_DIR/mit_ctrl_sim.pre_open31"
NEWBIN=1
if bash gazebo/deploy_host.sh > "$CAMPAIGN_DIR/open31_deploy.log" 2>&1 && grep -q "deploy ok" "$CAMPAIGN_DIR/open31_deploy.log" \
   && strings host-run/mit_ctrl_sim | grep -q "joint limits: clamps"; then
  say "deployed the OPEN-31 binary: $(tail -1 "$CAMPAIGN_DIR/open31_deploy.log")"
else
  say "DEPLOY FAILED - restoring the previous binary and skipping the joint-limit campaigns"
  DEPLOY_SRC="$CAMPAIGN_DIR/mit_ctrl_sim.pre_open31" bash gazebo/deploy_host.sh >> "$CAMPAIGN_DIR/open31_deploy.log" 2>&1
  NEWBIN=0
fi

run(){   # run <campaign-name> <reps> <speed> ; env comes from the caller
  local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps at $v (COURSES=${COURSES:-} ARMS=${ARMS:-})"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}

# ---- stage 3: the campaigns ----
JL=""   # appended to every later arm: empty = the binary's default (limits ON)
if [ "$NEWBIN" = 1 ]; then
  COURSES=hp_gap20 DUMP=1 ARMS="jl0:CTRL_JOINT_LIMITS=0 jl1:CTRL_JOINT_LIMITS=1" run open31_jointlimits 8 1.9
  C="$CAMPAIGN_DIR/open31_jointlimits.csv"
  p0=$(awk -F, '$2=="jl0"&&$4=="PASS"' "$C" | wc -l | tr -d ' '); p1=$(awk -F, '$2=="jl1"&&$4=="PASS"' "$C" | wc -l | tr -d ' ')
  if [ $((p0 - p1)) -ge 2 ]; then JL=",CTRL_JOINT_LIMITS=0"; say "joint limits REGRESSED ($p1 vs $p0) - later campaigns run with them OFF"; else say "joint limits: no regression ($p1 vs $p0) - later campaigns run the default (ON)"; fi
fi
COURSES=hp_gap20 ARMS="cg0:SIM_CONTACT_GATE=0,SIM_ESTERR=1$JL cg1:SIM_CONTACT_GATE=1,SIM_ESTERR=1$JL" run item5_contactgate 8 1.9
COURSES="dash:100" ARMS="cg0:SIM_CONTACT_GATE=0,SIM_ESTERR=1$JL cg1:SIM_CONTACT_GATE=1,SIM_ESTERR=1$JL" run item5_contactgate_dash 6 3.0
COURSES=hp_gap20 ARMS="flat:TERRAIN=flat$JL concrete:TERRAIN=concrete$JL" run item6_friction 6 1.9
COURSES=hp_gap20 ARMS="on0:BRIDGE_ORI_NOISE_DEG=0$JL on05:BRIDGE_ORI_NOISE_DEG=0.5$JL" run item6_orinoise 6 1.9
if [ "$NEWBIN" = 1 ]; then
  COURSES=wkc_finals DUMP=1 ARMS="jl0:CTRL_JOINT_LIMITS=0 jl1:CTRL_JOINT_LIMITS=1" run open31_jointlimits_wkc 6 1.9
fi
COURSES=wkc_finals ARMS="cg0:SIM_CONTACT_GATE=0,SIM_ESTERR=1$JL cg1:SIM_CONTACT_GATE=1,SIM_ESTERR=1$JL" run item5_contactgate_wkc 6 1.9
COURSES="dash:100" ARMS="d31:SPEED=3.1$JL d32:SPEED=3.2$JL d35:SPEED=3.5$JL" run dash_ceiling 6 3.0
say "open31 chain done"
