#!/bin/bash
# Chain B (2026-09-10 14:15): the first chain was stopped inside its
# item5_contactgate_dash campaign after the solo 100 m dash at 3.0 failed
# 3/3 on the freshly deployed OPEN-31 binary with the joint-limit clamp
# counters jumping to 600-900/s at cruise. That makes two things urgent
# before anything else spends the rig: (1) does the operational joint clamp
# break the sprint (limits off vs on, interleaved), and (2) if BOTH arms
# fail, did the sprint already fail on the previous binary (redeploy the
# backup, run it, redeploy the new one). Then the item 6 A/Bs, with the
# joint-limit flag set to whatever (1) says ships.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
LOG="$CAMPAIGN_DIR/open31_chainb.log"
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "[m]ission_runner.py" >/dev/null && ! pgrep -f "[o]pen28_subcourse.sh" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps at $v (COURSES=${COURSES:-} ARMS=${ARMS:-})"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
passes(){ awk -F, -v A="$2" '$2==A&&$4=="PASS"' "$CAMPAIGN_DIR/$1.csv" | wc -l | tr -d ' '; }
deploy(){ # $1 = source binary
  DEPLOY_SRC="$1" bash gazebo/deploy_host.sh > "$CAMPAIGN_DIR/chainb_deploy.log" 2>&1 && grep -q "deploy ok" "$CAMPAIGN_DIR/chainb_deploy.log"
}
say "chain B pid $$: waiting for the rig"
wait_idle; sleep 20; wait_idle
say "rig idle (phase $(phase))"

# (1) the sprint with the clamp off vs on, same binary, interleaved
COURSES="dash:100" DUMP=1 ARMS="jl0:CTRL_JOINT_LIMITS=0 jl1:CTRL_JOINT_LIMITS=1" run dash30_jointlimits 6 3.0
p0=$(passes dash30_jointlimits jl0); p1=$(passes dash30_jointlimits jl1)
JL=",CTRL_JOINT_LIMITS=0"; [ "$p1" -ge "$p0" ] && JL=""
say "dash 3.0: limits off $p0/6, on $p1/6 -> later campaigns run with '${JL:-default ON}'"

# (2) if the sprint fails without the clamp too, the previous binary answers whether it is a regression of the day
if [ "$p0" -le 3 ]; then
  OLD="$CAMPAIGN_DIR/mit_ctrl_sim.pre_open31"; NEW="$CAMPAIGN_DIR/mit_ctrl_sim.open31"
  cp host-run/mit_ctrl_sim "$NEW"
  if deploy "$OLD"; then
    say "previous (07:04) binary deployed for the regression check"
    COURSES="dash:100" ARMS="oldbin:CTRL_JOINT_LIMITS=0" run dash30_oldbin 6 3.0
    if deploy "$NEW"; then say "OPEN-31 binary redeployed"; else say "REDEPLOY OF THE OPEN-31 BINARY FAILED - host-run may hold the old one"; fi
  else
    say "old-binary deploy failed - skipping the regression check"; deploy "$NEW" || true
  fi
fi

# (3) the item 6 A/Bs, hp_gap20 at 1.9
COURSES=hp_gap20 ARMS="flat:TERRAIN=flat$JL concrete:TERRAIN=concrete$JL" run item6_friction 6 1.9
COURSES=hp_gap20 ARMS="on0:BRIDGE_ORI_NOISE_DEG=0$JL on05:BRIDGE_ORI_NOISE_DEG=0.5$JL" run item6_orinoise 6 1.9

# (4) if the sprint fails with the clamp off, bracket the OPEN-28 knobs that touch cruise
if [ "$p0" -le 3 ]; then
  COURSES="dash:100" ARMS="base:CTRL_JOINT_LIMITS=0 lead1:CTRL_JOINT_LIMITS=0,CTRL_MPC_SCHED_LEAD=1 lead3:CTRL_JOINT_LIMITS=0,CTRL_MPC_SCHED_LEAD=3 udp:CTRL_JOINT_LIMITS=0,GAZEBO_SOCK_DIR=" run dash30_bisect 5 3.0
fi
# (5) the wkc repeat of the joint-limit A/B, whichever way (1) went
COURSES=wkc_finals DUMP=1 ARMS="jl0:CTRL_JOINT_LIMITS=0 jl1:CTRL_JOINT_LIMITS=1" run open31_jointlimits_wkc 6 1.9
say "chain B done"
