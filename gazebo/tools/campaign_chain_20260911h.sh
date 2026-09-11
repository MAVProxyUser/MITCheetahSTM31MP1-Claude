#!/bin/bash
# Chain H (2026-09-11 03:00): deploy the speed-scheduled table lead (OPEN-34)
# once chain G has the rig, then verify it against the pinned default on the
# three cells that decided it - the 3.0 sprint (where the schedule should pass
# like knob 1), wkc_finals at 2.2 and hp_gap20 at 1.9 (where it must behave
# like knob 2). Interleaved; the 'sched' arm sets no lead so the schedule runs,
# the 'fixed2' arm pins today's default.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-68675}"
LOG="$CAMPAIGN_DIR/open31_chainh.log"
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "^python3 gazebo/conductor/mission_runner.py" >/dev/null && ! pgrep -f "^bash gazebo/tools/open28_subcourse.sh" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps at $v (COURSES=${COURSES:-} ARMS=${ARMS:-})"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
say "chain H pid $$: waiting for chain G (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null; do sleep 30; done
wait_idle; sleep 20; wait_idle
say "rig idle (phase $(phase))"
cp host-run/mit_ctrl_sim "$CAMPAIGN_DIR/mit_ctrl_sim.pre_sched_lead"
if bash gazebo/deploy_host.sh > "$CAMPAIGN_DIR/chainh_deploy.log" 2>&1 && grep -q "deploy ok" "$CAMPAIGN_DIR/chainh_deploy.log" \
   && strings host-run/mit_ctrl_sim | grep -q "table lead %d -> %d adopted"; then
  say "deployed the scheduled-lead binary: $(tail -1 "$CAMPAIGN_DIR/chainh_deploy.log")"
else
  say "DEPLOY FAILED - restoring the previous binary"; DEPLOY_SRC="$CAMPAIGN_DIR/mit_ctrl_sim.pre_sched_lead" bash gazebo/deploy_host.sh >> "$CAMPAIGN_DIR/chainh_deploy.log" 2>&1; exit 1
fi
ARMS2="sched:CTRL_MPC_LEAD_SLOW=2 fixed2:CTRL_MPC_SCHED_LEAD=2"
COURSES="dash:100" ARMS="$ARMS2" run dash30_sched 6 3.0
COURSES=wkc_finals ARMS="$ARMS2" run wkc22_sched 5 2.2
COURSES=hp_gap20 ARMS="$ARMS2" run hp19_sched 5 1.9
say "chain H done"
