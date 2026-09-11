#!/bin/bash
# Chain F2 (2026-09-10 23:50), replaces F before it started. Two independent
# knobs fix the solo 3.0 sprint in chains B and C: the contact-table lead at
# knob 1 (5/5, peaks 8-11 deg, aiding on) and velocity aiding OFF (4/5, peaks
# ~5 deg, lead at the default). Both changed after the 08-22 table (aiding
# went default-on 08-28; the lead rework shipped 09-10), so the shipping
# decision needs each knob's COST where the defaults were chosen: hp_gap20 at
# 1.9 (the lead default's home) and wkc_finals at 2.2 (the course's edge,
# 7/8 pitch trips on the reversal this morning). Four arms, interleaved.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-51307}"
LOG="$CAMPAIGN_DIR/open31_chainf.log"
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
say "chain F2 pid $$: waiting for chain E (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null; do sleep 30; done
wait_idle; sleep 20; wait_idle
say "rig idle (phase $(phase))"
ARMS4="dflt:CTRL_MPC_SCHED_LEAD=2 lead1:CTRL_MPC_SCHED_LEAD=1 noaid:SIM_VEL_AIDING=0 both:CTRL_MPC_SCHED_LEAD=1,SIM_VEL_AIDING=0"
COURSES=hp_gap20 ARMS="$ARMS4" run hp19_lead 6 1.9
COURSES=wkc_finals ARMS="$ARMS4" run wkc22_lead 5 2.2
COURSES="dash:100" ARMS="$ARMS4" run dash30_final 5 3.0
say "chain F2 done"
