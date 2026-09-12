#!/bin/bash
# Chain AH (2026-09-11 21:35): the shipped envelope at N=10. The trot-only
# lead 1 ships (20/20 full tier at 21:27); its two new rungs were measured at
# N=5 (wkc_finals 2.4 5/5, hp_gap20 2.5 5/5). Ten reps each, interleaved with
# the validated rung below as the control, recipe as served, the CSV's
# imu_gap_max_ms column saying which runs the host touched.
# Runs after chain AG (pid 72589).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-72589}"
LOG="$CAMPAIGN_DIR/open31_chainah.log"
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
say "chain AH pid $$: waiting for chain AG (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260911ag.sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; say "rig idle (phase $(phase)) - yaml trot lead: $(grep -m1 '^CTRL_MPC_LEAD_SLOW' host-run/ctrl_tuning.yaml | cut -c1-24)"
COURSES=wkc_finals ARMS="v24:SPEED=2.4 v22:SPEED=2.2" run wkc_ship_n10 10 2.4
COURSES=hp_gap20 ARMS="v25:SPEED=2.5 v23:SPEED=2.3" run hp_ship_n10 10 2.5
say "chain AH done"
