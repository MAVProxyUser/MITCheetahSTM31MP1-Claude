#!/bin/bash
# Chain Y (2026-09-11 15:25): the envelope the lead-1 default opens. On the
# schedule, wkc_finals died at 2.6 (0/5, chain P) and hp_gap20 at 2.5 was
# 3/5 (chain S); lead 1 carried the box at 2.6 5/5 and wkc 2.4 5/5. Ladders,
# interleaved, on the shipped yaml (lead 1 pinned):
#   wkc_finals 2.6 / 2.8, hp_gap20 2.5 / 2.7, 5 reps each rung.
# Runs after chain X (pid 78039, the fast suite).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-78039}"
LOG="$CAMPAIGN_DIR/open31_chainy.log"
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "^python3 gazebo/conductor/mission_runner.py" >/dev/null && ! pgrep -f "^bash gazebo/tools/open28_subcourse.sh" >/dev/null && ! pgrep -f "test_validated_missions.py" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps at $v (COURSES=${COURSES:-} ARMS=${ARMS:-})"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
say "chain Y pid $$: waiting for chain X (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260911x.sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; say "rig idle (phase $(phase)) - yaml lead: $(grep -m1 '^CTRL_MPC_SCHED_LEAD' host-run/ctrl_tuning.yaml | cut -c1-24)"
COURSES=wkc_finals ARMS="v26:SPEED=2.6 v28:SPEED=2.8" run wkc_lead1_top 5 2.6
COURSES=hp_gap20 ARMS="v25:SPEED=2.5 v27:SPEED=2.7" run hp_lead1_top 5 2.5
say "chain Y done"
