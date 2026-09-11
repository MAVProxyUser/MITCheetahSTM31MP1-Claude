#!/bin/bash
# Chain G (2026-09-11 02:45): tonight's default (knob 2, fixed table path)
# passed wkc_finals at 2.2 5/5; this morning's ladder, on the old alias path
# (CTRL_MPC_TABLE_ALIAS=1 + knob 1, recorded as "bit-for-bit the same
# physical lead"), went 1/8 at the same speed under an 86 % Spotlight load.
# Interleave the two on a quiet host: if ship fails and default passes, the
# alias path was never the same lead and the ladder's limit was measured on
# a configuration that no longer ships; if both pass, the morning's limit
# was the host.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-22812}"
LOG="$CAMPAIGN_DIR/open31_chaing.log"
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
say "chain G pid $$: waiting for chain F2 (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null; do sleep 30; done
wait_idle; sleep 20; wait_idle
say "rig idle (phase $(phase))"
COURSES=wkc_finals ARMS="dflt:CTRL_MPC_SCHED_LEAD=2 ship:CTRL_MPC_SCHED_LEAD=1,CTRL_MPC_TABLE_ALIAS=1" run wkc22_ship 5 2.2
say "chain G done"
