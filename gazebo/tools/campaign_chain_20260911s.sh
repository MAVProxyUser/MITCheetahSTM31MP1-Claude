#!/bin/bash
# Chain S (2026-09-11 11:15): the hairpin course's envelope TOP on the shipped
# slew recipe. Chain R put 2.3 at 5/5 (roll 13.4 deg) with the one 2.1 fall a
# 124 ms sensor freeze (harness), so the next rungs are 2.5 / 2.7, 5 reps
# interleaved, recipe as served. Runs after chain R (pid 98508) releases the rig.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
LOG="$CAMPAIGN_DIR/open31_chains.log"
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "mission_runner.py" >/dev/null && ! pgrep -f "^bash gazebo/tools/open28_subcourse.sh" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps at $v (COURSES=${COURSES:-} ARMS=${ARMS:-})"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
# chain R owns the rig until its own script exits; then the usual idle gate
while kill -0 98508 2>/dev/null && ps -p 98508 -o command= | grep -q "campaign_chain_20260911r.sh"; do sleep 20; done
wait_idle; sleep 30; wait_idle; say "chain S pid $$: rig idle (phase $(phase))"
COURSES=hp_gap20 ARMS="v25:SPEED=2.5 v27:SPEED=2.7" run hp_ladder_top 5 2.5
say "chain S done"
