#!/bin/bash
# Chain O (2026-09-11 08:50): the course recipe now carries WP_VSLEW=1.0
# (458be53); RECIPES are read at server start, so restart the conductor the
# SAFE way (conductor_ctl: /api/stop first, then kill, then relaunch) once
# chain N is done, and prove the recipe carries it: an arm that sets NOTHING
# (the recipe as shipped) against an explicit WP_VSLEW=0 override, at wkc 2.2
# and hp_gap20 1.9. The recipe arm's ctrl log must show [VSLEW] binding.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-57629}"
LOG="$CAMPAIGN_DIR/open31_chaino.log"
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
say "chain O pid $$: waiting for chain N (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null; do sleep 30; done
wait_idle; sleep 30; wait_idle
say "rig idle (phase $(phase)) - restarting the conductor to load the recipe"
python3 -c "import sys; sys.path.insert(0,'gazebo/conductor'); import conductor_ctl; conductor_ctl.restart_server('course recipe gained WP_VSLEW=1.0')" >> "$LOG" 2>&1
for i in $(seq 1 60); do sleep 2; ph=$(phase); [ -n "$ph" ] && break; done
say "conductor back (phase ${ph:-none}); recipe extra now: $(curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("recipes",{}).get("course",{}).get("extra"))' 2>/dev/null)"
COURSES=wkc_finals ARMS="recipe: off:WP_VSLEW=0" run wkc22_recipe 4 2.2
COURSES=hp_gap20 ARMS="recipe: off:WP_VSLEW=0" run hp19_recipe 4 1.9
say "chain O done"
