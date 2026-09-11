#!/bin/bash
# Chain AG (2026-09-11 20:00): the gap experiment at the dose that actually
# fell today. Chain AE's 30 ms hold once a second left expsquare passing on
# both arms with the same peaks (frozen 13.8/10.2 deg, dead-reckoned
# 13.2/11.5, clean 8.8/4.9 - rep 1); the host's real gaps were 21-56 ms and
# the six-minute walk (lissajous) was the case that died. So: 50 ms holds,
# expsquare (3 reps) and lissajous 11:9 (2 reps), clean / frozen /
# dead-reckoned, interleaved, recipe gaits. Runs after chain AF (pid 69359).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-69359}"
LOG="$CAMPAIGN_DIR/open31_chainag.log"
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "^python3 gazebo/conductor/mission_runner.py" >/dev/null && ! pgrep -f "^bash gazebo/tools/open28_subcourse.sh" >/dev/null && ! pgrep -f "test_validated_missions.py" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps (COURSES=${COURSES:-} ARMS=${ARMS:-} RECIPE_GAIT=${RECIPE_GAIT:-0})"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
say "chain AG pid $$: waiting for chain AF (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260911af.sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; say "rig idle (phase $(phase))"
export RECIPE_GAIT=1
G="BRIDGE_GAP_INJECT_MS=50"
COURSES="expsquare:5:12" ARMS="clean:BRIDGE_EXTRAP=0 gap50:$G,BRIDGE_EXTRAP=0 gap50x:$G,BRIDGE_EXTRAP=1" run expsquare_gap50 3 1.5
COURSES="lissajous:15:11:9" ARMS="clean:BRIDGE_EXTRAP=0 gap50:$G,BRIDGE_EXTRAP=0 gap50x:$G,BRIDGE_EXTRAP=1" run lissajous_gap50 2 1.5
say "chain AG done"
