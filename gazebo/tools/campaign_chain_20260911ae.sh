#!/bin/bash
# Chain AE (2026-09-11 18:25): can the bridge ride out a sensor-stream gap?
# OPEN-35's third class - gz-transport's loopback holding the IMU/joint
# topics 20-55 ms under host load - cannot be scheduled away from user space
# (the sim is not ours to put on the real-time band), so the bridge now has
# BRIDGE_EXTRAP=1: dead-reckon the orientation with the last body rate and the
# joints with their last velocities across a gap, up to 80 ms. Whether that
# is a rescue is measured with a SYNTHETIC hold on a quiet host
# (BRIDGE_GAP_INJECT_MS=30, once a second - the class's own shape), on the
# gait that proved most sensitive (walking: expsquare) and on the trot with
# continuous curvature (spiro). Three arms, interleaved, recipe gaits:
#   clean   - nothing injected, no extrapolation (the baseline)
#   gap30   - 30 ms hold every second, frozen state (today's failure mode)
#   gap30x  - the same hold, dead-reckoned across
# Runs after chain AD (pid 40929).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-40929}"
LOG="$CAMPAIGN_DIR/open31_chainae.log"
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "^python3 gazebo/conductor/mission_runner.py" >/dev/null && ! pgrep -f "^bash gazebo/tools/open28_subcourse.sh" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps (COURSES=${COURSES:-} ARMS=${ARMS:-} RECIPE_GAIT=${RECIPE_GAIT:-0})"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
say "chain AE pid $$: waiting for chain AD (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260911ad.sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; say "rig idle (phase $(phase))"
export RECIPE_GAIT=1
G="BRIDGE_GAP_INJECT_MS=30"
COURSES="expsquare:5:12" ARMS="clean:BRIDGE_EXTRAP=0 gap30:$G,BRIDGE_EXTRAP=0 gap30x:$G,BRIDGE_EXTRAP=1" run expsquare_gap 3 1.5
COURSES="spiro:9.0:8" ARMS="clean:BRIDGE_EXTRAP=0 gap30:$G,BRIDGE_EXTRAP=0 gap30x:$G,BRIDGE_EXTRAP=1" run spiro_gap 3 1.8
say "chain AE done"
