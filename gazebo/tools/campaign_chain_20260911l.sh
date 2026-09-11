#!/bin/bash
# Chain L (2026-09-11 06:20): the rig must not idle through the morning.
# Three follow-ups the night's closures named, each a real envelope question:
#   1. wkc_finals at 2.2 is a coin flip on the reversal's pitch trip;
#      WP_VSLEW (slew on the command's RISE out of the reversal) cut the
#      exit roll 14.5 -> 4.9 deg with a dose-response on hp_gap20 - does it
#      buy the 2.2 rung?  arms: off / 1.0 / 2.0 m/s^2, 5 reps.
#   2. OPEN-33: on concrete (mu 0.9 both sides) the hairpin at 1.9 sits
#      1.6 deg from the roll trip; a speed ladder ON concrete says where the
#      grippy-surface limit is.  arms: 1.7 / 1.9 / 2.1, 5 reps.
#   3. OPEN-33: orientation noise at 0.5 deg was a null; the DroneCAN/
#      Madgwick path is a larger dose.  arms: 0 / 1.5 / 3.0 deg, 5 reps.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
LOG="$CAMPAIGN_DIR/open31_chainl.log"
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
wait_idle; say "chain L pid $$: rig idle (phase $(phase))"
COURSES=wkc_finals ARMS="vs0:WP_VSLEW=0 vs1:WP_VSLEW=1.0 vs2:WP_VSLEW=2.0" run wkc22_vslew 5 2.2
COURSES=hp_gap20 ARMS="c17:TERRAIN=concrete,SPEED=1.7 c19:TERRAIN=concrete,SPEED=1.9 c21:TERRAIN=concrete,SPEED=2.1" run hp_concrete_ladder 5 1.9
COURSES=hp_gap20 ARMS="on0:BRIDGE_ORI_NOISE_DEG=0 on15:BRIDGE_ORI_NOISE_DEG=1.5 on30:BRIDGE_ORI_NOISE_DEG=3.0" run item6_orinoise_dose 5 1.9
say "chain L done"
