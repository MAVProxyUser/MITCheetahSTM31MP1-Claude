#!/bin/bash
# Chain BS (2026-09-13 12:05): the third marginal feature of the honest course at
# 2.6 - the wp11->wp13 S-bend (-50 then +40 deg, taken at cruise). Its peak pitch is
# 20 deg in every served-recipe run (8 deg at 2.4) with the BODY at 2.84 m/s in the
# window against a 2.60 stick, and 3 of 123 runs fell there. The lateral budget does
# not move that peak at 2.6 (19.2 at 1.5), so the lever to test is the measured-speed
# cap WP_VCAP_GAIN (off by default): pull the stick down by gain x the body's overshoot
# so the S-bend is entered at ~2.65 instead of 2.84. Scored on the S-bend peak
# (gazebo/tools/window_peak.py 11 13), not on falls. Arms 0 / 1.0 / 2.0, 8 reps,
# wkc_finals 2.6 on the served recipe. After chain BR (pid 89035; STOP_BR touched).
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "gazebo/conductor/mission_[r]unner" >/dev/null && ! pgrep -f "gazebo/tools/open28_subcourse[.]sh" >/dev/null && ! pgrep -f "unittests/test_validated_[m]issions" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
tm_wait(){ local n=0; while tmutil status 2>/dev/null | grep -q "Running = 1"; do [ $n -eq 0 ] && say "Time Machine backup in progress - holding (not a run)"; n=$((n+1)); sleep 30; done; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps at $v (COURSES=${COURSES:-} ARMS=${ARMS:-})"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
PREV="${PREV_CHAIN_PID:-89035}"
LOG="$CAMPAIGN_DIR/open38_chainbs.log"
say "chain BS pid $$: waiting for chain BR (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912br[.]sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; tm_wait; say "rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8)"
COURSES=wkc_finals ARMS="vcap0: vcap1:WP_VCAP_GAIN=1.0 vcap2:WP_VCAP_GAIN=2.0" run wkc26_vcap 8 2.6
say "S-bend scores:"; python3 gazebo/tools/window_peak.py 11 13 "vcap0=wkc26_vcap:vcap0" "vcap1=wkc26_vcap:vcap1" "vcap2=wkc26_vcap:vcap2" | tee -a "$LOG"
say "chain BS done"
