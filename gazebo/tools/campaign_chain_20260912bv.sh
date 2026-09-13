#!/bin/bash
# Chain BV (2026-09-13 14:35): the turn cap at 2.4, replicated where it acts and
# validated where it must be a no-op. Chain BU's A/B at wkc 2.6 on the served recipe
# (binary 5ed4a3ae): off 8/8 - S-bend (wp11->13) peak pitch median 20.0 deg, lap
# 96.3 s; WP_VTURN=2.4 8/8 - 8.4 deg, 96.9 s; WP_VTURN=2.2 7/8 - 6.9 deg, 100.0 s,
# the one fall a locomotionSafe leg-speed trip (FR foot 9 m/s -> RECOVERY_STAND) on
# the wp12 exit at run 6548. The wp03 corner window did not move (15.5/15.6/15.2).
# Before the cap can ship in the course recipe: (1) replicate off vs 2.4 at 2.6 -
# the +0.6 s and the 12 deg are one block so far; (2) the hairpin and the box at 2.6
# must be UNAFFECTED - every corner the cap could touch there is already braked
# harder by its fillet or its vertex stop, so any change is a bug; (3) exploration:
# with the S-bend capped, is 2.8 a rung (0/5, 3/5, 4/5 across budgets; failures at
# wp13 then wp15)? Runs after chain BU (pid 93991), strictly sequential, one dog.
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
score(){ local n="$1"
  say "$n S-bend (11->13):"; python3 gazebo/tools/window_peak.py 11 13 "off=$n:off" "vt24=$n:vt24" | tee -a "$LOG"
  say "$n wp03 corner (2->4):"; python3 gazebo/tools/window_peak.py 2 4 "off=$n:off" "vt24=$n:vt24" | tee -a "$LOG"
}
PREV="${PREV_CHAIN_PID:-93991}"
LOG="$CAMPAIGN_DIR/open38_chainbv.log"
say "chain BV pid $$: waiting for chain BU (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null && ps -p "$PREV" -o command= | grep -q "campaign_chain_20260912bu[.]sh"; do sleep 30; done
wait_idle; sleep 30; wait_idle; tm_wait
say "rig idle (phase $(phase)) - binary $(md5 -q host-run/mit_ctrl_sim | cut -c1-8), served recipe"
A="off: vt24:WP_VTURN=2.4"
COURSES=wkc_finals ARMS="$A" run wkc26_vturn2 8 2.6
score wkc26_vturn2
COURSES=hp_gap20 ARMS="$A" run hp26_vturn 6 2.6
COURSES=wkc_box ARMS="$A" run box26_vturn 6 2.6
COURSES=wkc_finals ARMS="$A" run wkc28_vturn 6 2.8
score wkc28_vturn
say "chain BV done"
