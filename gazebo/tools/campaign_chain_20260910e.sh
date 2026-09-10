#!/bin/bash
# Chain E (2026-09-10 14:45): two more candidates for the solo 3.0 sprint
# regression (OPEN-34), from widening the audit window to the table's real
# date (2026-08-22, commit e07345e/72f6066): the x_comp_integral clamp
# (default 1.0 since 08-27; a NEGATIVE value restores stock MIT's unbounded
# integrator, which is what the table ran with) and the four foot-contact
# LABEL sensors added to the proto on 09-04. The sensor test edits the
# tracked proto in place and restores it with git afterwards - the conductor
# rebuilds the fleet world from the proto at every launch.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
PREV="${PREV_CHAIN_PID:-50883}"
LOG="$CAMPAIGN_DIR/open31_chaine.log"
PROTO=gazebo/worlds/go1_speedway.sdf
say(){ echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
wait_idle(){ local ph; while :; do ph=$(phase); if ! pgrep -f "[m]ission_runner.py" >/dev/null && ! pgrep -f "[o]pen28_subcourse.sh" >/dev/null && [ "$ph" != running ] && [ "$ph" != launching ]; then return 0; fi; sleep 15; done; }
run(){ local n="$1" r="$2" v="$3"
  say "=== campaign $n: $r reps at $v (COURSES=${COURSES:-} ARMS=${ARMS:-})"
  CAMPAIGN_NAME="$n" bash gazebo/tools/open28_subcourse.sh "$r" "$v" > "$CAMPAIGN_DIR/$n.chain.log" 2>&1
  say "=== campaign $n finished: $(tail -1 "$CAMPAIGN_DIR/$n.done" 2>/dev/null || echo 'no marker')"
  awk -F, 'NR>1{k[$2]++; if($4=="PASS")p[$2]++} END{for(a in k) printf "    %s %d/%d PASS\n", a, p[a]+0, k[a]}' "$CAMPAIGN_DIR/$n.csv" | tee -a "$LOG"
  sleep 20; wait_idle
}
say "chain E pid $$: waiting for chain D (pid $PREV) to exit"
while kill -0 "$PREV" 2>/dev/null; do sleep 30; done
wait_idle; sleep 20; wait_idle
say "rig idle (phase $(phase))"
COURSES="dash:100" ARMS="base:CTRL_JOINT_LIMITS=0 xdrag_stock:CTRL_JOINT_LIMITS=0,CTRL_XDRAG_CLAMP=-1" run dash30_xdrag 5 3.0
if git diff --quiet -- "$PROTO"; then
  python3 gazebo/tools/add_foot_contacts.py "$PROTO" --remove > "$CAMPAIGN_DIR/chaine_strip.log" 2>&1
  say "proto foot-contact sensors stripped: $(grep -c foot_contact "$PROTO") 'foot_contact' left (was 4 sensors)"
  COURSES="dash:100" ARMS="nosens:CTRL_JOINT_LIMITS=0" run dash30_nosensors 6 3.0
  git checkout -- "$PROTO" && say "proto restored: $(grep -c 'type=\"contact\"' "$PROTO") contact sensors"
else
  say "proto has uncommitted changes - NOT running the sensor-strip arm"
fi
say "chain E done"
