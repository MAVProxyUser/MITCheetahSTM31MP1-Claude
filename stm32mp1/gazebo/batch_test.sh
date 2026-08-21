#!/bin/bash
# batch_test.sh - run many SITL gait configs back-to-back against ONE gz server.
#
# The slow part of iterating used to be tearing down and reloading Gazebo (~10 s
# per attempt). `reset: {all: true}` on /world/<w>/control restores the spawn
# pose in ~1 s and the HEADLESS server survives it (it only crashes when a GUI
# client is attached - run the GUI separately for demos, not for batches).
# NOTE: `reset: {model_only: true}` returns data:true but is a NO-OP - it does
# not move the robot, which silently invalidates every run after the first.
#
#   batch_test.sh <configfile>
#
# Each config line:  <label> | <binary> | <env assignments>
# ('#' comments and blank lines ignored).  Example:
#   crawl-fast | static_gait_sim | SG_VX=0.25 SG_T=0.9
#   trot-27    | mit_ctrl_sim    | SIM_VX=0 SIM_BODY_H=0.21
#
# Results table: survival, end pose, distance, mean speed.
set -u
CFG="${1:?usage: batch_test.sh <configfile>}"
RUN_S="${RUN_S:-40}"        # seconds per attempt
DIR="$(cd "$(dirname "$0")" && pwd)"
PYBIN="/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/ground/gazebo_bridge/venv/bin/python3"
OPMODELS="/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/ground/gazebo_bridge/models"
export GZ_SIM_RESOURCE_PATH="$DIR/unitree_ros/robots:$DIR/models:$OPMODELS"
export PATH="/opt/homebrew/bin:$PATH"
BOARD=${BOARD:-192.168.0.90}
HOSTIP=${HOSTIP:-192.168.0.75}
WORLD=go1_world
WORLD_FILE="${WORLD_FILE:-worlds/go1_farm.sdf}"   # worlds/go1_speedway.sdf for top-speed runs

start_stack() {
  pkill -f "gz sim -s" 2>/dev/null; pkill -f cheetah_gazebo_bridge 2>/dev/null; sleep 2
  ( cd "$DIR" && gz sim -s -r "$WORLD_FILE" > /tmp/gz.log 2>&1 & )
  sleep 5
  ( cd "$DIR" && BRIDGE_CONV=mit "$PYBIN" -u cheetah_gazebo_bridge.py > /tmp/bridge.log 2>&1 & )
  sleep 1
}

# Reset, then VERIFY. `reset: {all}` is fast but occasionally aborts the server,
# and a silently-failed reset makes every later run start from the previous
# run's crash pose - which invalidated a whole sweep before this check existed.
reset_model() {
  gz service -s /world/$WORLD/control --reqtype gz.msgs.WorldControl \
     --reptype gz.msgs.Boolean --timeout 3000 --req 'reset: {all: true}' >/dev/null 2>&1
  sleep 3
  local ok
  ok=$("$PYBIN" - <<'PYEOF' 2>/dev/null
import gz.transport13 as t, time, math
from gz.msgs10.pose_v_pb2 import Pose_V
n=t.Node(); r=[]
def cb(m):
  for p in m.pose:
    if p.name=='go1':
      x,y,z,w=p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w
      roll=math.degrees(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)))
      r.append((p.position.x,p.position.y,roll))
n.subscribe(Pose_V,'/world/go1_world/dynamic_pose/info',cb)
time.sleep(1.0)
print("OK" if r and abs(r[-1][0])<0.4 and abs(r[-1][1])<0.4 and abs(r[-1][2])<25 else "BAD")
PYEOF
)
  if [ "$ok" != "OK" ]; then
    echo "   (reset failed or server gone - reloading world)" >&2
    start_stack
    sleep 2
  fi
}

start_stack
echo "label                | outcome        | end x     y     z    roll | dist  | mean v | dyaw"
echo "---------------------+----------------+--------------------------+-------+--------+-----"

while IFS= read -r line; do
  case "$line" in ''|\#*) continue ;; esac
  LABEL=$(echo "$line" | cut -d'|' -f1 | xargs)
  BIN=$(echo "$line"   | cut -d'|' -f2 | xargs)
  ENVS=$(echo "$line"  | cut -d'|' -f3- | xargs)

  # server still alive? (a bad config can wedge it)
  pgrep -f "gz sim -s" >/dev/null || start_stack
  reset_model

  # -n: never read our stdin, or ssh swallows the rest of the config file
  ssh -n -o ConnectTimeout=15 $BOARD "/sbin/ip route replace $HOSTIP/32 dev eth0 2>/dev/null; \
      cd /usr/local/cheetah-mp1; $ENVS timeout $RUN_S chrt -f 80 ./$BIN $HOSTIP \
      stm32mp1-defaults.yaml mc-mit-ctrl-user-parameters.yaml >/tmp/run.log 2>&1" >/dev/null 2>&1 &
  SSHPID=$!

  "$PYBIN" - "$RUN_S" > /tmp/score.log 2>/dev/null <<'PY' &
import gz.transport13 as t, math, time, sys
from gz.msgs10.pose_v_pb2 import Pose_V
dur=float(sys.argv[1])+2
n=t.Node(); latest=[None]
def cb(m):
  for p in m.pose:
    if p.name=='go1':
      x,y,z,w=p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w
      roll=math.degrees(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)))
      pit=math.degrees(math.asin(max(-1,min(1,2*(w*y-z*x)))))
      yaw=math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
      latest[0]=(p.position.x,p.position.y,p.position.z,roll,pit,yaw)
n.subscribe(Pose_V,'/world/go1_world/dynamic_pose/info',cb)
t0=time.time()
while time.time()-t0 < dur:
    time.sleep(0.5)
    if latest[0]:
        x,y,z,r,pi,ya=latest[0]
        print(f"{time.time()-t0:6.2f} {x:8.3f} {y:8.3f} {z:6.3f} {r:8.2f} {pi:8.2f} {ya:8.2f}", flush=True)
PY
  SCOREPID=$!
  wait $SSHPID 2>/dev/null
  sleep 1; kill $SCOREPID 2>/dev/null; wait $SCOREPID 2>/dev/null

  python3 - "$LABEL" <<'PY'
import sys
rows=[]
for l in open('/tmp/score.log'):
    p=l.split()
    if len(p)==7: rows.append([float(v) for v in p])
label=sys.argv[1]
if not rows:
    print(f"{label:20s} | NO DATA"); raise SystemExit
DOWN=35.0
# The controller exits at the end of the run and the robot flops down; that
# collapse is not a gait failure, so drop the tail before judging.
if rows and rows[-1][0] > 3.0:
    tcut = rows[-1][0] - 2.5
    rows = [r for r in rows if r[0] <= tcut]
if not rows:
    print(f"{label:20s} | NO DATA"); raise SystemExit
stood=next((r for r in rows if r[3]>0.17 and abs(r[4])<DOWN and abs(r[5])<DOWN), None)
fell=None
if stood:
    fell=next((r for r in rows if r[0]>stood[0] and (abs(r[4])>DOWN or abs(r[5])>DOWN)), None)
end=rows[-1]
upright_end = abs(end[4])<DOWN and abs(end[5])<DOWN and end[3]>0.15
if stood and not fell and upright_end: outcome="UPRIGHT"
elif stood and fell:                   outcome=f"fell t={fell[0]:.0f}s"
elif not stood:                        outcome="never stood"
else:                                  outcome="down at end"
# distance measured from the moment it stood (ignore spawn settle)
base = stood if stood else rows[0]
# stop measuring at the fall, if there was one
useful = [r for r in rows if r[0] >= base[0] and (fell is None or r[0] <= fell[0])]
span = (useful[-1][0]-useful[0][0]) if len(useful) > 1 else 1e-9
if span <= 0: span = 1e-9
# PATH LENGTH, not straight-line: a run that veers still covers ground, and top
# speed is about ground covered per second, not displacement from the start.
dist = 0.0
for a,b in zip(useful, useful[1:]):
    dist += ((b[1]-a[1])**2 + (b[2]-a[2])**2)**0.5
end = useful[-1]
dyaw = end[6]-base[6]
while dyaw>180: dyaw-=360
while dyaw<-180: dyaw+=360
print(f"{label:20s} | {outcome:14s} | {end[1]:6.2f} {end[2]:6.2f} {end[3]:5.3f} {end[4]:6.1f} "
      f"| {dist:5.2f} | {dist/span:6.3f} | {dyaw:+5.0f}")
PY
done < "$CFG"


echo "(stack left running; pkill -f 'gz sim' when done)"
