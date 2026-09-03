#!/bin/bash
# run_mit_test.sh - one clean MIT_Controller SITL attempt with automatic scoring.
#   usage: run_mit_test.sh <label> <gait#> <extra-env...>
# Fresh gz world + bridge each run; loco always at 14 s; samples pose at 1 Hz
# for 40 s and reports survival time in LOCOMOTION + distance traveled.
set -u
LABEL="$1"; GAIT="$2"; shift 2
EXTRA_ENV="$*"

DIR="$(cd "$(dirname "$0")" && pwd)"
PYBIN="/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/ground/gazebo_bridge/venv/bin/python3"
OPMODELS="/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/ground/gazebo_bridge/models"
export GZ_SIM_RESOURCE_PATH="$DIR/unitree_ros/robots:$DIR/models:$OPMODELS"
export PATH="/opt/homebrew/bin:$PATH"
BOARD=192.168.0.90

ssh -o ConnectTimeout=10 $BOARD "sed -i 's/^cmpc_gait.*/cmpc_gait         : $GAIT/' /usr/local/cheetah-mp1/mc-mit-ctrl-user-parameters.yaml" 2>/dev/null

pkill -f "gz sim -s" 2>/dev/null; pkill -f cheetah_gazebo_bridge 2>/dev/null; sleep 2
cd "$DIR"
gz sim -s -r worlds/go1_farm.sdf > /tmp/gz.log 2>&1 &
sleep 4
BRIDGE_CONV=mit "$PYBIN" -u cheetah_gazebo_bridge.py > /tmp/bridge.log 2>&1 &
sleep 1

ssh -o ConnectTimeout=15 $BOARD "/sbin/ip route replace 192.168.0.75/32 dev eth0 2>/dev/null; cd /usr/local/cheetah-mp1; $EXTRA_ENV timeout 40 chrt -f 80 ./mit_ctrl_sim 192.168.0.75 stm32mp1-defaults.yaml mc-mit-ctrl-user-parameters.yaml >/tmp/mit.log 2>&1" &
SSH=$!

# sample once per second; note |roll|>35 => down
"$PYBIN" - > /tmp/score.log 2>/dev/null <<'PY' &
import gz.transport13 as t, math, time
from gz.msgs10.pose_v_pb2 import Pose_V
n=t.Node(); latest=[None]
def cb(m):
  for p in m.pose:
    if p.name=='go1':
      x,y,z,w=p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w
      roll=math.degrees(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)))
      pit=math.degrees(math.asin(max(-1,min(1,2*(w*y-z*x)))))
      latest[0]=(p.position.x,p.position.y,p.position.z,roll,pit)
n.subscribe(Pose_V,'/world/go1_world/dynamic_pose/info',cb)
t0=time.time()
while time.time()-t0 < 42:
    time.sleep(1.0)
    if latest[0]:
        x,y,z,r,pi=latest[0]
        print(f"{time.time()-t0:5.1f} {x:7.2f} {y:7.2f} {z:6.3f} {r:7.1f} {pi:7.1f}", flush=True)
PY
SCORE=$!
wait $SSH 2>/dev/null
sleep 1; kill $SCORE 2>/dev/null; wait $SCORE 2>/dev/null

# score: loco starts at 14 s (controller clock ~= our clock + ssh lag; detect from log)
python3 - "$LABEL" <<'PY'
import sys
rows=[]
for l in open('/tmp/score.log'):
    p=l.split()
    if len(p)==6: rows.append([float(v) for v in p])
label=sys.argv[1]
if not rows:
    print(f"[{label}] NO DATA"); sys.exit()
# find first standing (z>0.18), then first fall (|roll|>35) after it
stand=next((r for r in rows if r[3]>0.18 and abs(r[4])<20), None)
fall=None
if stand:
    fall=next((r for r in rows if r[0]>stand[0] and abs(r[4])>35), None)
end=rows[-1]
dist=((end[1]-rows[0][1])**2+(end[2]-rows[0][2])**2)**.5
up="FELL at t=%.0fs"%fall[0] if fall else "UPRIGHT at end"
print(f"[{label}] {up}; stood at t={'%.0f'%stand[0] if stand else 'never'}; end=(x={end[1]:.2f},y={end[2]:.2f},z={end[3]:.3f},roll={end[4]:.0f}); dist={dist:.2f}m")
PY