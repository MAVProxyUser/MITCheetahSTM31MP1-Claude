#!/bin/bash
# sim_up.sh [world.sdf] [--gui] - bring up the SITL stack with the resource path
# set. Forgetting GZ_SIM_RESOURCE_PATH makes gz fail to resolve the Go1 meshes
# and exit with "Failed to load a world", which looks exactly like a crash.
DIR="$(cd "$(dirname "$0")" && pwd)"
WORLD="${1:-worlds/go1_farm.sdf}"
PYBIN="/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/ground/gazebo_bridge/venv/bin/python3"
OPMODELS="/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/ground/gazebo_bridge/models"
export GZ_SIM_RESOURCE_PATH="$DIR/unitree_ros/robots:$DIR/models:$OPMODELS"
export PATH="/opt/homebrew/bin:$PATH"
pkill -f "gz sim" 2>/dev/null; pkill -f cheetah_gazebo_bridge 2>/dev/null; sleep 2
( cd "$DIR" && gz sim -s -r "$WORLD" > /tmp/gz.log 2>&1 & )
sleep 6
if ! pgrep -f "gz sim -s" >/dev/null; then echo "SERVER FAILED:"; tail -3 /tmp/gz.log; exit 1; fi
[ "$2" = "--gui" ] && { nohup gz sim -g > /tmp/gzgui.log 2>&1 & sleep 5; }
# Pin board->Mac traffic to ethernet EVERY stack-up. The board's eth0 and
# wlan0 sit on the same subnet, and after a reboot the route pin is gone: the
# controller's first packet then leaves via wlan0, the bridge learns the WIFI
# address as its peer, and the entire sensor stream runs over the lossy path -
# frozen state estimates, phantom launches, and hours of untraceable variance
# (this happened; the bridge log showed peer=192.168.0.14). Reboots must not
# be able to reintroduce it silently.
BOARD=${BOARD:-192.168.0.90}
ssh -n -o ConnectTimeout=8 $BOARD "/sbin/ip route replace ${MACIP:-192.168.0.75}/32 dev eth0" 2>/dev/null   && echo "board route pinned to eth0" || echo "WARNING: could not pin board route (board down?)"
( cd "$DIR" && BRIDGE_CONV=mit "$PYBIN" -u cheetah_gazebo_bridge.py > /tmp/bridge.log 2>&1 & )
sleep 2
echo "stack up: world=$WORLD  server=$(pgrep -f 'gz sim -s' | head -1)  bridge=$(pgrep -f cheetah_gazebo_bridge | head -1)"
