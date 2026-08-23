#!/bin/bash
# sim_up_n.sh <instance> [world.sdf] - bring up ONE isolated SITL stack.
#
# Three things must be isolated for N dogs to share a Mac; missing any one
# produces cross-talk that looks exactly like a physics bug:
#
#   GZ_PARTITION   Gazebo transport is topic-based and global by default. Two
#                  servers without partitions see each other's topics, so
#                  bridge 1 can feed dog 0's IMU into dog 1's estimator.
#   UDP ports      the bridge<->controller link. $SIM_INSTANCE shifts the pair
#                  (0 -> 9100/9101, 1 -> 9110/9111, ...) on BOTH sides.
#   log + pid      per instance, or they interleave and cannot be torn down
#                  individually.
#
# Deliberately does NOT pkill anything. The single-instance script opens with
# `pkill -f "gz sim"`, which in a parallel world kills every sibling.
set -u
INST="${1:-0}"
DIR="$(cd "$(dirname "$0")" && pwd)"
WORLD="${2:-worlds/go1_speedway.sdf}"
PYBIN="/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/ground/gazebo_bridge/venv/bin/python3"
OPMODELS="/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/ground/gazebo_bridge/models"
export GZ_SIM_RESOURCE_PATH="$DIR/unitree_ros/robots:$DIR/models:$OPMODELS"
export PATH="/opt/homebrew/bin:$PATH"
export GZ_PARTITION="cheetah$INST"
export SIM_INSTANCE="$INST"
RUN=/tmp/cheetah_inst_$INST; mkdir -p "$RUN"

( cd "$DIR" && exec gz sim -s -r "$WORLD" > /tmp/gz_$INST.log 2>&1 ) &
echo $! > "$RUN/gz.pid"
for i in $(seq 1 25); do sleep 1; kill -0 "$(cat $RUN/gz.pid)" 2>/dev/null || break
  grep -q "Serving world" /tmp/gz_$INST.log 2>/dev/null && break; done
if ! kill -0 "$(cat $RUN/gz.pid)" 2>/dev/null; then
  echo "[inst $INST] SERVER FAILED:"; tail -3 /tmp/gz_$INST.log; exit 1
fi
( cd "$DIR" && exec env BRIDGE_CONV=mit "$PYBIN" -u cheetah_gazebo_bridge.py \
    > /tmp/bridge_$INST.log 2>&1 ) &
echo $! > "$RUN/bridge.pid"
sleep 2
echo "[inst $INST] up: partition=$GZ_PARTITION ports=$((9100+10*INST))/$((9101+10*INST)) pids=$(cat $RUN/gz.pid),$(cat $RUN/bridge.pid)"
