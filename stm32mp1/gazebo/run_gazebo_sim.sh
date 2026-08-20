#!/usr/bin/env bash
# Launch the Cheetah<->Go1 Gazebo SITL on this Mac: Gazebo (headless by default)
# + cheetah_gazebo_bridge.py, lifetimes tied. Then run the controller on the MP1:
#
#   ssh 192.168.0.90 'cd /usr/local/cheetah-mp1 && ./jpos_ctrl_sim <this-mac-ip>'
#
#   run_gazebo_sim.sh            # headless
#   run_gazebo_sim.sh --gui      # with the Gazebo GUI
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PATH="/opt/homebrew/bin:$PATH"
export GZ_SIM_RESOURCE_PATH="$HERE/unitree_ros/robots${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
WORLD="$HERE/worlds/go1.sdf"

# find a python with the gz bindings (system python3.14 lacks them)
PY=""
for c in python3.13 python3.12 \
    "/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/ground/gazebo_bridge/venv/bin/python3"; do
  if "$c" -c "import gz.transport13, gz.msgs10.imu_pb2" >/dev/null 2>&1; then PY="$c"; break; fi
done
[ -z "$PY" ] && { echo "no python with gz bindings found"; exit 1; }
echo "python: $PY"

GUI=""; [ "${1:-}" = "--gui" ] && GUI="-g"

pids=()
cleanup() { echo; echo "stopping..."; for p in "${pids[@]}"; do kill "$p" 2>/dev/null; done; wait 2>/dev/null; }
trap cleanup EXIT INT TERM

echo "== gazebo =="
if [ -n "$GUI" ]; then
  gz sim -v1 "$WORLD" & pids+=($!)      # server+GUI
else
  gz sim -s -r -v1 "$WORLD" & pids+=($!)  # headless server, running
fi
sleep 3

echo "== bridge =="
"$PY" -u "$HERE/cheetah_gazebo_bridge.py" & pids+=($!)

MACIP=$(ipconfig getifaddr en0 2>/dev/null || ipconfig getifaddr en1 2>/dev/null || echo "<mac-ip>")
echo
echo "Gazebo + bridge up. On the MP1:"
echo "  ssh 192.168.0.90 'cd /usr/local/cheetah-mp1 && ./jpos_ctrl_sim $MACIP'"
echo "Ctrl-C here stops both."
wait
