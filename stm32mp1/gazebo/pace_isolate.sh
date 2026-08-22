#!/bin/bash
# Why does pacing manage 13.71 m on the farm but 0.1 m on the speedway?
#
# Two candidate causes, and they need separating before either result is
# reported:
#   (a) MY OWN INSTRUMENTATION. The farm matrix ran on a binary with no collapse
#       detector; the dash runs on one that exits when body height stays under
#       0.15 m for 0.5 s. Pacing is a rocking gait - if it dips and recovers,
#       the detector kills a run the old binary would have let continue.
#   (b) the world, despite both carrying identical damping/friction/limits.
#
# Same gait, same speed, same binary in all three cells; only the detector and
# the world change.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
G=stm32mp1/gazebo
PYBIN="/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/ground/gazebo_bridge/venv/bin/python3"
export PATH="/opt/homebrew/bin:$PATH"
OUT=/tmp/pace_iso; mkdir -p $OUT

# SIM_CHEATER deliberately absent - never set it, not even to 0. See dash_sweep.sh.
COMMON="SIM_HEADING_HOLD=1 SIM_MPC_ASYNC=0 SIM_WBC_DECIM=1 \
SIM_MPC_HORIZON=10 SIM_MPC_MS=26 SIM_SWING_H=0.11 SIM_VX_DELAY_S=4 SIM_GAIT=8 SIM_VX=0.6"

run() {  # $1 label  $2 world  $3 extra-env
  pkill -9 -f "gz sim" 2>/dev/null; pkill -f cheetah_gazebo_bridge 2>/dev/null; sleep 2
  bash $G/sim_up.sh "$2" > $OUT/$1.sim.log 2>&1; sleep 2
  ( cd host-run && env DYLD_LIBRARY_PATH=. $COMMON $3 \
      timeout 80 ./mit_ctrl_sim 127.0.0.1 stm32mp1-defaults.yaml \
      mc-mit-ctrl-user-parameters.yaml > $OUT/$1.ctrl.log 2>&1 ) &
  local cp=$!
  sleep 4
  local r; r=$("$PYBIN" $G/dash_trace.py 70 100 2>/dev/null | tail -1)
  kill $cp 2>/dev/null; wait $cp 2>/dev/null; pkill -f mit_ctrl_sim 2>/dev/null
  local fall; fall=$(grep -c "\[FALL\]" $OUT/$1.ctrl.log 2>/dev/null || echo 0)
  printf '%-28s %s   FALL-lines=%s\n' "$1" "$r" "$fall"
}

echo "pacing @0.6, 70 s each - isolating detector from world"
run "speedway_detector_ON"  worlds/go1_speedway.sdf   "SIM_FALL_EXIT=1"
run "speedway_detector_OFF" worlds/go1_speedway.sdf   "SIM_FALL_EXIT=0"
run "farmflat_detector_OFF" worlds/go1_farm_flat.sdf  "SIM_FALL_EXIT=0"
pkill -9 -f "gz sim" 2>/dev/null; pkill -f cheetah_gazebo_bridge 2>/dev/null
echo "logs: $OUT"
