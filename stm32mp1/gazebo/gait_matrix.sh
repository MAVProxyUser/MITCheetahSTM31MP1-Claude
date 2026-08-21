#!/bin/bash
# gait_matrix.sh [SIM_VX] [SIM_MPC_MS] - run every MIT ConvexMPCLocomotion gait
# against the same world and score survival / travel / safety trips.
# Gait numbers are ConvexMPCLocomotion's own selector (see the if-chain in
# run()): 1 bounding, 2 pronking, 4 standing, 5 trotRunning, 8 pacing,
# 9 trotting (default), 20 walking, 21 walking2, 22 galloping (>=10 is MIT omni).
DIR="$(cd "$(dirname "$0")" && pwd)"; cd "$DIR"
BOARD=${BOARD:-192.168.0.90}; MAC=${MAC:-192.168.0.75}
VX=${1:-0.3}; MS=${2:-45}; RUN=${RUN_S:-30}
WORLD=${WORLD_FILE:-worlds/go1_farm_flat.sdf}
PYBIN="/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/ground/gazebo_bridge/venv/bin/python3"
printf "%-14s %-6s %9s %7s %7s %7s %8s %7s\n" gait num dist_m speed up_s zmax outcome unsafe
printf "%-14s %-6s %9s %7s %7s %7s %8s %7s\n" ------------- ----- --------- ------- ------- ------- -------- -------
for spec in "bounding:1" "pronking:2" "standing:4" "trotRunning:5" "pacing:8" "trotting:9" "walking:20" "walking2:21" "galloping:22"; do
  NAME=${spec%%:*}; G=${spec##*:}
  ssh -n -o ConnectTimeout=15 $BOARD "cd /usr/local/cheetah-mp1 && sed -i 's/^cmpc_gait.*/cmpc_gait         : $G/' mc-mit-ctrl-user-parameters.yaml" 2>/dev/null
  ./sim_up.sh "$WORLD" >/dev/null 2>&1
  ssh -n -o ConnectTimeout=20 $BOARD "cd /usr/local/cheetah-mp1; SIM_MPC_MS=$MS SIM_VX=$VX timeout $RUN chrt -f 80 ./mit_ctrl_sim $MAC stm32mp1-defaults.yaml mc-mit-ctrl-user-parameters.yaml >/tmp/gm_$G.log 2>&1" &
  sleep 9
  R=$(timeout $((RUN-12)) "$PYBIN" "$DIR/gait_score.py" $((RUN-16)) 2>/dev/null)
  U=$(ssh -n -o ConnectTimeout=15 $BOARD "grep -c Unsafe /tmp/gm_$G.log" 2>/dev/null)
  printf "%-14s %-6s %s %7s\n" "$NAME" "$G" "$R" "${U:-?}"
  wait 2>/dev/null
done
