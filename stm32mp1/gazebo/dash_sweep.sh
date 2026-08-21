#!/bin/bash
# 100 m dash: for each gait, find the FASTEST commanded speed that covers the
# distance without going down.
#
# Ladder runs HIGH -> LOW and stops at the first speed that completes, so the
# answer per gait costs one run when the gait is good and a few when it is not.
#
# Ramp is deliberately gentle (SIM_VX_RAMP_S=12). Measured: trotting at 1.0 m/s
# fell after 3.62 m on a 3 s ramp but reached 0.92 m/s and ran 17.78 m on a 12 s
# one - a fast ramp masquerades as a speed ceiling, and the question here is
# what the GAIT can do.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
G=stm32mp1/gazebo
RUNDIR=host-run
PYBIN="/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/ground/gazebo_bridge/venv/bin/python3"
export PATH="/opt/homebrew/bin:$PATH"

WORLD="${WORLD:-worlds/go1_speedway.sdf}"   # bare 400x400 ground, no scenery
TARGET="${TARGET:-100}"
OUT="${OUT:-/tmp/dash_$(date +%H%M%S)}"
mkdir -p "$OUT"

# SIM_ZEROVEL_HOLD_GAIT: hold MIT's standing gait until there is a velocity to
# deliver. Without it a dynamic gait engages against a zero command and the
# flight-phase gaits collapse at engagement (measured: 0.00 m).
COMMON="SIM_ZEROVEL_HOLD_GAIT=1 SIM_HEADING_HOLD=1 SIM_MPC_ASYNC=0 SIM_CHEATER=1 \
SIM_WBC_DECIM=1 SIM_MPC_HORIZON=10 SIM_MPC_MS=26 SIM_SWING_H=0.11 \
SIM_VX_DELAY_S=4 SIM_VX_RAMP_S=12"

# gait:number:speed-ladder (high to low)
LADDERS="${LADDERS:-walk2:21:1.8,1.4,1.0 walk:20:1.8,1.4,1.0 trot:9:1.2,0.9,0.6 pace:8:1.0,0.8,0.6 trotrun:5:0.8,0.6,0.4 bound:1:1.4,1.0,0.6 pronk:2:1.0,0.6 gallop:22:1.0,0.6}"

printf '%-10s %8s %9s %9s %9s  %s\n' GAIT CMD_M/S T100_S V_FLY DIST_M RESULT
printf '%s\n' "---------------------------------------------------------------------"

for entry in $LADDERS; do
  gname="${entry%%:*}"; rest="${entry#*:}"
  gnum="${rest%%:*}"; speeds="${rest#*:}"

  for v in $(echo "$speeds" | tr ',' ' '); do
    LABEL="${gname}_v${v}"
    pkill -9 -f "gz sim" 2>/dev/null; pkill -f cheetah_gazebo_bridge 2>/dev/null
    sleep 2
    bash "$G/sim_up.sh" "$WORLD" > "$OUT/$LABEL.sim.log" 2>&1
    sleep 2

    ( cd "$RUNDIR" && env DYLD_LIBRARY_PATH=. $COMMON SIM_GAIT=$gnum SIM_VX=$v \
        timeout 260 ./mit_ctrl_sim 127.0.0.1 stm32mp1-defaults.yaml \
        mc-mit-ctrl-user-parameters.yaml > "$OUT/$LABEL.ctrl.log" 2>&1 ) &
    CPID=$!
    sleep 4
    R=$("$PYBIN" "$G/dash_trace.py" 250 "$TARGET" 2>/dev/null | tail -1)
    kill $CPID 2>/dev/null; wait $CPID 2>/dev/null
    pkill -f mit_ctrl_sim 2>/dev/null

    reached=$(echo "$R" | sed -n 's/.*reached=\([0-9]*\).*/\1/p')
    t100=$(echo "$R"    | sed -n 's/.*t100=\([0-9.]*\).*/\1/p')
    vfly=$(echo "$R"    | sed -n 's/.*v_fly=\([0-9.]*\).*/\1/p')
    dist=$(echo "$R"    | sed -n 's/.*maxdist=\([0-9.]*\).*/\1/p')
    : "${reached:=0}" "${t100:=0}" "${vfly:=0}" "${dist:=0}"

    if [ "$reached" = "1" ]; then
      printf '%-10s %8s %8ss %9s %9s  %s\n' "$gname" "$v" "$t100" "$vfly" "$dist" "CROSSED 100 m"
      break                      # fastest speed for this gait found
    else
      printf '%-10s %8s %9s %9s %9s  %s\n' "$gname" "$v" "-" "-" "$dist" "fell short"
    fi
  done
done

pkill -9 -f "gz sim" 2>/dev/null; pkill -f cheetah_gazebo_bridge 2>/dev/null
echo; echo "logs: $OUT"
