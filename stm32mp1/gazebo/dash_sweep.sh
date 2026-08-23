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

# STALE-BINARY GUARD. This harness runs ./mit_ctrl_sim out of host-run/, which
# is NOT what `cmake --build` writes - so a rebuild followed by a sweep happily
# measures the previous binary and reports it as the new one. Same failure class
# as the SIM_CHEATER trap (RULE ZERO in SKILLS.md): the runner not reflecting the
# code. Copy the freshly built binary every time, and say which one is running.
BUILT=host-build/user/MIT_Controller/mit_ctrl_sim
if [ -f "$BUILT" ]; then
  # deploy_host.sh, never a bare cp: overwriting a Mach-O in place invalidates
  # its signature and macOS then SIGKILLs it at exec with zero output, which
  # looks exactly like the robot failing instantly on every run.
  bash "$G/deploy_host.sh" || { echo "[harness] deploy failed - refusing to sweep" >&2; exit 1; }
else
  echo "[harness] WARNING: $BUILT not found; running whatever is in $RUNDIR" >&2
fi
echo "[harness] binary: $(date -r "$RUNDIR/mit_ctrl_sim" '+%Y-%m-%d %H:%M:%S')"

# Which user-parameter yaml to run. The SOLVER lives in here, and the solver
# turned out to matter more than every gait parameter combined: JCQP at the
# shipped rho 0.6 / 60 iters commands only ~0.25x bodyweight where the gait
# needs 2.0x, so the robot walks permanently crouched at z=0.13 and sinks until
# it collapses. qpOASES on the same problem commands 1.3-1.8x and holds
# z=0.285 against a 0.300 reference. Keep this switchable and always record it.
USERYAML="${USERYAML:-mc-mit-ctrl-user-parameters.yaml}"
echo "[harness] user yaml: $USERYAML"

WORLD="${WORLD:-worlds/go1_speedway.sdf}"   # bare 400x400 ground, no scenery
TARGET="${TARGET:-100}"
OUT="${OUT:-/tmp/dash_$(date +%H%M%S)}"
mkdir -p "$OUT"

# This block used to carry seven more variables - SIM_ZEROVEL_HOLD_GAIT,
# SIM_HEADING_HOLD, SIM_MPC_ASYNC, SIM_WBC_DECIM, SIM_MPC_HORIZON, SIM_MPC_MS,
# SIM_SWING_H. Every one of them was DEAD: the SIM_ -> CTRL_ rename moved the
# getenv() calls and nothing was left reading those names, so the harness had
# been silently running compiled defaults while appearing to pin a
# configuration. Audited 2026-08-23 against `grep -rl '"SIM_..."' user/ robot/`.
#
# They are deleted rather than renamed, because the defaults ARE the tuned
# values and are better than what the block asked for:
#   zero-vel hold, heading hold, inline MPC   - now unconditional in the code
#   horizon 10, swing height 0.11             - already the defaults
#   MPC segment 26 ms                         - now PER-GAIT (trotRunning 26,
#                                               everything else 22), which is
#                                               the measured answer; a flat 26
#                                               would be a regression for
#                                               trotting. $CTRL_MPC_MS still
#                                               overrides if a sweep needs it.
#
# SIM_CHEATER is DELIBERATELY ABSENT and must stay that way. Setting it at all -
# to any value, including 0 - is a lie: results measured with sim ground truth
# fed into the estimator do not describe what the robot can do. Ground truth is
# for MEASURING (dash_trace.py reads Gazebo pose); it never enters the loop.
COMMON="SIM_VX_DELAY_S=4 SIM_VX_RAMP_S=12"

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
        "$USERYAML" > "$OUT/$LABEL.ctrl.log" 2>&1 ) &
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
