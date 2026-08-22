#!/bin/bash
# Second pass of the gait max-speed search. Reads the coarse 0.6/1.0 matrix and
# bisects once for every gait that SURVIVED the low speed and FELL at the high
# one - those are the only gaits whose ceiling is actually bracketed. Gaits that
# fell at 0.6 have no ceiling to find, and gaits upright at 1.0 get pushed up
# instead.
#
#   refine_maxspeed.sh <coarse-results-file> [seconds]
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
COARSE="${1:?usage: refine_maxspeed.sh <coarse-results> [secs]}"
SECS="${2:-48}"
# SIM_CHEATER deliberately absent - never set it, not even to 0. See dash_sweep.sh.
COMMON="SIM_HEADING_HOLD=1 SIM_MPC_ASYNC=0 SIM_WBC_DECIM=1 SIM_MPC_HORIZON=10 SIM_MPC_MS=26 SIM_SWING_H=0.11 SIM_VX_DELAY_S=4"

# NOTE: macOS ships bash 3.2, which has NO associative arrays (`declare -A`).
# Under `set -u` that fails instantly with "unbound variable". Plain list.
GAITS="trot:9 walk:20 walk2:21 trotrun:5 bound:1 pronk:2 gallop:22 pace:8"

CFG=$(mktemp /tmp/refine_XXXX.cfg)
for entry in $GAITS; do
  g="${entry%%:*}"; gnum="${entry##*:}"
  lo=$(grep -E "^${g}_v06 " "$COARSE" 2>/dev/null | grep -c "UPRIGHT to end")
  hi=$(grep -E "^${g}_v10 " "$COARSE" 2>/dev/null | grep -c "UPRIGHT to end")
  if [ "$lo" = "1" ] && [ "$hi" = "0" ]; then
    VS="0.8"                    # ceiling already bracketed between 0.6 and 1.0
  elif [ "$hi" = "1" ]; then
    VS="1.4 1.8"                # still going at 1.0 - push until it breaks, so
                                # the table reports a real ceiling rather than
                                # "the fastest thing we happened to try"
  else
    continue                    # never survived 0.6; nothing to bracket
  fi
  for v in $VS; do
    echo "${g}_v${v/./}  SIM_GAIT=$gnum $COMMON SIM_VX=$v" >> "$CFG"
  done
done

if [ ! -s "$CFG" ]; then echo "[refine] nothing bracketed - no second pass needed"; exit 0; fi
echo "[refine] second pass:"; cat "$CFG"
bash "$ROOT/stm32mp1/gazebo/host_sweep.sh" "$CFG" "$SECS"
