#!/bin/bash
# run_valid.sh "<env vars>" <run_seconds> - one controller run with TRANSPORT
# VALIDITY GATING. The board's eth0 PHY flaps (dmesg: repeated "Link is Down"/
# "Link is Up"), and a run that overlaps a flap dies in ways indistinguishable
# from a controller bug: commands stop, the bridge watchdog folds the robot,
# and hours went into diagnosing "engage collapses" that were a dying link.
# A run only counts if eth0 stayed up for its entire duration.
BOARD=${BOARD:-192.168.0.90}; MAC=${MAC:-192.168.0.75}
ENVS="$1"; RUN=${2:-40}
CARRIER0=$(ssh -n -o ConnectTimeout=8 $BOARD 'cat /sys/class/net/eth0/carrier; dmesg | grep -c "eth0: Link"' 2>/dev/null)
[ -z "$CARRIER0" ] && { echo "INVALID (board unreachable before run)"; exit 2; }
set -- $CARRIER0; UP0=$1; FLAPS0=$2
[ "$UP0" != "1" ] && { echo "INVALID (eth0 down before run)"; exit 2; }
ssh -n -o ConnectTimeout=15 $BOARD "cd /usr/local/cheetah-mp1; $ENVS timeout $RUN chrt -f 80 ./mit_ctrl_sim $MAC stm32mp1-defaults.yaml mc-mit-ctrl-user-parameters.yaml >/tmp/run.log 2>&1"
AFTER=$(ssh -n -o ConnectTimeout=8 $BOARD 'dmesg | grep -c "eth0: Link"' 2>/dev/null)
if [ -z "$AFTER" ] || [ "$AFTER" != "$FLAPS0" ]; then
  echo "INVALID (eth0 flapped during run: $FLAPS0 -> ${AFTER:-unreachable})"; exit 2
fi
echo "VALID"
