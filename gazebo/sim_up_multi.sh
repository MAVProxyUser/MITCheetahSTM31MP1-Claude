#!/bin/bash
# sim_up_multi.sh <n> [world_base.sdf] [mission_spec | spacing_m]
#
# Spacing defaults to being DERIVED FROM THE MISSION - pass the same spec the
# controllers will run (star:10.514:5, atom:9.0:6, oval:40:5.0) and the lanes
# are sized to that course plus a fall margin, instead of a constant that is
# either wasteful or unsafe depending on which course you happen to run.
# ONE physics engine, N dogs, one bridge per dog. Contrast sim_up_n.sh, which
# runs N separate engines - this pays for physics once.
set -u
N="${1:-3}"
DIR="$(cd "$(dirname "$0")" && pwd)"
BASE="${2:-worlds/go1_speedway.sdf}"
SPACING="${3:-star:10.514:5}"
PYBIN="/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/ground/gazebo_bridge/venv/bin/python3"
OPMODELS="/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/ground/gazebo_bridge/models"
export GZ_SIM_RESOURCE_PATH="$DIR/unitree_ros/robots:$DIR/models:$OPMODELS"
export PATH="/opt/homebrew/bin:$PATH"
OUT="worlds/$(basename "${BASE%.sdf}")_x${N}.sdf"
RUN=/tmp/cheetah_multi; mkdir -p "$RUN"
python3 "$DIR/make_multi_world.py" "$DIR/$BASE" "$DIR/$OUT" "$N" "$SPACING" || exit 1
( cd "$DIR" && exec gz sim -s -r "$OUT" > /tmp/gz_multi.log 2>&1 ) &
echo $! > "$RUN/gz.pid"
for i in $(seq 1 30); do sleep 1; grep -q "Serving world" /tmp/gz_multi.log 2>/dev/null && break; done
kill -0 "$(cat $RUN/gz.pid)" 2>/dev/null || { echo "SERVER FAILED:"; tail -3 /tmp/gz_multi.log; exit 1; }
# READINESS GATE, not a fixed sleep. A bigger world takes longer to load, and
# starting a controller before its dog's sensors are advertised gives it
# garbage: the estimator goes NON-FINITE on the first tick and the dog falls
# before it ever stands. That failed at N>=4 while N<=3 happened to win the
# race, which looks exactly like a scaling ceiling and is not one.
echo "[multi] waiting for all $N dogs to advertise sensors..."
for i in $(seq 0 $((N-1))); do
  ok=0
  for t in $(seq 1 60); do
    if gz topic -l 2>/dev/null | grep -q "^/go1_$i/imu$"; then ok=1; break; fi
    sleep 1
  done
  [ "$ok" = 1 ] || { echo "[multi] go1_$i never advertised /go1_$i/imu - aborting" >&2; exit 1; }
done
echo "[multi] all $N dogs publishing"
for i in $(seq 0 $((N-1))); do
  ( cd "$DIR" && exec env BRIDGE_CONV=mit SIM_INSTANCE=$i SIM_MODEL=go1_$i \
      "$PYBIN" -u cheetah_gazebo_bridge.py > /tmp/bridge_multi_$i.log 2>&1 ) &
  echo $! > "$RUN/bridge_$i.pid"
done
# And wait for each bridge to actually be receiving, not merely started.
for i in $(seq 0 $((N-1))); do
  for t in $(seq 1 30); do
    grep -q "subscribed" /tmp/bridge_multi_$i.log 2>/dev/null && break
    sleep 1
  done
done
# $TRAILS=1 draws planned-vs-flown tracks, one daemon per dog, each in its own
# marker namespace and hue, with its planned track offset to its own lane.
# $GUI=1 attaches the render client - verified not to change results.
if [ "${TRAILS:-0}" = "1" ]; then
  SP=$(python3 - "$SPACING" <<'PYEOF'
import sys
a=sys.argv[1]
try:
    kind,rest=a.split(":",1); f=[float(x) for x in rest.split(":")]
    w = 2*f[0] if kind in ("star","circle","atom") else (2*f[1] if kind=="oval" else 2.0)
    print("%.3f" % (w + max(15.0, 0.5*w)))
except ValueError:
    print(a)
PYEOF
)
  for i in $(seq 0 $((N-1))); do
    ( cd "$DIR" && exec env SIM_MODEL=go1_$i "$PYBIN" -u trail_daemon.py         "$SPACING" 900 $i "$(python3 -c "print($SP*$i)")"         > /tmp/trail_$i.log 2>&1 ) &
    echo $! > "$RUN/trail_$i.pid"
  done
fi
[ "${GUI:-0}" = "1" ] && { ( exec gz sim -g > /tmp/gzgui_multi.log 2>&1 ) & echo $! > "$RUN/gui.pid"; sleep 5; }
sleep 3
echo "one engine, $N dogs:"
for i in $(seq 0 $((N-1))); do
  printf '  go1_%d  ports %d/%d  %s\n' $i $((9100+10*i)) $((9101+10*i)) \
    "$(grep -m1 'subscribed' /tmp/bridge_multi_$i.log 2>/dev/null || echo 'bridge starting')"
done
