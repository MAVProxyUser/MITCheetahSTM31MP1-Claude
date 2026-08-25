#!/bin/bash
# conductor.sh - start the Conductor control panel and open it.
#
# Does NOT rebuild mit_ctrl_sim. This is a launcher for the already-built,
# already-validated binary - if the code changed, deploy_host.sh and a fresh
# validation run come first, this comes after.
#
# Runs server.py under the venv Python (not system python3): it now subscribes
# to gz world pose itself (gz.transport13) to draw the fleet in-page rather
# than opening the native GUI, so it needs the same bindings trail_daemon.py
# always used.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYBIN="/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/ground/gazebo_bridge/venv/bin/python3"

# Refuse to start over stragglers from a previous run/sweep - the exact bug
# class documented in SKILL.md (a bridge still holding a port, topics from a
# dead partition) would otherwise silently corrupt the next launch.
for pid in $(pgrep -f 'gz[ ]sim|mit_ctrl[_]sim|cheetah[_]gazebo[_]bridge|conductor/server.py' 2>/dev/null); do
  [ "$pid" = "$$" ] && continue
  kill -9 "$pid" 2>/dev/null
done
sleep 1

"$PYBIN" "$HERE/server.py" &
SERVER_PID=$!
sleep 1
echo "Conductor: http://127.0.0.1:8420  (pid $SERVER_PID)"
command -v open >/dev/null && open "http://127.0.0.1:8420"
wait $SERVER_PID
