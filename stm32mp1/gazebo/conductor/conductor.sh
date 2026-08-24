#!/bin/bash
# conductor.sh - start the Conductor control panel and open it.
#
# Does NOT rebuild mit_ctrl_sim. This is a launcher for the already-built,
# already-validated binary - if the code changed, deploy_host.sh and a fresh
# validation run come first, this comes after.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Refuse to start over stragglers from a previous run/sweep - the exact bug
# class documented in SKILLS.md (a bridge still holding a port, topics from a
# dead partition) would otherwise silently corrupt the next launch.
for pid in $(pgrep -f 'gz[ ]sim|mit_ctrl[_]sim|cheetah[_]gazebo[_]bridge|conductor/server.py' 2>/dev/null); do
  [ "$pid" = "$$" ] && continue
  kill -9 "$pid" 2>/dev/null
done
sleep 1

python3 "$HERE/server.py" &
SERVER_PID=$!
sleep 1
echo "Conductor: http://127.0.0.1:8420  (pid $SERVER_PID)"
command -v open >/dev/null && open "http://127.0.0.1:8420"
wait $SERVER_PID
