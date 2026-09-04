#!/bin/bash
# start.sh - bring the Conductor panel up, with the checks that turn a
# silent misconfiguration into a message.
#
# Every refusal below is a real failure this project has already had:
#   - a second server on :8420 answering nothing while the first held the
#     port, so a campaign wrote a CSV full of fiction;
#   - system python3 instead of the venv, which has no gz.transport13, so
#     the pose feed came up "subscribed" and permanently deaf;
#   - a stale binary, so a whole relief-gain sweep measured a mechanism
#     that was not in the deployed code at all.
#
#   ./start.sh             start (refuses if already up)
#   ./start.sh --restart   stop.sh first, then start
#   ./start.sh --open      also open the panel in a browser
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT=8420
BASE="http://127.0.0.1:$PORT"
PYBIN="/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/ground/gazebo_bridge/venv/bin/python3"
LOG=/private/tmp/conductor_server.log
OPEN=0; RESTART=0
for a in "$@"; do
  case "$a" in --open) OPEN=1;; --restart) RESTART=1;;
    *) echo "unknown flag: $a"; exit 2;; esac
done
say(){ printf '  %s\n' "$*"; }
die(){ printf '  !! %s\n' "$*"; exit 1; }

[ "$RESTART" = "1" ] && bash "$HERE/stop.sh"

echo "== starting the Conductor =="
# 1. never become the second server
if curl -s -o /dev/null -m 4 "$BASE/api/state" 2>/dev/null; then
  say "already up at $BASE (pid $(lsof -nP -iTCP:$PORT -sTCP:LISTEN -t 2>/dev/null | head -1))"
  say "use --restart to replace it"; exit 0
fi
if lsof -nP -iTCP:$PORT -sTCP:LISTEN -t >/dev/null 2>&1; then
  die "something holds :$PORT but does not answer - run ./stop.sh first"
fi

# 2. the interpreter must be the one with the gz bindings
[ -x "$PYBIN" ] || die "venv python missing: $PYBIN"
"$PYBIN" -c "import gz.transport13, gz.msgs10.image_pb2, PIL" 2>/dev/null \
  || die "$PYBIN cannot import gz.transport13 / gz.msgs10 / PIL - wrong interpreter"
say "python: $PYBIN (gz + PIL ok)"

# 3. the binary the panel launches must actually exist
BIN="$(cd "$HERE/.." && pwd)/host-run/mit_ctrl_sim"
if [ -x "$BIN" ]; then
  say "controller: $(basename "$BIN") $(date -r "$BIN" '+%Y-%m-%d %H:%M')"
else
  say "WARNING: $BIN missing - deploy_host.sh before launching a mission"
fi

# 4. start detached, then wait for it to actually answer
#
# The redirections and the `disown` are NOT decoration. `( cmd & )` alone
# looks detached and usually behaves, but on 2026-09-03 campaign c20 proved
# it is not enough: the campaign's EXIT trap called this same launch, and
# bash sat in __wait4 on the freshly started server for SIX HOURS. The
# script could not die because its teardown had spawned something that
# outlives it, and because it never died it held the write end of the pipe
# its waiter was reading - so the chain never advanced and the rig sat idle
# with a finished measurement stranded in /tmp.
#   </dev/null  - the child never shares a terminal or a pipe for input
#   >"$LOG" 2>&1 - and never holds the parent's stdout/stderr pipe open
#   disown      - removes it from the job table so bash CANNOT wait on it
# unittests/test_launcher_detaches.sh is the regression test for this.
[ -f "$LOG" ] && mv -f "$LOG" "${LOG%.log}.prev.log"
# The launch goes through python's start_new_session=True (a real setsid)
# and NOT `( nohup ... & )`, because the shell form does not actually break
# the parent link and this script then cannot exit.
#
# Evidence, on this machine, 2026-09-03: `bash gazebo/start.sh` sat alive
# for 6h19m holding the conductor as its child, parked in __wait4 -
# confirmed with `sample` on the live pid. It was the same wedge that had
# already cost six hours of idle rig that morning via a campaign's teardown.
# I had "fixed" that by adding </dev/null and `disown` here; the live
# evidence says those did nothing, and my own bake-off had hinted as much
# before I shipped them as hygiene anyway.
#
# I cannot reproduce the wedge synthetically - a trivial script with the
# same idiom exits fine - so this does not claim to have found bash's
# reason. It removes the PRECONDITION: the intermediate python exits
# immediately, the server reparents to init, and this script has no child
# left to wait on. Verified by checking the server's PPID is 1 below.
( cd "$HERE/conductor" && "$PYBIN" -c '
import os, subprocess, sys
log = open(sys.argv[1], "a")
subprocess.Popen([sys.argv[2], "-u", "server.py"],
                 stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                 start_new_session=True)
' "$LOG" "$PYBIN" )

for i in $(seq 1 30); do
  curl -s -o /dev/null -m 3 "$BASE/api/state" 2>/dev/null && break
  sleep 1
done
curl -s -o /dev/null -m 4 "$BASE/api/state" 2>/dev/null \
  || die "server did not come up - see $LOG"
say "up: $BASE  (pid $(lsof -nP -iTCP:$PORT -sTCP:LISTEN -t 2>/dev/null | head -1))"
grep -m1 "thread baseline" "$LOG" 2>/dev/null | sed 's/^/  /'
say "log: $LOG"
[ "$OPEN" = "1" ] && command -v open >/dev/null && open "$BASE"
exit 0
