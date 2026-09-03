#!/bin/bash
# stop.sh - bring the SITL stack down SAFELY.
#
# Order is the whole point, and it is not obvious. `/api/stop` goes FIRST,
# because it routes through the conductor's own _reap_and_confirm(), which
# kills every child it spawned and CONFIRMS each one dead. Killing the
# server first instead leaves it unable to reap anything it owned: that is
# the unsafe restart CLAUDE.md documents, and it has already cost this
# project a real debugging session - an orphaned gz sim survived, its tail
# landed in the NEXT run's log, and three gaits were wrongly recorded as
# failing until the contamination was traced back here.
#
#   ./stop.sh            graceful: /api/stop, then the server, then strays
#   ./stop.sh --hard     also -9 anything matching the stack's patterns
set -u
PORT=8420
BASE="http://127.0.0.1:$PORT"
HARD=0; [ "${1:-}" = "--hard" ] && HARD=1

say(){ printf '  %s\n' "$*"; }
echo "== stopping the SITL stack =="

# 1. let the conductor reap its own children, and confirm it did
if curl -s -o /dev/null -m 5 "$BASE/api/state" 2>/dev/null; then
  say "conductor is up - POST /api/stop (reaps + confirms every child)"
  curl -s -m 120 -X POST "$BASE/api/stop" -o /tmp/.stop_resp 2>/dev/null
  say "  -> $(cat /tmp/.stop_resp 2>/dev/null | head -c 120)"
else
  say "conductor not answering - skipping /api/stop"
fi

# 2. now the server itself
PID=$(lsof -nP -iTCP:$PORT -sTCP:LISTEN -t 2>/dev/null | head -1)
if [ -n "${PID:-}" ]; then
  say "server pid $PID - TERM, then KILL if it lingers"
  kill "$PID" 2>/dev/null
  for _ in 1 2 3 4 5 6; do kill -0 "$PID" 2>/dev/null || break; sleep 1; done
  kill -9 "$PID" 2>/dev/null
else
  say "nothing listening on :$PORT"
fi

# 3. strays. Bracketed patterns so this never matches its own command line -
#    a pgrep that matches itself is the trap SKILL.md rule 3 exists for.
STRAY=$(pgrep -f 'gz[ ]sim|mit_ctrl[_]sim|cheetah[_]gazebo[_]bridge|conductor/[c]am_feed.py|conductor/[p]ose_feed.py' 2>/dev/null | grep -v "^$$\$" || true)
if [ -n "$STRAY" ]; then
  N=$(echo "$STRAY" | wc -l | tr -d ' ')
  say "$N stray process(es) left after the reap:"
  ps -o pid,etime,command -p $(echo "$STRAY" | tr '\n' ',' | sed 's/,$//') 2>/dev/null | tail -n +2 | cut -c1-110 | sed 's/^/      /'
  if [ "$HARD" = "1" ]; then echo "$STRAY" | xargs kill -9 2>/dev/null; say "killed (--hard)"
  else say "left alone - re-run with --hard to kill them"; fi
else
  say "no strays"
fi
echo "== down =="
