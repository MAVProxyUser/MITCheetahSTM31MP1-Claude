#!/bin/bash
# test_launcher_detaches.sh - guards against the c20 "six hours idle" failure.
#
# WHAT HAPPENED (2026-09-03): campaign c20 finished all 16 runs and wrote
# every CSV row at 11:08:49. Its EXIT trap then restarted the conductor and
# the shell parked in __wait4 on that server - confirmed by `sample` on the
# live pid, which showed the leaf frame. It never exited, so it never closed
# the pipe its waiter was reading, so the chain never advanced. The rig sat
# idle for 6h06 with a finished measurement stranded in /tmp.
#
# HONESTY NOTE: I could not reproduce that wedge synthetically - the repros
# that appeared to hang were contaminated by leftover children of earlier
# repro attempts. So this test does NOT assert bash's exit behaviour. It
# asserts the three structural properties that make the wedge survivable,
# each of which was violated on the day:
#
#   1. campaigns signal completion by FILE, written before any teardown;
#   2. waiters poll that file and carry a DEADLINE (never a bare pipe);
#   3. nothing but start.sh launches the conductor.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; ROOT="$(cd "$HERE/.." && pwd)"
FAIL=0; ok(){ printf '  ok   %s\n' "$*"; }; bad(){ printf '  FAIL %s\n' "$*"; FAIL=1; }
echo "== campaign completion protocol =="

export CAMPAIGN_DIR="$(mktemp -d)"; trap 'rm -rf "$CAMPAIGN_DIR"' EXIT
. "$ROOT/gazebo/tools/campaign_lib.sh"

# 1. the marker is a file, and it appears immediately
campaign_done unit_t1 "16/16 rows" >/dev/null
[ -f "$CAMPAIGN_DIR/unit_t1.done" ] && ok "campaign_done writes a marker file" \
                                    || bad "campaign_done wrote no marker"

# 2. a waiter returns on the marker...
t0=$SECONDS; wait_for_campaign unit_t1 60 >/dev/null; rc=$?
[ $rc = 0 ] && [ $((SECONDS-t0)) -lt 20 ] && ok "waiter returns on an existing marker" \
                                          || bad "waiter did not return on marker (rc=$rc)"

# 3. ...and gives up on its deadline instead of hanging forever
t0=$SECONDS; wait_for_campaign unit_never 16 >/dev/null 2>&1; rc=$?
el=$((SECONDS-t0))
[ $rc = 1 ] && [ $el -lt 60 ] && ok "waiter honours its deadline (${el}s, rc=1)" \
                              || bad "waiter did NOT stop at its deadline (${el}s, rc=$rc) - this is the 6-hour bug"

# 4. no campaign script may launch the conductor itself
echo "== only start.sh launches the conductor =="
STRAY=$(grep -rln 'nohup.*server\.py' "$ROOT/gazebo" 2>/dev/null | grep -v 'start\.sh' || true)
[ -z "$STRAY" ] && ok "start.sh is the only launcher under gazebo/" \
                || bad "these launch server.py directly instead of via start.sh:
$STRAY"
L="$(grep -n 'server\.py' "$ROOT/gazebo/start.sh" | grep nohup || true)"
case "$L" in *'</dev/null'*) ok "start.sh launch keeps the child off inherited stdin" ;;
                          *) bad "start.sh launch lost </dev/null" ;; esac
echo
[ "$FAIL" = 0 ] && echo "PASS" || echo "FAIL"; exit $FAIL
