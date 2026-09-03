#!/bin/bash
# campaign_lib.sh - the completion protocol for long campaigns.
#
# WHY THIS EXISTS (2026-09-03, cost: 6h06m of idle rig):
# campaign c20 finished all 16 runs cleanly at 11:08:49 and wrote every row
# to its CSV. Four seconds later its EXIT trap restarted the conductor, and
# the shell parked in __wait4 on the server it had just spawned - `sample`
# on the live pid proved it. It never exited. Two consequences, and the
# second is the expensive one:
#
#   1. it held the write end of the pipe feeding its waiter
#      (`bash c19.sh | grep C19_DONE`), so the waiter never saw EOF;
#   2. the chain gated on that waiter, so no follow-on work was queued and
#      the rig sat idle for six hours with a FINISHED measurement stranded
#      in /tmp.
#
# The measurement was never at risk. The SIGNAL was. So:
#
#   * a campaign announces completion by WRITING A FILE, the instant its
#     data is complete and BEFORE any teardown runs;
#   * a waiter polls that FILE, never a pipe - a pipe stays open as long as
#     any process holds the write end, including a wedged one;
#   * every waiter carries a DEADLINE, so a wedge costs minutes, not hours.
#
# I could not reproduce the __wait4 wedge synthetically (my attempts were
# contaminated by leftovers from earlier attempts), so this file does not
# claim to fix bash's exit behaviour. It removes the DEPENDENCE on it.
set -u
CAMPAIGN_DIR="${CAMPAIGN_DIR:-/tmp/cheetah_campaigns}"
mkdir -p "$CAMPAIGN_DIR"

# campaign_done <name> [note...]   - call the moment the data is complete.
campaign_done(){
  local n="$1"; shift || true
  printf '%s %s\n' "$(date '+%F %T')" "${*:-done}" > "$CAMPAIGN_DIR/$n.done"
  echo "[campaign] $n DONE -> $CAMPAIGN_DIR/$n.done"
}

# campaign_failed <name> [why...]
campaign_failed(){
  local n="$1"; shift || true
  printf '%s FAILED %s\n' "$(date '+%F %T')" "${*:-}" > "$CAMPAIGN_DIR/$n.done"
  echo "[campaign] $n FAILED -> $CAMPAIGN_DIR/$n.done"
}

# wait_for_campaign <name> <deadline_seconds>
#   returns 0 when the marker appears, 1 on deadline. NEVER blocks forever.
wait_for_campaign(){
  local n="$1" deadline="${2:-7200}" waited=0
  while [ ! -f "$CAMPAIGN_DIR/$n.done" ]; do
    sleep 15; waited=$((waited+15))
    if [ "$waited" -ge "$deadline" ]; then
      echo "[campaign] WAITER DEADLINE: $n did not finish in ${deadline}s - NOT waiting longer" >&2
      return 1
    fi
  done
  echo "[campaign] $n signalled: $(cat "$CAMPAIGN_DIR/$n.done")"
  return 0
}
