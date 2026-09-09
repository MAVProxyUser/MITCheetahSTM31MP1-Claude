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
. "$(dirname "${BASH_SOURCE[0]}")/paths.sh"   # persistent root - never /tmp

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

# THE SNAPSHOT MUST BE FROM THE RUN YOU JUST DID.
#
# shm segments outlive the process that created them (deliberately - see
# ShmTrace.h), so when a run aborts before the controller starts, BOTH
# $RUN_DIR/ctrl_0.log and the shm ring still hold the PREVIOUS run's contents.
# dump_snapshot returns them happily and the campaign records a complete,
# plausible, entirely fictitious data point.
#
# Measured: round 4 of wkc_settle_ab recorded run 3479 SIX times - three arms
# x two reps - all reporting the same fall at the same place, because its first
# six runs never started. Rounds 1-3 were clean, so this is not constant, which
# is exactly why it has to be checked every run rather than assumed.
#
# Every campaign should write this into its CSV and drop rows whose id repeats.
# WEAKER than campaign_launched_run_id: ctrl_0.log is itself stale when a
# launch is refused, so this agrees with a stale ring instead of catching it.
# Kept for recording the id in a CSV; use the launched id for the check.
campaign_run_id(){   # -> the controller run id behind the CURRENT ctrl_0.log
  grep -oE '\[RUNID\] run=[0-9]+' "$RUN_DIR/ctrl_0.log" 2>/dev/null |
    tail -1 | grep -oE '[0-9]+'
}

# A WEDGED FLEET EATS A CAMPAIGN SILENTLY.
#
# Measured on open28_subcourse: a gz sim process survived its run, the
# conductor stayed at phase "running", and every later launch was refused
# ("a fleet is already active - stop it first"). The runner waited out its
# gate and returned verdict NONE, so 7 of 24 rows were fictitious - the ring
# still held an older run, which only the run-id guard caught. The campaign
# reported a full sweep it had not performed.
#
# So: consecutive NONE verdicts are a HOST problem, not a result. Clear it
# once, and if that does not take, stop rather than keep writing rows.
# Call as: campaign_health_gate "$VERDICT"
CAMPAIGN_NONE_STREAK=0
campaign_health_gate(){   # $1 = this run's verdict
  if [ "${1:-NONE}" = "NONE" ]; then
    CAMPAIGN_NONE_STREAK=$((CAMPAIGN_NONE_STREAK+1))
  else
    CAMPAIGN_NONE_STREAK=0; return 0
  fi
  if [ "$CAMPAIGN_NONE_STREAK" -eq 2 ]; then
    echo "  [health] 2 consecutive NONE verdicts - clearing the fleet and carrying on"
    curl -s -m 15 -X POST http://127.0.0.1:8420/api/stop >/dev/null 2>&1
    sleep 10
    pkill -f "gz[ ]sim -s -r" 2>/dev/null
    sleep 5
  elif [ "$CAMPAIGN_NONE_STREAK" -ge 4 ]; then
    echo "  [health] 4 consecutive NONE verdicts after a clear - the host is not"
    echo "           running missions. Stopping rather than writing more fiction."
    return 1
  fi
  return 0
}

# THE ONLY RUN ID THAT CANNOT BE STALE.
#
# mission_runner prints "[runner] launched run N" to its OWN stdout, after the
# conductor has accepted the launch. That file is truncated per invocation, so
# unlike $RUN_DIR/ctrl_0.log (which a refused launch leaves holding the
# previous run) it cannot carry a previous run's number. If the launch was
# refused the line is simply absent, which is itself the answer.
#
# Pass the result to shm_reaper.dump_snapshot(expect_run_id=...) and it will
# refuse to archive a ring belonging to some other run.
campaign_launched_run_id(){   # $1 = the runner's stdout log
  grep -oE '\[runner\] launched run [0-9]+' "$1" 2>/dev/null |
    tail -1 | grep -oE '[0-9]+'
}

# A CSV ROW THAT SPLIT IN TWO IS SILENT CORRUPTION.
#
# `W=$(grep -c PATTERN FILE || echo 0)` looks defensive and is a landmine:
# grep -c EXITS 1 when the count is zero, so it prints "0" AND the fallback
# prints "0", and the variable becomes $'0\n0'. Every run that reached zero
# waypoints therefore wrote a two-line record. Found in five scripts and four
# campaigns; the orphan tail then parses as its own row with the course name
# in the waypoint column, which is how "collapsed 2/2, 100% fell" appeared in
# a summary table.
#
# Nothing flagged it. Call this at the end of a campaign so it does.
campaign_check_csv(){   # $1 = csv path
  local bad
  bad=$(awk -F, 'NR>1 && $1 !~ /^[0-9][0-9]:/' "$1" 2>/dev/null | wc -l | tr -d ' ')
  if [ "${bad:-0}" -gt 0 ]; then
    echo "  WARNING: $1 has $bad row(s) that do not start with a timestamp -"
    echo "           a field contained a newline and split the record. Repair"
    echo "           before trusting any count from this file."
    return 1
  fi
  return 0
}

# DO NOT START A CAMPAIGN ON TOP OF ANOTHER ONE.
#
# Measured: a yaw-cap sweep was still on its last rep when the next campaign
# launched. The old campaign's final row landed in the NEW campaign's csv, and
# its `.done` marker - written seconds later - terminated the new campaign's
# watcher immediately, which then reported a one-row result for a 90-run
# experiment. Neither campaign was wrong; the overlap was.
#
# Two things have to be true before a campaign starts: nothing else is running,
# and no marker is lying around from last time. Call this first.
campaign_claim(){   # $1 = campaign name
  local name="$1" other
  other=$(pgrep -fl "gazebo/tools/.*\.sh" 2>/dev/null |
          grep -v "campaign_lib" | grep -v "[[:space:]]$$[[:space:]]" |
          grep -vc "^$$ " || true)
  if pgrep -f "[m]ission_runner.py" >/dev/null 2>&1; then
    echo "  REFUSING to start $name: a mission is already running."
    echo "  Wait for it, or stop it - overlapping campaigns write into each"
    echo "  other's csv and each other's markers."
    return 1
  fi
  rm -f "$CAMPAIGN_DIR/$name.done" "$CAMPAIGN_DIR/$name.failed"
  return 0
}
