#!/bin/bash
# run_queue.sh - keep the rig working through a list of jobs, unattended,
# without any way to wedge.
#
# WHY THIS EXISTS. Over 2026-09-03/04 this rig lost roughly EIGHT HOURS to
# harnesses that stopped without saying so:
#   * a finished campaign parked in __wait4 inside its own teardown, held the
#     pipe its waiter was reading, and the chain never advanced (6h06m);
#   * start.sh did the same thing and sat for 6h19m holding the conductor;
#   * the watchdog written to catch the first one could not fire, because it
#     treated the conductor's resting phase ("done") as busy;
#   * and when the watchdog was fixed and DID fire, it wrote to a log file
#     that nothing read - so the rig still sat idle 87 minutes.
#
# The lesson is not "be careful". It is that every waiting construct needs a
# DEADLINE and every detection needs an ACTION. So:
#
#   * every job runs under `timeout` - a job CANNOT run forever, whatever it
#     does internally;
#   * the queue never reads a pipe and never waits on a marker; it waits on
#     its own child, which timeout guarantees will die;
#   * a failed or timed-out job does not stop the queue - it is logged and
#     the next job starts, because an unattended rig that stops at the first
#     error wastes the night;
#   * before each job the conductor is health-checked and restarted if it is
#     not answering;
#   * progress goes to a log with timestamps AND to a marker per job, so a
#     human (or I) can see where it got to at a glance.
#
#   bash gazebo/tools/run_queue.sh QUEUEFILE [PER_JOB_TIMEOUT_S]
#
# QUEUEFILE: one job per line, "<name> <command...>". Blank lines and #
# comments ignored.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.." || exit 1
QF="${1:?usage: run_queue.sh QUEUEFILE [PER_JOB_TIMEOUT_S]}"
JOB_TIMEOUT="${2:-5400}"
. "$(dirname "${BASH_SOURCE[0]}")/paths.sh"
LOG="$LOG_DIR/queue.log"
STATE="$LOG_DIR/queue.state"
BASE=http://127.0.0.1:8420

say(){ printf '%s %s\n' "$(date '+%F %T')" "$*" | tee -a "$LOG"; }

ensure_conductor(){
  curl -s -o /dev/null -m 6 "$BASE/api/state" 2>/dev/null && return 0
  say "  conductor not answering - restarting"
  timeout 90 bash gazebo/stop.sh >/dev/null 2>&1
  timeout 120 bash gazebo/start.sh >/dev/null 2>&1
  for i in $(seq 1 20); do
    curl -s -o /dev/null -m 5 "$BASE/api/state" 2>/dev/null && { say "  conductor back"; return 0; }
    sleep 3
  done
  say "  CONDUCTOR WILL NOT START - jobs will likely fail, continuing anyway"
  return 1
}

say "== queue start: $QF (per-job cap ${JOB_TIMEOUT}s) =="
n=0
while IFS= read -r line; do
  case "$line" in ''|\#*) continue ;; esac
  name="${line%% *}"; cmd="${line#* }"
  n=$((n+1))
  echo "$name" > "$STATE"
  ensure_conductor
  say "-- job $n: $name"
  t0=$SECONDS
  timeout "$JOB_TIMEOUT" bash -c "$cmd" >> "$LOG" 2>&1
  rc=$?
  el=$((SECONDS - t0))
  case $rc in
    0)   say "-- job $n: $name OK (${el}s)" ;;
    124) say "-- job $n: $name TIMED OUT at ${JOB_TIMEOUT}s - killed, moving on" ;;
    *)   say "-- job $n: $name exited $rc (${el}s) - moving on" ;;
  esac
  printf '%s %s rc=%s %ss\n' "$(date '+%F %T')" "$name" "$rc" "$el" >> "$LOG_DIR/queue.results"
done < "$QF"
say "== queue done: $n jobs =="
echo "QUEUE_IDLE" > "$STATE"
