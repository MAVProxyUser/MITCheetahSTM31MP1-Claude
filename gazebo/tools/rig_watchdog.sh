#!/bin/bash
# rig_watchdog.sh - notice an IDLE RIG within minutes instead of hours.
#
# On 2026-09-03 a finished campaign wedged in its teardown at 11:08 and the
# rig sat idle until 17:15 - six hours - because the only thing watching was
# a `grep` on a pipe the wedged process still held open. Nothing was
# watching the RIG. This is.
#
# Every INTERVAL it asks one question: has the conductor's run_id advanced,
# or is a mission in flight? If neither has been true for STALE seconds, it
# writes a loud line to the alert log with WHAT was still alive, so the
# cause is in the record at the moment it happens rather than reconstructed
# six hours later.
#
#   bash gazebo/tools/rig_watchdog.sh [INTERVAL=300] [STALE=1800]
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.." || exit 1
INTERVAL="${1:-300}"; STALE="${2:-1800}"
BASE=http://127.0.0.1:8420
ALERT=/tmp/cheetah_rig_idle.log
last_run=""; last_change=$(date +%s)

while :; do
  now=$(date +%s)
  st=$(curl -s -m 6 "$BASE/api/state" 2>/dev/null)
  if [ -z "$st" ]; then
    echo "$(date '+%F %T') ALERT conductor not answering :8420" >> "$ALERT"
  else
    rid=$(printf '%s' "$st" | python3 -c "import json,sys;d=json.load(sys.stdin);print('%s|%s'%(d.get('run_id'),d.get('phase')))" 2>/dev/null)
    phase="${rid#*|}"
    if [ "$rid" != "$last_run" ]; then last_run="$rid"; last_change=$now; fi
    # BUSY IS A PROCESS QUESTION, NOT A VOCABULARY QUESTION.
    #
    # This used to say `[ "$phase" != "idle" ]`, which meant the watchdog
    # considered the rig busy whenever the phase was anything else - and the
    # conductor's resting phase after a completed run is "done", not "idle".
    # So it never fired. Found the only way it could be: the rig sat idle for
    # 33 minutes with a STALE of 1800 s and /tmp/cheetah_rig_idle.log had not
    # even been created. A watchdog that cannot alarm is worse than none,
    # because it is BELIEVED - I had cited this one as the reason the morning's
    # six-hour idle could not recur.
    #
    # Ask the operating system instead: is a mission process actually running?
    # That has no vocabulary to drift, and it is what "the rig is working"
    # means. The phase string is now only a hint, never the test.
    if pgrep -f 'mission_[r]unner.py|gz[ ]sim' >/dev/null 2>&1; then
      last_change=$now
    fi
    idle=$((now - last_change))
    if [ "$idle" -ge "$STALE" ]; then
      {
        echo "$(date '+%F %T') ALERT RIG IDLE ${idle}s (run_id/phase stuck at $rid)"
        echo "  campaign shells still alive:"
        ps -eo pid,ppid,etime,command 2>/dev/null | grep -E '/tmp/(c[0-9]+|chain_|ab_)' | grep -v grep | cut -c1-140 | sed 's/^/    /'
        echo "  campaign markers:"; ls -t /tmp/cheetah_campaigns/*.done 2>/dev/null | head -5 | sed 's/^/    /'
      } >> "$ALERT"
      last_change=$now   # re-arm so it reports every STALE, not every tick
    fi
  fi
  sleep "$INTERVAL"
done
