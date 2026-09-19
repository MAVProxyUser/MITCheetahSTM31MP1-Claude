#!/bin/bash
# cull_mediaanalysisd.sh - keep mediaanalysisd dead. The operator cannot stop it
# any other way; `com.apple.mediaanalysisd` is ALREADY `=> disabled` in this
# user's launchd disabled list and something else still launches it (OPEN-35
# decision #7). Each instance dies having used ~0.01 s of CPU, so this holds it at
# effectively zero cost - but left unwatched it is not free: between 2026-09-18
# and 09-19, with no culler running, one instance accumulated 424 CPU-MINUTES and
# sat at 19 % sustained while the rig's IMU delivery degraded from 11.5 ms to
# 33.2 ms median gap.
#
# It is a committed tool rather than a shell loop because a shell loop dies with
# the session, which is exactly how those 424 minutes happened.
#
# Kills ONLY a process whose argv ends in /mediaanalysisd. It deliberately does
# NOT touch mediaanalysisd-access.xpc, which is a different binary.
set -u
INTERVAL="${INTERVAL:-2}"
LOG="${CULL_LOG:-${CHEETAH_DATA:-$HOME/Desktop/Cheetah/rundata}/campaigns/cull_mediaanalysisd.log}"
kills=0; started=$(date +%s)
echo "$(date '+%m-%d %H:%M:%S') culler up (pid $$), interval ${INTERVAL}s" >> "$LOG"
while :; do
  for p in $(pgrep -x mediaanalysisd 2>/dev/null); do
    a=$(ps -o args= -p "$p" 2>/dev/null)
    case "$a" in
      */mediaanalysisd) kill -9 "$p" 2>/dev/null && kills=$((kills+1)) ;;
    esac
  done
  # a heartbeat every ~30 min so the log shows the rate without spamming per kill
  now=$(date +%s)
  if [ $(( (now - started) % 1800 )) -lt "$INTERVAL" ] && [ "$kills" -gt 0 ]; then
    echo "$(date '+%m-%d %H:%M:%S') $kills kills in $(( (now-started)/60 )) min" >> "$LOG"
  fi
  sleep "$INTERVAL"
done
