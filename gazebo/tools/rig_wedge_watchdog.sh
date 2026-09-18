#!/bin/bash
# rig_wedge_watchdog.sh - notice when the rig is stuck at phase=running and clear it.
#
# WHY THIS EXISTS (2026-09-17 22:41 -> 2026-09-18 03:29, ISSUES OPEN-41).
# The conductor's log poller is the only thing that notices a run finishing and
# tears the sim down. It is spawned per launch, and a transient PermissionError
# reading ctrl_0.log propagated out of it and killed the thread. The fleet then
# sat at phase=running with an orphaned `gz sim` at 56 % CPU, the chain spun in
# its own wait_idle, and NOTHING NOTICED FOR 4 HOURS 44 MINUTES. A single
# POST /api/stop cleared it instantly.
#
# The narrow `except FileNotFoundError` is fixed in server.py, but that fix only
# takes effect when the conductor is restarted, and the class of failure - "the
# poller dies, so phase never leaves running" - has other members. This is the
# recovery net: an external deadline, which is what the tree's own rule asks for
# ("marker files + deadline waiters", not just measurement).
#
# THE WEDGE TEST, and why it is this and not a simple timer: a healthy rig
# ARCHIVES a ctrl log at every launch, so the newest file in the archive advances
# every run. A wedge is therefore "phase has been `running` continuously AND the
# archive has not gained a file for STALL_MIN minutes". A long legitimate case
# still archives on its way in, so it does not trip this.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh 2>/dev/null || true
DATA="${CHEETAH_DATA:-$HOME/Desktop/Cheetah/rundata}"
ARCH="$DATA/conductor/archive"
LOG="${WEDGE_LOG:-$DATA/campaigns/rig_wedge_watchdog.log}"
STALL_MIN="${STALL_MIN:-15}"     # no new archived ctrl log for this long = wedged
POLL_S="${POLL_S:-60}"
MAX_CLEARS="${MAX_CLEARS:-6}"    # refuse to loop forever clearing the same wedge

say(){ echo "$(date '+%m-%d %H:%M:%S') $*" | tee -a "$LOG"; }
phase(){ curl -s -m 5 http://127.0.0.1:8420/api/state | python3 -c 'import sys,json;print(json.load(sys.stdin).get("phase"))' 2>/dev/null; }
newest_age_min(){
  python3 - "$ARCH" <<'PY'
import glob,os,sys,time
fs=glob.glob(os.path.join(sys.argv[1],"*_ctrl_0.log"))
print(int((time.time()-max(os.path.getmtime(f) for f in fs))/60) if fs else -1)
PY
}

say "wedge watchdog up (pid $$): stall=${STALL_MIN} min, poll=${POLL_S}s, max clears=${MAX_CLEARS}"
clears=0
while :; do
  sleep "$POLL_S"
  ph=$(phase)
  [ "$ph" != "running" ] && continue
  age=$(newest_age_min)
  [ "${age:--1}" -lt 0 ] && continue
  if [ "$age" -ge "$STALL_MIN" ]; then
    # Confirm once more before acting - a single sample could catch a slow launch.
    sleep 60
    ph2=$(phase); age2=$(newest_age_min)
    if [ "$ph2" = "running" ] && [ "${age2:-0}" -ge "$STALL_MIN" ]; then
      clears=$((clears+1))
      say "WEDGED: phase=running and the archive has not gained a ctrl log for ${age2} min (>= ${STALL_MIN})."
      say "  orphans before the clear: $(ps -Axo pid,etime,args | grep -cE '[g]z sim|cheetah_gazebo_bridge') process(es)"
      curl -s -m 15 -X POST http://127.0.0.1:8420/api/stop >/dev/null 2>&1
      sleep 10
      say "  cleared via /api/stop (clear $clears of $MAX_CLEARS); phase now $(phase)"
      if [ "$clears" -ge "$MAX_CLEARS" ]; then
        say "GIVING UP: $MAX_CLEARS clears means the wedge is not transient. Leaving the rig alone for a human."
        exit 1
      fi
    fi
  fi
done
