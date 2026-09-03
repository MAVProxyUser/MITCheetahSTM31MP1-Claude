#!/bin/bash
# c22 - OPEN-26, the decisive one: WHICH of the estimator's blocks is wrong?
#
# The trace has the estimator's z and vz and they disagree ~2.5x during a
# level collapse (z descends 0.31 m/s, vz never reports worse than -0.12).
# With no ground truth in the trace we cannot say which is lying. pose_feed.py
# already emits gz's true [x,y,z,yaw] and needs no rebuild.
#
# FIRST ATTEMPT BURNED A RUN: I passed --warmup 0, so pose_feed declared the
# subscription deaf in 0.0s and exited by design, and rep1 recorded "truth=0
# lines". The instrument must be PROVEN to have run - so rep1 now aborts the
# whole campaign if the truth file is empty, rather than collecting eight
# runs of nothing.
set -u
cd /Users/kfinisterre/Desktop/Cheetah/Cheetah-Software
. gazebo/tools/campaign_lib.sh
PYBIN="/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/ground/gazebo_bridge/venv/bin/python3"
N="${1:-8}"; SPEED="${2:-3.5}"; OUT=/tmp/c22.csv
mkdir -p /tmp/c22; rm -f /tmp/c22/truth_rep*.jsonl
echo "wall,rep,verdict,truth_lines,snapshot,truth" > $OUT
for r in $(seq 1 $N); do
  TRUTH=/tmp/c22/truth_rep$r.jsonl
  ( timeout 400 python3 gazebo/conductor/mission_runner.py --terrain flat \
      --slot "dash:30" --gait walking --speed "$SPEED" --dash 0 \
      --wait-for-gate 1800 --extra "WP_CLOSE_LEG=0" > /tmp/c22_run.log 2>&1 ) & RP=$!
  for i in $(seq 1 90); do pgrep -f 'gz[ ]sim' >/dev/null 2>&1 && break; sleep 1; done
  # The three gz-transport env vars server.py sets on ITSELF (server.py:85-135).
  # Without them the subscribe returns ok and not one message ever arrives -
  # a partition mismatch prints no error, which is exactly what burned rep1
  # of the previous attempt. GZ_RELAY is the OPEN-21/22 fix: discovery
  # multicast on this host leaves via en0, never loopback.
  ( cd gazebo/conductor && GZ_PARTITION=cheetah_fleet GZ_IP=127.0.0.1 \
        GZ_RELAY=127.0.0.1 "$PYBIN" -u pose_feed.py --world go1_world \
        --names go1_0=0 --rate 200 --warmup 30 --deaf-after 600 \
        </dev/null > "$TRUTH" 2>/tmp/c22/pf_rep$r.log & disown )
  wait $RP
  pkill -f 'pose_feed.py --world go1_world' 2>/dev/null
  V=$(grep -oE "VERDICT: [A-Z]+" /tmp/c22_run.log | head -1 | awk '{print $2}')
  TL=$(wc -l < "$TRUTH" 2>/dev/null | tr -d ' ')
  SNAP=$(python3 -c "
import sys; sys.path.insert(0,'gazebo')
import shm_reaper
print(shm_reaper.dump_snapshot(0,'${V:-NONE}_c22') or 'NONE')" 2>/dev/null | tail -1)
  echo "  rep$r ${V:-NONE}  truth=$TL lines  $(basename "${SNAP:-NONE}")"
  echo "$(date +%H:%M:%S),$r,${V:-NONE},$TL,${SNAP:-NONE},$TRUTH" >> $OUT
  # PROVE THE INSTRUMENT RAN before spending seven more reps on it
  if [ "$r" = 1 ] && [ "${TL:-0}" -lt 100 ]; then
    echo "  ABORT: rep1 captured $TL ground-truth lines - the instrument is not running."
    sed 's/^/    pose_feed: /' /tmp/c22/pf_rep1.log 2>/dev/null | head -5
    campaign_failed c22 "instrument dead: rep1 truth=$TL lines"; exit 1
  fi
done
campaign_done c22 "$(tail -n +2 $OUT | wc -l | tr -d ' ') reps, $(grep -cE ',(FELL|FAIL),' $OUT) non-PASS"
