#!/bin/bash
# campaign_truth.sh - OPEN-26's workhorse: run a cell, shadow it with gz
# ground truth, and archive the controller's own ring for every run.
#
# Replaces the c22 -> c23 -> c26 -> c27 -> c28 lineage, which was five sed
# edits of one script and drifted a little each time. Parameterised instead.
#
#   bash campaign_truth.sh NAME REPS "ARM:gait:terrain:speed" ["ARM2:..."]
#
# Arms alternate EVERY rep - a blocked comparison on this rig is not a
# comparison. With one arm it is just a harvest.
#
# THE DUMP IS RETRIED. c28 aborted on rep 1 because dump_snapshot returned
# None once; running the identical command by hand seconds later worked. The
# controller has just exited and the conductor is still reaping when the dump
# fires, so it is a race, not a layout mismatch (the reader shouts loudly for
# that case now). A transient miss must not kill a campaign - but a PERSISTENT
# one must, because a campaign that records "NONE" for every run silently
# wastes the night. Three tries, then abort.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.." || exit 1
. gazebo/tools/campaign_lib.sh
PYBIN="/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/ground/gazebo_bridge/venv/bin/python3"
NAME="${1:?name}"; REPS="${2:?reps}"; shift 2
ARMS=("$@"); [ ${#ARMS[@]} -gt 0 ] || { echo "need at least one arm"; exit 2; }
. gazebo/tools/paths.sh
DIR="$CAMPAIGN_DIR/$NAME"; mkdir -p "$DIR"; OUT="$CAMPAIGN_DIR/$NAME.csv"
echo "wall,arm,rep,gait,terrain,speed,verdict,truth_lines,snapshot,truth,contact" > "$OUT"

dump_with_retry(){   # $1 = tag -> echoes path or NONE
  local tag="$1" p
  for try in 1 2 3; do
    p=$(python3 -c "
import sys; sys.path.insert(0,'gazebo')
import shm_reaper
print(shm_reaper.dump_snapshot(0,'$tag') or 'NONE')" 2>/dev/null | tail -1)
    [ "${p:-NONE}" != "NONE" ] && { echo "$p"; return 0; }
    sleep 3
  done
  echo NONE; return 1
}

one(){ local arm="$1" rep="$2" gait="$3" terr="$4" spd="$5"
  local TRUTH="$DIR/truth_${arm}_$rep.jsonl"
  local CONTACT="$DIR/contact_${arm}_$rep.jsonl"
  ( timeout 400 python3 gazebo/conductor/mission_runner.py --terrain "$terr" \
      --slot "dash:30" --gait "$gait" --speed "$spd" --dash 0 \
      --wait-for-gate 1800 --extra "WP_CLOSE_LEG=0" > "$DIR/run.log" 2>&1 ) & local RP=$!
  for i in $(seq 1 90); do pgrep -f 'gz[ ]sim' >/dev/null 2>&1 && break; sleep 1; done
  # the three gz env vars server.py sets on itself; without them the
  # subscribe returns ok and not one message ever arrives
  ( cd gazebo/conductor && GZ_PARTITION=cheetah_fleet GZ_IP=127.0.0.1 \
      GZ_RELAY=127.0.0.1 "$PYBIN" -u pose_feed.py --world go1_world \
      --names go1_0=0 --rate 200 --warmup 30 --deaf-after 600 \
      </dev/null > "$TRUTH" 2>"$DIR/pf_${arm}_$rep.log" & disown )
  # GROUND-TRUTH FOOT CONTACT - labels only, never a control input. The real
  # EDU dog has no contact sensors; these exist so a sensorless estimator can
  # be scored against truth instead of against the gait schedule, which is the
  # thing under suspicion. Sim-only, own process, own file.
  ( cd gazebo/conductor && GZ_PARTITION=cheetah_fleet GZ_IP=127.0.0.1 \
      GZ_RELAY=127.0.0.1 "$PYBIN" -u contact_feed.py --world go1_world \
      --model go1_0 --rate 200 --stale 0.01 \
      </dev/null > "$CONTACT" 2>"$DIR/cf_${arm}_$rep.log" & disown )
  wait $RP
  pkill -f 'pose_feed.py --world go1_world' 2>/dev/null
  pkill -f 'contact_feed.py --world go1_world' 2>/dev/null
  local V TL SNAP
  V=$(grep -oE "VERDICT: [A-Z]+" "$DIR/run.log" | head -1 | awk '{print $2}')
  TL=$(wc -l < "$TRUTH" 2>/dev/null | tr -d ' ')
  SNAP=$(dump_with_retry "${V:-NONE}_$NAME")
  local CL; CL=$(wc -l < "$CONTACT" 2>/dev/null | tr -d ' ')
  echo "  $arm rep$rep $gait/$terr@$spd ${V:-NONE} truth=$TL contact=${CL:-0} snap=$([ "$SNAP" = NONE ] && echo NONE || echo ok)"
  echo "$(date +%H:%M:%S),$arm,$rep,$gait,$terr,$spd,${V:-NONE},$TL,$SNAP,$TRUTH,$CONTACT" >> "$OUT"
  if [ "$SNAP" = NONE ]; then
    FAILS=$((FAILS+1))
    if [ "$FAILS" -ge 3 ]; then
      echo "  ABORT: 3 snapshot dumps failed - the instrument is not working"
      campaign_failed "$NAME" "3 failed dumps"; exit 1
    fi
  fi
  if [ "$rep$arm" = "1${ARMS[0]%%:*}" ] && [ "${TL:-0}" -lt 100 ]; then
    echo "  ABORT: no ground truth on the first run (truth=$TL)"
    sed 's/^/    /' "$DIR/pf_${arm}_$rep.log" 2>/dev/null | head -4
    campaign_failed "$NAME" "no ground truth"; exit 1
  fi
}

FAILS=0
for r in $(seq 1 "$REPS"); do
  for a in "${ARMS[@]}"; do
    IFS=: read -r an ag at asp <<< "$a"
    one "$an" "$r" "$ag" "$at" "$asp"
  done
done
P=$(awk -F, '$7=="PASS"' "$OUT" | wc -l | tr -d ' '); T=$(( $(wc -l < "$OUT") - 1 ))
echo "  RESULT $NAME: $P/$T PASS, $((T-P)) non-PASS"
campaign_done "$NAME" "$T runs, $P PASS"
