#!/bin/bash
# Compact archived shm_trace snapshots older than AGE_H hours (default 24) in
# place: X.json -> X.json.zst (zstd -3, ~8x on these traces; reversible with
# `zstd -d`). Campaign CSVs keep the .json path; readers resolve it through
# gazebo/tools/snapio.py. Runs every file at background QoS so a live campaign
# is not the one paying for it. No retention: nothing is deleted, only packed.
#   usage: archive_compact.sh            (AGE_H=24 by default)
#          AGE_H=6 archive_compact.sh
set -u
DATA="${CHEETAH_DATA:-$HOME/Desktop/Cheetah/rundata}"
DIR="${SHM_ARCHIVE:-$DATA/conductor/archive/shm_trace}"
AGE_MIN=$(( ${AGE_H:-24} * 60 ))
LOG="$DATA/campaigns/archive_compact.log"
before=$(du -sk "$DIR" | cut -f1); n=0; fail=0
while IFS= read -r -d '' f; do
  if taskpolicy -b nice -n 19 zstd -q --rm -T1 -3 "$f"; then n=$((n+1)); else fail=$((fail+1)); fi
done < <(find "$DIR" -name '*.json' -mmin +"$AGE_MIN" -print0)
after=$(du -sk "$DIR" | cut -f1)
echo "$(date '+%F %T') compacted $n snapshots older than ${AGE_H:-24} h ($fail failed): $((before/1024)) MB -> $((after/1024)) MB; disk free $(df -h "$DIR" | awk 'NR==2{print $4}')" | tee -a "$LOG"
