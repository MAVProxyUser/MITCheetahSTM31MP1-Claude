#!/bin/bash
# Shell half of gazebo/paths.py - see that file for why none of this may live
# in /tmp. Source it: . gazebo/tools/paths.sh
_PATHS_HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_REPO="$(cd "$_PATHS_HERE/../.." && pwd)"
CHEETAH_DATA="${CHEETAH_DATA:-$(cd "$_REPO/.." && pwd)/rundata}"
export CHEETAH_DATA
RUN_DIR="$CHEETAH_DATA/conductor"
ARCHIVE_DIR="$RUN_DIR/archive/shm_trace"
CAMPAIGN_DIR="$CHEETAH_DATA/campaigns"
LOG_DIR="$CHEETAH_DATA/logs"
export RUN_DIR ARCHIVE_DIR CAMPAIGN_DIR LOG_DIR
mkdir -p "$RUN_DIR" "$ARCHIVE_DIR" "$CAMPAIGN_DIR" "$LOG_DIR" 2>/dev/null
