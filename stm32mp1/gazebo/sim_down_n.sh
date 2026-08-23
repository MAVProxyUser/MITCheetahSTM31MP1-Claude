#!/bin/bash
# sim_down_n.sh <instance> - tear down ONE instance, leaving siblings alone.
# By PID, from the files sim_up_n.sh wrote: pattern-matching on "gz sim" would
# take out every dog on the machine.
set -u
INST="${1:-0}"; RUN=/tmp/cheetah_inst_$INST
for f in "$RUN"/*.pid; do
  [ -f "$f" ] || continue
  p=$(cat "$f"); [ -n "$p" ] && kill -9 "$p" 2>/dev/null
  pkill -9 -P "$p" 2>/dev/null      # and its children
  rm -f "$f"
done
