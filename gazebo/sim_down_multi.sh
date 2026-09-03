#!/bin/bash
set -u
RUN=/tmp/cheetah_multi
for f in "$RUN"/*.pid; do [ -f "$f" ] || continue
  p=$(cat "$f"); [ -n "$p" ] && { kill -9 "$p" 2>/dev/null; pkill -9 -P "$p" 2>/dev/null; }
  rm -f "$f"; done
