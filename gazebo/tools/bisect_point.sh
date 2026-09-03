#!/bin/bash
# bisect_trot.sh <sha> [N]  -  one bisect point for OPEN-24.
#
# Checks <sha> out in the persistent worktree, incremental-builds
# mit_ctrl_sim there, deploys it through gazebo/deploy_host.sh (DEPLOY_SRC,
# so it gets the fresh inode / re-sign / proves-it-loads path - never a
# bare cp into host-run/), runs N trotting@2.5 flat dashes through the
# conductor, and tallies from mission_runner's OWN summary line
# ("PASS=a FAIL=b FELL=c") - not from the archive, which is one run behind.
# The current binary is ALWAYS restored at the end, even on failure.
set -u
REPO=/Users/kfinisterre/Desktop/Cheetah/Cheetah-Software
WT=/tmp/bisect_wt
SHA="${1:?sha}"; N="${2:-4}"
OUT=/tmp/bisect_results.csv
[ -f $OUT ] || echo "wall,sha,subject,run,verdict,pass,fail,fell,nav_lines" > $OUT
restore(){ echo "== restore HEAD binary =="; ( cd "$REPO" && bash gazebo/deploy_host.sh 2>&1 | tail -1 | sed 's/^/  /' ); }
trap restore EXIT
cd "$REPO"
SUBJ=$(git log -1 --format=%s "$SHA" | cut -c1-60)
echo "######## bisect point $SHA  ($SUBJ) ########"; date
git -C "$WT" checkout -q "$SHA" 2>&1 | tail -1
git -C "$WT" checkout -q -- . 2>/dev/null          # drop any previous variant's edits
git -C "$WT" submodule update --init --recursive >/dev/null 2>&1 || true
# VARIANT=/tmp/variant_x.sh applies a one-line change on top of <sha> (the
# candidate fixes are single lines; a variant is cheaper and more honest than
# a synthetic commit). Rows are labelled sha:variant.
if [ -n "${VARIANT:-}" ]; then
  ( cd "$WT" && bash "$VARIANT" ) || { echo "!! variant failed to apply"; exit 1; }
  SHA="$SHA:$(basename "$VARIANT" .sh | sed 's/^variant_//')"
  echo "  variant applied: $SHA"
fi
( cd "$WT/host-build" && make -j3 mit_ctrl_sim > /tmp/bisect_build_$SHA.log 2>&1 ); RC=$?
B=$(find "$WT/host-build" -name mit_ctrl_sim -type f | head -1)
[ $RC -eq 0 ] && [ -n "$B" ] || { echo "!! build failed (rc=$RC) - see /tmp/bisect_build_$SHA.log"; grep -iE " error " /tmp/bisect_build_$SHA.log | head -3; exit 1; }
echo "  built: $B"
DEPLOY_SRC="$B" bash gazebo/deploy_host.sh 2>&1 | tail -1 | sed 's/^/  /' || { echo "!! deploy failed"; exit 1; }
curl -s -o /dev/null -m 6 http://127.0.0.1:8420/api/state || bash gazebo/start.sh >/dev/null 2>&1
P=0
for r in $(seq 1 $N); do
  timeout 400 python3 gazebo/conductor/mission_runner.py --terrain flat --slot "dash:30" \
    --gait trotting --speed 2.5 --dash 0 --wait-for-gate 1800 --extra "WP_CLOSE_LEG=0" > /tmp/bisect_run.log 2>&1
  S=$(grep -oE "PASS=[0-9]+ FAIL=[0-9]+ FELL=[0-9]+" /tmp/bisect_run.log | tail -1)
  V=$(grep -oE "VERDICT: [A-Z]+" /tmp/bisect_run.log | head -1 | awk '{print $2}')
  pa=$(echo "$S" | grep -oE "PASS=[0-9]+" | cut -d= -f2); fa=$(echo "$S" | grep -oE "FAIL=[0-9]+" | cut -d= -f2); fe=$(echo "$S" | grep -oE "FELL=[0-9]+" | cut -d= -f2)
  NL=$(grep -c "\[nav\] wp" /tmp/cheetah_conductor/ctrl_0.log 2>/dev/null)
  # A controller that dies at startup is INVALID, not a bad point. The first
  # point ever run read 0/4 for exactly this reason (a Release build with
  # the yaml reads compiled out) and was nearly recorded as a regression.
  if [ "${NL:-0}" -eq 0 ] && { grep -q "terminating due to uncaught exception" /tmp/cheetah_conductor/ctrl_0.log 2>/dev/null || [ "$(wc -l < /tmp/cheetah_conductor/ctrl_0.log 2>/dev/null || echo 0)" -lt 5 ]; }; then
    echo "  run $r: INVALID - controller died at startup:"; head -3 /tmp/cheetah_conductor/ctrl_0.log | cut -c1-120 | sed 's/^/      /'
    echo "$(date +%H:%M:%S),$SHA,\"$SUBJ\",$r,INVALID,,,,0" >> $OUT
    echo "BISECT_POINT_INVALID $SHA"; exit 2
  fi
  [ "${pa:-0}" = "1" ] && P=$((P+1))
  echo "  run $r: ${V:-NONE}  $S  nav=$NL"
  echo "$(date +%H:%M:%S),$SHA,\"$SUBJ\",$r,${V:-NONE},${pa:-},${fa:-},${fe:-},$NL" >> $OUT
  sleep 3
done
echo "== $SHA: $P/$N PASS at trotting@2.5 flat dash =="
echo "BISECT_POINT_DONE $SHA $P/$N"
