#!/bin/bash
# c15 - OPEN-24, the comparison that can actually carry a causal claim.
# noinit read 6/6 and HEAD 4/7, but in separate blocks. Here the two
# binaries alternate EVERY run (deploy_host.sh DEPLOY_SRC from saved
# copies - fresh inode, re-sign, startup proof, ~2 s), so both arms share
# every minute of host state. N=8 each. Tally from mission_runner's own
# summary line.
set -u
cd /Users/kfinisterre/Desktop/Cheetah/Cheetah-Software
N="${1:-8}"; OUT=/tmp/c15.csv
echo "wall,arm,rep,verdict,pass,fell,nav" > $OUT
CAMP=gazebo/conductor/campaign.py; python3 $CAMP "c15" "starting" 0 0 ""; python3 $CAMP heartbeat & HB=$!
restore(){ DEPLOY_SRC=/tmp/bin_head bash gazebo/deploy_host.sh 2>&1 | tail -1 | sed 's/^/  [restore] /'; kill $HB 2>/dev/null; python3 $CAMP clear; }
trap restore EXIT
[ -f /tmp/bin_head ] && [ -f /tmp/bin_noinit ] || { echo "!! need /tmp/bin_head and /tmp/bin_noinit"; exit 1; }
run(){ local arm="$1" rep="$2" bin="$3"
  DEPLOY_SRC="$bin" bash gazebo/deploy_host.sh > /tmp/c15_deploy.log 2>&1 || { echo "!! deploy of $arm failed"; cat /tmp/c15_deploy.log | tail -3; exit 1; }
  curl -s -o /dev/null -m 6 http://127.0.0.1:8420/api/state || timeout 60 bash gazebo/start.sh >/dev/null 2>&1
  timeout 400 python3 gazebo/conductor/mission_runner.py --terrain flat --slot "dash:30" --gait trotting --speed 2.5 --dash 0 --wait-for-gate 1800 --extra "WP_CLOSE_LEG=0" > /tmp/c15_run.log 2>&1
  local S V NL pa fe; S=$(grep -oE "PASS=[0-9]+ FAIL=[0-9]+ FELL=[0-9]+" /tmp/c15_run.log | tail -1)
  V=$(grep -oE "VERDICT: [A-Z]+" /tmp/c15_run.log | head -1 | awk '{print $2}')
  pa=$(echo "$S" | grep -oE "PASS=[0-9]+" | cut -d= -f2); fe=$(echo "$S" | grep -oE "FELL=[0-9]+" | cut -d= -f2)
  NL=$(grep -c "\[nav\] wp" /tmp/cheetah_conductor/ctrl_0.log 2>/dev/null)
  echo "  $arm rep$rep  ${V:-NONE}  $S  nav=$NL"
  echo "$(date +%H:%M:%S),$arm,$rep,${V:-NONE},${pa:-},${fe:-},$NL" >> $OUT; sleep 3; }
for r in $(seq 1 $N); do
  python3 $CAMP "c15" "rep $r of $N" $r $N "HEAD vs noinit, interleaved"
  run HEAD   $r /tmp/bin_head
  run NOINIT $r /tmp/bin_noinit
done
python3 - <<'PY'
import csv, collections
c=collections.defaultdict(lambda:[0,0,0])
for r in csv.DictReader(open("/tmp/c15.csv")):
    if r["verdict"] in ("NONE",""): continue
    c[r["arm"]][1]+=1; c[r["arm"]][0]+= r["verdict"]=="PASS"; c[r["arm"]][2]+= r["fell"]=="1"
for a in ("HEAD","NOINIT"): p,n,f=c[a]; print("  %-7s %d/%d PASS  (%d FELL)" % (a,p,n,f))
PY
echo C15_DONE
