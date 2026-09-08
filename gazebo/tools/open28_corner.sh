#!/bin/bash
# OPEN-28: is the mid-course collapse a CORNER, and at what angle?
#
# What is known: on wkc_finals at 1.9 the mid-course falls are OPEN-26's
# mechanism (pitch runs past 28.65 deg, ESTOP, legs zeroed, down), they are
# DETERMINISTIC (three arms on the same rep produce identical numbers to two
# decimals), and dash:60 at 1.9 is 3/3 PASS - so a straight does not do it.
# The one trace read closely showed a heading change 79 -> 135 deg immediately
# before. 16 of 23 falls had |wz| > 0.25 rad/s in the preceding 2 s, but that
# has NO control: a course full of turns will show turning before anything.
#
# corner:<leg_m>:<angle_deg> is one isolated corner with a real approach and a
# real exit, so it answers the question directly instead of by correlation.
# Sweep the angle at the speed that fails, interleaved by angle within each rep
# so a drifting host cannot order the arms.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
. gazebo/tools/campaign_lib.sh
NAME=open28_corner
N="${1:-6}"; V="${2:-1.9}"; LEG="${3:-12}"
ANGLES="${ANGLES:-30 45 60 90 120 135}"
DIR="$CAMPAIGN_DIR/$NAME"; mkdir -p "$DIR"; OUT="$CAMPAIGN_DIR/$NAME.csv"
[ -s "$OUT" ] || echo "wall,angle,rep,verdict,waypoints,fall,peak_pitch,peak_roll,run_id,snapshot" > "$OUT"
FAILS=0

dump_with_retry(){
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

one(){ local ang="$1" rep="$2"
  timeout 300 python3 gazebo/conductor/mission_runner.py --terrain flat \
    --slot "corner:$LEG:$ang" --gait trotting --speed "$V" --dash 0 \
    --wait-for-gate 1800 > "$DIR/run.log" 2>&1
  local L="$RUN_DIR/ctrl_0.log" V_ W F SNAP
  V_=$(grep -oE "VERDICT: [A-Z]+" "$DIR/run.log" | head -1 | awk '{print $2}')
  W=$(grep -c 'reached wp' "$L" 2>/dev/null || echo 0)
  F=$(grep -oE '\[FALL\] [a-z]+' "$L" 2>/dev/null | tail -1 | awk '{print $2}')
  SNAP=$(dump_with_retry "a${ang}r${rep}_${NAME}_${V_:-NONE}")
  local PP PR
  read -r PP PR <<< "$(python3 - "$SNAP" <<'PY'
import sys,json
p=sys.argv[1]
if p=="NONE": print(""); raise SystemExit
try:
    R=[x for x in json.load(open(p))["records"] if x.get("pitch") is not None]
    # peak BEFORE any E-stop: classify at the event, not at the corpse
    k=next((i for i in range(1,len(R)) if R[i]["op_mode"]==2 and R[i-1]["op_mode"]!=2), len(R))
    W=[x for x in R[:k] if x["t"] > 5.0]
    print("%.1f %.1f" % (max(abs(x["pitch"]) for x in W)*57.2958,
                         max(abs(x["roll"])  for x in W)*57.2958))
except Exception: print("")
PY
)"
  echo "  angle=$ang rep$rep ${V_:-NONE} wp=$W ${F:-nofall} peak pitch=${PP:-?} roll=${PR:-?}"
  echo "$(date +%H:%M:%S),$ang,$rep,${V_:-NONE},$W,${F:-none},${PP:-},${PR:-},$(campaign_run_id),$SNAP" >> "$OUT"
  if [ "$SNAP" = NONE ]; then
    FAILS=$((FAILS+1))
    [ "$FAILS" -ge 3 ] && { echo "  ABORT: 3 failed dumps"; campaign_failed "$NAME" "3 failed dumps"; exit 1; }
  else FAILS=0; fi
}

for r in $(seq 1 "$N"); do for a in $ANGLES; do one "$a" "$r"; done; done
echo "  --- fell, by corner angle (leg=${LEG}m at ${V} m/s) ---"
for a in $ANGLES; do
  f=$(awk -F, -v A="$a" '$2==A && $6!="none"' "$OUT"|wc -l|tr -d ' ')
  n=$(awk -F, -v A="$a" '$2==A' "$OUT"|wc -l|tr -d ' ')
  pp=$(awk -F, -v A="$a" '$2==A && $7!=""{print $7}' "$OUT"|sort -n|awk '{v[NR]=$1}END{if(NR)print v[int((NR+1)/2)]}')
  echo "    ${a}deg: fell $f/$n   median peak pitch ${pp:-?} deg"
done
campaign_done "$NAME" "corner angle sweep done"
