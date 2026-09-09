#!/bin/bash
# OPEN-28: is it the FEATURES, or is it just being out there for 200 seconds?
#
# Everything short passes. dash:60 at 1.9 is 3/3. Isolated corners 30-180 deg
# are 0/66, worst peak pitch 14.9 against a 28.65 limit. Three sequences cut
# from wkc_finals' own turn list are 0/30 at ten reps each - the hairpin is
# measurably the hardest (median peak pitch 16.1 vs 7.5 and 8.3) but it never
# reaches the limit. Meanwhile the full 197 m course falls mid-run in roughly
# a fifth to a third of attempts.
#
# The remaining difference is DURATION. wkc_finals is ~230-280 s; every
# reproducer so far is 30-60 s. Anything cumulative - estimator drift, a slow
# integrator, thermal-like state in the controller - is invisible to all of
# them by construction.
#
# Three arms, interleaved every rep:
#   wkcfin   course:wkc_finals   the POSITIVE CONTROL. If this does not fall,
#                                nothing else here means anything.
#   longeasy course:long_easy    ~200 m of the gentlest turn wkc contains
#                                (40 deg) - same duration, no hard feature
#   long     dash:200            ~110 s of pure straight - duration, no turns
#
# If the two long arms fall and the short reproducers do not, it is duration.
# If only wkc_finals falls, it is its specific features and duration is a red
# herring.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
. gazebo/tools/campaign_lib.sh
NAME=open28_duration
N="${1:-6}"; V="${2:-1.9}"
ARMS=("wkcfin:course:wkc_finals" "longeasy:course:long_easy" "long:dash:200")
DIR="$CAMPAIGN_DIR/$NAME"; mkdir -p "$DIR"; OUT="$CAMPAIGN_DIR/$NAME.csv"
[ -s "$OUT" ] || echo "wall,arm,rep,verdict,waypoints,fall,secs,peak_pitch,peak_roll,run_id,snapshot" > "$OUT"
FAILS=0

dump_with_retry(){   # $1 = tag, $2 = the run id the runner said it launched
  local tag="$1" want="${2:-}" p
  for try in 1 2 3; do
    p=$(python3 -c "
import sys; sys.path.insert(0,'gazebo')
import shm_reaper
w='$want'
print(shm_reaper.dump_snapshot(0,'$tag', expect_run_id=(w or None)) or 'NONE')" 2>/dev/null | tail -1)
    [ "${p:-NONE}" != "NONE" ] && { echo "$p"; return 0; }
    sleep 3
  done
  echo NONE; return 1
}

one(){ local arm="$1" slot="$2" rep="$3" t0=$SECONDS
  timeout 500 python3 gazebo/conductor/mission_runner.py --terrain flat \
    --slot "$slot" --gait trotting --speed "$V" --dash 0 \
    --wait-for-gate 1800 > "$DIR/run.log" 2>&1
  local L="$RUN_DIR/ctrl_0.log" V_ W F SNAP RID
  V_=$(grep -oE "VERDICT: [A-Z]+" "$DIR/run.log" | head -1 | awk '{print $2}')
  W=$( { grep -c 'reached wp' "$L" 2>/dev/null || echo 0; } | head -1 )
  F=$(grep -oE '\[FALL\] [a-z]+' "$L" 2>/dev/null | tail -1 | awk '{print $2}')
  RID=$(campaign_launched_run_id "$DIR/run.log")
  SNAP=$(dump_with_retry "${arm}r${rep}_${NAME}_${V_:-NONE}" "$RID")
  local PP PR
  read -r PP PR <<< "$(python3 - "$SNAP" <<'PY'
import sys,json
p=sys.argv[1]
if p=="NONE": print(""); raise SystemExit
try:
    R=[x for x in json.load(open(p))["records"] if x.get("pitch") is not None]
    k=next((i for i in range(1,len(R)) if R[i]["op_mode"]==2 and R[i-1]["op_mode"]!=2), len(R))
    W=[x for x in R[:k] if x["t"] > 5.0]
    print("%.1f %.1f" % (max(abs(x["pitch"]) for x in W)*57.2958,
                         max(abs(x["roll"])  for x in W)*57.2958))
except Exception: print("")
PY
)"
  echo "  $arm rep$rep ${V_:-NONE} wp=$W ${F:-nofall} $((SECONDS-t0))s peak pitch=${PP:-?} roll=${PR:-?} run=$RID"
  echo "$(date +%H:%M:%S),$arm,$rep,${V_:-NONE},$W,${F:-none},$((SECONDS-t0)),${PP:-},${PR:-},$RID,$SNAP" >> "$OUT"
  if [ "$SNAP" = NONE ]; then
    FAILS=$((FAILS+1))
    [ "$FAILS" -ge 3 ] && { echo "  ABORT: 3 failed dumps"; campaign_failed "$NAME" "3 failed dumps"; exit 1; }
  else FAILS=0; fi
  campaign_health_gate "${V_:-NONE}" || { campaign_failed "$NAME" "host not running missions"; exit 1; }
}

for r in $(seq 1 "$N"); do
  for a in "${ARMS[@]}"; do
    an="${a%%:*}"; sl="${a#*:}"
    one "$an" "$sl" "$r"
  done
done
echo "  --- fell, by arm (at ${V} m/s) ---"
for a in "${ARMS[@]}"; do
  an="${a%%:*}"
  f=$(awk -F, -v A="$an" '$2==A && $6!="none"' "$OUT"|wc -l|tr -d ' ')
  n=$(awk -F, -v A="$an" '$2==A' "$OUT"|wc -l|tr -d ' ')
  pp=$(awk -F, -v A="$an" '$2==A && $8!=""{print $8}' "$OUT"|sort -n|awk '{v[NR]=$1}END{if(NR)print v[int((NR+1)/2)]}')
  s=$(awk -F, -v A="$an" '$2==A{t+=$7;n++}END{if(n)printf "%.0f",t/n}' "$OUT")
  echo "    $an: fell $f/$n   median peak pitch ${pp:-?} deg   mean ${s:-?}s"
done
campaign_done "$NAME" "duration vs features"
