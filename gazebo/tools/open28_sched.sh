#!/bin/bash
# OPEN-28: IS GAZEBO'S SILENT TORQUE CLIP THE MECHANISM?
#
# Every moving crossing: >=6 s of nominal trot, then z drops ~3 cm in ~0.3 s
# and attitude jumps 25 deg in 0.2 s. Joints, height and attitude depart in
# the same ~50 ms; commanded torque follows 0.18 s LATER. The controller is
# fighting it, not causing it.
#
# The controller clamps torque at setMaxTorqueCheetah3(208.5). The bridge
# applies the impedance law with no clamp. Gazebo enforces the SDF's 23.7 /
# 35.55 N*m and tells nobody. The QP may ask for f_max=175 N vertical and
# +-70 N in the cone per foot, whose joint torques at the Go1's levers exceed
# those limits. Two arms, interleaved, both with BRIDGE_DUMP on so commanded
# torque per joint is recorded at 100 Hz:
#   base   GO1_EFFORT_SCALE=1   the limits as shipped
#   x2     GO1_EFFORT_SCALE=2   the sim can deliver what is asked
# If x2 removes the crossings AND the base arm's dump shows |tau_ff| crossing
# the limit before the sink, the clip is the mechanism. The fix is then on the
# controller side - the real motors do not scale.
# OPEN-28: no single FEATURE reproduces it, so test the SEQUENCES.
#
# What has been ruled out on wkc_finals at 1.9:
#   dash:60 straight            3/3 PASS
#   corner:12:{30..135}         0/36 fell, median peak pitch 5.2-6.7 deg
#   corner:8:{150..180}         passing too, peak pitch 12.4 at the 180 reversal
# All far below SafetyChecker's 28.65. An isolated feature at this speed is
# easy; the full course falls mid-run about a fifth of the time. So what the
# course adds is the CHAINING - features with only 6-7 m of recovery between
# them - not any one of them.
#
# Three sub-courses cut straight from wkc_finals' own turn list, each with a
# real 18 m approach and a 20 m exit:
#   wkc_weave    0,18 75,7 -75,6 75,6 -75,6 0,20     the zig-zag
#   wkc_box      0,18 90,6 90,6 90,7 0,20            three 90s in a row
#   wkc_hairpin  0,18 90,7 -180,9 0,20               the reversal in context
# Interleaved by course within each rep so a drifting host cannot order them.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
. gazebo/tools/paths.sh
. gazebo/tools/campaign_lib.sh
NAME=open28_sched
N="${1:-8}"; V="${2:-1.9}"
COURSES="${COURSES:-wkc_weave wkc_box wkc_hairpin}"
# ARMS lets one course be run under several env settings, interleaved every rep -
# for a dose-response on a knob rather than a comparison of shapes. Format is
# NAME:ENV, e.g. ARMS="yaw08:WP_MAX_YAWRATE=0.8 yaw12:WP_MAX_YAWRATE=1.2".
ARMS="${ARMS:-}"
campaign_claim "$NAME" || exit 1   # no overlapping campaigns, no stale markers
DIR="$CAMPAIGN_DIR/$NAME"; mkdir -p "$DIR"; OUT="$CAMPAIGN_DIR/$NAME.csv"
[ -s "$OUT" ] || echo "wall,course,rep,verdict,waypoints,fall,peak_pitch,peak_roll,yawsat,peak_wz,run_id,bridge_dump,snapshot" > "$OUT"
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

one(){ local crs="$1" rep="$2" env="${3:-}"
  timeout 300 python3 gazebo/conductor/mission_runner.py --terrain flat \
    --slot "course:$crs" --gait trotting --speed "$V" --dash 0 \
    --wait-for-gate 1800 --extra "${env:+$env }BRIDGE_DUMP=$DIR/bridge_{RUN}.csv" > "$DIR/run.log" 2>&1
  local L="$RUN_DIR/ctrl_0.log" V_ W F SNAP RID
  V_=$(grep -oE "VERDICT: [A-Z]+" "$DIR/run.log" | head -1 | awk '{print $2}')
  W=$( { grep -c 'reached wp' "$L" 2>/dev/null || echo 0; } | head -1 )
  F=$(grep -oE '\[FALL\] [a-z]+' "$L" 2>/dev/null | tail -1 | awk '{print $2}')
  RID=$(campaign_run_id)
  local YS; YS=$( { grep -c 'YAWSAT' "$L" 2>/dev/null || echo 0; } | head -1 )
  SNAP=$(dump_with_retry "${crs}r${rep}_${NAME}_${V_:-NONE}" "$(campaign_launched_run_id "$DIR/run.log")")
  local PP PR
  read -r PP PR WZ <<< "$(python3 - "$SNAP" <<'PY'
import sys,json,math
p=sys.argv[1]
if p=="NONE": print(""); raise SystemExit
try:
    R=[x for x in json.load(open(p))["records"] if x.get("pitch") is not None]
    k=next((i for i in range(1,len(R)) if R[i]["op_mode"]==2 and R[i-1]["op_mode"]!=2), len(R))
    W=[x for x in R[:k] if x["t"] > 5.0]
    C=[x for x in W if x.get("vx") is not None and math.hypot(x["vx"],x["vy"])>1.0]
    wz=max((abs(x["wz"]) for x in C if x.get("wz") is not None), default=0.0)
    print("%.1f %.1f %.2f" % (max(abs(x["pitch"]) for x in W)*57.2958,
                              max(abs(x["roll"])  for x in W)*57.2958, wz))
except Exception: print("")
PY
)"
  local LBL="$crs"; [ -n "${env:-}" ] && LBL=$(echo "$env" | tr -d ' =' )
  echo "  $LBL rep$rep ${V_:-NONE} wp=$W ${F:-nofall} peak pitch=${PP:-?} roll=${PR:-?} wz=${WZ:-?} yawsat=${YS:-0} run=$RID"
  echo "$(date +%H:%M:%S),$LBL,$rep,${V_:-NONE},$W,${F:-none},${PP:-},${PR:-},${YS:-0},${WZ:-},$RID,$DIR/bridge_$RID.csv,$SNAP" >> "$OUT"
  if [ "$SNAP" = NONE ]; then
    FAILS=$((FAILS+1))
    [ "$FAILS" -ge 3 ] && { echo "  ABORT: 3 failed dumps"; campaign_failed "$NAME" "3 failed dumps"; exit 1; }
  else FAILS=0; fi
  campaign_health_gate "${V_:-NONE}" || { campaign_failed "$NAME" "host not running missions"; exit 1; }
}

for r in $(seq 1 "$N"); do
  if [ -n "$ARMS" ]; then
    for a in $ARMS; do one "$COURSES" "$r" "${a#*:}" ; done
  else
    for c in $COURSES; do one "$c" "$r"; done
  fi
done
echo "  --- fell, by sub-course (at ${V} m/s) ---"
for c in $COURSES; do
  f=$(awk -F, -v A="$c" '$2==A && $6!="none"' "$OUT"|wc -l|tr -d ' ')
  n=$(awk -F, -v A="$c" '$2==A' "$OUT"|wc -l|tr -d ' ')
  pp=$(awk -F, -v A="$c" '$2==A && $7!=""{print $7}' "$OUT"|sort -n|awk '{v[NR]=$1}END{if(NR)print v[int((NR+1)/2)]}')
  echo "    $c: fell $f/$n   median peak pitch ${pp:-?} deg"
done
# stale-snapshot guard: a repeated run id means the run never started and the
# ring still held the previous one (see campaign_run_id).
dup=$(awk -F, 'NR>1{print $9}' "$OUT" | sort | uniq -d | wc -l | tr -d ' ')
[ "$dup" -gt 0 ] && echo "  WARNING: $dup repeated run id(s) - those rows are stale, drop them"
campaign_done "$NAME" "sub-course sweep done"
