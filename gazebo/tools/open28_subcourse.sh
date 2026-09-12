#!/bin/bash
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
NAME="${CAMPAIGN_NAME:-open28_subcourse}"     # CAMPAIGN_NAME=x gives a campaign its own CSV and dir
N="${1:-8}"; V="${2:-1.9}"
COURSES="${COURSES:-wkc_weave wkc_box wkc_hairpin}"
# ARMS lets one course be run under several env settings, interleaved every rep -
# for a dose-response on a knob rather than a comparison of shapes. Format is
# NAME:ENV, e.g. ARMS="yaw08:WP_MAX_YAWRATE=0.8 yaw12:WP_MAX_YAWRATE=1.2".
# Several variables in one arm are comma-separated: "old:BRIDGE_RX_THREAD=1,CTRL_MPC_TABLE_ALIAS=1".
# The arm NAME is the label in the CSV.
ARMS="${ARMS:-}"
# DUMP=1 records the bridge's per-joint command stream (100 Hz) for every run,
# which the per-exchange scorers read; the path lands in a bridge_dump column.
DUMP="${DUMP:-0}"
campaign_claim "$NAME" || exit 1   # no overlapping campaigns, no stale markers
# Pack snapshots older than 24 h while this campaign runs (background QoS,
# bounded, one at a time): the archive has no retention and the disk sat at
# 96 % on 2026-09-11 (ISSUES OPEN-35). Readers resolve .json/.json.zst alike.
pgrep -f "^bash gazebo/tools/archive_compact.sh" >/dev/null || ( nohup bash gazebo/tools/archive_compact.sh >/dev/null 2>&1 & )
DIR="$CAMPAIGN_DIR/$NAME"; mkdir -p "$DIR"; OUT="$CAMPAIGN_DIR/$NAME.csv"
[ -s "$OUT" ] || echo "wall,course,rep,verdict,waypoints,fall,peak_pitch,peak_roll,yawsat,peak_wz,run_id,bridge_dump,snapshot,loop_max_ms,speed,imu_gap_max_ms,mission_t_s,max_dwell_s" > "$OUT"
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

one(){ local crs="$1" rep="$2" env="${3:-}" arm="${4:-}"
  env="${env//,/ }"
  # SPEED=<m/s> inside an arm's env sets that arm's cruise instead of $V, so a
  # speed ladder can be run INTERLEAVED (one rung per rep) rather than as
  # blocks - blocks are not a comparison on this rig. The token is consumed
  # here and not passed to the controller.
  local v="$V" terr="${TERRAIN:-flat}" tok rest=""
  # TERRAIN=<kind> inside an arm's env sets that arm's surface kind (flat,
  # concrete, grass, ...) so a friction A/B interleaves like a speed ladder.
  # Consumed here, not passed to the controller.
  for tok in $env; do case "$tok" in SPEED=*) v="${tok#SPEED=}";; TERRAIN=*) terr="${tok#TERRAIN=}";; *) rest="${rest:+$rest }$tok";; esac; done
  env="$rest"
  [ "$DUMP" = 1 ] && env="${env:+$env }BRIDGE_DUMP=$DIR/bridge_{RUN}.csv"
  # a COURSES entry with a colon is a raw slot spec (dash:100, star:10.514:5);
  # a bare name is a course file under gazebo/courses/.
  local slot="course:$crs"; case "$crs" in *:*) slot="$crs";; esac
  # A Time Machine backup makes the conductor REFUSE the launch; the runner
  # then waits inside its own 300 s deadline and the harness books a NONE for
  # a run that never existed (2026-09-11 11:24: two NONEs and a fleet clear
  # for a 20-minute backup). Wait it out HERE, before the deadline starts.
  local tmw=0
  while tmutil status 2>/dev/null | grep -q "Running = 1"; do
    [ "$tmw" -eq 0 ] && echo "  [gate] Time Machine backup in progress - waiting before launching (not a run)"
    tmw=$((tmw+1)); sleep 30
  done
  [ "$tmw" -gt 0 ] && echo "  [gate] Time Machine finished after ~$((tmw*30)) s - launching"
  # RECIPE_GAIT=1: run the slot on its recipe's own gait and speed (the suite's
  # full-tier cases - walking recipes, spiro - are defined that way; forcing
  # --gait trotting --speed V would turn them into something else). $v then
  # only labels the CSV.
  local gaitargs=(--gait trotting --speed "$v")
  [ "${RECIPE_GAIT:-0}" = 1 ] && gaitargs=()
  timeout 900 python3 gazebo/conductor/mission_runner.py --terrain "$terr" \
    --slot "$slot" ${gaitargs[@]+"${gaitargs[@]}"} --dash 0 \
    --wait-for-gate 1800 ${env:+--extra "$env"} > "$DIR/run.log" 2>&1
  local L="$RUN_DIR/ctrl_0.log" V_ W F SNAP RID
  V_=$(grep -oE "VERDICT: [A-Z]+" "$DIR/run.log" | head -1 | awk '{print $2}')
  W=$( { grep -c 'reached wp' "$L" 2>/dev/null || echo 0; } | head -1 )
  F=$(grep -oE '\[FALL\] [a-z]+' "$L" 2>/dev/null | tail -1 | awk '{print $2}')
  RID=$(campaign_run_id)
  local YS; YS=$( { grep -c 'YAWSAT' "$L" 2>/dev/null || echo 0; } | head -1 )
  SNAP=$(dump_with_retry "${crs//:/_}r${rep}_${NAME}_${V_:-NONE}" "$(campaign_launched_run_id "$DIR/run.log")")
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
  local LBL="$crs"
  if [ -n "${arm:-}" ]; then LBL="$arm"; elif [ -n "${env:-}" ]; then LBL=$(echo "$env" | tr -d ' =' ); fi
  local BD=""; [ "$DUMP" = 1 ] && BD="$DIR/bridge_${RID}.csv"
  local IG0; IG0=$(grep -v "peer=None" "$RUN_DIR/bridge_0.log" 2>/dev/null | grep -oE 'imu_gap_max=[0-9.]+' | cut -d= -f2 | sort -n | tail -1)
  # THE RUN'S SHAPE (ISSUES OPEN-38): the mission time and the longest a
  # single waypoint index stayed active while the [nav] lines kept coming.
  # Every wkc_finals and hp_gap20 verdict before 2026-09-12 was a DOUBLE
  # LAP - 171 s for a 197 m course at 2.4, and wp7 held for 75 s while the
  # body swept the whole course - and no column said so. A mission time
  # far over course length / cruise, or a dwell of tens of seconds on one
  # index, is a mission-shape problem, not a robot result.
  local MT DW; MT=$(grep -oE 'MISSION COMPLETE t=[0-9.]+' "$L" 2>/dev/null | tail -1 | cut -d= -f2)
  DW=$(awk '/^\[nav\] wp[0-9]+\/[0-9]+ /{ idx=$2; t=$NF; sub(/^t=/,"",t); sub(/s$/,"",t); if (idx!=last){ if (last!="") { d=tl-t0; if (d>m) m=d }; last=idx; t0=t }; tl=t } END{ if (last!="") { d=tl-t0; if (d>m) m=d }; printf "%.1f", m+0 }' "$L" 2>/dev/null)
  echo "  $LBL rep$rep ${V_:-NONE} wp=$W ${F:-nofall} peak pitch=${PP:-?} roll=${PR:-?} wz=${WZ:-?} yawsat=${YS:-0} run=$RID imu_gap=${IG0:-?}ms mission_t=${MT:-none}s dwell=${DW:-?}s"
  # the control loop's worst period in this run (ms) - a host-stall column,
  # so a run that survived a 469 ms freeze and one that ran clean are not
  # the same row (2026-09-10: three of six control runs carried 44-469 ms)
  local LM; LM=$(grep 'ctrl loop' "$L" 2>/dev/null | grep -oE 'maxPeriod=[0-9.]+' | cut -d= -f2 | sort -n | tail -1)
  # the sim's sensor stream, from the bridge's own 1 Hz line: the worst gap
  # between two IMU messages in this run (OPEN-35's third class - gz-transport
  # loopback holds under host load; 21-45 ms in the walking runs that fell on
  # 2026-09-11, 4-6 ms in the ones that passed). A run with a gap over ~15 ms
  # is not the robot's evidence.
  local IG; IG=$(grep -v "peer=None" "$RUN_DIR/bridge_0.log" 2>/dev/null | grep -oE 'imu_gap_max=[0-9.]+' | cut -d= -f2 | sort -n | tail -1)
  echo "$(date +%H:%M:%S),$LBL,$rep,${V_:-NONE},$W,${F:-none},${PP:-},${PR:-},${YS:-0},${WZ:-},$RID,$BD,$SNAP,${LM:-},$v,${IG:-},${MT:-},${DW:-}" >> "$OUT"
  if [ "$SNAP" = NONE ]; then
    FAILS=$((FAILS+1))
    [ "$FAILS" -ge 3 ] && { echo "  ABORT: 3 failed dumps"; campaign_failed "$NAME" "3 failed dumps"; exit 1; }
  else FAILS=0; fi
  campaign_health_gate "${V_:-NONE}" || { campaign_failed "$NAME" "host not running missions"; exit 1; }
}

for r in $(seq 1 "$N"); do
  if [ -n "$ARMS" ]; then
    for a in $ARMS; do one "$COURSES" "$r" "${a#*:}" "${a%%:*}"; done
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
dup=$(awk -F, 'NR>1{print $11}' "$OUT" | sort | uniq -d | wc -l | tr -d ' ')
[ "$dup" -gt 0 ] && echo "  WARNING: $dup repeated run id(s) - those rows are stale, drop them"
# A fall preceded by a sensor freeze (the bridge stalled under a running
# controller) is the host's, not the robot's (OPEN-35): split the verdicts
# per arm before anyone reads the counts above as the robot's.
echo "  --- harness-fall check (state freeze in the second before the event) ---"
python3 gazebo/tools/campaign_freeze_report.py "$NAME" 2>/dev/null | sed 's/^/  /' | tail -n 8
campaign_done "$NAME" "sub-course sweep done"
