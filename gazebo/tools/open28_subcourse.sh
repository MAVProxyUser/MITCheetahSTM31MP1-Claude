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
# An arm may carry several variables joined by commas (no spaces - the arm list
# splits on whitespace): "ramp40:WP_LIEDOWN_EDAMP0=40,WP_LIEDOWN_RAMP_MS=600" -
# the commas become spaces on the --extra line (2026-09-14).
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
# FOREGROUND since 2026-09-15 00:50 (was nohup'd into the first run): the packer is
# a host tenant like any other (zstd -3 -T1 at background QoS still costs the sim
# real time - OPEN-39's sample deficits do not care about nice), so it runs to
# completion in the idle gap before the first launch, capped at five minutes.
pgrep -f "^bash gazebo/tools/archive_compact.sh" >/dev/null || timeout 300 bash gazebo/tools/archive_compact.sh >/dev/null 2>&1

# DISK GUARD (2026-09-15 23:00, ISSUES OPEN-35). The snapshot archive has no
# retention policy and grows ~6 GB/day COMPACTED (496-665 snapshots a day at
# ~11 MB each); compaction is keeping up (4160 packed, 0 unpacked) so what is
# left is genuine data, and only the operator can decide to delete it. What the
# harness can do is refuse to produce garbage: below DISK_WARN_GB the campaign
# says so in its own log every time, and below DISK_STOP_GB it stops the CHAIN
# cleanly (touching every STOP_?? marker) instead of letting runs fail in
# confusing ways, logs truncate and the conductor wedge on a full volume. A
# clean stop with a loud reason beats a night of corrupt data; RULE ONE's answer
# is to free space, not to keep writing into 5 GB.
DISK_WARN_GB="${DISK_WARN_GB:-20}"; DISK_STOP_GB="${DISK_STOP_GB:-8}"
DISK_FREE_GB=$(df -g / | tail -1 | awk '{print $4}')
if [ "${DISK_FREE_GB:-999}" -lt "$DISK_STOP_GB" ]; then
  echo "DISK GUARD: only ${DISK_FREE_GB} GB free (floor ${DISK_STOP_GB} GB) - STOPPING the chain instead of writing into a full volume."
  echo "  the archive is $(du -sh "$CHEETAH_DATA/conductor/archive" 2>/dev/null | cut -f1) with no retention policy; thin it or move it, then clear the STOP markers."
  # stop whatever chains are actually alive, by their own two-letter marker
  for cp in $(pgrep -fl 'campaign_chain_20260912[a-z][a-z][.]sh' 2>/dev/null | grep -oE 'campaign_chain_20260912[a-z][a-z]' | grep -oE '[a-z][a-z]$'); do
    m="$CAMPAIGN_DIR/STOP_$(echo "$cp" | tr 'a-z' 'A-Z')"; touch "$m"; echo "  touched $(basename "$m")"
  done
  exit 3
elif [ "${DISK_FREE_GB:-999}" -lt "$DISK_WARN_GB" ]; then
  echo "DISK GUARD: ${DISK_FREE_GB} GB free (warn ${DISK_WARN_GB} GB, floor ${DISK_STOP_GB} GB) - about $(( (DISK_FREE_GB - DISK_STOP_GB) * 4 )) hours of runs left at ~6 GB/day."
fi
DIR="$CAMPAIGN_DIR/$NAME"; mkdir -p "$DIR"; OUT="$CAMPAIGN_DIR/$NAME.csv"
[ -s "$OUT" ] || echo "wall,course,rep,verdict,waypoints,fall,peak_pitch,peak_roll,yawsat,peak_wz,run_id,bridge_dump,snapshot,loop_max_ms,speed,imu_gap_max_ms,mission_t_s,max_dwell_s,imu_rx_min" > "$OUT"
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
    --wait-for-gate 1800 ${env:+--extra "$(echo "$env" | tr ',' ' ')"} > "$DIR/run.log" 2>&1
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
  # THE SAME STREAM, COUNTED (ISSUES OPEN-39, 2026-09-13): the fewest IMU
  # samples the bridge received in any cruise second (cmd_rx >= 400/s). A
  # second can lose twenty samples as several SHORT gaps - none over 15 ms,
  # none a held sample the freeze scan sees - and both locomotionSafe
  # leg-speed trips on record sat in exactly such a second (480/s, 482/s)
  # against one cruise second in 2859 with 15 or more missing. 500 is clean.
  local IR; IR=$(grep -v "peer=None" "$RUN_DIR/bridge_0.log" 2>/dev/null | grep -E 'cmd_rx=([4-9][0-9][0-9]|[1-9][0-9]{3})/s' | grep -oE 'imu_rx=[0-9]+' | cut -d= -f2 | sort -n | head -1)
  # THE RUN'S SHAPE (ISSUES OPEN-38): the mission time and the longest a
  # single waypoint index stayed active while the [nav] lines kept coming.
  # Every wkc_finals and hp_gap20 verdict before 2026-09-12 was a DOUBLE
  # LAP - 171 s for a 197 m course at 2.4, and wp7 held for 75 s while the
  # body swept the whole course - and no column said so. A mission time
  # far over course length / cruise, or a dwell of tens of seconds on one
  # index, is a mission-shape problem, not a robot result.
  local MT DW; MT=$(grep -oE 'MISSION COMPLETE t=[0-9.]+' "$L" 2>/dev/null | tail -1 | cut -d= -f2)
  DW=$(awk '/^\[nav\] wp[0-9]+\/[0-9]+ /{ idx=$2; t=$NF; sub(/^t=/,"",t); sub(/s$/,"",t); if (idx!=last){ if (last!="") { d=tl-t0; if (d>m) m=d }; last=idx; t0=t }; tl=t } END{ if (last!="") { d=tl-t0; if (d>m) m=d }; printf "%.1f", m+0 }' "$L" 2>/dev/null)
  echo "  $LBL rep$rep ${V_:-NONE} wp=$W ${F:-nofall} peak pitch=${PP:-?} roll=${PR:-?} wz=${WZ:-?} yawsat=${YS:-0} run=$RID imu_gap=${IG0:-?}ms imu_rx_min=${IR:-?}/s mission_t=${MT:-none}s dwell=${DW:-?}s"
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
  echo "$(date +%H:%M:%S),$LBL,$rep,${V_:-NONE},$W,${F:-none},${PP:-},${PR:-},${YS:-0},${WZ:-},$RID,$BD,$SNAP,${LM:-},$v,${IG:-},${MT:-},${DW:-},${IR:-}" >> "$OUT"
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
