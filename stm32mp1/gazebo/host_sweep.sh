#!/bin/bash
# Mac-first sweep harness: run the SAME controller natively, headless, many
# configs back to back. The board needs a cross-compile + scp + a ~90 s run per
# data point and its eth0 flaps under load; here a data point is one process
# start. Use this to bang the math out, then cross-compile the identical source
# for the board and confirm.
#
#   host_sweep.sh <configfile> [seconds]
#
# configfile: one run per line, "<label>  KEY=VAL KEY=VAL ..."; # comments ok.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
G="$ROOT/stm32mp1/gazebo"
RUNDIR="$ROOT/host-run"
PYBIN="/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/ground/gazebo_bridge/venv/bin/python3"
export PATH="/opt/homebrew/bin:$PATH"

CFG="${1:?usage: host_sweep.sh <configfile> [seconds]}"
SECS="${2:-70}"
WORLD="${WORLD:-worlds/go1_farm_flat.sdf}"
OUT="${OUT:-/tmp/host_sweep_$(date +%H%M%S)}"
mkdir -p "$OUT"

printf '%-22s %8s %8s %8s %8s  %s\n' LABEL DIST_M UPRIGHT MAXLOOP DRIFT_E VERDICT
printf '%s\n' "--------------------------------------------------------------------------------"

while IFS= read -r line; do
  [ -z "$line" ] && continue
  case "$line" in \#*) continue ;; esac
  LABEL="${line%%[[:space:]]*}"
  ENVS="${line#*[[:space:]]}"

  # Fresh stack every run: a stale gz holds the sensor systems and silently
  # produces a dead-sensor world (measured - it cost a live demo).
  pkill -9 -f "gz sim" 2>/dev/null
  pkill -f cheetah_gazebo_bridge 2>/dev/null
  sleep 2
  bash "$G/sim_up.sh" "$WORLD" > "$OUT/$LABEL.sim.log" 2>&1
  sleep 2
  # Sensor pre-flight: no NAVSAT means no GPS origin and nav never starts.
  if ! timeout 5 gz topic -e -t /navsat -n 1 2>/dev/null | grep -q latitude; then
    printf '%-22s %8s %8s %8s %8s  %s\n' "$LABEL" - - - - "DEAD SENSORS (skipped)"
    continue
  fi

  ( cd "$RUNDIR" && env DYLD_LIBRARY_PATH=. $ENVS \
      timeout "$SECS" ./mit_ctrl_sim 127.0.0.1 stm32mp1-defaults.yaml \
      mc-mit-ctrl-user-parameters.yaml > "$OUT/$LABEL.ctrl.log" 2>&1 ) &
  CPID=$!
  sleep 4
  timeout "$SECS" "$PYBIN" "$G/pose_trace.py" $((SECS - 6)) > "$OUT/$LABEL.trace" 2>/dev/null &
  TPID=$!
  # End the run the moment the controller exits. RobotRunner's fall detector
  # exits the process once the robot is on its side, so a failed config costs
  # seconds instead of the whole timeout.
  while kill -0 $CPID 2>/dev/null; do sleep 0.5; done
  kill $TPID 2>/dev/null; wait $TPID 2>/dev/null

  python3 - "$OUT/$LABEL.trace" "$OUT/$LABEL.ctrl.log" "$LABEL" <<'PY'
import sys, re
trace, ctrl, label = sys.argv[1], sys.argv[2], sys.argv[3]
rows = []
for ln in open(trace, errors='ignore'):
    m = re.search(r't=\s*(\S+)s E=\s*(\S+) N=\s*(\S+) z=(\S+) dist=\s*(\S+)m', ln)
    if m:
        rows.append(tuple(float(x) for x in m.groups()))
if not rows:
    print(f'{label:<22} {"-":>8} {"-":>8} {"-":>8} {"-":>8}  NO TRACE'); sys.exit()
# upright = last sample before the body drops below 0.15 m and stays there
FLOOR = 0.15
upright_t, dist_at_fall, drift = rows[-1][0], rows[-1][4], rows[-1][1]
for i, r in enumerate(rows):
    if r[3] < FLOOR and all(x[3] < FLOOR for x in rows[i:]):
        upright_t, dist_at_fall, drift = r[0], r[4], r[1]
        break
loops = [float(x) for x in re.findall(r'maxRuntime=([0-9.]+)', open(ctrl, errors='ignore').read())]
maxloop = max(loops) if loops else 0.0
ctrl_txt = open(ctrl, errors='ignore').read()
fell = upright_t < rows[-1][0] - 1.0 or '[FALL]' in ctrl_txt
verdict = f'fell @ {upright_t:.0f}s' if fell else 'UPRIGHT to end'
if 'MISSION COMPLETE' in ctrl_txt:
    import re as _re
    ts = _re.findall(r'MISSION COMPLETE .*?t=([0-9.]+)s', ctrl_txt)
    verdict = f'STAR DONE {ts[-1]}s' if ts else 'STAR DONE'
print(f'{label:<22} {dist_at_fall:8.2f} {upright_t:7.0f}s {maxloop:7.2f}m {drift:8.2f}  {verdict}')
PY
done < "$CFG"

pkill -9 -f "gz sim" 2>/dev/null; pkill -f cheetah_gazebo_bridge 2>/dev/null
echo; echo "logs: $OUT"
