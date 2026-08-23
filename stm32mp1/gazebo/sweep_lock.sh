#!/bin/bash
# MUTEX FOR SWEEPS. Every harness here opens with `pkill -9 -f "gz sim"`, so two
# running at once do not merely contend for CPU - they murder each other's
# simulator mid-run and report the wreckage as data. That has now happened
# twice: once destroying a corner sweep, and once destroying BOTH an atom
# ladder (three runs reported 0/143 waypoints) and the fall-signature
# collection that killed it (runs cut off at 25 samples).
#
# Discipline did not work. So: source this at the top of every sweep script.
# It takes an exclusive lock and refuses to start if another sweep holds it.
#
#   source stm32mp1/gazebo/sweep_lock.sh   # exits 1 if a sweep is running
#
LOCKDIR="${TMPDIR:-/tmp}/cheetah_sweep.lock"
if ! mkdir "$LOCKDIR" 2>/dev/null; then
  echo "[lock] REFUSING TO START - another sweep holds $LOCKDIR" >&2
  echo "[lock] holder: $(cat "$LOCKDIR/owner" 2>/dev/null || echo unknown)" >&2
  echo "[lock] if that sweep is dead: rm -rf $LOCKDIR" >&2
  exit 1
fi
echo "$0 pid=$$ started=$(date '+%H:%M:%S')" > "$LOCKDIR/owner"
trap 'rm -rf "$LOCKDIR"' EXIT INT TERM
echo "[lock] held by $0 (pid $$)"

# ---------------------------------------------------------------------------
# LOAD GATE. Measurements need an idle machine, and "needs" is not a hope - the
# control loop targets 2.0 ms and this Mac delivers 2.5-3.0 ms idle but 7-16 ms
# with a compile or Spotlight indexing running. At 8 ms the loop is running at
# a quarter of its intended rate and the physics the run measured is not the
# physics we meant to measure. A whole oval feasibility ladder was taken at a
# median-worst 10.44 ms before this existed.
#
# So: refuse to start until the machine settles, and give up loudly rather than
# quietly measuring rubbish.
SWEEP_LOAD_MAX="${SWEEP_LOAD_MAX:-4.0}"     # 1-min load average, 14-core M4 Max
SWEEP_LOAD_WAIT="${SWEEP_LOAD_WAIT:-900}"   # s to wait for it to settle

sweep_load() { sysctl -n vm.loadavg | awk '{print $2}'; }

sweep_wait_for_idle() {
  local waited=0 l
  l=$(sweep_load)
  if awk -v a="$l" -v b="$SWEEP_LOAD_MAX" 'BEGIN{exit !(a<b)}'; then
    echo "[load] $l - machine is quiet, starting"; return 0
  fi
  echo "[load] $l exceeds $SWEEP_LOAD_MAX - HOLDING. Something else is using this"
  echo "[load] machine (compile? indexing?). Waiting up to ${SWEEP_LOAD_WAIT}s."
  while [ "$waited" -lt "$SWEEP_LOAD_WAIT" ]; do
    sleep 30; waited=$((waited+30)); l=$(sweep_load)
    if awk -v a="$l" -v b="$SWEEP_LOAD_MAX" 'BEGIN{exit !(a<b)}'; then
      echo "[load] $l after ${waited}s - settled, starting"; return 0
    fi
    [ $((waited % 120)) -eq 0 ] && echo "[load] still $l after ${waited}s..."
  done
  echo "[load] ============================================================" >&2
  echo "[load] GIVING UP: load still $l after ${SWEEP_LOAD_WAIT}s." >&2
  echo "[load] NOT starting - results taken now would be junk. Free the" >&2
  echo "[load] machine and re-run, or raise \$SWEEP_LOAD_MAX deliberately." >&2
  echo "[load] ============================================================" >&2
  return 1
}

# LOOP HEALTH, measured properly. `maxPeriod` is the maximum over an entire
# run - one scheduling hiccup in 20,000 ticks dominates it, so it flags healthy
# runs and says nothing about how starved the loop really was. Measured on
# three back-to-back single-dog runs:
#
#     p50    p95     over-4ms     result
#     2.48   2.49     0.0 %       PASS 40.3 s
#     2.48   3.01     1.3 %       PASS 40.5 s
#     2.47   8.98    10.9 %       FAIL
#
# The median never moves. What separates the failure is the TAIL. So health is
# the fraction of report intervals that overran, and p95 alongside it.
# Echoes: "<p50> <p95> <over_pct>"
sweep_loop_stats() {
  local log="$1"
  grep -o "maxPeriod=[0-9.]*" "$log" 2>/dev/null | cut -d= -f2 | sort -n | awk '
    {a[NR]=$1; if($1>4.0) o++}
    END{ if(NR==0){print "? ? ?"; exit}
         printf "%.2f %.2f %.1f\n", a[int(NR*0.5)+1], a[int(NR*0.95)+1], 100*o/NR }'
}

# CULPRIT SAMPLER. Knowing a run missed its deadlines is half the story; the
# other half is WHAT took the core. Run this in the background for the duration
# of a run and it records the top non-simulation process every few seconds, so
# a run flagged sick can name its cause instead of leaving it a mystery.
#
#   sweep_watch_start <logfile>   ... run ...   sweep_watch_stop
sweep_watch_start() {
  local out="$1"
  : > "$out"
  ( while true; do
      ps -Ao pcpu,comm -r 2>/dev/null | awk 'NR>1 && $1>15 {
          if ($2 !~ /gz|mit_ctrl_sim|Python|python/) { print strftime("%H:%M:%S"), $1, $2 }
        }' >> "$out"
      sleep 3
    done ) &
  SWEEP_WATCH_PID=$!
}
sweep_watch_stop() { [ -n "${SWEEP_WATCH_PID:-}" ] && kill "$SWEEP_WATCH_PID" 2>/dev/null; SWEEP_WATCH_PID=""; }

# Top culprit from a sampler log: "<name> <peak %>", or "-" if the machine was
# quiet and the deadline misses have some other cause.
sweep_culprit() {
  [ -s "$1" ] || { echo "-"; return; }
  awk '{ if ($2+0 > peak[$3]) peak[$3]=$2+0 }
       END { best=""; bv=0; for (k in peak) if (peak[k]>bv) {bv=peak[k]; best=k}
             if (best=="") print "-"; else printf "%s@%.0f%%\n", best, bv }' "$1"
}

# REAL-TIME FACTOR. The hard ceiling when N dogs share one engine: the
# controller runs on WALL CLOCK at 500 Hz, so if the simulator advances slower
# than real time the two drift apart and every dog fails at once, with the
# control loop looking perfectly healthy. Measured: RTF 1.001 at 3 dogs (all
# pass), 0.759 at 6 (all fail with 0% loop overruns).
# NOTE real_time_factor is on the SAME line as its value in /stats.
sweep_rtf() {
  local secs="${1:-10}"
  timeout "$secs" gz topic -e -t /stats 2>/dev/null \
    | grep -oE "real_time_factor: [0-9.]+" | awk '{s+=$2;n++} END{if(n)printf "%.3f",s/n; else print "?"}'
}

# Called by harnesses after each run: pause if the loop is being starved.
SWEEP_SICK_STREAK=0
# Pass the OVER-PERCENT here, not the max. Threshold 5 %: 1.3 % was a healthy
# passing run, 10.9 % was a failure.
sweep_health_check() {
  local over="$1"
  if awk -v x="${over:-0}" 'BEGIN{exit !(x>5.0)}'; then
    SWEEP_SICK_STREAK=$((SWEEP_SICK_STREAK+1))
  else
    SWEEP_SICK_STREAK=0; return 0
  fi
  if [ "$SWEEP_SICK_STREAK" -ge 3 ]; then
    echo "[load] !! 3 runs in a row with >5% of loop intervals overrunning (last ${over}%)."
    echo "[load] !! The machine is loaded; these results are not trustworthy."
    echo "[load] !! Pausing until it settles."
    sweep_wait_for_idle || echo "[load] !! continuing anyway - RESULTS FROM HERE ARE SUSPECT" >&2
    SWEEP_SICK_STREAK=0
  fi
}
# NO STRAGGLERS. Killing a sweep leaves gz/bridge/controller processes behind
# for a few seconds, and the next sweep's FIRST batch inherits them: topics from
# a dead partition, a bridge still holding a port, a controller still driving.
# Measured symptom - three dogs "failing" identically with the estimator diverging
# 250 times and one stuck at N=0.00 E=0.00 for 170 s, which read exactly like a
# controller regression and was nothing of the sort.
# Count REAL simulation processes. `pgrep -f` matches command lines, and any
# shell that merely MENTIONS these names - a heredoc writing a harness, a grep,
# this function's own caller - matches too. That self-match has now bitten three
# times in three different forms (pkill killing its own shell, a pgrep waiter
# waiting on itself, and this guard refusing to start because the script being
# written contained the word). So: match the command line, then keep only PIDs
# whose EXECUTABLE is actually one of ours, and never count this process tree.
sweep_sim_pids() {
  local pid comm
  for pid in $(pgrep -f 'gz[ ]sim|mit_ctrl[_]sim|cheetah[_]gazebo[_]bridge' 2>/dev/null); do
    [ "$pid" = "$$" ] && continue
    [ "$pid" = "$PPID" ] && continue
    comm=$(ps -o command= -p "$pid" 2>/dev/null)
    case "$comm" in
      *bash*-c*|*"pgrep"*|*"grep"*) continue ;;      # a shell talking about them
      gz\ sim*|*/mit_ctrl_sim*|./mit_ctrl_sim*|*cheetah_gazebo_bridge.py*)
        echo "$pid" ;;
    esac
  done
}
sweep_wait_clean() {
  local n
  for i in $(seq 1 20); do
    n=$(sweep_sim_pids | wc -l | tr -d ' ')
    [ "$n" = "0" ] && { [ "$i" -gt 1 ] && echo "[lock] machine clear after ${i}s"; return 0; }
    sleep 1
  done
  echo "[lock] REFUSING TO START: $n simulation processes still alive." >&2
  for p in $(sweep_sim_pids); do ps -o pid=,command= -p "$p" 2>/dev/null | cut -c1-90 >&2; done
  return 1
}
sweep_wait_clean || exit 1
sweep_wait_for_idle || exit 1
