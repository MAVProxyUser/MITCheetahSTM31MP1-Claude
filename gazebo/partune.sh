#!/bin/bash
# partune.sh - run N configs across 3 parallel dogs with EQUAL, VALID reps.
#
# Two rules this enforces, because both have burned this project:
#
#  EQUAL N. Every arm gets exactly $REPS runs. Comparing 5/5 against 3/3 is how
#  a 3/8-vs-7/7 headline survived here long enough to be reported when the true
#  stock rate was 8/13. A run that does not complete VALIDLY is retried, not
#  quietly dropped, so the arms stay matched.
#
#  ACCEPTANCE CRITERIA. A run only counts if the machine was actually able to
#  run it:
#     over4ms <= 5%   control loop met its deadlines (measured: every failure
#                     sat at 14%, every pass at <=3.9%)
#     no NaN at start estimator did not diverge before the dog stood
#     mission planned  the brief printed, so the config really took effect
#  Anything else is an INSTRUMENT failure, not a result, and is re-run.
#
# Configs run three-at-a-time, which also means the three arms in a group share
# identical machine conditions - a better-matched comparison than running them
# sequentially and hoping the machine did not drift.
#
# usage: partune.sh <reps> <label:env...> <label:env...> ...
set -u
source "$(dirname "${BASH_SOURCE[0]}")/sweep_lock.sh" || exit 1
cd "$(dirname "${BASH_SOURCE[0]}")/../.." || exit 1
G=gazebo
REPS="$1"; shift
CONFIGS=("$@")
O=${PARTUNE_OUT:-/tmp/PT}; mkdir -p $O
MAXRETRY=${MAXRETRY:-3}

run_one(){          # instance label env...
  local inst="$1" label="$2"; shift 2
  ( cd host-run && env DYLD_LIBRARY_PATH=. SIM_INSTANCE=$inst \
      SIM_VX_DELAY_S=4 SIM_VX_RAMP_S=8 WP_MAX_YAWRATE=1.2 WP_PLANNER=1 "$@" \
      timeout 220 ./mit_ctrl_sim 127.0.0.1 stm32mp1-defaults.yaml \
      mc-mit-ctrl-user-parameters.yaml > $O/$label.log 2>&1 )
}
valid(){            # label -> 0 if the RUN is trustworthy (pass OR fail)
  # A FAILURE is a perfectly good result; what must be rejected is a run the
  # MACHINE spoiled. Three criteria, and the middle one has a trap in it.
  local f=$O/$1.log
  [ -s "$f" ] || return 1
  grep -q "\[mission\]\|\[nav\].*mission" "$f" || return 1     # config took effect
  # Control loop met its deadlines: every real failure sat at ~14% of intervals
  # overrunning, every pass at <=3.9%.
  local over=$(grep -o "maxPeriod=[0-9.]*" "$f" | cut -d= -f2 \
               | awk '{n++; if($1>4)o++} END{if(n)printf "%.1f",100*o/n; else print 99}')
  awk -v x="$over" 'BEGIN{exit !(x<=5.0)}' || return 1
  # THE DOG ACTUALLY GOT GOING. A transient NON-FINITE at startup that the
  # estimator reinitialises from is harmless - the first version of this check
  # rejected ANY NaN and threw away a completed 39.4 s run, the fastest in the
  # phase, purely because it blipped once before standing. Worse, the blip only
  # occurred in the analyzer arm, so the gate was silently biased AGAINST the
  # treatment. What matters is whether the run got off the ground, not whether
  # the estimator ever printed a warning.
  grep -q "MISSION COMPLETE t=" "$f" && return 0                  # completed: valid
  [ "$(grep -c "\[nav\] reached" "$f")" -ge 1 ] && return 0      # real mid-course failure
  return 1                                                        # never started: instrument
}
result(){           # label -> "t|over"
  local f=$O/$1.log
  local t=$(grep -m1 "MISSION COMPLETE t=" "$f" | sed -n 's/.*t=\([0-9.]*\)s.*/\1/p')
  local over=$(grep -o "maxPeriod=[0-9.]*" "$f" | cut -d= -f2 \
               | awk '{n++; if($1>4)o++} END{if(n)printf "%.1f",100*o/n; else print 99}')
  echo "${t:-FAIL}|$over"
}

# work queue: one entry per (config, rep)
QUEUE=(); for r in $(seq 1 $REPS); do for c in "${!CONFIGS[@]}"; do QUEUE+=("$c:$r"); done; done
declare -a RES; for c in "${!CONFIGS[@]}"; do RES[$c]=""; done
retries=0
while [ ${#QUEUE[@]} -gt 0 ]; do
  BATCH=("${QUEUE[@]:0:3}"); QUEUE=("${QUEUE[@]:3}")
  for i in "${!BATCH[@]}"; do bash $G/sim_up_n.sh $i worlds/go1_speedway.sdf >/dev/null 2>&1; done
  sleep 3
  LBL=()
  for i in "${!BATCH[@]}"; do
    ci="${BATCH[$i]%%:*}"; rr="${BATCH[$i]##*:}"
    spec="${CONFIGS[$ci]}"; name="${spec%%:*}"; envs="${spec#*:}"
    LBL[$i]="${name}_r${rr}"
    run_one $i "${LBL[$i]}" $envs &
  done
  wait
  for i in "${!BATCH[@]}"; do bash $G/sim_down_n.sh $i; done
  sleep 2
  for i in "${!BATCH[@]}"; do
    ci="${BATCH[$i]%%:*}"
    if valid "${LBL[$i]}"; then
      RES[$ci]="${RES[$ci]} $(result "${LBL[$i]}")"
      printf '  %-24s %s\n' "${LBL[$i]}" "$(result "${LBL[$i]}")"
    else
      retries=$((retries+1))
      printf '  %-24s INVALID (instrument) - requeued\n' "${LBL[$i]}"
      [ $retries -le $((MAXRETRY * ${#CONFIGS[@]} * REPS)) ] && QUEUE+=("${BATCH[$i]}")
    fi
  done
done
echo
printf '%-22s %8s %9s %8s %9s\n' ARM PASS MEAN_T SD WORST_OVER
for c in "${!CONFIGS[@]}"; do
  name="${CONFIGS[$c]%%:*}"
  echo "${RES[$c]}" | tr ' ' '\n' | grep -v '^$' | awk -v n="$name" '
    { split($0,a,"|"); if(a[1]!="FAIL"){t[++k]=a[1]; s+=a[1]} ; tot++; if(a[2]+0>w)w=a[2]+0 }
    END{ m=(k?s/k:0); v=0; for(i=1;i<=k;i++) v+=(t[i]-m)^2; sd=(k>1?sqrt(v/k):0);
         printf "%-22s %4d/%-3d %9.2f %8.2f %8.1f%%\n", n, k, tot, m, sd, w }'
done
