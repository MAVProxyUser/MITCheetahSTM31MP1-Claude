# SKILLS.md — STM32MP1 Cheetah port: the commands

Board IP and Mac IP move with DHCP — set them once. See `CLAUDE.md` for why/traps.

```bash
BOARD=192.168.0.90     # STM32MP1 (re-check the lease)
MAC=192.168.0.75       # this Mac's LAN IP (Gazebo/bridge host)
```

## One-time setup (Mac)
```bash
brew tap messense/macos-cross-toolchains
brew install arm-unknown-linux-gnueabihf          # arm-unknown-linux-gnueabihf-{gcc,g++,...}
brew install gz-harmonic                          # Gazebo Sim 8 (for the SITL)
```

## Cross build
```bash
stm32mp1/build.sh                 # -> mp1-build/robot/{jpos_ctrl, jpos_ctrl_sim, stand_sim}
stm32mp1/build.sh clean           # wipe + reconfigure (needed after Eigen-align / ABI changes)
stm32mp1/tools/build_tools.sh     # -> stm32mp1/tools/bin/{unitree_probe, imu_probe}
```

## Deploy to the board
```bash
stm32mp1/deploy.sh push $BOARD /usr/local/cheetah-mp1     # build + stage + scp (self-contained, $ORIGIN rpath)
stm32mp1/deploy.sh                                        # stage only, no board access
# quick single-binary refresh:
arm-unknown-linux-gnueabihf-strip mp1-build/robot/stand_sim -o /tmp/stand_sim
scp /tmp/stand_sim $BOARD:/usr/local/cheetah-mp1/
```

## METHOD: ground truth only, measured on an idle machine

Every real finding in this port came from a measurement. Every wrong turn came
from reasoning ahead of the data and then looking for confirmation. These rules
are written from actual mistakes made here, not from principle.

### The only four sources of truth
1. **The Unitree decompile** (`docs/LEGGED_SPORT_REVERSE.md`) - the factory
   controller is the same MIT codebase, so its constants and structure are
   authoritative for a Go1.
2. **The URDF / SDK headers** - but **assume they may be wrong**. They disagree
   with each other and with the binary: three different joint-limit sets exist
   (URDF +-49.5 deg, Legged_sport's operational +-55, `go1_const.h` +-60), and
   this port had used the most permissive as a PHYSICAL stop.
3. **MIT's source** - upstream intent, including its bugs and its TODOs
   (`ContactEstimator` is an admitted pass-through, not a detector).
4. **Your own instrumentation** - logs, traces, and printed internals.

If a claim cannot be traced to one of those four, it is a hypothesis, and it
gets labelled as one.

### Rules

**Never reason ahead of the data.** State a falsifiable prediction BEFORE the
run, then let it stand or die. The cycle-time theory for the ~1 m/s wall
predicted that a 16 ms segment would clear 1.4 m/s; it failed at the same
distance as 26 ms, so the theory was dropped. That is the pattern to repeat.

**Verify the knob took effect before interpreting the result.** A change that
silently does nothing looks exactly like a change that does not help.
- the MPC contact-table patch fired (`6/10 horizon steps ... forced to stance`)
  and still did nothing - checking told us the mechanism was wrong, not absent;
- the GPS aiding "engaged" (origin printed) but its correction was inert,
  because MIT caps the x,y covariance so the Kalman gain was ~0;
- `SIM_MPC_MS` was verified as `dtMPC: 0.026 -> 0.020` before its result was
  believed.

**Measure the quantity the hypothesis is about.** Distance-walked comes from
Gazebo TRUTH, so it cannot show estimator error: a run scored a clean 83 m while
the estimate was 4.50 m wrong the whole way. Absolute position is NOT observable
from leg odometry (relative) + IMU - only GPS closes it.

**Repeat every marginal cell before claiming it.** This port has documented
run-to-run variance and it caught three claims in one session. Bounding at
1.0 m/s went "speed-responsive" (1 run) -> "broken" (2 runs) -> bimodal, ~50%
(4 runs: 5.23 / 0.04 / 0.03 / 5.52).

**A stop-at-first-success ladder is biased DOWNWARD.** It reported trotting at
0.6 m/s; 0.9 actually holds 176 m. Always re-test the speed ABOVE a reported
ceiling. And a gait whose viable speed is below your lowest rung reads as
"never crosses" - `walking` did, until 0.8 was tried.

**Re-measure documented numbers before building on them.** The recorded
real-estimator figure was 0.65 m and was quoted all day as a caveat; the actual
value was 57.6 m. Documentation written earlier is a hypothesis about the
present.

**Default new behaviour OFF until measured.** The flight-phase cost gate shipped
default-ON and cut solved MPC force from 39-42 N/foot to 6.1 N/foot.

### Your instrument is part of the system: verify it before believing a FAILURE

Four failures in one session came from the measuring apparatus, not the robot.
A wrong instrument that reports SUCCESS gets caught eventually; one that reports
FAILURE manufactures a finding that stops the investigation dead.

| what broke | how it lied |
|---|---|
| `getenv("SIM_CHEATER")` non-null for `"0"` | `SIM_CHEATER=0` still enabled cheater mode, so every "real estimator" result was a ground-truth run. Identical cheater/real numbers were read as "the estimator is fine" instead of "the flag does nothing". |
| fall detector threshold 0.15 m | The robot WALKS at ~0.175 estimated and dips to 0.149 on the stand-up transient, so the detector aborted every real-estimator run while Gazebo truth showed 0.197 and still walking. Nearly reported as "walking2 fails on the real estimator". |
| flight-phase cost gate | Shipped default-ON; cut solved MPC force 39-42 -> 6.1 N/foot. |
| debug print in the wrong branch | An early `continue` skipped it, so a firing correction reported as "never fires". |

Rules that follow:
- **Parse env VALUES, never test the pointer.** `getenv("X")` is non-null for
  `X=0`. Use `getenv("X") && atoi(getenv("X")) != 0`.
- **`SIM_CHEATER` is NEVER set to `0`. It is UNSET.** The code now parses the
  value correctly, but this rule stands regardless of any future code change:
  do not rely on parsing being correct, rely on the variable being ABSENT for
  the real estimator. Every "real estimator" claim in this port's history was
  wrong for exactly the opposite mistake - do not risk it recurring.
- **Audit the HARNESS env block, not just the source.** A retraction was
  written into CLAUDE.md saying every dash time was a cheater number - and
  `dash_sweep.sh` went on setting `SIM_CHEATER=1` in its `COMMON=` line for
  weeks afterward, producing a fresh "record" table that was read off and
  reported before anyone opened the script. Fixing the CODE and documenting the
  LESSON does nothing while the runner still exports the flag. Before believing
  any sweep result, `grep` the harness for the variables the result depends on,
  and launch with `env -u` for anything that must be absent.
- **A result is only as clean as the process that produced it.** Read the
  invocation before reporting the number, every time - not the number first and
  the invocation only when something looks odd. Nothing looked odd here; the
  table was internally consistent, reproducible to ~1%, and entirely invalid.
- **A detector must not depend on the quantity most likely to be wrong.** The
  fall detector reads ESTIMATED height, so it inherits estimator error. Keep a
  wide margin below the true operating value (walking ~0.175 -> trip at 0.10,
  not 0.15), and prefer attitude/kinematics over an estimated scalar.
- **When two configurations give IDENTICAL results, suspect the switch before
  concluding "no effect."** 57.64 vs 57.66 m and 4.50 vs 4.50 m were both the
  flag doing nothing.
- **Before reporting a negative result, disable your own instrumentation and
  re-run.** `SIM_FALL_EXIT=0` was what revealed the robot had been fine all along.


### CPU hygiene: measurements need an idle machine
On the board, building and running happen on different machines. On the Mac they
compete, and the SITL is sensitive to it.

- **Never compile, run r2/objdump analysis, or start a second sweep while a
  measurement is in flight.** A `make -j8` during a sweep turned a known-good
  21.23 m run into 3.54 m; the tell was the worst control-loop time going
  1.0 -> 4.5 ms.
- Check the loop time in every result. If `MAXLOOP` is far above ~2 ms, the run
  is contaminated and must be discarded, not interpreted.
- Kill stale `gz sim` with `pkill -9` between runs - a soft kill leaves a zombie
  holding the sensor systems, which silently produces a dead-sensor world.
- **Restore any harness edit immediately.** Pointing `host_sweep.sh` at an
  experimental yaml would have silently contaminated every later run.
- `pgrep -f "<pattern>"` MATCHES ITS OWN COMMAND LINE - a waiting shell reports
  the thing it is waiting for as already running. Use
  `ps -Ao pid,command | grep ... | grep -v "bash -c"`, or check for an output
  file.


## Mac-first workflow (develop here, then cross-compile for the board)

The board is ~11x slower and its eth0 flaps under load, so the math gets banged
out natively on the Mac first. Same source, same code paths - only the ISA and
the two Linux-only drivers differ.

```bash
cmake -B host-build -DSTM32MP1_HOST=ON -DSTM32MP1_MIT=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build host-build -j8 --target mit_ctrl_sim
cp host-build/user/MIT_Controller/mit_ctrl_sim host-run/
# run from host-run/ - the RPATH is $ORIGIN (a Linux-ism), so on macOS:
cd host-run && DYLD_LIBRARY_PATH=. ./mit_ctrl_sim 127.0.0.1 \
    stm32mp1-defaults.yaml mc-mit-ctrl-user-parameters.yaml
```

**Never build while a measurement is running** - on the board, compiling and
running are on different machines; here they compete. A `make -j8` during a
sweep turned a known-good 21.23 m run into 3.54 m (worst control-loop time
1.0 -> 4.5 ms is the tell).

## Measurement harnesses (Mac)

```bash
stm32mp1/gazebo/host_sweep.sh <configfile> [secs]   # many env configs, one fresh stack each
stm32mp1/gazebo/dash_sweep.sh                       # 100 m dash: fastest speed per gait
stm32mp1/gazebo/refine_maxspeed.sh <results> [secs]  # bisect/push bracketed speed ceilings
stm32mp1/gazebo/summarize_runs.py <results...>       # consolidate into the result tables
stm32mp1/gazebo/dash_trace.py <max_s> [target_m]     # 10 Hz pose trace, times the line crossing
```

Config line format: `<label>  KEY=VAL KEY=VAL ...`; the label must carry the
speed (`walk2_v10` -> 1.0 m/s) for the scorer to bin it.

## Reverse-engineering `Legged_sport` (see docs/LEGGED_SPORT_REVERSE.md)

```bash
tools/reversing/extract_consts.py <addr>    # mov/movk + fmov float immediates in a function
tools/reversing/extract_pool.py   <addr>    # resolve adrp+ldr -> .rodata constant pool
tools/reversing/find_qweights.py            # locate the MPC cost vector in .rodata
```

## Key runtime knobs

| env | default | what |
|---|---|---|
| `SIM_GAIT` | yaml | gait id: 9 trot, 20 walking, 21 walking2, 8 pacing, 1 bound, 2 pronk, 22 gallop, 5 trotRunning |
| `SIM_VX` / `SIM_VX_RAMP_S` | - / 3 | commanded speed and ramp. **A 3 s ramp masquerades as a speed ceiling** - use 12 s to measure a gait |
| `SIM_HEADING_HOLD` | 1 | integrate the yaw reference instead of re-slaving it to the measurement (0 = stock MIT) |
| `SIM_YAW_ERR_MAX` | 0.40 | how far the heading reference may lead; competes with body height |
| `SIM_MPC_MS` | 36 | MPC segment ms. **26-30 is the working range**; 36 was a board-compute compromise that capped the walk |
| `SIM_MPC_ASYNC` | 0 | solve on a worker thread; costs nothing at 26 ms and lets a slow board use a fast segment |
| `SIM_FALL_EXIT` / `SIM_FALL_DEG` / `SIM_FALL_Z` | 1 / 50 / 0.15 | end a run on tip-over OR collapse |
| `SIM_WBC_DECIM` | 1 | run WBIC every Nth tick, caching its outputs between |
| `SIM_CHEATER` | 0 | feed sim ground truth to the estimator |
| `WP_MISSION` | - | `star:<r>:<n>` / `circle:<r>:<n>` / `outback:<m>` |
| `WP_YAW_SIGN` / `WP_TURN_FLOOR` / `WP_MAX_YAWRATE` | 1 / 0.65 / 1.2 | nav: measured turn sign, arc-vs-pivot floor, turn authority |


## Run on real hardware (board)
```bash
ssh $BOARD 'cd /usr/local/cheetah-mp1 && sudo ./board_setup.sh'   # bring up can0, list ttySTM*
ssh $BOARD 'cd /usr/local/cheetah-mp1 && sudo ./run.sh'           # jpos_ctrl (SCHED_FIFO as root)
# subsystem probes (bring-up, before the control loop):
ssh $BOARD 'cd /usr/local/cheetah-mp1 && ./imu_probe can0 -1'     # CAN IMU: gyro/accel/quat + Hz
ssh $BOARD 'cd /usr/local/cheetah-mp1 && ./unitree_probe /dev/ttySTM1 4000000 0'  # one motor (needs RS485)
```

## Gazebo Go1 SITL
```bash
# Mac: Gazebo (headless) + bridge, one command (Ctrl-C stops both):
stm32mp1/gazebo/run_gazebo_sim.sh            # --gui to watch the Go1
# Board: the controller, pointing at the Mac:
ssh $BOARD "cd /usr/local/cheetah-mp1 && ./stand_sim $MAC"       # holds/squats a Go1 stance (clean demo)
ssh $BOARD "cd /usr/local/cheetah-mp1 && ./jpos_ctrl_sim $MAC"   # JPos sine sweep
```
Regenerate the world after editing `make_world.py`:
```bash
cd stm32mp1/gazebo
gz sdf -p unitree_ros/robots/go1_description/urdf/go1.urdf > /tmp/go1_raw.sdf
python3 make_world.py /tmp/go1_raw.sdf        # -> worlds/go1.sdf
```
Inspect gz topics live:
```bash
export GZ_SIM_RESOURCE_PATH=$PWD/unitree_ros/robots
gz sim -s -r worlds/go1.sdf &
gz topic -l | grep -E 'imu|air_pressure|navsat|joint_state|cmd_force'
```

## Debug a crash on the board
```bash
ssh $BOARD 'dmesg -c >/dev/null'                                  # clear first (traps only)
# capture a core (systemd Storage=none, so route it ourselves):
ssh $BOARD 'cd /usr/local/cheetah-mp1
  echo /tmp/jc-core.%p > /proc/sys/kernel/core_pattern; ulimit -c unlimited
  for i in $(seq 1 15); do timeout 4 ./jpos_ctrl.dbg ... >/dev/null 2>&1; [ $? = 139 ] && break; done
  gdb --batch -nx -ex "bt 30" ./jpos_ctrl.dbg /tmp/jc-core.*'
# push the UNSTRIPPED binary for symbols:  scp mp1-build/robot/jpos_ctrl $BOARD:/usr/local/cheetah-mp1/jpos_ctrl.dbg
```

## Board admin
```bash
ssh $BOARD 'systemctl stop ninjapilot && systemctl disable ninjapilot'   # free the cores (kills the CAN IMU stream)
ssh $BOARD 'free -m; uptime; ip -br addr'
```

## Gaits and waypoint missions (Go1 SITL)

The bridge must run in the MIT abstract joint convention for every gait below:

```bash
cd stm32mp1/gazebo
export GZ_SIM_RESOURCE_PATH="$PWD/unitree_ros/robots:$PWD/models:/path/to/NinjaPilot/ground/gazebo_bridge/models"
gz sim -s -r worlds/go1_farm.sdf &                       # headless server (farm world, solid buildings)
BRIDGE_CONV=mit python3 cheetah_gazebo_bridge.py &       # gz python bindings: use the OpenPilot venv
gz sim -g &                                              # OPTIONAL GUI - only for watching, never for batches
```

Always `chrt -f 80` on the board, and force the route over ethernet:

```bash
ssh $BOARD "/sbin/ip route replace $MAC/32 dev eth0; cd /usr/local/cheetah-mp1 && \
  SG_VX=0.2 SG_T=1.0 SG_H=0.26 chrt -f 80 ./static_gait_sim $MAC"
```

### Statically-stable crawl — `static_gait_sim` (the reliable one)
`SG_VX` m/s · `SG_T` cycle s · `SG_H` body height m · `SG_SHIFT` lateral CoM shift m ·
`SG_LIFT` foot lift m · `SG_TURN` differential-stride turn bias

### Dynamic trot — `trot_sim` (the fast one, still being tuned)
`TR_V` m/s · `TR_T` cycle s · `TR_DUTY` stance fraction (>0.5 = double support) ·
`TR_H` height · `TR_LIFT` · `TR_KV` Raibert gain · `TR_KP_ROLL/KD_ROLL/KP_PITCH/KD_PITCH`
attitude feedback · `TR_KP_YAW/KD_YAW` heading hold · `TR_KP_J/KD_J` joint PD

### MIT convex MPC + WBC — `mit_ctrl_sim`
`SIM_MODE` final FSM mode (1 stand, 3 balance, 4 locomotion) · `SIM_STAND_S/SIM_BAL_S/SIM_LOCO_S`
stage times · `SIM_SKIP_BAL=1` go 1->4 directly · `SIM_VX` + `SIM_VX_RAMP_S` velocity ramp ·
`SIM_BODY_H` MPC body height · `SIM_MPC_MS` gait segment ms · `SIM_CHEATER=1` feed the
estimator sim ground truth · `STM32MP1_EST_DBG=1` 20 Hz `[EST]`/`[LEG]` dumps

### Waypoint missions (OpenPilot path planner)
```bash
ssh $BOARD "/sbin/ip route replace $MAC/32 dev eth0; cd /usr/local/cheetah-mp1 && \
  WP_MISSION=circle:3:8 WP_ACCEPT=0.5 SG_VX=0.25 SG_T=0.9 SG_H=0.26 \
  chrt -f 80 ./static_gait_sim $MAC"
```
`WP_MISSION=circle:<radius_m>:<points>` or `outback:<metres>` · `WP_ACCEPT` acceptance
radius m · `WP_LOOP` repeat forever. Progress prints as `[nav] reached wpNN ...`.

## Batch gait testing (do not hand-run sweeps)
```bash
cat > /tmp/cfg.txt <<'CFG'
label-a | static_gait_sim | SG_VX=0.2 SG_T=1.0
label-b | trot_sim        | TR_V=0.5 TR_DUTY=0.70
CFG
RUN_S=25 stm32mp1/gazebo/batch_test.sh /tmp/cfg.txt
```
Prints outcome (UPRIGHT / fell at t / never stood), end pose, distance, mean speed
and yaw drift per config. It starts the world+bridge once, resets between runs, and
verifies each reset actually took effect (see CLAUDE.md for why that check exists).

## Headless video capture
```bash
python3 stm32mp1/gazebo/record_video.py out.mp4 25 /chase_cam
```
Records the `chase_cam` sensor on the robot's trunk straight into ffmpeg. Needs no
GUI and does not spam "Saved image to:" toasts the way `/gui/screenshot` polling did.
