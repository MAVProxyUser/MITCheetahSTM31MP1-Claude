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
