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
