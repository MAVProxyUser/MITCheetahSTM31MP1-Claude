# MIT Cheetah on the STM32MP1 (Octavo OSD32MP1)

A port of MIT Biomimetics' [Cheetah-Software](https://github.com/mit-biomimetics/Cheetah-Software)
to run its locomotion control stack on an **Octavo OSD32MP1** (STM32MP157: dual
Cortex-A7 running Linux + a Cortex-M4), driving **Unitree RS485 legs**, with a
**Gazebo (Go1) software-in-the-loop** simulation for development and — the end goal —
**OpenPilot-style waypoint navigation**.

> Fork of MIT Cheetah-Software (BSD-3, see `LICENSE`). All original build/run docs
> for the desktop + Mini Cheetah still apply; this README covers the STM32MP1 port,
> which lives under **`stm32mp1/`**. See also `CLAUDE.md` (rules + traps) and
> `SKILLS.md` (the exact commands).

## Why the STM32MP1

The MIT stack ran the locomotion controller on an x86 computer and the motor I/O on
separate SPI "spine" boards, with a Jetson only for perception. The STM32MP1 collapses
**control brain + motor I/O onto one chip**: the A7/Linux cores run the Cheetah
controller, and motor I/O (Unitree RS485, or CAN sensors) runs on the same SoC. No
GPU is needed for locomotion — perception/ROS stays separate and optional.

## Status

| Piece | State |
|---|---|
| Cross-build (Mac → armv7) | ✅ `libbiomimetics.a` + JCQP/osqp/etc. build for Cortex-A7 |
| Unitree RS485 actuator driver | ✅ compiles + probe; **not hardware-validated** (no adapter yet) |
| CAN IMU (DroneCAN) + AHRS | ✅ validated on the live bus (~490 Hz) |
| `jpos_ctrl` on the board | ✅ runs the control loop on real hardware |
| **Gazebo Go1 SITL** | ✅ **controller on the MP1 stands/squats a simulated Go1 over UDP, with IMU/baro/GPS** |
| MIT_Controller (MPC + WBC) locomotion | 🚧 in progress |
| OpenPilot waypoint navigation | ⬜ planned |

## Quick start

**Cross-compile (on a Mac with the toolchain):**
```bash
brew tap messense/macos-cross-toolchains && brew install arm-unknown-linux-gnueabihf
stm32mp1/build.sh                 # -> mp1-build/robot/{jpos_ctrl,jpos_ctrl_sim,stand_sim}
stm32mp1/deploy.sh push <board-ip> /usr/local/cheetah-mp1
```

**Gazebo SITL (Go1) — Gazebo on the Mac, controller on the board:**
```bash
stm32mp1/gazebo/run_gazebo_sim.sh                 # Mac: Gazebo (headless) + bridge
ssh <board-ip> 'cd /usr/local/cheetah-mp1 && ./stand_sim <mac-ip>'   # board: stand + squat
```

See `SKILLS.md` for hardware bring-up, the sensor/actuator probes, and debugging.

## Layout of the port

```
stm32mp1/
  toolchain.cmake            arm-unknown-linux-gnueabihf, Cortex-A7 + NEON, static libstdc++
  build.sh deploy.sh         cross build + package/scp to the board
  robot_main.cpp             hardware entry (JPos)
  config/                    board robot + user parameter YAMLs
  tools/                     unitree_probe, imu_probe (standalone bring-up)
  lcm_shim/                  null LCM (no glib) so the robot links headless
  gazebo/                    Go1 SITL: world generator, bridge, run script, README
robot/
  include/Stm32mp1HardwareBridge.h, src/Stm32mp1HardwareBridge.cpp   headless bridge
  {include,src}/rt/rt_unitree.*   Unitree A1/B1 RS485 driver
  {include,src}/rt/rt_can_imu.*   DroneCAN compact-stream IMU + Madgwick AHRS
  {include,src}/rt/rt_gazebo.*    UDP backend to the Gazebo bridge
third-party/
  eigen/                     vendored Eigen 3.4.0 (for the cross build)
  unitree_motor_sdk/         vendored Unitree A1/B1 wire protocol + CRC
  JCQP/simd_compat.h         NEON shim replacing JCQP's x86 AVX2
```

## Credits

Built on MIT Biomimetics **Cheetah-Software**. Unitree Go1 model from
`unitreerobotics/unitree_ros`. Gazebo SITL pattern adapted from the author's
OpenPilot/NinjaPilot `gazebo_bridge`.
