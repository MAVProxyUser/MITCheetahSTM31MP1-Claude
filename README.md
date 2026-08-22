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
| MIT_Controller (MPC + WBC) locomotion | ✅ **walks** — 100 m continuous under `walking2` on the real estimator; 4 of MIT's 8 gaits cross 100 m |
| OpenPilot waypoint navigation | ⚠️ **GPS star mission completes under convex MPC** (5 × 10.1 m legs) — timings were taken in cheater mode; awaiting an honest re-measure |
| Mac-first host build | ✅ same source builds natively (`-DSTM32MP1_HOST=ON`) for fast iteration |
| Robot model vs the real Go1 | ✅ **corrected against Unitree's own binary** (see `docs/LEGGED_SPORT_REVERSE.md`) |

## Measured locomotion (Mac SITL)

100 m dash on flat ground, fastest speed each gait completes it. Measured on
the robot's **own state estimate** - `SIM_CHEATER` is absent from the
environment and the harnesses no longer set it. Gazebo ground truth is used
only to MEASURE distance; it never enters the control loop.

| gait | max speed | 100 m time | cruise | next rung up |
|---|---|---|---|---|
| `walking2` (21) | 0.8 m/s | **121.9 s** | 0.83 m/s | 1.0 -> 10.4 m |
| `walking` (20) | 0.6 m/s | **162.4 s** | 0.61 m/s | 0.8 -> 10.2 m |
| `pacing` (8) | 0.6 m/s | **183.5 s** | 0.54 m/s | 0.8 -> 5.3 m |
| `trotting` (9) | 0.6 m/s | **185.8 s** | 0.53 m/s | 0.8 -> 78.7 m* |
| `trotRunning` (5) | — | never crosses | — | fails 0.8 / 0.6 / 0.4 |
| `bounding` / `pronking` / `galloping` | — | never cross | — | — |

\* marginal, under repeat - a stop-at-first-success ladder biases the answer
downward, and this trap has already understated trot by 50% once.

**Four of MIT's eight gaits run 100 m on the real estimator.** None did before
this work: the previous honest baseline was ~21 m (walking2 at 1.0 m/s) and
35 m at 0.6 m/s.

### Ground truth vs the real estimator: the gap is the whole story

The same sweep with sim ground truth fed to the estimator is one to two rungs
faster on every gait:

| gait | ground truth | real estimator |
|---|---|---|
| `trotting` | 1.0 m/s, 106.5 s | 0.6 m/s, 185.8 s |
| `walking2` | 1.0 m/s, 107.0 s | 0.8 m/s, 121.9 s |
| `walking` | 0.8 m/s, 130.7 s | 0.6 m/s, 162.4 s |
| `pacing` | 0.8 m/s, 131.1 s | 0.6 m/s, 183.5 s |

trot is hit hardest: it covers 100 m at 1.0 m/s on ground truth and goes down
after **4.4 m** at the same command on its own estimate. **The state estimator,
not compute and not gait tuning, is what caps this port today.**

Reality check against the machine: a Go1 Air does 2.5 m/s, a Pro 3.5-3.7, an Edu
sprints to 4.7. **This stack tops out at 0.83 m/s on its own sensors** - about a
sixth of the sprint.

> **Note on earlier revisions of this file.** Every 100 m time published here
> before 2026-08-22, and the claim that the GPS star mission ran "with no ground
> truth anywhere," were produced with cheater mode active. `getenv("SIM_CHEATER")`
> is non-null for `"0"`, so `SIM_CHEATER=0` still ENABLED it; separately, the
> sweep harnesses hardcoded `SIM_CHEATER=1` in their env block. Both are fixed.
> The star-mission time has not yet been re-measured honestly and is withdrawn
> until it has been.

## Reverse-engineering the factory controller

`docs/LEGGED_SPORT_REVERSE.md` documents the analysis of Unitree's shipped
`Legged_sport`: it is a direct MIT Cheetah-Software fork (MIT's source tree
verbatim in its DWARF paths, MIT licence shipped alongside), which makes it an
authoritative reference for parameterising this stack for a Go1. It corrected
six wrong constants in this port — including a knee gear ratio (9.4995, not
6.33) that removes a field this port had invented, and an MPC inertia tensor
that was under-estimating roll and yaw by ~25%.

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
