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
| MIT_Controller (MPC + WBC) locomotion | ✅ **runs** — 100 m in **40.4 s at 2.65 m/s** under `trotting`, on the real estimator, no falls |
| OpenPilot waypoint navigation | ⚠️ **GPS star mission completes under convex MPC** (5 × 10.1 m legs) — timings were taken in cheater mode; awaiting an honest re-measure |
| Mac-first host build | ✅ same source builds natively (`-DSTM32MP1_HOST=ON`) for fast iteration |
| Robot model vs the real Go1 | ✅ **corrected against Unitree's own binary** (see `docs/LEGGED_SPORT_REVERSE.md`) |

## Measured locomotion (Mac SITL)

100 m dash on flat ground. Measured on the robot's **own state estimate** —
cheater mode is deleted from the codebase, so ground truth cannot reach the
control loop; it is used only to MEASURE distance.

### The dash, after fixing the solver

| gait | speed | 100 m time | cruise | runs |
|---|---|---|---|---|
| **`trotting` (9)** | **2.5 m/s** | **40.4 s** | **2.65 m/s** | crossed, confirmation running |
| **`trotting` (9)** | **2.0 m/s** | **47.1 s** | **2.24 m/s** | **4/4 crossed — 47.1/47.2/47.1/47.2 s** |

The four runs at 2.0 m/s agree to **0.1 s (0.2%)**, with **zero safety trips,
zero falls**, body height held at mean 0.287 m against a 0.300 m reference, and
a worst control-loop iteration of 1.02 ms against a 2.0 ms budget. `trotting` at
3.0 m/s reaches 81 m (marginal, under repeat); 3.5 and 4.0 fail at ~24 m.

**2.65 m/s cruise exceeds the Go1 Air's 2.5 m/s rating** and reaches into Go1
Pro territory (3.5–3.7 m/s). The previous honest figure for this port was
0.53 m/s.

### What changed: the QP solver was never converging

The stack shipped `use_jcqp: 1`, adopted as a speed win on the STM32 (82 ms vs
qpOASES' 198 ms) and validated by a sweep run against a **standing** trot. Under
a *moving* gait it does not reach the optimum, and the consequence is that the
MPC asks for a fraction of the force needed to hold the body up:

| solver | 2-foot Fz/mg | 4-foot Fz/mg | mean body z |
|---|---|---|---|
| JCQP `rho 0.6/60` (shipped) | 0.25 | 0.45 | 0.128 m |
| JCQP `rho 2.0/300` | 0.38 | 0.69 | 0.132 m |
| **qpOASES** | **1.27** | **1.76** | **0.275 m** |
| *required* | *2.00* | *1.00* | *0.300 m* |

A gait with stance duty `d` must command `m·g/d` while its feet are down. JCQP
commanded about a fifth of that, so the robot walked permanently crouched at
0.13 m and sank until it collapsed flat (`roll=0 pitch=-0 z=0.028`) — not
tipped over. Five times the iterations barely moved it, so this is not tuning.

This also retires the project's longest-standing open question — *"why is the
MPC satisfied at z=0.204 when its reference says 0.30?"* It was never a cost
weight, never MIT's `Kp_stance = 0`, and never the state estimator (which
`[ESTERR]` shows tracks forward velocity to within 0.13 m/s right up to the
fall). The solver was returning a fifth of the required force, and every gait
number this project recorded before now was measured on top of that.

Caveats, both measured: qpOASES is **not** a universal win — `walking2` failed
at every speed tried with it, where JCQP crossed at 0.8 m/s. And qpOASES cost
198–218 ms on the STM32's A7, so it cannot run inline there against a 26 ms MPC
segment; the board needs the async path or a re-measured reduced solve.

### Previous results (JCQP, superseded)

Kept because they are what every earlier claim rests on:

| gait | speed | 100 m | cruise |
|---|---|---|---|
| `walking2` (21) | 0.8 m/s | 121.9 s | 0.83 m/s |
| `walking` (20) | 0.6 m/s | 162.4 s | 0.61 m/s |
| `pacing` (8) | 0.6 m/s | 183.5 s | 0.54 m/s |
| `trotting` (9) | 0.6 m/s | 185.8 s | 0.53 m/s |
| `trotRunning` (5) | — | never crosses | — |

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
sprints to 4.7. **This stack now cruises at 2.65 m/s on its own sensors** —
past the Air's rating, inside Pro territory, and about 56% of the Edu sprint.

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
