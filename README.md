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
| MIT_Controller (MPC + WBC) locomotion | ✅ **runs** — 100 m dash in **24.8 s at 4.70 m/s** under `trotRunning`; 6 of 8 gaits complete a mission |
| OpenPilot waypoint navigation | ✅ **100 m GPS star, 13/13 at 42.6 s**, judged PASS (settles + lies down) — real estimator, cheater mode deleted |
| Mac-first host build | ✅ same source builds natively (`-DSTM32MP1_HOST=ON`) for fast iteration |
| Robot model vs the real Go1 | ✅ **corrected against Unitree's own binary** (see `docs/LEGGED_SPORT_REVERSE.md`) |

## The 100 m star mission — repeatable

5 legs of 20.0 m, five 144° corners, GPS waypoint navigation, real state
estimator. Ends with a judged PASS: the robot decelerates, settles upright, and
lies down under damping. Arriving at the last waypoint is not finishing.

| cruise | passes | time | course speed |
|---|---|---|---|
| **2.0 m/s** | **13 / 13** | **42.5–42.7 s** | 2.35 m/s |
| 2.5 m/s | ~67% | 41.5–41.8 s | 2.41 m/s |
| 3.0 m/s | ~40% | 40.7–40.8 s | 2.46 m/s |

2.5 m/s is one second faster and fails one run in three. **2.0 m/s has never
failed** — that is the number to quote.

```bash
cd host-run && env DYLD_LIBRARY_PATH=. \
  SIM_GAIT=9 SIM_VX=2.0 SIM_VX_DELAY_S=4 SIM_VX_RAMP_S=8 \
  WP_MISSION=star:10.514:5 WP_ACCEPT=1.5 WP_MAX_YAWRATE=1.2 WP_PLANNER=1 \
  ./mit_ctrl_sim 127.0.0.1 stm32mp1-defaults.yaml mc-mit-ctrl-user-parameters.yaml
```

### The open problem, stated honestly

Above 2.0 m/s the robot fails by **collapsing** — `roll=0 pitch=0`, flat — not
by tipping. The only quantity separating a passing run from a failing one is
body height (0.212–0.239 m passing, 0.185–0.200 failing); commanded force is
identical in both. Eleven levers aimed at cornering (gait switching, banking,
crouch, angle grading, hairpin pivots, lateral budget, yaw authority, braking
rate, acceptance radius, lookahead, MPC segment) all failed to move it, because
they act on the **plan** and the failure is in force **delivery**.

Next step is measuring achieved vs commanded foot force through a corner — a
whole-body-controller question, not a planner one.

## Measured locomotion (Mac SITL)

100 m dash on flat ground. Measured on the robot's **own state estimate** —
cheater mode is deleted from the codebase, so ground truth cannot reach the
control loop; it is used only to MEASURE distance.

### The 100 m star mission, every gait ranked

5 legs of 20.0 m (circumradius 10.514 m), real state estimator, with the
Apollo-derived body planner. "course" is 100 m of waypoint path over elapsed
time — the robot's own path is shorter, because the planner cuts corners inside
the 1.0 m acceptance radius.

| rank | gait | commanded | time | course m/s | runs |
|---|---|---|---|---|---|
| **1** | **`trotting`** | 2.0 m/s | **44.4 s** | **2.25** | **3/3** |
| 2 | `trotRunning` | 1.5 m/s | 54.8 s | 1.82 | 1/1 |
| 3 | `walking` | 1.5 m/s | 56.4 s | 1.77 | **3/3** |
| 4 | `bounding` | 1.5 m/s | 57.2 s | 1.75 | 1/3, marginal |
| 5 | **`galloping`** | 0.8 m/s | 103.7 s | 0.96 | 1/1 |
| — | `walking2`, `pacing`, `pronking` | — | no completion | — | 0 |

**Galloping runs a waypoint mission** — it had never travelled more than ~13 m
in this port's history. It needed both the solver fix (so a 40% duty gait gets
the `m·g/duty` it needs) and the planner (so it is never asked to corner at a
speed its flight phase cannot redirect from).

### Straight-line speed does NOT predict mission speed

| gait | duty | 100 m dash | 100 m star |
|---|---|---|---|
| `trotRunning` | 40% | **4.70 m/s — 1st** | 1.5 m/s — 2nd |
| `trotting` | 50% | 3.46 m/s — 2nd | **2.0 m/s — 1st** |
| `galloping` | 40% | never crosses | 0.8 m/s |

The fastest gait in a straight line is the worst at cornering. A 40% duty gait
is fully airborne ~20% of every cycle, and a body in flight has no feet to push
against — it cannot redirect itself, generate yaw authority, or arrest roll. The
flight phase that buys top speed is the same thing that costs turning authority.

### The 100 m dash, every gait, after fixing the solver

Fastest reliable speed for each of MIT's eight gaits, all on the real state
estimator with qpOASES. 2.0 m/s is the qualifying floor — a gait that cannot
hold 2.0 is out of the running for "fastest". Every entry is repeated; the
`runs` column is the count that actually crossed.

| rank | gait | num | commanded | segment | 100 m | cruise | runs |
|---|---|---|---|---|---|---|---|
| **1** | **`trotRunning`** | 5 | **4.0 m/s** | 26 ms | **24.8 s** | **4.70 m/s** | **3/3** |
| 2 | `trotting` | 9 | 3.1 m/s | 22 ms | 32.2 s | 3.46 m/s | **5/5** |
| 3 | `walking` | 20 | 2.25 m/s | 22 ms | 42.0 s | 2.57 m/s | **3/3** |
| — | `bounding` | 1 | — | — | 99.9 m of 100 | — | 0/2 |
| — | `galloping` | 22 | — | — | fails 12.8 m | — | 0/2 |
| — | `pronking` | 2 | — | — | fails 10.6 m | — | 0/2 |
| — | `walking2` | 21 | — | — | fails 5.6 m | — | 0/2 |
| — | `pacing` | 8 | — | — | fails 0.4 m | — | 0/2 |

**`trotRunning` wins at 24.8 s / 4.70 m/s**, three runs for three, zero falls.
`trotting`'s full confirmed ladder: 3.1 (5/5, 32.2 s), 3.0 (3/3, 33.2 s), 2.75
(2/2, 37.5 s), 2.5 (3/3, 40.5 s), 2.0 (4/4, 47.1 s).

**A flight phase is worth 36%.** The two fastest gaits are `trotRunning`
(flight phase, 4.70 m/s) and `trotting` (no flight, 3.46 m/s). Trot is capped by
how far a stance foot can sweep before running out of leg — the same geometry
that makes the segment speed-dependent — and a flight phase removes that
constraint. This is measured, not argued.

Above each gait's reliable ceiling the behaviour goes **stochastic**, and single
crossings there mean nothing: `trotRunning` at 4.5 crossed once at 22.5 s
(5.29 m/s) then failed twice (94.9 m, 78.3 m) — 2 of 4; `trotting` at 3.15
crossed once at 31.7 s then failed three times.

> ### Read the top-end numbers with scepticism
>
> 4.70 m/s matches the Go1 Edu's rated sprint on a 12.8 kg machine, and the
> stochastic band above it reached 5.29 m/s. A controller in SITL matching or
> beating the manufacturer's flat-out figure says more about the simulation than
> the controller. Two specific reasons:
> - **foot friction is μ=2.0** in these worlds (set earlier to stop feet
>   skating); the URDF says **0.6**. At 4–5 m/s traction is doing enormous work.
> - **no actuator dynamics**: commanded torque is applied directly, with no
>   motor current limit, no thermal derating, no RS485 latency and unbounded
>   joint velocity.
>
> The defensible claim is narrower and still large: **the controller is no
> longer what limits this port.** None of these figures are hardware-validated.

### Above 3.1 the gait goes stochastic (repeat before believing)

| commanded | crossings | note |
|---|---|---|
| 3.15 | **2 of 5** | crossed at 31.7 s once, then failed at 69.9 / 35.6 / 85.2 m |
| 3.17 | 0 of 1 | |
| 3.18 | 1 of 1 | crossed at 31.5 s, 3.57 m/s — single sample, inside the marginal band |
| 3.2 | 0 of 2 | fails at 31–38 m |

3.15 was very nearly published as a 31.7 s record off its first run. It does not
reproduce. Anything above 3.1 m/s here is a coin flip, and a single crossing in
that band means nothing — this project has been burned by exactly this before
(bounding at 1.0 m/s, trot's ladder understating it by 50%).

### The gait segment is SPEED-DEPENDENT (and was on the "ruled out" list)

3.0 m/s only works at a 22 ms MPC segment. The shipped 26 ms fails at 67 m:

| segment | cycle | 3.0 m/s result |
|---|---|---|
| 18 ms | 0.18 s | fails, 15.6 m |
| **22 ms** | **0.22 s** | **100 m in 33.2 s** |
| 26 ms (default) | 0.26 s | fails, 67.3 m |
| 30 ms | 0.30 s | fails, 24.8 m |

A sharp optimum, and it moves with speed — 2.0–2.75 m/s run best at 26 ms.
The reason is geometric: stance duration × commanded speed is the distance the
stance foot must sweep, so a faster gait needs a shorter cycle to keep the
sweep inside the leg's ~0.318 m horizontal reach. Cycle time was on this
project's *"swept without success, do not spend time on this again"* list —
swept while the solver was starving the robot of force.

### Each speed tier fails differently — do not re-fix the solved one

| speed | failure | evidence | status |
|---|---|---|---|
| ≤2.75 | force starvation | `Fz/mg = 0.25` vs 2.00 needed; sank flat to z=0.028 | **fixed** — qpOASES |
| 3.0 at 26 ms | vertical oscillation | force fine (2.45–2.64×), body vz ±0.5–0.7 m/s | **fixed** — 22 ms segment |
| >3.1 | at the machine's limit | achieved cruise saturates ~3.5 m/s; commanding more just overshoots | **open** |

A correction worth recording: the tier above 3.1 was first investigated as a
"3.5 m/s wall" and ~20 configurations were swept there (segment, horizon,
orientation gains, swing clearance) — all flat, all failing at 20–28 m. That was
the wrong question. Commanded and achieved speed differ (3.0 commanded cruises
at 3.46 m/s), so every one of those runs was asking for ~3.9 m/s, past the
machine's ceiling, and could not have succeeded whatever the parameter. A target
the robot cannot reach makes every lever look inert.

Eliminated by measurement at that tier, and still valid as facts: orientation
gains (`Kd_ori` 40/60/80 → 27.7/27.5/26.9 m, flat), swing clearance (0.11→27.7,
0.14→22.1, 0.17→13.8 m), joint torque (peak knee 26.4 of 35.55 N·m), leg reach
(max 0.393 of 0.430 m), and contact timing — new `[CONTACT]` instrumentation
shows the feet are on the ground when the schedule says so at both 3.0 and 3.5
(0.0% vs 0.1% of stance samples airborne).

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

### The estimator was blamed for this, and it was not the cause

Under the broken solver, feeding the estimator ground truth bought one to two
speed rungs on every gait, which looked conclusive:

| gait | ground truth | real estimator |
|---|---|---|
| `trotting` | 1.0 m/s, 106.5 s | 0.6 m/s, 185.8 s |
| `walking2` | 1.0 m/s, 107.0 s | 0.8 m/s, 121.9 s |
| `walking` | 0.8 m/s, 130.7 s | 0.6 m/s, 162.4 s |
| `pacing` | 0.8 m/s, 131.1 s | 0.6 m/s, 183.5 s |

**That conclusion was wrong.** `[ESTERR]` (`SIM_ESTERR=1`) logs the estimate
against truth in the same body frame — truth logged, never fed to the
controller — and the LinearKF tracks forward velocity to within **0.13 m/s**
right up to the moment of the fall (true 1.101 vs estimated 1.029 at the worst
point). The large errors appear only *after* the robot is down and integrating
phantom motion. Consequence, not cause.

What ground truth was actually doing was masking the force deficit: a robot
walking crouched at 0.13 m is close enough to failing that any extra error
tips it over, so removing estimator error postponed the collapse without
addressing it. With the solver fixed, `trotting` runs at **3.0 m/s on its own
sensors** — 3× faster than it ever managed *with* ground truth under JCQP.
The estimator was never the wall.

Reality check against the machine: a Go1 Air does 2.5 m/s, a Pro 3.5-3.7, an Edu
sprints to 4.7. **This stack now cruises at 3.46 m/s on its own sensors** —
past the Air's rating, inside Pro territory, and 74% of the Edu sprint.

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
