# CLAUDE.md — STM32MP1 Cheetah port: rules, architecture, and traps

Read before changing the port. Companion to `SKILL.md` (commands) and `README.md`.

## The board (Octavo OSD32MP1-RED)

- STM32MP157: **dual Cortex-A7, armv7l 32-bit**, NEON/VFPv4; + a Cortex-M4 (unused here).
- OpenSTLinux (Yocto dunfell), kernel 5.10.10 **PREEMPT (not PREEMPT_RT)**, **glibc 2.31**,
  gcc/g++ 9.3.0. **426 MB RAM, no swap.** `/usr/local` has room; `/` is tight.
- On-board it has gcc/make/python3 but **no cmake, no Eigen, no LCM, no `mkswap`/`swapon`**.
- Reach it by IP (mDNS `.local` has been flaky): current lease has been `192.168.0.90`.
  The Mac's LAN IP (Gazebo/bridge host) has been `192.168.0.75`. **DHCP moves — re-check.**

## Build: cross-compile on the Mac, never on the board

Native on-board builds thrash (426 MB, no swap) and lack cmake/Eigen/LCM. Instead:

- Toolchain: **`brew install arm-unknown-linux-gnueabihf`** (messense tap; gcc 15, glibc 2.28
  base). Link **dynamic glibc** (2.28 ≤ board's 2.31, so it runs) + **static libstdc++/libgcc**
  (the board only has gcc-9's runtime). See `stm32mp1/toolchain.cmake`.
- `stm32mp1/build.sh` runs cmake with `-DSTM32MP1_BUILD=ON -DCMAKE_TOOLCHAIN_FILE=... -DCMAKE_POLICY_VERSION_MINIMUM=3.5`.
- The `STM32MP1_BUILD` CMake path: Cortex-A7 flags (**no `-march=native`**), **no `-Werror`**,
  `NO_SIM`, static `biomimetics`, vendored Eigen, gates out Qt/EtherCAT/VectorNav/LORD/qpOASES/Goldfarb.

### TRAP: Eigen + ARMv7 NEON alignment
The MIT/Eigen code assumes x86 tolerates misaligned SIMD; **ARMv7 NEON faults** on it and the
kernel can't fix NEON alignment traps → `dmesg`: `Alignment trap: not handling instruction`.
Fix (already applied, in all TUs): build with **`-DEIGEN_MAX_ALIGN_BYTES=0 -DEIGEN_MAX_STATIC_ALIGN_BYTES=0`**.
It's ABI-sensitive → any change needs a **clean rebuild**.

### TRAP: JCQP hand-written x86 AVX2
`third-party/JCQP/Cholesky*Solver.cpp` used `<immintrin.h>`/`__m256`. Replaced by
`third-party/JCQP/simd_compat.h` (scalar/NEON, `std::fma` to keep fused-rounding numerics).

### TRAP: gcc-15 template bodies
`Goldfarb_Optimizer/Array.hh` `GMatr::operator=` references non-existent members → gcc-15 errors.
Goldfarb is gated out of the port for now (only MIT_Controller's WBC needs it — fix it there).

### LCM without glib
The robot code only uses `lcm::LCM`/`ReceiveBuffer` with publish/subscribe — it never encodes.
So `stm32mp1/lcm_shim/lcm/lcm-cpp.hpp` is a **null LCM** (no-op templated pub/sub) and the LCM
types are hand-written **POD** headers in `lcm-types/cpp/*.hpp` (force-added; the repo ignores that dir).
No glib, headless. Swap in a real UDPM shim later for networked operator tooling.

## Runtime architecture

`RobotRunner` (unchanged MIT control loop) reads `SpiData` / writes `SpiCommand` and reads
`VectorNavData`. `Stm32mp1HardwareBridge` wires those to a **Backend**:
- **HARDWARE**: `rt_unitree` (RS485 motors) + `rt_can_imu` (DroneCAN IMU).
- **GAZEBO**: `rt_gazebo` (UDP to the Gazebo bridge). Set via `setGazebo()`.

`SpiData`/`SpiCommand` (SpineBoard.h) are **layout-identical** to `spi_data_t`/`spi_command_t`
(static_assert + memcpy). Leg order 0-3 = FR,FL,RR,RL; joint 0/1/2 = abad/hip/knee.

### Unitree RS485 (`rt_unitree`) — NOT hardware-validated
A1/B1 protocol vendored at `third-party/unitree_motor_sdk` (Unitree ships no armv7 lib/source).
Serial is MP1-specific: custom baud via `termios2/BOTHER`, hardware RS485 DE via `TIOCSRS485`.
**Validate on a bench when the fast RS485 adapter arrives**: field scalings (T×256, W×128,
Pos×16384/2π, K_P×2048, K_W×1024), FOC mode value, CRC word count `(sizeof>>2)-1`, per-joint
gear(9.1)/sign/offset.

### CAN IMU (`rt_can_imu`)
Reads the OP Revo Redux **compact single-frame DroneCAN stream** on `can0`: **msg 20500 gyro,
20501 accel**, int16[3] LE raw counts (dps=raw/16.4 ±2000dps; m/s²=raw·9.80665/16384 ±2g).
Madgwick AHRS synthesizes the quaternion. Filter by **message type only** (`imu_node_id=-1`) —
the node id is allocator-assigned (seen as 123, not the docs' 124).

**KEY FINDING:** the compact stream is **kept alive by `fw_realposix` (ninjapilot.service)** and
is stateful/intermittent — stop realposix and it dies. For standalone hardware use, Cheetah needs
its own **DroneCAN participant** that keeps the stream alive and can **tear down + restart an
already-running stream** (deferred; Gazebo sim sensors sidestep this for development).

## Gazebo SITL (Go1)

Gazebo Sim 8 (gz-harmonic) on the **Mac**; controller on the **board**; UDP between.
`make_world.py` turns `unitree_ros` `go1.urdf` into a gz world with IMU + baro + GPS sensors,
a joint-state publisher, and 12 force inputs. `cheetah_gazebo_bridge.py` runs the Unitree motor
PD (`τ = kp·Δq + kd·Δq̇ + τ_ff`) and speaks UDP to the controller (`:9100` cmd in, `:9101` sensors out).
Joint map is **identity** (Go1 hip/thigh/calf = Cheetah abad/hip/knee) and stands correctly; tune
`SIGN`/`OFFSET` in the bridge for other conventions. Baro/GPS reach the controller via
`gazebo_get_aux()` — reserved for waypoint nav, not consumed by control today.
gz Python bindings: system python3.14 lacks them; use the OpenPilot venv or **python3.13**.

### Debugging on the board
- Kernel logs **alignment traps** to `dmesg` (not plain segfaults). `dmesg -c` to clear first.
- systemd-coredump has Storage=none, so `echo /tmp/jc-core.%p > /proc/sys/kernel/core_pattern`
  + `ulimit -c unlimited`, run until it crashes, then `gdb bt` (board has gdb 9.1). Binaries are
  `-no-pie`, so the dmesg `ip` maps directly. Push the **unstripped** `mp1-build/robot/*` for symbols.
- Config files load relative to the run dir (`THIS_COM=""`). `JPosInitializer` needs
  `config/initial_jpos_ctrl.yaml` present or it OOB-crashes on an empty target vector.

## Don't

- Don't build on the board. Don't use `-march=native`. Don't change the Eigen align flags without
  a clean rebuild. Don't re-enable Goldfarb/qpOASES/SOEM/vectornav in the port without gating them
  so they can't break the working `jpos/sim/stand` build. Don't trust the DroneCAN node id from docs.

## Locomotion findings (Aug 2026): how the Go1 got from A to B

**WORKING**: `static_gait_sim` — statically-stable crawl (analytic IK + lateral CoM
shift, pure joint PD at 500 Hz, abstract convention). Walked 2.1 m upright in 66 s,
then a 10-min endurance run. Knobs: `SG_VX/SG_T/SG_SHIFT/SG_LIFT/SG_H/SG_TURN`.
Run the bridge with `BRIDGE_CONV=mit`. This is the A→B workhorse for waypoint nav.

Fixes that got the MIT stack (and everything else) healthy — each was load-bearing:
- **Joint convention**: the MIT stack works in the Cheetah *abstract* leg frame;
  the Go1 URDF differs by hip/knee sign. `BRIDGE_CONV=mit` in the bridge applies
  `SIGN=[1,-1,-1]` per leg to q/qd/tau symmetrically. Pure-PD controllers mask a
  wrong map (PD is sign-consistent automatically); only tau_ff exposes it.
- **Joint damping/friction in the sim**: the URDF ships `damping=0 friction=0`;
  real actuators reflect ~0.4 N·m·s/rad (MIT: 0.01 rotor × 6.33²). Without it every
  force-based controller (WBIC balance, MPC loco) rings up at ~1.5 Hz and flips.
  Worlds now carry `damping=1.0 friction=0.2` per joint (and foot μ=2.0 vs the
  URDF's 0.6, which let feet skate inward and collapse the support polygon).
- **convexMPC hard-codes mini-cheetah body**: `RobotState.h m=9`, inertia
  `(.07,.26,.242)` — gated Go1 values (13.1 kg, scaled inertia) behind
  `USE_GO1_MODEL`. With m=9 the MPC under-supports the Go1 by ~30% and it
  free-falls at gait start (`imu_az≈3.3` signature).
- **FSM staging**: enter LOCOMOTION *through* BALANCE_STAND (sequencer does
  1→3→4, `SIM_BAL_S`). Jumping 1→4 makes the MPC see a 9 cm height step and
  command ~2× bodyweight (a leap). Velocity must be *ramped* (`SIM_VX_RAMP_S`),
  never stepped.
- **JPos crouch**: old MIT crouch (±0.6 abad splay) rested the trunk on the ground
  and let feet scrub — random per-run stance skew. New crouch (0,-1.3,2.5 abstract)
  keeps the trunk clear and feet under hips.
- **Swing stance width**: ConvexMPCLocomotion's `.065` lateral foot offset is
  mini-cheetah's abad link; Go1's is `.08` (patched via `_abadLinkLength`).
- **Cheater mode** (`SIM_CHEATER=1`): bridge streams sim ground truth
  (pos/quat/vWorld) in the sensor packet; Stm32mp1HardwareBridge feeds
  CheaterState. Used to prove the estimator was NOT the tip-over cause.
- **Debug**: `STM32MP1_EST_DBG=1` dumps `[EST]` (estimator) + `[LEG0..3]`
  (q, foot p, force/tau ff) at 20 Hz.

**STATUS of MIT MPC+WBC**: BALANCE_STAND is stable (with all of the above);
trot/walk gaits still tumble within ~1 s of gait start — all four legs fold in the
first 50 ms (whole-body command transient at gait entry, under investigation).
The convex-MPC A→B path is deferred; the static crawl carries the mission.

## Gaits, speed limits, and waypoint navigation (Aug 2026, part 2)

### Three gaits, and which to use
| binary | gait | measured | stability | use |
|---|---|---|---|---|
| `static_gait_sim` | statically-stable crawl, one leg at a time, lateral CoM shift | 0.06-0.15 m/s | **rock solid** (10-min runs, full circle missions) | waypoint missions, the mission workhorse |
| `trot_sim` | dynamic trot, diagonal pairs, Raibert footholds + attitude feedback | ~0.1 m/s sustained; upright to ~0.5 m/s cmd | marginal above 0.5 m/s cmd | speed work in progress |
| `mit_ctrl_sim` | MIT convex MPC + WBC | BALANCE_STAND only | trot/walk tumble ~1 s after gait start | reference implementation |

**Reality check on speed.** A Go1 does 2.5-3.7 m/s in the real world and 4.7 m/s
flat out, but that is the *hardware* limit, not this stack's. Two things cap us
well below that today, and neither is the A7's compute (the control loop runs
1.4 ms of a 2 ms budget):
  1. A statically-stable crawl can never be fast - it is a sequence of static
     poses by definition. 0.1-0.3 m/s is the ceiling for this gait class.
  2. The dynamic gaits (ours and MIT's) both destabilise well before Go1 speeds
     through the UDP SITL loop. Real speed needs force control that survives
     that latency - see below.

### Speed: what was won, and the wall that is left (measured)
Starting point was 0.1 m/s. Three fixes took the trot to **~1.1 m/s sustained**
(21.6 m straight, upright, roll 2.5 deg) - a 10x gain, all of them real bugs:
1. **Swing touchdown scrub.** In the body frame a stance foot moves backward at
   v, so a swing profile arriving with ZERO body-frame velocity lands moving
   FORWARD at v over the ground and brakes the robot every step. The swing is
   now a cubic Hermite whose end slopes equal the stance sweep rate.
2. **Landing slam.** `LIFT*sin(pi*s)` hits the ground at `pi*LIFT/T_sw` ~ 2 m/s.
   `LIFT*(1-cos(2*pi*s))/2` has zero vertical velocity at both ends.
3. **qdDes pinned at zero.** The joint D term then fights every intentional
   motion - at kd=3 and real swing speeds that is >10 Nm per joint of pure
   braking. qdDes now comes from differentiating the commanded angles.
Plus **stance force feed-forward** (each stance leg gets bodyweight/n through
J^T), which cut steady-state joint tracking error from 0.25 rad to 0.13.

### Where ~1 m/s comes from (literature vs this port)
| approach | robot | max speed |
|---|---|---|
| PD + ILC (learned feed-forward torque) | Go1 hardware | 0.4 m/s |
| Bezier curves + Cartesian impedance | Go1 | 1.0 m/s |
| **this port's hand-rolled trot** | Go1 sim | **~1.1 m/s** |
| contact-implicit MPC | Go1 | 3.0 m/s |
| Unitree factory controller (RL) | Go1 | 4.7 m/s |

The split is not position-vs-force control - the ETH RL policy also emits joint
target positions into a PD controller and still goes fast. The split is whether
the REFERENCE TRAJECTORY is dynamically consistent:
  * hand-drawn kinematic foot paths (this port, the Bezier work, PD-ILC) plateau
    around 1 m/s;
  * references from an optimal control problem - MIT's SRBM + QP, or ETH's
    variable-height inverted pendulum, `r_ddot = (r - x_cop)(h_ddot+g)/r_z + g` -
    reach 3 m/s and beyond.
Two specific deficiencies of the kinematic plan show up at speed: body height is
pinned at a constant H (a VHIPM lets it breathe, which is what buys a flight
phase), and foothold choice never reasons about where the centre of pressure has
to be for the CoM acceleration the gait is asking for.

**The wall: ~1.1 m/s, and it is structural, not tuning.** Measured cruise speed
is flat at 0.6-1.1 m/s no matter what is commanded (1.0 / 1.8 / 2.5 / 3.5 all
land in the same band, and above ~2 m/s commanded it simply falls over). Swept
without success: cycle time 0.24-0.52 s, duty 0.54-0.80, body height 0.24-0.28,
joint kp 60-320, kd 3-14, lift 0.06-0.10, stride/reach out to 0.30 m, and the
trot / pace / bound / pronk pair patterns (`TR_GAIT`).

Why it caps: this is a POSITION-controlled gait. The only thing that decides
ground reaction force is joint tracking error, so the controller cannot choose
how hard each foot pushes, cannot exploit a flight phase, and cannot reject a
touchdown impulse except by stiffening (which was measured to make it worse).
Cartesian impedance at the published Go1 gains (Kxd=1000 N/m, Bxd=44 Ns/m, via
`TR_KP_C`/`TR_KD_C`; joint PD at kp=120 Nm/rad is ~2700 N/m at the foot, nearly
3x stiffer) was also tried: 0.64-0.80 m/s, same band.

**The credible route to real-world speed is a dynamically-consistent reference** -
finishing MIT's convex MPC, which is already ported here and only tumbles ~1 s
after gait entry. This trot is a good 1 m/s waypoint gait and a poor sprinter.

### What made the dynamic trot work at all
- **Attitude feedback sign.** Extending the leg on the side that is DROPPING
  means `dz < 0` (foot further below the body), i.e.
  `dz = (kp*roll + kd*wx)*SIDE + (kp*pitch + kd*wy)*FRONT`. The opposite sign is
  positive feedback: the robot rolls onto the same side within ~2 s on *every*
  run - a suspiciously repeatable failure, which is the tell.
- **Duty factor > 0.5** (`TR_DUTY`, default 0.65). A pure trot (duty 0.5) is
  always on exactly two feet and free to roll about the support diagonal the
  whole time. Overlapping the pairs gives an all-four double-support window.
- **Heading hold** (`yawRef` integrating the commanded yaw rate). Without it the
  trot spirals and ends up travelling sideways relative to its start heading.

### Waypoint navigation (OpenPilot port)
`WaypointNav.{hpp,cpp}` ports NinjaPilot's `PathPlanner` arrival logic:
- **half-plane arrival** - a waypoint counts as reached once you cross the plane
  through it perpendicular to the inbound leg (within `corridor*accept_radius`),
  not just inside the acceptance sphere. Without it a vehicle that overshoots by
  centimetres is commanded back to the point and hooks around it;
- acceptance radius, plus optional **confirm-arrival** (speed under a threshold
  held for a dwell) for missions that want a precise stop on a corner;
- pure-pursuit steering to a yaw rate, easing speed near the point and when
  badly mis-aimed.
Position comes from **GPS** (`gazebo_get_aux()` -> equirectangular projection
about the first fix), which is exactly the path the real dog will use over CAN;
heading from the state estimator.

Run a mission: `WP_MISSION=circle:<radius>:<points>` or `outback:<metres>`,
with `WP_ACCEPT` (acceptance radius) and `WP_LOOP`. Verified: 3 m circle,
8 breadcrumbs, GPS-driven, upright throughout.

**Frame convention (easy to get backwards).** Gazebo world is ENU (x=East,
y=North). The dog spawns yaw=+90 deg so body-x points north. The estimator zeroes
its initial yaw, so `rpy[2]` is CCW-from-north and compass bearing (positive
toward EAST) is its NEGATIVE. The gait's turn command is CCW-positive, so the
nav's steering output is negated on the way in.

### SITL harness (use these, they are much faster than doing it by hand)
- `batch_test.sh <configfile>` - many gait configs against ONE gz server, with
  survival / distance / mean speed / yaw drift scoring per run.
  **`reset: {model_only: true}` is a NO-OP** (returns `data:true`, moves nothing)
  and silently invalidates every run after the first; `reset: {all: true}` works
  but occasionally aborts the server, so the harness verifies the reset actually
  happened and reloads the world when it did not.
- `record_video.py <out.mp4> <seconds>` - headless capture from the `chase_cam`
  sensor mounted on the trunk. The old `/gui/screenshot` polling popped a
  "Saved image to:" toast per frame and stalled the GUI.
- Spawn is `z=0.08`, belly on the deck, and every controller boots **limp** for
  1 s before standing - the real Go1 procedure (lie flat -> power on -> stand ->
  walk), which also makes each SITL run start from an identical settled pose.
- The farm world keeps `agriculture_world` **with collision**, so buildings and
  fences are solid; probe spheres measured the terrain around spawn as flat to
  ~1 mm over +-6 m, so footing is honest and the feet do not clip.

## Function audit: how these gaits sit against MIT's pipeline

Worth stating plainly, because the custom gaits deviate in one structural way
that had a real safety consequence.

**Shared and unchanged** (every binary in this port goes through it):
`Stm32mp1HardwareBridge` -> `RobotRunner` (PeriodicTask, 500 Hz) ->
`_stateEstimator->run()` -> `setupStep()` (`legController->updateData(spiData)`,
`zeroCommand()`, `setEnabled()`, `setMaxTorque()`) -> `JPosInitializer` until
initialised -> `_robot_ctrl->runController()` -> `finalizeStep()`
(`legController->updateCommand(spiCommand)`). Leg data, the estimator, the
Jacobian/FK, the torque assembly in `LegController::updateCommand` (tau_ff +
forceFeedForward + Cartesian PD through J^T, plus joint PD) and the
SpiCommand/SpiData wire format are all stock MIT.

**Where the custom gaits deviate.** `MIT_Controller::runController()` is a thin
wrapper around `ControlFSM::runFSM()`, which does:
1. `operatingMode = safetyPreCheck()` - `SafetyChecker::checkSafeOrientation()`,
   trips at `|roll|` or `|pitch| >= 0.5 rad`;
2. on failure force **ESTOP -> the PASSIVE state** (legs limp);
3. run the current FSM state, then `safetyPostCheck()`, which clamps
   `checkPDesFoot()` (desired foot position) and `checkForceFeedForward()`.

`StaticGaitController` and `TrotController` subclass `RobotController` directly
and write leg commands themselves, so they never enter `ControlFSM` and
inherited **none** of that: a fallen robot kept running its gait, grinding its
legs - fine in sim, gear-stripping on hardware.

**Closed** by `SafetyCheck.hpp`, which reproduces step 1 and 2 in the same
spirit: sustained attitude beyond a threshold latches a fault and the legs go
limp and stay limp until a deliberate restart. Attitude, not acceleration, is
the discriminator - a legged robot takes a genuine impact spike on every
footfall, so acceleration cannot separate "trotting hard" from "fallen over",
whereas a working quadruped is never on its side or its face. The trip is 35 deg
held 0.25 s (`SAFE_ROLL_DEG` / `SAFE_PITCH_DEG` / `SAFE_HOLD_S`), a little
looser than MIT's 28.6 deg because a hard corner touches that transiently.

**Still not reproduced from `safetyPostCheck()`**: the `checkPDesFoot` /
`checkForceFeedForward` clamps. The custom gaits bound foot targets in their own
IK (`clampf` on reach and abad angle) rather than through MIT's checker, so the
protection exists but is not the same code path.

## Waypoint steering: what fixed the long arcs

Three changes turned a mission that looped huge arcs and loitered on each point
into one that captures every waypoint at 0.28-0.29 m and cruises the legs
between them:

1. **Yaw by rotating the stance feet about the body axis**, not by differential
   stride length. For body yaw rate w, a foot under hip (hx,hy) must travel
   at -(w x r) = (+w*hy, -w*hx) in the body frame. Differential stride alone
   delivered 0.09 rad/s against a 0.6 rad/s command - the dog had to arc metres
   to come round. Body-axis rotation gives ~0.25 rad/s.
   **Clamp the per-step foot displacement**: an unclamped 1.6 rad/s command at a
   0.85 s cycle asks for 25 cm of lateral foot travel, saturates every joint and
   rotates *nothing* - the mission sat pinned for 90 s with yaw commanded at max.
2. **The nav-to-gait turn sign flipped with it.** Measured, not assumed:
   `SG_TURN=+0.6` now yields a CLOCKWISE turn (world yaw -217 deg in 15 s),
   which matches nav's compass-sense output, so it passes straight through. The
   old differential-stride turn was CCW-positive.
3. **Nav shaping**: acceptance radius 0.25 m (hit the point, not a buffer), no
   speed taper into the waypoint (tapering made it loiter on top and fire the
   arrival test late), and turn-first speed shaping - but with a floor, because
   a hard `v=0` pivot deadlocks whenever the achievable yaw rate is below the
   commanded one.

**Heading fusion is correct** and was ruled out by measurement: commanding a
steady turn moved Gazebo truth yaw 88 -> 0 deg while the nav's bearing went
0 -> +87 deg. Bearing = -yaw_est, so those agree.

## Terrain: still an open gap (SG_TERRAIN, default OFF)

The gaits command every foot to a fixed depth below the body, which assumes flat
ground. On the farm mesh the robot spawns on a 7.6 cm rise (measured by dropping
probe spheres) and gets levered up on one leg - high-centred enough that it
cannot pivot, which stalled a whole mission at zero waypoints. A real dog just
feels the ground and stops the leg where it lands.

A first attempt at this (`SG_TERRAIN=1`) probes downward on the back half of the
swing and remembers a per-leg ground height. It is **off by default because it
tips the robot**: contact is inferred from the measured knee angle versus the
previous *commanded* one, which is a lagged tracking error rather than a contact
force, so it false-triggers and latches wrong heights. Doing it properly needs
the swing-leg torque jump at touchdown (`LegController` already computes
`datas[leg].tauEstimate`) and a plane fit across the four contact heights
instead of four independent per-leg offsets.

## Applying the literature: ILC, contact detection, VHIPM

**Iterative Learning Control** (`SG_ILC=1`, from the Go1 PD-ILC work). A gait
repeats, so its tracking error is mostly repeatable - gravity on the rear legs,
leg inertia, sag under load - and a PD controller structurally cannot remove it:
a steady error is the price of the torque it is producing. ILC keeps a
feed-forward torque per (leg, gait-phase bin, joint) and folds in a slice of the
last cycle's error each time round (lp=0.20, ld=0.10, leaky, clamped +-12 Nm).
**Measured: mean |q - qDes| fell 0.1413 -> 0.0276 rad, a 5x reduction.**
Ground speed did not change (0.14 m/s either way) - the crawl's speed is limited
by gait geometry, not by tracking - but the legs now go where they are told,
which is what terrain following and real hardware need.
CAVEAT: ILC assumes a REPEATING trajectory. Turning and speed changes make the
learned table stale, so it should be keyed by command (the source work keeps a
torque library per commanded velocity) or frozen while the command moves.

**Contact detection.** The sim sends only q and qd - no joint torque - so
contact is inferred from kinematics: `LegController` runs FK every tick, so
`datas[leg].p` is where the foot actually is. Two attempts:
  1. knee angle vs the previous COMMANDED angle - a lagged tracking error, not
     contact. False-triggered and rolled the robot over.
  2. absolute foot-position error - also wrong: the legs lag several cm under
     load *everywhere*, so every leg "finds ground" in mid-air, stops
     descending, and the robot never stands (body pinned at 0.13 m).
  3. what it uses now: a RATE test - the command is still descending but the
     foot is not, held 80 ms. Immune to constant lag.
Feeding that into a per-leg contact-terminated stand (each leg presses down
until IT finds the floor, so the body ends up parallel to whatever it stands on)
is implemented behind `SG_TERRAIN=1`. It improves the stand on the farm mesh
(body 0.114 -> 0.183) but does NOT yet get the robot walking there.

**VHIPM foot placement** (`TR_VHIPM=1`, from the ETH RL+MBOC reference model).
`r_ddot = (r - x_cop)(h_ddot + g)/r_z + g`, read backwards, says where the foot
must go: to command CoM acceleration a, the centre of pressure must sit
`a * r_z / (h_ddot + g)` from the CoM - plant BEHIND to accelerate, AHEAD to
brake. Two things stop being free parameters:
  * the capture-point gain is `sqrt(r_z / (h_ddot+g))` = 0.169 s at a 0.28 m
    stance, where this gait had a guessed 0.08;
  * `h_ddot` is measured from the body's vertical acceleration, which is the
    part a constant-height gait throws away.
A/B at 1.0 m/s commanded: VHIPM 1.00 m/s vs hand-tuned 1.02 m/s - the derived
gain reproduces the hand-tuned one, which is a good validation of the model but
not a speed win on a gait whose ceiling is elsewhere.

**Published Go1 joint gains, for reference** (this port started at kp=120, the
stiff end, and high P specifically costs compliance on uneven ground):
GainAdaptor P=28 D=0.7 | PD-ILC kp=90 kd=4 | Bezier+impedance ~1000 N/m at the
foot (~45 Nm/rad equivalent). Exposed as `SG_KP` / `SG_KD`, default 70.

## Mission result (the thing this was all for)

Full 5-point star, GPS-driven, controller on the STM32MP1, **on the farm**:

```
[nav] reached wp00 (N=-2.43 E= 1.76) dist=0.29
[nav] reached wp01 (N= 0.93 E=-2.85) dist=0.30
[nav] reached wp02 (N= 0.93 E= 2.85) dist=0.28
[nav] reached wp03 (N=-2.43 E=-1.76) dist=0.30
[nav] reached wp04 (N= 3.00 E= 0.00) dist=0.28
[nav] MISSION COMPLETE
```

Every waypoint captured within 0.30 m of the point itself - not a generous
acceptance buffer - cruising at the commanded 0.28 m/s between them and pivoting
at the corners. Repro:

```bash
stm32mp1/gazebo/sim_up.sh worlds/go1_farm_flat.sdf --gui
python3 stm32mp1/gazebo/trail_daemon.py star:3:5 500 &
ssh $BOARD "cd /usr/local/cheetah-mp1 && WP_MISSION=star:3:5 WP_ACCEPT=0.3 \
  SG_VX=0.28 SG_T=0.85 SG_H=0.30 chrt -f 80 ./static_gait_sim $MAC"
```

NOTE the crawl is marginally stable and has real run-to-run variance: the same
command fell over twice on this world before completing cleanly. That variance -
not the world, not the terrain - explains several "the farm is broken" detours
in the log above. Treat a single failed run as noise and repeat it.

## Fixing MIT's ConvexMPCLocomotion (not reinventing it)

The custom gaits were a detour. MIT already ships trotting/bounding/pronking/
galloping/walking/pacing/trotRunning; the job is making THEM run on a Go1. What
was actually wrong, in order of how much it mattered:

1. **`locomotionSafe()` aborts locomotion on mini-cheetah geometry.**
   `FSM_State_Locomotion.cpp` limits lateral foot position to **0.18 m**, which
   is mini-cheetah's (abad link 0.062 m). The Go1's abad link is 0.08 m so its
   feet legitimately stand ~30% wider, the rear legs cross 0.18 m within ~1 s of
   gait entry, and **failing this check sends the FSM to RECOVERY_STAND, which
   folds all four legs**. Every "the MPC tumbles at gait start" note in this
   file was that check firing - not dynamics. Gated to 0.24 m for Go1.
   The same line also carries an upstream typo: `std::fabs(p_leg[1] > 0.18)`
   takes `fabs` of a *bool*, so only the positive side was ever tested.
2. **`walking` and `walking2` were unreachable.** The gait selector stops at 8
   and everything else falls through to `trotting`, so two of MIT's own gaits
   could not be selected at all. Now 10 = walking (4-beat), 11 = walking2
   (diagonal pairs at 7/10 duty, i.e. a 40% double-support overlap - the same
   property that made this port's hand-rolled trot stable).
3. Go1 constants gated where they were still mini-cheetah: body height
   0.29 -> 0.30 (ConvexMPCLocomotion x2, FSM_State_BalanceStand init), swing
   height .06 -> .07 and .05 -> .055, and the sparse-MPC solver's `mass = 9` /
   inertia (unused at `cmpc_use_sparse = 0`, but a 30% mass error waiting for
   whoever enables it).

**What is now proven working** (measured, not assumed):
- the MPC+WBC machinery itself is fine: `cmpc_gait=4` (standing) runs the full
  locomotion path with **0 safety trips** and holds 0.330 m;
- **the state estimator is exonerated** - cheater mode (sim ground truth fed
  straight to the estimator) still rolls over, so the fault is not estimation;
- **the MPC commands correct forces** - ~78 N vertical on each stance foot of
  the diagonal against a 128 N robot, and exactly 0 on the swing pair;
- **the torques reach the sim** - bridge at 500 cmd/s, knee 6.16 Nm, which is
  what the stance geometry needs for ~61 N at the foot.
- `cmpc_gait=10` (walking) at 0.3 m/s commanded: **1 safety trip in 19 s**,
  body at 0.321 m. Compare thousands of trips for trotting.

**What is still wrong**: `walking` is stable but does not translate, and rolls
over by 0.5 m/s commanded. With forces, state and transport all verified good,
what is left is contact timing / phase margin through the SITL loop. Gait period
was swept (`SIM_MPC_MS` 27/45/60) without fixing it.

## Why MIT's gaits could not run: the A7 cannot solve the dense MPC inline

Measured, after fixing everything else: **the convex-MPC solve costs 60-105 ms
on this Cortex-A7 against a 2 ms control period.** The control loop reports
`maxRuntime=62-105 ms` for the ticks that solve, i.e. ~30-50 missed control
periods, and it happens on the FIRST tick of LOCOMOTION. Stance legs run at
`Kp_stance = 0*Kp` by MIT's design (all support comes from MPC/WBC force), so
the robot free-falls through the stall, drops ~7 cm, and rolls out. That is the
"tumbles ~1 s after gait start" symptom, start to finish.

What it is NOT (all measured, all ruled out):
- not the QP iteration budget: `nWSR` 100 / 25 / 10 -> 105 / 103 / 104 ms;
- not the horizon: 10 / 6 / 4 -> 81 / 56 / 57 ms (so not O(n^3) QP work);
- not the WBC: `use_wbc` 1 / 0 -> 58 / 85 ms, the MPC path dominates either way;
- not unvectorised code, though that WAS a real bug - see below.

It is the dense linear algebra in `SolverMPC`: `qH = 2*(B_qp^T * S * B_qp + ...)`
is ~4M double-precision MACs at horizon 10, and this board has no FP throughput
to spare. MIT's x86 UP board did the same solve in 1-2 ms and never had to care.
The fix that keeps MIT's design is to run the solve ASYNCHRONOUSLY, which is
what MIT's own hardware does (MPC at 30-40 Hz, leg control at 500 Hz).

### Build bug found along the way
`third-party/qpOASES/CMakeLists.txt` did a blanket
`set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb -w")`, **clobbering the parent's flags**,
so the hottest code in the stack was compiled for a generic ARM baseline with
no `-mcpu=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard` and without the project's
Eigen alignment defines. Now appends under `STM32MP1_BUILD` instead. (It did not
move the MPC number much, but a third-party library silently dropping the
project's ABI-sensitive Eigen flags is a trap worth removing regardless.)

## Two more upstream bugs in MIT's locomotion

1. **Lateral capture point is ~22x too weak.** In `ConvexMPCLocomotion.cpp` the
   Raibert terms read
   ```
   pfx_rel = vWorld[0] * (.5 + bonus) * stance_time + ...
   pfy_rel = vWorld[1] * .5 * stance_time * dtMPC   + ...
   ```
   Same capture-point quantity, but y carries an extra `* dtMPC` (0.045 s). So
   sideways velocity goes essentially uncorrected by foot placement: measured
   `vB_y` climbing 0.013 -> 0.466 m/s over ~1 s while roll ran away. Removed.
2. **Gait numbers >= 10 are reserved.** `run()` does
   `if(gaitNumber >= 10) { gaitNumber -= 10; omniMode = true; }`, so adding
   `walking`/`walking2`/`galloping` at 10/11/12 silently selected
   trotting/bounding/pronking instead - an entire gait matrix measured the wrong
   thing. They now live at 20/21/22 and bypass the omni rewrite.

## Go1 adaptation of MIT constants (verified against the URDF)

The URDF matches Unitree's published spec exactly - abad +-49 deg, thigh
-39..258 deg, shank -161..-51 deg, 23.7 / 23.7 / 35.55 Nm, 13.101 kg total - and
`buildGo1()` matches the URDF. Corrections made:
- `_motorTauMax` is MOTOR-side (`ActuatorModel` does
  `tau_joint = gearRatio * clamp(tau_motor)`). It held 23.7, the JOINT figure,
  so the model believed 150 Nm per joint. Now 3.744 (23.70/6.33), plus a new
  `_kneeMotorTauMax = 5.616` (35.55/6.33) because the Go1's knee is 1.5x the hip
  at the SAME gear ratio, which MIT's single-value struct could not express
  (mini-cheetah used a bigger knee gear instead). NOTE: `buildActuatorModels()`
  is only referenced by unit tests, so this is model correctness, not behaviour.
- `_maxLegLength` 0.40 -> **0.385**: reach is set by the KNEE LIMIT, not the
  links. The shank cannot pass -51 deg, so the leg never straightens and
  hip-to-foot maxes at `sqrt(l2^2+l3^2+2*l2*l3*cos(0.888)) = 0.385 m`.
- MPC per-foot force cap 120 N -> **175 N**, holding MIT's own ratio (120 N for
  a 9 kg / 88 N mini-cheetah = 1.36x bodyweight) for the 13.1 kg / 128 N Go1.
  250 N was tried and made the stand sit lower.
- Entry height ramp: BALANCE_STAND settles the Go1 at 0.21-0.26 m while
  locomotion's nominal `_body_height` is 0.30, and handing the MPC that 9 cm
  step on the first tick made it LAUNCH the robot (measured z 0.211 -> 0.342 at
  0.32 m/s vertical, then a roll-out). `_body_height` now ramps from the height
  the robot is actually at. A mini-cheetah never sees this because its balance
  stand already sits at its locomotion height.

## Measured gait matrix (`gait_matrix.sh`, farm, 0.5 m/s commanded)

| gait | num | dist before fall | up for | outcome |
|---|---|---|---|---|
| standing | 4 | 0.08 m | 18 s | upright |
| walking | 20 | 0.18 m | 18 s | upright |
| pronking | 2 | 0.81 m | 13.8 s | fell |
| bounding | 1 | 0.34 m | 14.2 s | fell |
| pacing | 8 | 0.40 m | 3.9 s | fell |
| galloping | 22 | 0.29 m | 6.2 s | fell |
| trotRunning | 5 | 0.20 m | 6.7 s | fell |
| trotting | 9 | 0.15 m | 6.8 s | fell |
| walking2 | 21 | 0.24 m | 6.1 s | fell |

Every one of them is limited by the same 60-105 ms solve stall, not by gait
tuning. **Fix the async solve before tuning any of these numbers.**

## What the real Go1 runs (dumped from the robot, Aug 2026)

`/home/pi/Unitree_latest/autostart/sportMode/bin/Legged_sport` is a **direct fork
of MIT Cheetah-Software**. Its build paths are MIT's tree verbatim:
```
/home/pi/src_legged_sport/common/src/Controllers/FootSwingTrajectory.cpp
/home/pi/src_legged_sport/common/include/Dynamics/Quadruped.h
/home/pi/src_legged_sport/user/MIT_Controller/Controllers/convexMPC/dance.cpp
/home/pi/src_legged_sport/third-party/{qpOASES,Goldfarb_Optimizer,ParamHandler}
```
and it ships `reference_license/Cheetah-Software/LICENSE` (MIT, 2019 MIT
Biomimetic Robotics Lab) alongside drake, pinocchio, ocs2, towr, rbdl, quad-sdk,
dwl, xpp, free_gait and kindr. So this port and the factory controller are the
same codebase, which makes the robot a direct reference.

### The compute gap, measured

| | Go1 (factory) | this port |
|---|---|---|
| SoC | Raspberry Pi CM4, BCM2711 | Octavo OSD32MP1 (STM32MP157) |
| CPU | **4x Cortex-A72 @ 1.5 GHz** (part 0xd08) | **2x Cortex-A7 @ 650 MHz** |
| kernel | 5.4.81-**rt45** PREEMPT_**RT** | 5.10.10 PREEMPT (not RT) |
| relative compute | **~11x** | 1x |

(per core: 1.5/0.65 GHz x 4.7/1.9 DMIPS/MHz = 5.7x, times 2x the cores.)
Unitree also runs a genuinely real-time kernel; this board does not.

**This is the honest reason MIT's gaits do not simply run here.** It is not
missing constants - those are all fixed now - it is that the factory controller
has ~11x the compute and a PREEMPT_RT kernel to do the same job.

### Their thread structure (`ps -eLo pid,tid,cls,rtprio,comm`)

```
FF 95  x2      <- highest: comms / control
FF 90  x2
FF 85  x3
TS     x2      <- SCHED_OTHER, non-RT (background)
```
A three-tier RT priority stack plus two non-RT threads, over 4 cores. That
independently validates the structure this port arrived at the hard way - the
MPC worker MUST sit below the control loop (here: control/motor at FIFO 49, MPC
worker at FIFO 20 pinned to cpu1) - but they can afford to spread it over four
cores where this board has two.

### WBC decimation: fixes timing, does NOT fix height

`SIM_WBC_DECIM` runs WBIC every Nth tick and holds its joint commands in
between (it writes qDes/qdDes/kp/kd/tau, which the leg controller keeps applying
on skipped ticks). Measured:

| WBC rate | worst control loop | body z |
|---|---|---|
| every tick (stock) | 55.89 ms | 0.224 |
| every 5 ticks | **5.45 ms** | 0.214 |
| every 10 ticks | 6.63 ms | 0.223 |

So decimation buys back the control loop (56 -> 5.5 ms) at no cost in height -
but it disproves the hypothesis below: the body parks at ~0.22 whether the WBC
runs every tick, every fifth, or not at all. Something else is holding it down,
and BodyPosTask is not it.

### THE WBC IS NOT OPTIONAL

The binary contains `WBIC`, `LocomotionCtrl`, `BodyOriTask`, `LinkPosTask`,
`ContactSpec` and Goldfarb_Optimizer - 154 matching symbols. The factory
controller runs the **full MPC + WBIC stack**. This port currently sets
`use_wbc: 0` because WBIC costs ~50 ms/tick here, and that is very likely why
the body parks at 0.204 m against a 0.30 m reference: `BodyPosTask` /
`BodyOriTask` are exactly what servo body pose, and with `Kp_stance = 0` there
is nothing else doing it. Re-enabling the WBC - probably decimated to run every
N ticks rather than every tick - is the next thing to try.

### Gaits they actually ship

Trotting, Bounding, Pronking, Galloping, Standing, Trot Running, **Walking**,
**Walking2**, Pacing, Jumping, plus their own **JumpingTrot**. They use Walking
and Walking2 - the two MIT's stock selector makes unreachable through the
`gaitNumber >= 10` omni rewrite, which this port had to fix to reach them at all.

### Joint limits: the SDK disagrees with the URDF

`unitree_legged_sdk/include/unitree_legged_sdk/go1_const.h`:

| joint | URDF | Unitree SDK (enforced) |
|---|---|---|
| hip/abad | +-49 deg | **+-60 deg** |
| thigh | -39 .. **258** deg | -38 .. **170** deg |
| calf | -161 .. -51 deg | **-156 .. -48** deg |

The URDF's 258 deg thigh is not reachable in practice. Reach from the SDK's
-48 deg calf is 0.389 m (this port uses 0.385 from the URDF's -51 deg - close
enough to leave alone), and the wider +-60 deg abad means the lateral foot
limit can be more generous than the URDF implies.

### URDF vs reality (applied)
The abad limit is now **+-1.047 rad (+-60 deg)** in all four worlds, matching
`go1_const.h` rather than the URDF's +-0.863 (+-49 deg). The firmware on the
real machine allows +-60, lateral foot placement is the axis roll stability
depends on, and simulating it 11 deg tighter than the robot is a self-inflicted
handicap on exactly the wrong axis. The thigh's URDF range (-39..258 deg) is
left alone for now even though the SDK enforces -38..170 - it is permissive
rather than restrictive, so it costs nothing until a controller tries to use it.
Still worth getting from the real dog: measured joint damping/dry friction (the
URDF ships 0 and this port guessed 0.6/0.2), and real foot friction (URDF 0.6,
this port uses 2.0 to stop the feet skating).

### Per-robot drift calibration
`/var/local/unitree_legged_config.yaml` carries ONLY drift trim, calibrated per
machine - on the test dog: `walk_x -0.015, walk_yaw +0.002, run_x -0.03,
run_y +0.023, run_yaw +0.015`. Unitree does not eliminate residual drift, they
measure it per robot and subtract it. Worth remembering before chasing this
port's ~1.3 cm/s in-place crawl drift as a bug.

## Where MIT's locomotion actually stands now

The chain of failures, each found by measurement, in the order they had to be
fixed. Every one of them was a real bug, and none of them was gait tuning:

1. `locomotionSafe()`'s 0.18 m lateral foot limit (mini-cheetah's abad link)
   aborted LOCOMOTION into RECOVERY_STAND, which **folds all four legs**. This
   alone accounts for every earlier "the MPC tumbles at gait start" note.
2. Gait numbers >= 10 collide with MIT's omni rewrite, so walking/walking2/
   galloping at 10/11/12 silently ran trotting/bounding/pronking - an entire
   gait matrix measured the wrong gait. Moved to 20/21/22.
3. The lateral capture-point term is ~22x too weak upstream (`* dtMPC` on the y
   term only), so sideways velocity went uncorrected.
4. Locomotion stepped `_body_height` from BALANCE_STAND's actual 0.21-0.26 m to
   0.30, and the MPC answered by LAUNCHING the robot (z 0.211 -> 0.342 at
   0.32 m/s). Now ramped from the height it is actually at.
5. **The real blocker: the dense MPC solve costs 60-200 ms on this A7** against
   a 2 ms control period. Inline, it stalled the loop for 30-50 periods at the
   worst possible moment; with `Kp_stance = 0` (MIT's design - stance support is
   pure MPC force) the robot free-falls through the stall.

Fixed by running the solve on a worker thread (MIT's own hardware architecture:
MPC 30-40 Hz, leg control 500 Hz). **Worst control-loop iteration in locomotion:
5.33 ms, down from 56-105 ms.** Thread priority mattered as much as threading:
FIFO 49 (inherited) preempts the control loop and changes nothing; SCHED_OTHER
is starved to one solve per 28 s; FIFO 20 pinned to cpu1 works.

Two configuration wins worth knowing:
- **`use_jcqp: 1`** - MIT ships two solvers and this port was on qpOASES. JCQP
  solves the same problem in 82 ms vs 198 ms here. One yaml line, 2.5x.
- **`use_wbc: 0`** - the WBIC costs ~50 ms per tick on this board (async MPC
  with WBC: 56 ms/tick; without: 5.7 ms). MIT's non-WBC path is supported.

**Status: gaits enter locomotion cleanly and stay upright, but do not travel.**
The MPC updates at only ~5-13 Hz (73-200 ms/solve) against MIT's 30-40 Hz, and
its solutions come back all-zero. The velocity command is verified reaching it
(`pad=0.300 -> _x_vel_des=0.300`, correct gait id). A static-equilibrium
bootstrap now holds the robot up until the first solution lands, because with
Kp_stance = 0 there is otherwise NO support for the 73-240 ms that first solve
takes - the robot collapsed before its first MPC answer ever arrived, which is
why later solves saw a robot already on the floor.

### The zero-force problem: SOLVED (a race I introduced with the async solve)

`setup_problem()` calls `resize_qp_mats()`, which `setZero()`s S, fmat, qH, qg
and the rest. Both the control thread (in `solveDenseMPC`) and the new worker
called it, so the control thread was wiping the matrices the worker was midway
through building. The worker's problem came out with **|S| = 0 and |fmat| = 0** -
no state cost and no friction constraints - so the QP reduced to
`min alpha*||u||^2` and BOTH solvers correctly returned zero force. Not a solver
bug, not a model bug: a data race in this port's own threading.

Only the worker calls `setup_problem()` now. Measured immediately after:

| | before | after |
|---|---|---|
| `\|S\|` state cost | 0 | 162 |
| `\|fmat\|` friction | 0 | 34.6 |
| `\|q_soln\|` | 0 | 118-185 |
| stance forces | 0 N | -47, -68, -59, -65 N |

Those are correct ground reaction forces for a 128 N robot, and with them every
gait except pacing now survives a full 18 s run with zero safety trips.

### Old notes on narrowing it (kept for the method)

Everything feeding the QP is verified correct, and BOTH solvers still return an
all-zero force vector. Measured at the first solve, robot standing at 0.29 m:

```
[MPCIN]  p=0.14 0.00 0.29   v=0 0 0   yaw=-0.00  h=10
[MPCIN]  table[0..7]=1001 1101        traj[0..5]=-0.00 -0.00 -0.00 0.14 0.00 0.29
[MPCIN]  r(foot rel CoM) x=0.17 0.17 -0.20 -0.20   z=-0.29 x4
[MPCMAT] m=13.10  I=(0.1020 0.3790 0.3520)
         |A_qp|=11.6  |B_qp|=2.13  |X_d|=1.01  |U_b|=6.32e+11
```
State, contact schedule, reference, foot geometry, mass, inertia and the
prediction matrices are all sane and Go1-correct (`|U_b|` is MIT's BIG_NUMBER on
the unbounded friction-cone rows, not a bug). `x_0`'s 13th element is -9.8 as it
should be, so it is not a missing-gravity problem. And it is not the solver:
- JCQP: 70-100 ms/solve, all-zero forces;
- qpOASES: 198-218 ms/solve, 16 consecutive solves, all-zero forces.
It is also not the JCQP tuning, though that WAS separately wrong and is now
fixed: the yaml shipped `rho 1e-07` (JCQP default 2), `sigma 1e-08` (1e-5),
`terminate 0.1` (1e-3) - a tolerance so loose the solver can converge at its
zero initial guess. MIT never tuned them because their default is use_jcqp=0.

So: the QP is well-formed, both solvers agree, and the agreed optimum is zero.
The next step is to dump `q_soln` directly (rather than the rotated `f_ff`) to
confirm the solution really is zero rather than the readback being wrong, and
then to check `S` (the weight matrix) and `qg` - if `S` came out zero the cost
has no state-tracking term and zero force IS the correct optimum.

### The remaining wall, quantified

With the race fixed the MPC produces correct forces, and the limit is now purely
solve rate versus solution quality:

| horizon | solve time | MPC rate | stance forces | verdict |
|---|---|---|---|---|
| 10 (double) | 349 ms | ~3 Hz | -73 -60 0 -66 N | correct forces, far too slow |
| 10 (float) | 230 ms | ~4.3 Hz | -66 0 0 -67 N | correct forces, still too slow |
| 10 (float, rho 0.6, 60 iters) | **92 ms** | **~11 Hz** | **-67 0 0 -68 N** | correct forces, 3.8x faster |
| 6 | 97 ms | ~10 Hz | -19 x4 (76 N) | fast enough-ish, UNDER-SUPPORTS |
| 4 | 76 ms | ~13 Hz | -7 x4 (28 N) | badly under-supports |

The robot needs 128 N. A short horizon under-actuates because MIT's cost weights
were tuned against horizon 10, so shortening it is not free - it needs the
weights re-tuned, or the horizon kept and the solve made ~12x faster.
MIT runs this at 30-40 Hz.

Also seen and now guarded: the QP occasionally returns **non-finite** forces,
which propagate through J^T into joint torques and Gazebo rejects them
("Invalid joint force value [nan]"). On real hardware a NaN torque is undefined
behaviour, so the worker drops any non-finite solution and keeps the previous
one. It fired intermittently at horizon 6.

### Solver tuning: 349 ms -> 92 ms with the SAME answer

JCQP is ADMM, and ADMM convergence is dominated by `rho`. The shipped value was
`rho 1e-07` (JCQP's own default is 2) with `max_iter 10000`. Sweeping it against
the known-good answer (-66 N per stance foot of the diagonal, 128 N robot):

| rho | iters | solve | forces | note |
|---|---|---|---|---|
| 2 | 200 | 238-265 ms | -69 -66 -66 | MIT-ish default, converged, slow |
| 2 | 60 | 91 ms | -25 | under-converged |
| 2 | 25 | 67-77 ms | -11 | badly under-converged |
| 0.1 | 60 | 97-102 ms | -151 -149 | overshoots |
| 1.0 | 60 | 92 ms | -46 -46 | undershoots |
| **0.6** | **60** | **92 ms** | **-67 -68** | **converged, 2.6x faster** |

So `rho 0.6 / max_iter 60` reaches the same solution as `rho 2 / max_iter 200`
in a third of the time. With single precision on top, the horizon-10 solve went
349 ms -> 92 ms overall.

Two things that did NOT help, so do not spend time on them again:
- exploiting that `S` is diagonal (`S.diagonal().asDiagonal()` in the
  `B^T S B` triple product): 230 -> 223 ms, 3%. The cost is the ADMM
  iterations, not the matrix setup. The change is kept because it is free and
  numerically identical, but it is not a lever.
- `nWSR` on the qpOASES path, and the MPC horizon by itself.

### The height problem, narrowed (start here)

The MPC now commands correct forces at ~11 Hz and the robot still parks at
z = 0.204 against a 0.30 reference, upright, without travelling. What is ruled
out so far:
- the reference is right: `[MPC] bodyH=0.300` and `traj[5] = 0.30`;
- the estimate is right (and NaN is now guarded);
- the forces are right in MAGNITUDE: -67 N on each foot of the stance diagonal,
  134 N under a 128 N robot - but that is only ~5% above bodyweight, and 5% will
  not lift a body 10 cm;
- stance Cartesian stiffness does NOT fix it: `SIM_KP_STANCE` 0 / 0.15 / 0.4
  all park at 0.204-0.205. (MIT ships `Kp_stance = 0*Kp` - all support is meant
  to come from MPC force. The knob is left in place, defaulting to stock 0.)

So the question is why a z error of -0.096 m against a Q weight of 50 produces
only 5% extra force. Look at `X_d` over the whole horizon, not just the first
step - if the reference z is 0.30 at every step while the model predicts the
body cannot get there in one horizon, the optimiser will trade the z error away
against the input cost. Compare `pz_err` and `x_comp_integral` too, and try
raising Q[5] to see whether the solution responds at all - if it does not, the
z row of `B_qp` is the thing to inspect.

### Older note

The MPC now commands CORRECT forces at ~11 Hz, and the robot still sits at
z ~= 0.204 against a `_body_height` target of 0.30, so it stands crouched and
does not travel (best 0.38-0.67 m in an 18 s run). Commanding exactly bodyweight
holds a robot wherever it already is - to RISE it needs more, so the question is
why the MPC is satisfied at 0.204 when its reference says 0.30. Check, in order:
(a) that `trajAll`'s z entry really is 0.30 and not being overwritten by the
entry-height ramp, (b) the z weight in `Q` (index 5, value 50) against the
achieved error, and (c) whether `Fr_des`/`f_ff` is being applied to the legs
every tick or only on MPC update ticks.

**Next, in priority order:**
1. **Get the solve from 230 ms to ~30 ms.** Single precision is now DONE (349
   -> 230 ms, 1.5x: JCQP was instantiated `QpProblem<double>` with
   `.cast<double>()` on every matrix, even though convexMPC's `fpt` is already
   float and the A7 has no double-precision SIMD; `QpProblem<float>` and
   `CholeskySparseSolver<float>` are now instantiated, which needed AMD's
   diagnostic `Info` array pinned to double since its API is not templated).
   Remaining ideas, in order: profile inside `solve_mpc` to see whether the time
   is the dense triple product `B_qp^T S B_qp` or the ADMM iterations; exploit
   that `S` is DIAGONAL (MIT builds it with `S.diagonal() = ...`, so
   `B^T S B` should never be a full matrix multiply); cut `jcqp_max_iter` from
   200 once it is known how many iterations are actually used.
2. Or re-tune `Q` for horizon 4-6 so a short horizon still commands bodyweight -
   at horizon 6 the solver only asks for 76 N under a 128 N robot.
2. Get the solve under ~30 ms so the MPC can run at 30 Hz. Horizon barely moves
   it (10/6/4 -> 81/56/57 ms), and nor does `nWSR`, so the cost is the dense
   algebra (`qH = 2*(B_qp^T S B_qp + ...)`, ~4M double MACs at horizon 10).
   Single-precision, or a smaller state, is the lever - not solver tuning.
3. Only then tune gaits. Anything measured before the MPC runs at rate is noise:
   the same gait/config swings between 0 and 3000+ safety trips run to run.

## THE MIT TROT WALKS (end of the Fable session)

**Final measured result: 11.40 m of continuous ConvexMPC+WBIC trotting** on the
STM32MP1 (VX=0.4, ~60 s of walking at ~0.19 m/s delivered, curving left without
a heading hold, transport-VALID, cheater state). Progression within one
session: could not survive gait engage -> 0.83 m -> 1.37 m -> 2.51 m -> 5.92 m
-> 11.40 m.

The working configuration:
```
SIM_MPC_ASYNC=0          # inline again: the reduced solver fits MIT's segment
SIM_MPC_HORIZON=10  SIM_MPC_MS=36
SIM_WBC_DECIM=2          # WBC every other tick, outputs CACHED between runs
SIM_SWING_H=0.09         # swing clearance: the single biggest distance lever
SIM_VX_DELAY_S=4  SIM_GAIT_WAIT_MS (default 600)
yaml: use_wbc 1, use_jcqp 1, jcqp_rho 0.6, jcqp_max_iter 60
```

The three final killers, in the order they were unmasked (all found via the
Mac-side bridge dump - board-side printf on the FIFO control thread corrupts
the very runs it instruments):
1. **WBC decimation sent zeros.** setupStep() zeroCommand()s EVERY tick, so on
   WBC-skipped ticks the legs got zero gains/torque - 250 Hz healthy/zero
   chatter at decim 2, 80% zeros at decim 5. Fixed by caching the WBC outputs
   and rewriting them on skipped ticks. This alone took the trot from
   "collapses at engage, always" to walking.
2. **The board's eth0 PHY flaps** (one 107-minute outage; a re-drop 62 s after
   recovery). Runs that overlap a flap die exactly like controller bugs (the
   bridge watchdog folds the robot when commands stop). `run_valid.sh` gates
   every measurement on carrier-before + dmesg-link-delta-after. HARDWARE
   ATTENTION NEEDED: cable / port / PSU on the OSD32MP1's ethernet.
3. **Swing clearance**: the surviving walks died on foot scuffs; 0.07 -> 0.09 m
   swing height doubled then quadrupled the distance (SIM_SWING_H).

Solver work that made inline viable again (details in earlier sections):
349 ms -> 32 ms per solve = contact-only reduction (port of MIT's own
qpOASES-path elimination to the JCQP path) x single precision x rho 0.6/60
iters. 32 ms fits inside MIT's own 36-45 ms MPC segment, so MIT's synchronous
semantics are restored and the whole async apparatus (worker, pipeline,
bootstrap, kSeg) is now OPTIONAL (SIM_MPC_ASYNC=1) rather than required.

With the REAL estimator (no cheater): walks 0.65 m then falls - the LinearKF
under a trot on this transport is the next frontier.

## Mac-first development (Aug 2026): the same source, built natively

The board is ~11x slower, its eth0 flaps under sustained load, and every
iteration cost a cross-compile + scp + a ~90 s SITL run. So the port now also
builds NATIVELY on the development Mac and talks to the local Gazebo over
loopback UDP - the SAME source, the same `STM32MP1_BUILD` code paths, only a
different ISA:

```bash
cmake -B host-build -DSTM32MP1_HOST=ON -DSTM32MP1_MIT=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build host-build -j8 --target mit_ctrl_sim
# run from host-run/ with DYLD_LIBRARY_PATH=. (the RPATH is $ORIGIN, a Linux-ism)
```

`STM32MP1_HOST` turns on `STM32MP1_BUILD` (so every downstream gate, the LCM
shim, `THIS_COM=""`, `USE_GO1_MODEL` and the Eigen alignment defines are
identical) and only drops the Cortex-A7 arch flags and the two Linux-only
drivers (`rt_unitree` RS485, `rt_can_imu` SocketCAN). What had to be made
portable to get there:
- `Stm32mp1HardwareBridge.{h,cpp}`, `rt_gazebo.{h,cpp}` and the `rt_*` headers
  were wrapped in `#ifdef linux` as WHOLE FILES; the class and the UDP backend
  are portable, so the guards moved inward to just `mlockall`/`sched_setscheduler`
  and the hardware backends;
- the MPC worker's `cpu_set_t` affinity + `SCHED_FIFO` is now `#ifdef __linux__`.

### THE trap: `PeriodicTask` free-runs on non-Linux

`common/src/Utilities/PeriodicTask.cpp` uses a **timerfd**, and upstream MIT
guards the timerfd calls with `#ifdef linux` but leaves **no wait at all** in
the `#else` path. So every "periodic" task busy-loops. Measured on macOS: the
500 Hz control loop ran at **~1.9 MHz - 3700x real time** (28.3 MILLION log
lines in a 75 s run), which advances every gait timer, MPC segment counter and
integrator at nonsense rates; the robot fell over inside a metre and the FSM
looked like it was ESTOP-ing for no reason. Fixed with a portable
`std::this_thread::sleep_until` on an ABSOLUTE deadline (a relative sleep
accumulates the runtime as drift), re-basing rather than spinning to catch up
when a period is overrun. After the fix: 222 log lines, 500-501 cmd/s at the
bridge, zero transport stalls.

### What the Mac buys (measured)

| | STM32MP1 board | Mac (native) |
|---|---|---|
| worst control-loop iteration | 5.33 ms | **0.69-1.61 ms** |
| turnaround per data point | cross-compile + scp + run | one process start |
| transport | eth0 (flaps under load) | loopback, 0 stalls |

Caveats that keep the board authoritative: no `SCHED_FIFO`, so worst-case
period is 3.0 ms against a 2.0 ms target (mean is exactly right); and Gazebo
plus the controller share the CPU, so the sim's real-time factor swings
0.88-1.16. **Tune here, confirm there.**

### First result from the Mac harness: MPC segment time was capping the walk

`stm32mp1/gazebo/host_sweep.sh <configfile> [seconds]` runs many configs back
to back, headless, one fresh stack each (with the sensor pre-flight check that
a stale `gz sim` otherwise defeats). Five configs in ~7 minutes found this:

| config | distance | outcome |
|---|---|---|
| board best (`SIM_MPC_MS=36`, `WBC_DECIM=2`) | 2.23 m | fell @ 26 s |
| WBC every tick (`DECIM=1`) | 2.87 m | fell @ 26 s |
| **`SIM_MPC_MS=30`** | **18.86 m** | **upright to end of run** |
| `SIM_MPC_MS=26` | 19.83 m | upright to end of run |
| `SIM_MPC_MS=26`, swing 0.11 | 19.98 m | upright to end of run |

The 36 ms MPC segment was never a gait choice - it was forced on the board
because the reduced solve costs 32 ms and has to FIT INSIDE the segment to run
inline. That compromise is itself what capped the trot. With the segment at
MIT's own ~30 ms the trot stops falling over and the run ends because the clock
runs out, not because the robot does. **Backport consequence: the board needs
the solve under ~26 ms to run a 30 ms segment inline, or it must use the async
path (`SIM_MPC_ASYNC=1`) to decouple segment time from solve time.**

## Heading hold: upstream never had any

`ConvexMPCLocomotion::_SetupCommand` ends with

```cpp
_yaw_des = data._stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
```

which re-slaves the yaw REFERENCE to the yaw MEASUREMENT every tick. The
heading error the MPC ever sees is one timestep of commanded turn - exactly
zero when walking straight - so there is **no heading regulation at all** and
the robot keeps whatever direction it drifts into. That is the 11.4 m trot that
ended 5 m off course, and the same reason the sweep runs above drift 2-4 m east.

MIT integrates `world_position_desired` properly a few hundred lines later, so
position gets a real reference and heading does not. `_yaw_des` is now
integrated the same way (`_yaw_des += dt * _yaw_turn_rate`), initialised from
the measurement on `firstRun` and while the standing gait is active, unwrapped
onto the same branch as the measurement (rpy wraps at +-pi; an unwrapped
reference hands the MPC a 2*pi error and spins the robot), and saturated at
0.40 rad of correction so a disturbance cannot wind up into a violent turn.
`SIM_HEADING_HOLD=0` restores stock behaviour for A/B.

## Waypoint missions: real legs, no opening pivot

`.30 m isn't a test` - the star mission now runs at **r=5.3 m, i.e. 10.1 m
(33 ft) chords**, and `makeStar` rotates the whole pattern so **wp00 lies due
north**, which is the direction the dog spawns facing. The mission therefore
opens with a straight leg instead of a ~140 degree pivot in place - the crawl's
least stable move, and the one that faceplanted a live demo. `trail_daemon.py`
applies the identical rotation so the drawn plan still overlays the path.

Result, GPS-driven, controller on the STM32MP1, farm world, GUI up:

```
[nav] reached wp00 (N= 5.30 E= 0.00) dist=0.34
[nav] reached wp01 (N=-4.29 E= 3.12) dist=0.34
[nav] reached wp02 (N= 1.64 E=-5.04) dist=0.35
[nav] reached wp03 (N= 1.64 E= 5.04) dist=0.33
[nav] reached wp04 (N=-4.29 E=-3.12) dist=0.33
[nav] MISSION COMPLETE
```

Five 10 m legs, every waypoint captured inside 0.35 m of the point itself.


## Waypoint navigation for the MIT stack (not just the crawl)

The crawl drives nav from inside `StaticGaitController` because it owns its
gait. The MIT stack must not be reached into like that, and does not need to be:
`ConvexMPCLocomotion` already consumes a velocity and a yaw rate through
`DesiredStateCommand`, which is exactly the interface an operator's stick uses.
So `mit_sim_main.cpp`'s `navThread()` writes the SAME two channels the bridge
sequencer and a gamepad write:

```
leftStickAnalog[1]  -> _x_vel_des
rightStickAnalog[0] -> _yaw_turn_rate
```

Nothing in the FSM, the MPC or the WBC knows navigation exists. The sequencer
still stands the robot, enters LOCOMOTION, lets the gait engage and ramps
velocity; nav waits for that ramp and then owns both channels. Because wp00 is
due north and the dog spawns facing north, the handover happens mid-stride on
the correct heading.

Sign, derived rather than guessed: MIT's yaw rate is CCW-positive (it integrates
into `_yaw_des`, a world yaw), nav's is compass-sense (positive toward east =
clockwise), so `rightStickAnalog[0] = -navW`. `WP_YAW_SIGN` overrides it.

Heading datum: bearing is taken RELATIVE to the yaw at mission start
(`-(rpy[2] - yaw_ref)`), which works identically whether the estimator zeroes
its yaw (VectorNav path) or reports true world yaw (cheater path) - the two
differ by exactly that constant. Verified against Gazebo truth: nav reported
hdg=170 deg while the world pose said 170.3 deg.

### Two bugs this uncovered

1. **`GamepadCommand` was never zero-initialised.** `Stm32mp1HardwareBridge`
   memset its motor scratch buffers but not the operator command channel, so it
   started full of stack garbage. The nav driver's "wait until the sequencer has
   ramped the stick to `SIM_VX`" gate saw a value that already exceeded the
   target and took the stick at t=0.0 s - before the robot had stood up - which
   spun it 170 degrees off heading before the first waypoint. In sim that looks
   like a nav bug; on hardware it is a velocity command nobody asked for.
2. **Block-buffered stdout loses the whole log when a run is killed.** The sweep
   harness ends runs with SIGTERM (timeout, or the fall detector), and the
   default block buffering on a redirected stdout meant a run could produce a
   ZERO-BYTE log. `setvbuf(stdout, nullptr, _IOLBF, 0)` in main.

## Fall detector: end failed runs immediately

MIT's `ControlFSM::safetyPreCheck()` E-stops on attitude (0.5 rad -> PASSIVE),
but the PROCESS keeps running, so a run that ended on its side still burned the
whole timeout before the harness moved on. `RobotRunner::run()` now carries a
fall detector next to the NaN guard: sustained roll or pitch beyond
`SIM_FALL_DEG` (default 50 deg, deliberately well past MIT's own E-stop so it
only fires on a genuine fall) for `SIM_FALL_HOLD_S` (0.5 s) zeroes the legs,
flushes one `[FALL]` line and exits. `SIM_FALL_EXIT=0` disables it.

Attitude is the right discriminator, for the same reason `SafetyCheck.hpp` uses
it: a legged robot takes a real acceleration spike on every footfall, so
acceleration cannot separate "trotting hard" from "fallen over", whereas a
working quadruped is never on its side or its face.


## Model ground truth from Unitree's own binary (see docs/LEGGED_SPORT_REVERSE.md)

`Legged_sport` ships **unstripped with DWARF**, so its build paths and its
compiled-in constants are readable. It is a direct MIT Cheetah-Software fork
(MIT's tree verbatim, 9,739 symbols, MIT licence shipped alongside), which makes
it an authoritative reference for how MIT's stack should be parameterised for a
Go1. Several of this port's "Go1 adaptations" turned out to be guesses that the
binary contradicts. All now corrected:

| what | this port had | Unitree's binary | why it matters |
|---|---|---|---|
| `_kneeGearRatio` | 6.33 | **9.4995** | 23.70/6.333 = 35.55/9.4995 = 3.742 Nm - ONE motor drives all 12 joints; the strong knee is GEARING |
| `_kneeMotorTauMax` | 5.616 (invented) | **does not exist** | the field was a workaround for the wrong gear ratio; REMOVED, `Quadruped.h` back to upstream MIT |
| `_maxLegLength` | 0.385 | **0.430** | bounds foot placement (swing planner + `SafetyChecker` maxPDes) - 0.385 is a self-imposed 10% shorter stride |
| `_batteryV` | 21.6 | **24.0** | |
| MPC inertia | (0.102, 0.379, 0.352) | **(0.13662, 0.425578, 0.460359)** | ours was mini-cheetah scaled by mass ratio: **roll -25%, yaw -24%** |
| world abad limit | ±1.047 (±60°) | **±0.863938 (±49.5°)** | we used the SDK's permissive SOFTWARE bound as a PHYSICAL stop |

**MIT's cost weights are untouched by Unitree** - `Q = {0.25,0.25,10, 2,2,50,
0,0,0.3, 0.2,0.2,0.1}` appears verbatim twice in `.rodata`, and `alpha = 4e-5`
matches. So this port's use of MIT's Q is right, and the "short horizon
under-supports" behaviour is a genuine property of those weights, not a porting
error.

### Three joint-limit sets, and we had used the wrong layer
| source | abad | thigh | calf | what it is |
|---|---|---|---|---|
| binary + URDF | ±49.5° | -39.3..257.9° | -161.5..-50.9° | **mechanical** |
| binary, 2nd set | ±55° | -33..165° | -151..-53° | operational clamp (round degrees) |
| `go1_const.h` | ±60° | -38..170° | -156..-48° | permissive SDK bound |

The worlds now use the mechanical set. The operational set belongs as a
controller-side clamp and is **not yet enforced anywhere in this port**.

### What Unitree built that MIT (and we) lack
- **27 FSM states** vs MIT's 12 (`Dance`, `TwoLegHop`, `TurnOverMove`, `Space`,
  `WallowStand`, `PreStand`, `StandDown`, `ZeroTorque`, ...).
- `ConvexMPCLocomotion` split into **`runSwingLegControl` / `runContactLegControl`**,
  plus `trajPlanner` and **`zeroVelTransitionAmend`**; MIT's `updateMPCIfNeeded`
  and the whole sparse path are gone (this port dropped sparse too).
- `OffsetDurationGait` gained **`getFlightState`, `getCurrentHybridMode`,
  `getItNxtHybMode`, `getContactStateExpected`** - explicit flight-phase and
  hybrid-mode machinery. This port's bounding/pronking/galloping/trotRunning are
  exactly the flight-phase gaits, and they collapse at gait ENGAGEMENT with
  velocity commanded to zero. Strong hint the missing piece is this, not tuning.
- `zeroVelTransitionAmend(int* mpcTable, ControlFSMData&)` **amends the MPC
  contact table around zero-velocity transitions**, with a debounce so one tick
  of zero does not trigger a gait change (pseudocode in the doc; constants
  0.01 m/s threshold, 0.7 duty divisor, 0.42 offset). MIT has no equivalent -
  it hands the contact table to the MPC unmodified. Our gaits die at exactly
  that event.


## Implementing the RE findings: what worked, what did not

Three things were ported from `Legged_sport` into this stack. The process is
worth recording because two of the three steps were wrong first.

**1. `getFlightState()` (Gait.h/.cpp).** All-four-legs-in-swing detector, ported
from `OffsetDurationGait::getFlightState` (0xf7880). Upstream MIT has no
equivalent, so it has no way to know a gait even HAS a flight phase.

**2. Per-leg stance/swing durations.** MIT's `OffsetDurationGait` does
`(void)leg; return dtMPC * _stance;` - one scalar for all four legs. Unitree's
build of the SAME class indexes a per-leg array (0xf7420 / 0xf7438). MIT only
supports per-leg timing in `MixedFrequncyGait`.

**3. Zero-velocity gait hold - and the dead guard it exposed.**

FIRST ATTEMPT, WRONG: patch the MPC contact table so every all-swing horizon
step is forced back to stance. Measured: **no effect whatsoever** - pronking
still reached 0.00 m and collapsed at 10 s with the amendment confirmed firing
(`6/10 horizon steps were airborne ... forced to stance`). It cannot work: the
table only tells the MPC what to SOLVE AGAINST, while the gait's own
contact/swing states still swing the legs. The result is the MPC commanding
forces into feet that are in the air - strictly worse than leaving it alone.

Re-reading the binary showed Unitree does not rewrite the schedule at all: it
raises a TRANSITION REQUEST (`data+0x36a`). They leave the flight gait rather
than patch it.

Implementing that exposed the actual defect: **this port's standing-gait entry
window is gated behind `if (_mpcAsync)`**, and the solve runs INLINE by default
(`SIM_MPC_ASYNC=0` is the working config). So nothing had ever stopped a dynamic
gait from engaging while the sequencer still held velocity at zero - dead code
for the entire gait matrix. `zeroVelHold()` now applies in both modes.

### Result: trotRunning went from broken to working

| gait | before | after |
|---|---|---|
| **trotRunning (5)** | 2.96 m, fell @ 24 s | **16.72 m, upright to end (0.6 m/s)** |
| **bounding (1)** | 1.84 m, fell @ 24 s | **BIMODAL at 1.0 m/s** - see below |
| pronking (2) | fell | 0.03-0.12 m, 18 s at every speed - still broken |
| galloping (22) | fell | 0.04-0.06 m, 18 s at every speed - still broken |

trotRunning is REPRODUCIBLE - four runs at 0.6 m/s gave 16.72 / 16.59 / 16.82 /
16.82 m, all upright to the end of the run (1.4% spread). Its ceiling is 0.6:
0.8 falls at 34 s and 1.0 at 28 s.

**Bounding at 1.0 m/s is BIMODAL, and this is a caution about single runs.** Four
runs at identical settings:

    5.23 m / 30 s     0.04 m / 18 s     0.03 m / 18 s     5.52 m / 30 s

Two clean ~5.4 m runs and two collapses at engagement - roughly a coin flip. It
was reported here as "speed-responsive" off the first run and then as "broken"
off the next two; neither was right. It is a genuine but MARGINALLY STABLE
operating point, and it is distinct from pronking/galloping, which never exceed
0.12 m at any speed tried (0.3 / 0.6 / 1.0). At 0.6, 1.4 and 1.8 bounding gives
~0.02-0.05 m, so the window really is around 1.0.

Pronking (all four legs in phase) and galloping do not respond to speed at all,
consistent with a true all-airborne phase leaving the MPC with no contacts - the
friction cone forces every foot force to zero and the body is purely ballistic,
so landing depends entirely on timing and inertia.

**Method note: repeat every marginal cell before believing it.** This port has
documented run-to-run variance and it bit three separate claims in one session.

### Also ruled out by the binary
`Kp_stance = 0` is NOT this port's bug. Unitree's stance gain vector
(`.rodata 0x2fe0f8`) is zeros, same as MIT. All support genuinely is meant to
come from MPC force.

### Left as a knob, not a guess
MIT shifts the contact schedule one MPC step into the future
(`iter = (i + _iteration + 1) % _nIterations`); Unitree's build of the same
function (0xf7758) does **not**. Plausibly MIT compensating for solve latency,
and this port's latency is larger than either of theirs - so it is exposed as
`SIM_MPC_SCHED_LEAD` (default 1 = MIT) rather than silently changed.


## Final measured state (Mac SITL, corrected model + RE fixes)

> **INVALID - CHEATER NUMBERS. DO NOT QUOTE.** Every row below was produced by
> `dash_sweep.sh` while its `COMMON=` block hardcoded `SIM_CHEATER=1`, i.e. with
> Gazebo ground truth fed into the estimator. The retraction further down this
> file was written and the harness was never checked, so the same contaminated
> table was re-measured and re-reported afterwards. The flag is now removed from
> `dash_sweep.sh`, `refine_maxspeed.sh` and `pace_isolate.sh`. Real-estimator
> replacements go in the section at the end of this file.

| gait | max speed | 100 m | cruise | notes |
|---|---|---|---|---|
| walking2 (21) | 1.0 | 106.8 s | 0.94 | 1.4 fails x3 |
| trotting (9) | **0.9** | **120.4 s** | 0.83 | 176 m x2 at 0.9; 1.0 fails x2 |
| pacing (8) | 0.8 | 130.5 s | 0.77 | 1.0 fails x2 |
| trotRunning (5) | 0.6 | 186.1 s | 0.53 | 0.8 fails x2 |
| walking (20) | - | never | - | 17.4/17.6 m, endurance-limited |
| bounding (1) | - | never | - | bimodal at 1.0: ~5 m in 3 of 5 runs |
| pronking (2) | - | never | - | <0.3 m at every speed |
| galloping (22) | - | never | - | <0.1 m at every speed |

### METHOD TRAP: a stop-at-first-success ladder is biased DOWNWARD

`dash_sweep.sh` tries speeds high-to-low and stops at the first that completes.
That is efficient and **wrong for a gait that is bimodal near its limit**: one
unlucky run permanently records the lower speed. It reported trotting at 0.6
m/s. Re-checking the speed above found 0.9 holds 176 m twice, with 1.0 failing
twice - so the ladder had understated trot by **50%**, and its 100 m time by
51 seconds (171.8 -> 120.4 s).

Re-checks of the other three crossings confirmed the ladder (walking2 1.0,
pacing 0.8, trotRunning 0.6), so this is not a systematic offset - it is a
sampling failure that strikes unpredictably. **Always repeat the speed above a
reported ceiling.**

### Levers tried on pronking/galloping that did NOT work
- zero-velocity gait hold: fixes ENTRY (0.00 m -> survives to 18 s) but not
  operation;
- speed: 0.3 / 0.6 / 1.0 all identical;
- heavier command smoothing (`SIM_CMD_FILTER` 0.10 vs 0.01): no effect. NOTE this
  was a proxy - Unitree's 0.998/0.994/0.992 filters live in `trajPlanner` and act
  on the trajectory reference, not on MIT's operator-command filter, so this
  rules out the lever tested, not Unitree's actual smoothing;
- patching the MPC contact table: no effect, and structurally wrong (see above).

Remaining candidate: Unitree's swing/stance split (`runSwingLegControl` /
`runContactLegControl`) and `trajPlanner` proper, reversed only structurally.


## The yaw bug: why `walking` "could not be explained"

`walking` was recorded here as endurance-limited for unknown reasons - ~17 m and
down, every time. That was a stopping decision, not a finding. ONE instrumented
run settled it: yaw climbed 0.09 -> 0.51 rad over the run and the robot went
down when it saturated.

**Mechanism.** The MPC is handed a yaw ANGLE reference (`pBody_RPY_des[2]`,
cost weight 10) but the yaw RATE reference (`vBody_Ori_des[2]`) is just the
commanded turn rate - exactly ZERO when walking straight. So nothing actively
unwinds a heading error; only the angle error opposes it, and this port clamps
that at 0.40 rad. A 4-beat gait kicks yaw on every single-leg step, the error
saturated the clamp, and the gait lost. `walking2` (diagonal pairs) is
yaw-balanced and never showed it - same MPC, same weights, opposite outcome.

**Fix.** Proportional heading feedback into the RATE channel,
`vBody_Ori_des[2] = _yaw_turn_rate + (-kp * yaw_error)`, active only while
holding heading. Clean gain curve, which is what says the mechanism is right:

| `SIM_YAW_RATE_KP` | distance |
|---|---|
| 0 (stock) | 9.3 m, fell |
| **1.5** | **93.6 m, upright** |
| 3.0 | 21.4 m, over-correction |

Measured NOT to help trot/pacing (both yaw-balanced), which is the right
signature for a mechanism-specific fix rather than a general stability nudge.

## THE REAL ESTIMATOR IS NO LONGER THE BLOCKER

Everything measured in this file before today used `SIM_CHEATER=1`. The recorded
real-estimator figure was **0.65 m**. That number was stale by ~50x and it was
being trusted instead of re-measured. Current, `SIM_CHEATER=0`:

| gait | speed | distance (84 s) | drift |
|---|---|---|---|
| walking2 | 1.0 m/s | **57.64 m** | 0.05 m |
| pacing | 0.8 m/s | 46.62 m | 0.99 m |
| trotting | 0.6 m/s | 35.21 m | 0.51 m |

**And the full GPS star mission completes on the real estimator in 57.1 s -
identical to cheater mode.** GPS -> nav -> real LinearKF -> convex MPC -> legs,
5 x 10.1 m legs and five 144 degree corners, 0.80 m/s average. Was 85.3 s.

Baro/GPS absolute aiding is implemented (`SIM_ABS_AIDING=1`, opt-in, verified to
engage) and is a measured NULL on these gaits - 35.21 vs 35.36 m, 57.64 vs
57.64 m. Correct reason: they never lose contact, so leg odometry never drops
out and there is no observability hole to fill. Its value is reserved for the
flight gaits, whose all-swing windows are what blow the covariance up.

## Flight gaits: five approaches, all failed - and what that RULES OUT

> **VOID - RE-TEST BEFORE TRUSTING ANY OF THIS.** Everything below was measured
> while `use_jcqp: 1` was returning roughly a fifth of the required ground
> reaction force under any moving gait (see the solver section). Pronking needs
> 80 N/foot and "stock plateaus at 66.1 N" was read as a gait problem; against a
> solver that under-commands by 4-8x it is not evidence about the gait at all.
> A lever that changes nothing while the robot is being handed a fifth of the
> force it needs has been MASKED, not tested. The flight gaits have not been
> retried on qpOASES yet.

pronking and galloping still collapse ~2 s after gait engagement. Recorded so
nobody repeats them:

1. **Patching the MPC contact table** (force airborne steps to stance) - null.
   Cannot work: the table only tells the MPC what to solve against, while the
   gait still swings the legs, so the MPC pushes into feet that are in the air.
2. **Ballistic vertical reference** (launch velocity from the flight duration) -
   null. Stock MIT already commands sensible pronking forces; the reference was
   not what was broken.
3. **Flight-phase cost gating** (drop z/vz cost on airborne steps) - **HARMFUL**.
   Cut solved force from 39-42 N/foot to 6.1 N/foot. The cost is what makes the
   optimiser command force at the CONTACT steps; dropping it on 6 of 10 steps
   removed most of the objective. Shipped default-ON by mistake, reverted.
4. **Raising ADMM convergence** (rho 2.0 / 300 iters) - reaches the force
   (81.7 N/foot vs the 80 the 40% duty needs, where stock plateaus at 66.1) but
   stalls the control loop to 5.55 ms inline, and still falls.
5. **Async + converged solve** - loop clean at 1.00 ms, forces available, STILL
   falls. Also degraded bounding (0.22 m vs its usual ~5 m).

**What this eliminates:** it is not force availability (the solver can find
them), not compute (async runs clean), not stance stiffness (Unitree's is zero
too), not the height reference, and not `locomotionSafe` (no RECOVERY_STAND
transitions in any failing log). The remaining candidate is the swing/foothold
path during the 60% of the cycle all four legs are airborne - Unitree's
`runSwingLegControl`, reversed only structurally.

### Arithmetic worth keeping
Pronking is `durations(4,4,4,4)` of 10 = **40% stance**, so holding height needs
`m*g/duty` = 128/0.4 = **320 N total, 80 N per foot**. Stock plateaus at 66.1 N
(83%) and the body sinks at ~g/2 - which matches the measured monotonic descent
0.290 -> 0.140 exactly. The force cap (175 N/foot) is nowhere near binding.

### Solver tuning is per-gait, and ours is global
`jcqp_rho 0.6 / 60 iters` was swept against a TROT. Pronking's QP is harder and
comes out ~17% short at those settings. Any future convergence tuning should be
checked per gait rather than assumed to transfer.


## RETRACTION: every "real estimator" result before this was a cheater run

`getenv("SIM_CHEATER")` is non-null whenever the variable is SET, so
`SIM_CHEATER=0` **still enabled cheater mode**. Everything previously recorded
here as real-estimator performance was ground truth. Fixed to parse the value.

### The honest real-estimator baseline (walking2, SIM_CHEATER=0)

| config | speed | result |
|---|---|---|
| cheater (ground truth) | 1.0 m/s | 57.5 m, upright |
| **real estimator** | **1.0 m/s** | **~21 m, falls @ 44 s** (5 runs: 20.68/21.35/21.12/25.24/21.30) |
| **real estimator** | **0.6 m/s** | **34.95 m, upright to end** |

**The estimator caps SPEED, not capability.** Ground truth supports 1.0 m/s; the
real LinearKF supports ~0.6. Every 100 m dash time in this file is a CHEATER
number and overstates the robot.

### The fall detector was aborting valid runs
`SIM_FALL_Z` was 0.15 m, but the robot WALKS at ~0.175 estimated and dips to
0.149 on the stand-up transient - so the detector killed every real-estimator run
while Gazebo truth showed 0.197 and still walking. Threshold now 0.10, arms at
0.25. NOTE it reads the ESTIMATE, so it inherits estimator error; on hardware it
should key off attitude and kinematics instead.

## GPS/baro aiding: correct diagnosis, wrong layer

Absolute position IS unobservable from MIT's KF - leg odometry is relative, and
MIT additionally caps the x,y covariance every tick
(`_P.block(0,0,2,2) /= 10`), so the filter is permanently overconfident about
the one quantity it never measures. Measured drift: 4.5 m over 83 m.

Aiding was implemented (sequential update, `SIM_ABS_AIDING`, plus `SIM_KF_UNCAP`
to lift MIT's covariance cap) and a frame bug found and fixed: the GPS origin was
captured at first fix, AFTER stand-up had moved the robot, while the estimator's
origin is the spawn pose. Harm grew as the correction got gentler
(21 -> 5.8 -> 0.20 m), which is the signature of a BIAS, not noise.

**CORRECTION, checked against Unitree's binary: the covariance cap is NOT a bug.**
`LinearKFPositionVelocityEstimator<float>::run()` (0x1c11e0, 7864 bytes) contains
the IDENTICAL structure at 0x1c26a0-0x1c2778: zero the (0,2)/(2,0) 2x16/16x2
cross-blocks, a threshold check, then `div_assign_op<float,float>` on the (0,0)
2x2 block against the constant 10.0 - byte-for-byte the same as MIT's
`_P.block(0,0,2,2) /= 10`. Unitree ships this on hardware that walks at
2.5-4.7 m/s, so it is not what caps this port's speed, and `SIM_KF_UNCAP` was
solving a problem that does not exist. Kept as an opt-in flag (default off,
harmless) but the diagnosis behind it was wrong.

Also confirmed from the same disassembly pass: `high_suspect_number = 100` and
`foot_process_noise_position = 0.002` are exact matches to this port's values -
both were guesses that turned out right, now verified rather than assumed.

**With frames aligned it still does not help locomotion, and adds a
catastrophic tail:**

| | samples (m) | mean | worst |
|---|---|---|---|
| no aiding | 20.68 / 21.35 / 21.12 / 25.24 / 21.30 | 21.9 | 20.68 |
| GPS aiding | 25.51 / 21.33 / **0.25** / 19.13 | 16.6 | **0.25** |

**Conclusion - position drift does not destabilise walking; CORRECTING it does.**
The controller needs consistent RELATIVE motion to balance, not absolute
position. Injecting GPS into the control estimate steps the tracking error and
the MPC fights it. Fuse GPS in the NAVIGATION layer only - which is what
`WaypointNav` already does, reading `gazebo_get_aux()` directly. Aiding stays
OFF by default.

Unitree's alternate Q vector (`.rodata 0x2fe390`, 10x MIT's position weight) was
also tested: 21.25 m alone, 25.61 m with aiding - both inside baseline scatter.
Neutral at 1.0 m/s.


## Contact detection: implemented, tested, REGRESSES the real estimator

MIT's `ContactEstimator` is an admitted pass-through (its own header: "this will
need to change once we move contact detection to C++") - the KF believes a foot
is down because the GAIT SCHEDULE says so, and inflates that foot's measurement
noise 100x (`high_suspect_number`) whenever the schedule calls it swing.

Implemented from kinematics + IMU (`SIM_CONTACT_DETECT=1`): the lowest foot by
FK is the contact candidate (a RELATIVE test, so it does not depend circularly
on the body-height estimate), vetoed by a free-fall check on the accelerometer
(specific force well under 1g means nothing is bearing load).

**Measured on the real estimator, walking2 @1.0 m/s, three repeats each:**

| | samples (m) | 
|---|---|
| detection off | 21.34 (+ 5 earlier baselines: 20.68-25.24) |
| **detection on** | **5.64 / 5.67 / 5.71** |

Reproducible to within 0.07 m across three runs - this is a real, systematic
regression, not variance.

**Mechanism (why, not just that):** MIT's KF does not use contact as a boolean.
`phase` feeds a `trust` that ramps over a 0.2 window at each end of stance
(`PositionVelocityEstimator.cpp`), so the filter fades a foot's measurement in
and out smoothly rather than switching it on and off. This implementation
overwrites that graded phase with a two-level signal
(`std::max(sched, 0.5)` / `std::min(sched, 0.5)`), which throws away exactly the
ramp information the trust computation depends on. Detection answered "is this
foot down," which the schedule already answers adequately (no RECOVERY_STAND
transitions, no evidence of schedule/reality mismatch on the working gaits) -
the phase within stance was never the problem.

Default OFF (`SIM_CONTACT_DETECT`, opt-in). If revisited: use detection to
CORRECT the schedule's phase when they disagree, rather than replacing it -
preserve the ramp, only intervene when the disagreement is real.


## Foundational audit (per direct instruction): URDF + decompile, block by block

Stopped adding estimator features. Went back to verify every constant against
ground truth (URDF + Unitree's binary) rather than build on top of an
unverified foundation. Found three real bugs, all independently confirmed by
TWO sources agreeing:

### Rotor mass/inertia: copy-pasted from mini-cheetah, never updated

`Go1.h`'s rotor block was byte-identical to `MiniCheetah.h`'s (mass 0.055,
inertia diag 33/33/63 x1e-6) - a straight copy that nobody had updated for the
Go1's actual rotors. Corrected from the URDF's `hip_rotor`/`thigh_rotor`/
`calf_rotor` links (mass 0.089, spin-axis I=111.842e-6, radial I=59.647e-6).

**Independent confirmation**: 59.646999 and 111.842003 appear as immediates in
Unitree's `buildMiniCheetah<float>()` - found during the FIRST reversing pass
of this project and not connected to this bug until now. Two unrelated sources
(URDF, binary) agree to 6 significant figures.

Total robot mass: was 12.45 kg with the wrong rotor mass; is now **12.859 kg**
against Unitree's own binary constant of **12.840 kg** - 0.15% agreement.

### Rotor locations: two of three were placeholder guesses

`_abadRotorLocation` was a literal copy of `_abadLocation` (impossible - they
are different joints). `_hipRotorLocation`/`_kneeRotorLocation` followed
mini-cheetah's pattern (rotor co-located with the joint) rather than the Go1's
actual near-zero motor-housing offsets. Corrected from URDF joint origins, with
the sign/frame convention verified against `Quadruped.cpp`'s actual consuming
code (`withLegSigns`, confirmed leg 0 = FR maps stored (x,y,z) -> physical
(x,-y,z)) rather than assumed - checked against the three locations that were
ALREADY correct (`_abadLocation`/`_hipLocation`/`_kneeLocation`) to derive the
rule before applying it to the three that were wrong.

### Motor electrical params (motorKT, motorR): confirmed NOT load-bearing

Identical to mini-cheetah's. Traced their only consumer
(`buildActuatorModels()`) and confirmed it is unit-test-only, never called from
the control loop - so this is unverified but also inert, not a live bug.

## The covariance cap: corrected diagnosis (Unitree ships the identical code)

`LinearKFPositionVelocityEstimator<float>::run()` (0x1c11e0, 7864 bytes)
disassembled at 0x1c26a0-0x1c2778: zero (0,2)/(2,0) cross-blocks, threshold
check, `div_assign_op<float,float>` on the (0,0) 2x2 block against 10.0 -
byte-for-byte MIT's `_P.block(0,0,2,2) /= 10`. Unitree ships this on hardware
that walks 2.5-4.7 m/s, so it is NOT what caps this port's speed.
`SIM_KF_UNCAP` was solving a problem that does not exist; the earlier
diagnosis in this file was wrong and is corrected here.

Also confirmed exact matches from the same pass: `high_suspect_number = 100`,
`foot_process_noise_position = 0.002`.

## The SITL never tested orientation estimation at all

`VectorNavOrientationEstimator::run()` is a pass-through of
`vectorNavData->quat`. In the Gazebo SITL that value is `msg.orientation` from
Gazebo's IMU sensor plugin - and the SDF has no `<orientation>` noise block on
any world's IMU sensor, so it reports the link's exact simulated pose. **Every
"real estimator" run measured today had perfect, noise-free orientation.**

This reframes the whole 21 m / 57 m gap: it is not sensor realism, it is the
LinearKF's position/velocity integration (leg odometry) degrading the estimate
even under best-case orientation and IMU input. Unitree's real hardware has
genuine VectorNav noise and still outperforms this by ~5x, which says the gap
is in the filter's leg-odometry handling itself, not in a lack of simulated
sensor error.


## WBIC gains: Unitree has far more tuned variants than MIT ships

`LocomotionCtrl<float>::_ParameterSetup` reads gains the same way ours does -
from a config path, not hardcoded per-call - so the binary cannot be read for
"the one true gain value" the way a physical constant can. What IS visible and
verified: the symbol table has `Kp_body`, `Kp_body_stance`, `Kp_body_running`,
`Kp_body_stairs`, `Kp_ori`, `Kp_ori_stairs`, `Kp_joint`, `Kp_joint_stance`,
`Kp_joint_swing`, `Kp_joint_swing_running` - MIT/this port ships ONE set
(`Kp_body`/`Kd_body`/`Kp_ori`/`Kd_ori`/`Kp_joint`/`Kd_joint`) used for every
gait and every phase. Unitree tunes per-situation.

Raw values were read from `.rodata` and are NOT reported here as verified: a
first pass misread `Kp_ori` as an 8-float (Kp+Kd) block when it is actually
4 floats, spilling into the next symbol's memory and producing a nonsense
number. The variables are not uniformly sized, so pattern-matching the layout
is unsafe without confirming each one's actual read site in
`_ParameterSetup`'s disassembly - not done here, flagged as a genuine gap
rather than a guess.

**What's safe to act on without that work**: the STRUCTURE - Unitree's
per-gait/per-phase gain switching is real and this port's single fixed gain
set is a plausible source of the marginal stability seen at the top of every
gait's speed range (bounding's coin-flip at 1.0 m/s, trot's wall at 1.0). Worth
a proper `_ParameterSetup` disassembly before touching any WBIC yaml value.


## WBIC body damping: Kd_body 16->40, Kd_ori proportionally - real, but gait-specific

Following the WBIC gain audit above (structural finding: Unitree ships far more
tuned gain variants than this port's single fixed set), tested the concrete
lever directly rather than stopping at "can't verify Unitree's exact number":
raised this port's own `Kd_body` (16 -> 40) and `Kd_ori` ([26,18,10] ->
[40,40,20]), leaving `Kp_body`/`Kp_ori` untouched, and measured on every speed
that fails today.

| config | baseline (m) | high-Kd (m) | ratio | still fails? |
|---|---|---|---|---|
| **trot @1.0** | 5.47 | **60.03 / 41.41 / 46.43** (3 runs) | **~9-11x** | NO on 1 of 3, collapses late on the other 2 |
| trot @1.2 | 4.48 | 6.61 | 1.5x | yes |
| walking2 @1.4 | 1.38 | 7.44 | 5.4x | yes |
| pacing @1.0 | 5.29 | 5.89 | 1.1x | yes |

**Honest read: this is not a general speed unlock.** It is a large, reproducible
fix for trot specifically at its previously-marginal 1.0 m/s (3 independent
runs, all far above baseline, none of the old ~5-10 m failures). The other
three speeds - all PAST their gait's known ceiling - see modest improvement
but still fail. The fix restores a marginal-but-should-work speed to
reliability; it does not push gaits past their existing ceiling.

Applied to `stm32mp1/deploy_pkg/mc-mit-ctrl-user-parameters.yaml` (what
actually ships to the board). `config/mc-mit-ctrl-user-parameters.yaml` was
found to have DRIFTED from the deployed config (`Kp_body=100/Kd_body=10` vs
deployed `70/16`) - left unchanged pending investigation into which is meant
to be the reference copy; not touched here to avoid a second, unrelated
change riding along with this one.


## Fall detector: re-verified against Gazebo truth, not just our own estimate

Directly requested: pull the fall detector out of the loop (`SIM_FALL_EXIT=0`)
and re-check today's headline WBIC finding against something the detector
cannot influence - Gazebo's actual simulated position, read independently of
this port's own estimator or logging.

| config | detector | distance | source |
|---|---|---|---|
| baseline (Kd_body=10, stock) | **OFF** | z collapses to 0.057 at **5.30 m**, stays down for the remaining 70 s of a 100 s run | Gazebo truth |
| high-Kd (Kd_body=40) | **OFF** | **69.53 m** before going down | Gazebo truth |

**69.53 / 5.30 = 13.1x** - LARGER than the 11.0x measured earlier with the
detector active (60.03/5.47). The detector was not inflating this result; the
baseline's 5.3 m failure is confirmed genuine (the robot is simply down and
stationary for 70 straight seconds, not an early-exit artifact).

This does not mean the detector is trustworthy in general - it separately
caused a real problem earlier this session (killed valid real-estimator runs
at a 0.15 m threshold too close to normal standing height, ~0.175-0.197 m).
The lesson is narrower: THIS finding was independently reproduced without it,
so it does not rest on the detector being correct.


## THE ACTUAL SPEED WALL: JCQP never converges, and starved the robot of force

Found by running at the TARGET speed (2.0 m/s) and instrumenting the failure,
instead of laddering down to whatever worked. Laddering down measures success;
it never measures the thing that breaks.

### What breaks at 2.0 m/s: the robot sinks, it does not tip

`[FALL] collapsed: roll=0 deg pitch=-0 deg z=0.028` - flat on the floor with the
body level. Height decays 0.222 -> 0.139 while body vz oscillation grows to
0.55 m/s. Control loop stays at 1.2-1.4 ms against a 2.0 ms budget throughout,
so it is not a compute stall.

### The estimator is NOT the cause (it was blamed, wrongly)

`[ESTERR]` ($SIM_ESTERR=1) logs the estimate against ground truth in the SAME
body frame - truth is logged, never fed to the controller. Through the entire
ramp to the fall the LinearKF tracks forward velocity to within 0.13 m/s:

| t | true vx | est vx | err |
|---|---|---|---|
| 23.1 | 0.266 | 0.245 | -0.021 |
| 27.5 | 0.916 | 1.041 | +0.125 |
| 28.3 | 1.101 | 1.029 | -0.072 |

The dramatic errors (dvx > +1.0) appear only AFTER the robot is down, when it
integrates phantom motion from a fallen machine. Consequence, not cause.
NOTE `dp` in that log compares different frames (the estimator zeroes initial
yaw, so its x is forward while Gazebo truth is ENU) and is meaningless; vT/vE
are both body-frame and are the valid comparison.

### The vertical force budget, measured directly

To HOLD height, a gait with stance duty d must command `m*g/d` while feet are
down; commanding exactly `m*g` during stance averages `m*g*d` and the body falls
at `(1-d)*g`. `[MPCZ]` ($SIM_MPCZ=1) prints what the solver actually asked for.
Same gait, same speed, same everything except the solver:

| solver | 2-foot Fz/mg | 4-foot Fz/mg | mean body z |
|---|---|---|---|
| JCQP `rho 0.6 / 60` (what shipped) | 0.25 | 0.45 | 0.128 |
| JCQP `rho 2.0 / 300` | 0.38 | 0.69 | 0.132 |
| **qpOASES** | **1.27** | **1.76** | **0.275** |
| *required* | *2.00* | *1.00* | *0.300* |

**JCQP commands about one fifth of what qpOASES commands on the identical
problem, and 5x the iterations barely moves it.** This is not solver tuning -
the ADMM solve does not reach the optimum on this problem at all.

**This retires a question that sat open for the whole project**: "why is the MPC
satisfied at z=0.204 when its reference says 0.30?" It was never a cost-weight
mystery, never a `B_qp` z-row bug, and never MIT's `Kp_stance = 0`. The solver
was returning a fifth of the required force, and every gait number in this file
before now was measured on top of that.

### How it hid for so long

`use_jcqp: 1` was adopted as a SPEED win on the STM32 (82 ms vs 198 ms), and the
`rho 0.6 / 60` sweep that blessed it was run against a **STANDING trot**, where
the QP is easy and it genuinely does converge (-67 N/foot, matching `rho 2/200`).
Nobody rechecked it under a MOVING gait. The documented warning was even written
down - "solver tuning is per-gait, and ours is global" - and then not acted on.

On the Mac qpOASES costs 0.6-1.7 ms against a 2.0 ms budget, so the speed
argument that motivated JCQP does not apply here at all.

**Backport consequence for the board**: qpOASES cost 198-218 ms on the A7 versus
JCQP's 82 ms, so it cannot run inline against a 26 ms MPC segment there. The
board needs either the async path (`SIM_MPC_ASYNC=1`) or a re-measured
contact-reduced qpOASES. Do not assume the Mac result transfers unmeasured.

### Not a universal win - characterise before believing

qpOASES is transformative for `trotting` and appears HARMFUL for `walking2`,
which failed at every speed tried with it (including 1.0 m/s, where JCQP crossed
at 0.8). Both are recorded; neither is assumed to generalise.

## FINAL STATE: the 100 m star, and the honest hypothesis

### The numbers

| cruise | passes | time | per-leg | course m/s | verdict |
|---|---|---|---|---|---|
| **2.0 m/s** | **13/13** | **42.5-42.7 s** | 8.5 s | 2.35 | **the repeatable answer** |
| 2.5 m/s | ~20/30 (67%) | 41.5-41.8 s | 8.3 s | 2.41 | 1 s faster, fails 1 in 3 |
| 3.0 m/s | ~4/10 | 40.7-40.8 s | 8.1 s | 2.46 | fastest recorded, unreliable |

100 m dash spot-check on the same build - no regressions:

| gait | commanded | time | cruise |
|---|---|---|---|
| trotRunning | 4.0 | 24.8 s | 4.69 m/s |
| trotting | 3.0 | 33.3 s | 3.32 m/s |
| walking | 2.25 | 41.6 s | 2.60 m/s |
| trotting | 2.0 | 47.4 s | 2.23 m/s |

### What is IN the working configuration

Every one measured, none inherited:

| piece | value | why |
|---|---|---|
| solver | qpOASES (`use_jcqp: 0`) | JCQP returns ~1/5 the required force under a moving gait |
| gait | trotting (9) | the all-rounder: only gait that completes this course reliably |
| MPC segment | 22 ms | speed/gait scheduled; a mid-run switch cost a whole cell once |
| braking `a_lon` | 1.5 below 2.2 m/s, 0.4 above | the zone must outrun the real stopping distance |
| yaw clamp | `omega <= a_lat/v` | a constant clamp is wrong at both ends |
| acceptance | 1.5 m | raises fillet radius AND shortens path |
| lateral budget | 2.5 m/s^2 | roll 27 deg at 3.0, trip at 28.6 |
| end of mission | decelerate, settle, lay down, judge PASS/FAIL | arriving is not finishing |

### What is OUT, and measured to be out

| idea | result |
|---|---|
| gait switching (2- and 3-tier) | switches correctly; indistinguishable from trotting alone |
| banking into the turn | 9/13 vs baseline 11/16 - both 69%, NEUTRAL |
| corner crouch | 0/2, drives min height to 0.187 - actively harmful |
| angle-graded corner speed | 2-3 s slower, same reliability |
| hairpin pivot | 50.0 s 2/2 - most repeatable measured, 8 s slower |
| more/less lateral budget | no gain either direction |
| more yaw authority | roll 27 -> 72 deg for no time |

### THE HYPOTHESIS

The 2.5 m/s failures are a **collapse**, not a cornering failure: `roll=0
pitch=0`, flat, and the ONLY quantity that separates a passing run from a
failing one is body height - 0.212-0.239 when it works, 0.185-0.200 when it
does not. Commanded force is identical either way (0.85-0.88 x mg).

That is why eleven levers aimed at cornering - gait, bank, crouch, grading,
pivot, lateral budget, yaw authority, braking rate, acceptance, lookahead,
segment - all failed to move it. **They act on the plan; the failure is in
force DELIVERY.** The MPC asks for the right thing and the body still sinks.

**Predicted cause:** the WBIC/leg controller cannot hold the commanded stance
force through the combined decelerate-and-turn transient, so the body loses a
few centimetres each corner and, on a bad run, crosses the height at which the
gait can no longer recover.

**How to test it, and the test that would falsify it:** log ACHIEVED foot force
(`LegController::datas[].tauEstimate` through the Jacobian) against COMMANDED
`Fr_des` through a corner. If achieved tracks commanded, this hypothesis is
WRONG and the sink is coming from the swing/contact schedule instead. If
achieved falls short exactly where height drops, it is a WBIC tracking problem
and the fix is in the whole-body controller, not the planner.

Nothing in the planner will fix this. That is the strongest claim this session
supports, and it is supported by eleven negative results.

## THE REPEATABLE 100 m STAR - final measured state

Two configurations, and the choice between them is a real trade, not a ranking.

| | cruise | time | reliability | spread |
|---|---|---|---|---|
| **RELIABLE** | **2.0 m/s** | **42.6-42.7 s** | **7/7 (100%)** | 0.1 s |
| FAST | 2.5 m/s | 41.6-41.8 s | 6/8 (75%) | 0.2 s |

2.5 m/s buys **1.0 second** for a **1-in-4 failure rate**. On a course you intend
to repeat, that is a bad trade; the 2.0 config has never failed.

Both are precise when they run - a 0.1-0.2 s spread across a 100 m five-corner
mission - which says the remaining failures are a threshold being crossed, not
noise accumulating.

### `unittests/path_analysis.py` validated against real data, and it earned its keep immediately

Applied to the trotRunning-on-smooth-circle fall from earlier tonight
(pulled `planned`/`positions[0].trail` straight from a still-warm
`/api/state` before anything else launched and overwrote it) rather than
only the synthetic self-test. Result: corners 1-9 read CLEAN (0.01-0.09m
closest approach), corners 10-12 flip to **HAIRPIN** (overshoot 0.27-0.35m
- a real overshoot-and-correct signature), and corner 13 onward shows
monotonically GROWING "closest approach" distances tracing the shape of
the still-unwalked remainder of the circle - exactly what a STATIONARY
fallen robot compared against a moving sequence of planned vertices should
produce, not a bug in the tool. The fall itself happened right where the
hairpinning starts: the follower was already overshooting and correcting
for 2-3 corners before the actual tip-over, which is a genuinely useful,
specific diagnostic lead for the still-open "why does trotRunning fail
continuous curvature" question two sections up - worth checking whether
this same overshoot-then-correct pattern precedes trotRunning's other
smooth-circle fall too, before guessing at a cause.

### BANK INTO THE TURN - the first thing to touch the actual failure

Every running animal drops its shoulder and LEANS into a corner. That is not
style, it is force routing: a turn needs lateral acceleration `a = v*omega`, and
a body held LEVEL must generate all of it as SHEAR at the feet. Shear at the
feet is what pushes the body down through its own stance - which is exactly the
collapse this course fails on. Leaning puts the ground reaction along the body
axis instead, the same reason a banked track corners faster than a flat one:

    theta = atan(a_lat / g)      -> 14.3 deg at the 2.5 m/s^2 budget

Computed from the controller's OWN commands (it already knows v and omega), so
no plumbing from the planner. `$CTRL_BANK` scales it, 0 = off, 1 = full bank.

| config | passes | time | min body z |
|---|---|---|---|
| level (baseline) | 2/3 | 41.6-41.8 s | 0.195-0.245 |
| **bank 1.0** | **3/3** | 41.7-41.8 s | **0.221-0.235** |
| bank + 4 cm crouch | **0/2** | - | **0.187-0.193** |

**RETRACTED ON REPETITION.** That 3/3 was three runs and it did not reproduce:
a five-run confirmation gave 2/5, with min height 0.200-0.211 rather than
0.221-0.235. Combined, banking is 5/8 at 2.5 m/s against a baseline of 8/11 -
no measurable improvement, possibly worse.

At 3.0 m/s it looks better (2/3 vs baseline 1/3) but those samples are far too
small to separate. An interleaved A/B is running, because every comparison above
is between BATCHES taken at different times, which cannot distinguish an effect
from drift.

I claimed "the first change to touch the actual failure mode" from three runs.
That is the exact error documented four times already in this file, made again -
and it is worth leaving in rather than editing out, because the pattern is the
lesson: a promising mechanism plus a small sample produces a confident wrong
claim every single time.

**And crouching HURTS, which corrects the animal analogy for this machine.**
Animals lower their CoM into a turn; this robot is ALREADY too low there,
sinking to 0.19 m, so deliberately adding crouch pushes it straight through the
failure threshold (0/2, min z 0.187). The lean is what pays. The crouch is what
the robot is already suffering from involuntarily.

### GRADED CORNERING AND A THREE-TIER GAIT: both built, both neutral

Two structural improvements, both correct in principle and neither moving the
result - which is itself the finding.

**Angle-graded corner speed** replaced the hard pivot/arc switch, because that
switch was a TRANSITION BUG of exactly the kind gait switching had: pivoting the
star's true hairpin and arcing the rest failed 0/3, always at an ARCED corner
AFTER the pivot. Exiting one corner treatment into another is a discontinuity.
The lateral budget now scales smoothly with turn angle (full below 80 deg,
scaled by `corner_scale_min` at 160 deg, interpolated between), so no corner is
a special case. Measured at 2.5 m/s:

| grading | passes | time |
|---|---|---|
| 1.0 (none) | 2/3 | 41.6 s |
| 0.55 | 2/3 | 43.7-43.8 s |
| 0.4 | - | 44.7 s |

Same reliability, up to 3 s slower. Same verdict as the uniform a_lat reduction,
and for the same reason: **corner speed is not the limiting factor.**

**Three-tier gait schedule** (trotRunning on straights / trotting in the middle
/ walking in the tight, leaning on trotting as the all-rounder). Switches fire
correctly, twice per run:

| | 2.5 m/s | 3.0 m/s |
|---|---|---|
| three-tier | 41.5-41.6 s, 2/3 | 40.7 s, 1/2 |
| trotting only | 41.6-41.8 s, 4/5 | 40.8 s, 1/3 |

Indistinguishable. 40.7 s is the fastest star recorded and it is one run.

Both are kept - they are the right structure, and on a course with varied corner
angles they should pay. On THIS course they cannot, because the failure is not
where they act.

### HEIGHT IS THE DISCRIMINATOR - and the failure is a DEPARTURE, not a droop

Instrumenting the force budget shows the force command is INNOCENT:

| run | result | min Fz/mg | min body z |
|---|---|---|---|
| r1 | PASS 41.6 s | 0.86 | 0.212 |
| r2 | FAIL 4/5 | 0.87 | **0.200** |
| r3 | FAIL 2/5 | 0.88 | **0.185** |
| r4 | PASS 41.6 s | 0.85 | 0.224 |
| r5 | PASS 41.6 s | 0.88 | 0.239 |

Commanded force is identical across passes and failures. HEIGHT is the
discriminator, and it reproduced independently over twelve later interleaved
runs at 2.5 (minimum height reached: passes 0.231-0.251, failures
0.179-0.207).

**Three corrections to the earlier reading of this, all from 5 Hz height logs:**

1. **It is not a slow sink.** The robot cruises at 0.26-0.29 for tens of
   seconds and then DEPARTS, losing 8 cm in about 0.6 s at up to 0.35 m/s:

   ```
   PASS   0.283 0.277 0.271 0.271 0.272 0.286 0.286 0.277 0.272 0.270 0.287
   FAIL   0.274 0.268 0.260 0.246 0.228 0.198   <- gone
   ```

   The earlier "min z" table reports the BOTTOM of that departure, not a
   height the robot was holding. Anything built to react to a droop reacts to
   the wrong shape of event.

2. **`roll=0 pitch=0` describes the aftermath, not the event.** What actually
   trips is `SafetyChecker::checkSafeOrientation` (|roll| or |pitch| >= 0.5
   rad). The `[FALL]` line is printed 0.5 s later, after the bridge watchdog
   has already made the legs limp and the body has settled flat. So this is
   NOT evidence that attitude was fine - attitude is what failed the check.
   Do not cite the flat-and-level signature as proof of pure force starvation.

3. **The failures are on STRAIGHTS, not in corners.** Last nav line before
   loss, twelve-run sweep at 2.5:

   | run | waypoint | v | yaw rate | t |
   |---|---|---|---|---|
   | on_1 | wp3 | 2.50 | -0.13 | 29.7 s |
   | on_3 | wp1 | 2.46 | -0.15 | 11.7 s |
   | on_4 | wp1 | 2.50 | -0.08 | 11.4 s |
   | on_6 | wp3 | 2.24 | +0.29 | 30.8 s |
   | off_2 | wp3 | 2.30 | -0.22 | 30.8 s |
   | off_5 | wp3 | 2.46 | -0.14 | 30.3 s |
   | off_4 | wp4 | 2.50 | **-1.00** | 41.0 s |

   Six of seven at |yaw rate| <= 0.3 rad/s and full speed. The earlier note
   that "failures are CONSISTENTLY at wp 3/5 - a specific corner" was wrong:
   wp3 is right, corner is not. They cluster at the two places the robot first
   reaches its commanded speed - the top of the initial ramp (~11.5 s) and the
   wp2->wp3 straight (~30.5 s).

   This is the real reason eleven cornering levers all measured neutral. They
   were aimed at the wrong part of the course.

### THE REACTIVE HEIGHT GOVERNOR (Zhang et al. 2022)

`common/include/Controllers/HeightGovernor.h`, from "Mechanism analysis of
cheetah's high-speed locomotion based on digital reconstruction", Biomimetic
Intelligence and Robotics 2 (2022) 100033.

The paper's finding worth having: the cheetah holds body height at a FIXED
FRACTION of leg length (0.55 fore, 0.57 hind) while the virtual leg length
underneath swings by more than 2x within one cycle. Stance height is a
REGULATED VARIABLE. Section 4.7 adds the mechanism - the stance leg sits at
its lowest manipulability, the posture that "can withstand a greater force" -
which is also why crouching into a corner is the wrong instinct for a robot
whose failure mode is a vertical force deficit.

Stock has no such regulation: a constant 0.30 m reference and an unmeasured,
load-dependent droop underneath it. The governor closes the loop with two
levers - trim the reference up, and give up forward speed to buy vertical
force - both triggered on a predicted departure from the robot's own cruise
height. `CTRL_HGOV=0` disables it. Planner side: `plannedHeightBias()`
(`WP_HBIAS`, off by default) pre-loads margin for the corner ahead.

**Two dead versions, both worth not repeating:**

| version | why it failed |
|---|---|
| symmetric setpoint at 0.55 x leg | The Go1 cruises ABOVE 0.239, so the loop spent whole runs pushing the reference DOWN to its floor, 2 cm below stock, and started climbing only at 0.198. 1/5 waypoints vs stock's 2/5. The animal's ratio is a FLOOR here, not a target. |
| one-sided, absolute trigger at 0.239 | Never fired. Over six armed runs the reference moved 6 mm and the derate engaged zero times - so the "no effect" A/B was two identical controllers. Trigger must be departure-from-cruise, not absolute height. |

Third version triggers on `h_pred = h_debobbed + 0.2 * dh/dt` against a
self-tracked cruise height. The de-bobbing is not optional: raw dh/dt carries
+/-0.25 m/s of gait oscillation, which a 0.3 s lead turned into a phantom
0.08 m departure on a run that passed.

## CAMPAIGN RESULTS: equal valid N, 3 dogs in parallel, everything <= 3.5 m/s

Six reps per arm, arms run SIMULTANEOUSLY so they share machine conditions, and
a run only counts if it passed its acceptance gate (loop tail <= 5 %, config
took effect, the dog actually got going). Every headline below was then
re-confirmed SINGLE-DOG.

### THE WINS

| course | was | now | config |
|---|---|---|---|
| star | 39.40 s 6/6 | **38.25 s 6/6** (SD 0.15) | trotRunning 3.5, `WP_ALAT=3.25` |
| oval | 37.88 s 6/6 | **30.48 s 6/6** (SD 0.13) | trotRunning 3.5, analyzer, `WP_VSUS=2.6` |
| atom | 58.94 s | **58.97 s 6/6** (SD 0.09) | trotting 2.1, NO analyzer |

Single-dog confirmation: star 38.2/38.3/38.1, oval 30.2/30.5/30.5, atom 58.8.

### THE ANALYZER PAYS ONLY WHERE THE COURSE HAS MIXED REGIMES

| course | regime mix | analyzer value |
|---|---|---|
| star | all transient corners | NONE - 39.35 vs 39.40, inside noise |
| atom | 96 % sustained curve | NONE - bare @1.9 is 63.90, analyzed @2.1+cap1.9 is 63.83 |
| oval | 72 % straight / 28 % sustained | **-19.5 %** - 30.48 vs 37.88 |

This is the whole thesis, stated honestly. On a UNIFORM course, "cap the
sustained segments" and "lower the global speed" are the same operation, and a
hand-picked constant does just as well. The planner earns its keep only where
the robot should be doing DIFFERENT THINGS IN DIFFERENT PLACES - full speed on
the oval's straights, governed speed through its two ends. That is why the oval
had to be built before the idea could be tested at all.

### RETRACTED

**The atom "reliability win" was a speed reduction in disguise.** Reported as
analyzer 6/6 vs bare 5/6. Then bare @1.9 measured 6/6 at 63.90 s - identical to
the analyzed 63.83 s - and bare @2.1 measured 6/6 on a re-run where it had
previously given 5/6. So the comparison rested on ONE failure, and the analyzer
adds nothing on this course.

**Height pre-load (WP_HBIAS) does not survive.** It helped once, in one
marginal configuration (trotting 2.5, 4/5 -> 5/5, peak pitch 0.420 -> 0.308).
Since then: on the star at 3.5 it COSTS 0.88 s for no reliability gain
(all arms already 6/6), and on the marginal atom config it made things WORSE
(5/6 against bare's 6/6). Two failures to reproduce. Treat the original result
as a one-off until something explains it.

### CLIFFS, NOT SLOPES - every limit found this campaign is sharp

    star lateral budget   3.25 -> 6/6 38.25 s | 3.5 -> 5/6 | 4.5 -> 0/6
    atom sustained cap    1.9  -> 6/6         | 2.2 -> 0/6
    oval sustained cap    2.6  -> 6/6 30.48 s | 2.8 -> 0/6

Nothing degrades gracefully. The rising standard deviation is the only advance
warning: on the star it goes 0.07 (a_lat 2.75), 0.08 (3.0), 0.15 (3.25), then
failures at 3.5. Watch the spread, not the mean.

### UNEXPLAINED: the trotting dash fails in parallel and passes alone

    trotting 3.0, 100 m dash, single dog      33.4 s, reliable (matches table)
    trotting 3.0, 100 m dash, 3 in parallel   0/6, falls at 17-58 m

Loop health is perfect in both (p50 2.48 ms, 0 % over 4 ms) and RTF is 1.005 on
every instance, so it is neither scheduler starvation nor desynchronisation.
trotRunning and walking dashes are FINE in parallel; the star, atom and oval
missions all reproduce single-dog to within 0.1-0.3 s. CAUSE NOT ISOLATED.
Until it is, confirm any dash result single-dog.

## THE MISSION ANALYZER: decide once, up front, with the whole route in view

`common/include/Planning/MissionAnalyzer.h`. Everything about a route that can
be known IS known before the dog takes a step - the geometry is fixed, so the
curvature, the speed each point allows, which turns last long enough to need a
different gait, and where the time is actually lost are all computable in
advance. This port kept re-deriving that at 50 Hz, badly, from filtered
signals, and it produced three of the worst bugs in this file.

`analyze()` cuts the route into SEGMENTS carrying what the robot needs there:

    regime        straight / transient corner / sustained curve
    radius_min    tightest point
    v_cap         what the curvature allows
    gait          which gait, decided from the turn's DURATION
    height_bias   stance-height margin to pre-load BEFORE it
    time_cost     seconds this segment loses against free cruising
    blame_cost    ...charged to the FEATURE that caused it

At runtime the robot does `segmentAhead(s, lead)` - a lookup, not a
computation. `$WP_ANALYZER=1`.

### DURATION, NOT SEVERITY - the distinction the old decider could not make

The decider always keyed off `minPlannedSpeedAhead`, i.e. the MAGNITUDE of the
demand. Magnitude cannot separate these, and they want opposite gaits:

    star vertex   very high kappa, over in a metre. Planner brakes, robot
                  powers through, flight gait never tested - trotRunning is
                  32/32 on the star across 2.5-3.3 m/s.
    atom lobe     moderate kappa held for ten-plus metres, twenty gait cycles
                  with no recovery - trotRunning 3/8 where trotting is 7/8.

Both read as "slow ahead". `curveRunAhead()` measures the RUN LENGTH of
continuous curvature instead, and `regimeAhead()` classifies it. The rule that
falls out is physical, not tuned: a flight gait can spend a metre or two in a
turn and cannot spend twenty gait cycles there.

The classifier reproduces three independently measured results from geometry
alone, before the dog moves:

    star   9 segments, 0 sustained, 0 gait changes   (matches trotRunning 32/32)
    atom   1 sustained covering 125 of 130 m -> trotting  (matches 7/8 vs 3/8)
    oval   2 sustained, 4 changes a lap              (the course built for it)

### BLAME THE CAUSE, NOT THE PLACE

The first cost map reported the star's worst segment as a STRAIGHT at +3.48 s.
A straight is not slow because it is a straight - it is braking for the corner
ahead. The map now walks the profile and hands every metre of deficit to the
turn responsible (decelerating -> the turn ahead, accelerating -> the turn
behind). The star's costliest feature is then its FIRST CORNER at +4.30 s
(R=0.28 m), which is actionable in a way the other number never was.

### SUSTAINED TURNING HAS ITS OWN SPEED ENVELOPE

`MissionPolicy::v_sustained_max`, applied two-pass (classify -> cap -> re-plan
-> re-classify). Measured on the oval: it fails at a 3.0 m/s cruise at EVERY
radius tried - 3.0, 5.0 and 7.0 m - while the same robot takes the star's
transient corners at 3.3 (8/8) and runs 100 m straight at 3.0. R=7.0 is only
1.29 m/s^2 of lateral load, so curvature is not what binds. A per-point
lateral-acceleration budget cannot express this, because the budget is
evaluated at a point and the constraint is about duration.

CAVEAT: that measurement was taken at a median-worst loop period of 10.44 ms
(see the deadline-miss section) and has NOT been re-confirmed on a quiet
machine. Treat it as a lead, not a result.

## THE OVAL: a course where a gait switch can pay

`WP_MISSION=oval:<straight_m>:<radius_m>` - `WaypointNav::makeOval`. Two long
straights joined by two constant-radius 180s. Neither existing course rewards
switching, for opposite reasons: the star's corners are instantaneous (nothing
to switch away from), the atom is curvature everywhere (nothing to switch to).

    oval:40:5.0   2 x 40 m straight + 2 x 15.7 m of continuous R=5.0
                  111 m lap, sustained regime 28% of it

Feasibility, measured: 2.0 works at R=3.0 (41.5 s), 2.5 at R=5.0 (37.7 s),
3.0 fails at EVERY radius. Note R=3.0 at 2.5 gets STUCK - the robot cannot
track it and circles wide outside the turn.

## MISSED CONTROL DEADLINES: the instrument problem that invalidated a sweep

The control loop targets 2.0 ms. If macOS does not schedule it on time it wakes
late, and forces computed for 2 ms get applied for 9 - a 4.5x impulse, and the
dog goes over. Invisible in the result except as an unexplained fall.

`maxPeriod` is the WRONG metric: it is a maximum over ~20,000 ticks, so one
hiccup flags a healthy run. Measured on three back-to-back single-dog runs:

    p50    p95     over-4ms    result
    2.48   2.49     0.0 %      PASS 40.3 s
    2.48   3.01     1.3 %      PASS 40.5 s
    2.47   8.98    10.9 %      FAIL

The median never moves. The TAIL is what separates them. So health is the
fraction of intervals that overran, and `sweep_loop_stats` reports p50/p95/over.
Threshold 5%: every failure across every scale sat at 14%, every pass at <=3.9%.

`sweep_lock.sh` now also refuses to start above a 4.0 load average, pauses a
sweep after three consecutive sick runs, and samples the top non-simulation
process so a sick run NAMES its culprit instead of leaving "hiccup" unexplained.

Sweeps taken before this existed, by median-worst period:

    overnight (150 runs)        2.99 ms   healthy
    trotRunning 32/32 confirm   2.99 ms   healthy
    oval feasibility           10.44 ms   CONTAMINATED
    first analyzer tune         5.04 ms   CONTAMINATED

## PARALLEL DOGS: three, verified; four breaks

    3 separate engines   12/12 dogs, 40.4-40.7 s, over4ms 0.0%   (4 reps)
    3 in one engine       9/9  dogs, 40.4-40.5 s, RTF 1.000      (3 reps)
    single-dog reference        40.4 s

Three dogs give the same time to a tenth of a second with zero loop overruns.
3x throughput, nothing degrades.

FOUR OR MORE FAILS in both architectures: every dog hits `STATE ESTIMATE WENT
NON-FINITE` and falls before standing. What it is NOT: RTF (1.000 at N=4 and 5
with dogs running), loop starvation (p50 2.87-2.92, over4ms 0%), sensor wiring
(each bridge correctly on its own /go1_N/imu), a startup race (a readiness gate
did not fix it), or settling (20 s did not either). CAUSE NOT ISOLATED.

Use `sim_up_n.sh <i>` for separate engines (best evidence) or `sim_up_multi.sh
<n>` for one engine with N robots. `make_multi_world.py` namespaces every
sensor topic per model and derives lane spacing FROM THE MISSION - star 36 m,
atom 33 m, oval 25 m - rather than a constant that is either wasteful or unsafe
depending on which course you run.

`partune.sh <reps> <label:env...>` runs configs three-at-a-time with equal N
per arm and re-runs anything that fails its validity gate (loop tail, estimator
NaN, config actually took effect) rather than dropping it.

### THE GAIT CHOICE WAS BACKWARDS, AND IT DISSOCIATES BY COURSE

This port has run the star on TROTTING throughout, on the reasoning that it is
"the best all-rounder measured, the one to lean on", and has treated
trotRunning as the straight-line gait that is fatal in corners. Measured, 8
runs per arm:

| star @ | trotRunning (5) | trotting (9) |
|---|---|---|
| 2.5 | **8/8, 40.4 s** | 7/8, 41.8 s |
| 2.7 | **8/8, 39.8 s** | 0/5 |
| 3.0 | **8/8, 39.6 s** | - |
| 3.3 | **8/8, 39.4 s** | - |

**32/32 across four speeds, and faster at every one.** Trotting cannot complete
2.7 at all. Every star result in this file before this point is measured
against the wrong baseline gait.

But it reverses on the atom, and the reversal is the interesting part:

| atom @ | trotRunning | trotting |
|---|---|---|
| 2.1 | 3/8 | **7/8, 58.7 s** |
| 2.3 | 0/8 | **3/8, 54.9 s** |

The two courses want opposite gaits, and the reason is their shape. The star is
long straights joined by five discrete corners: a flight gait pays off on the
straights, and the planner brakes so hard for the vertices that the flight
phase is never tested in them. The atom is continuously curving with no
straights and no recovery anywhere - and a flight gait cannot sustain that.

So there is no single best gait, which is what the gait DECIDER was built for.
Note the decider was measured neutral (3/5 vs 3/5) while configured
fast=trotRunning, corner=trotting - on the STAR, where dropping to trotting is
dropping into the worse gait. It has never been tested on a course whose shape
actually rewards the switch.

### THE HEIGHT GOVERNOR IS NEUTRAL - three corrections, ending in a retraction

Reported here in three successive versions as p=0.026, then p=0.10, then not
significant. The final interleaved measurement, 10 pairs at 2.5 on the star:

| arm | passes |
|---|---|
| governor | 7/10 |
| stock | 5/10 |

Fisher p = 0.65. **The reactive height governor does not improve the star.**
The 10/10 that produced the earlier claim came from the version whose reference
trim had no deadband - and that same version was destroying the 100 m dash
(trotRunning 4.0 fell at 33.9 m against a 24.8 s table). Fixing the dash
removed the star benefit, which is the honest reading: the benefit was the
side-effect of a bug.

What DOES work is the predictive half, and it is the half that was built first
and left switched off:

| WP_HBIAS | passes | mean peak \|pitch\| | time |
|---|---|---|---|
| 0 | 4/5 | 0.420 | 41.9 s |
| 0.02 | 5/5 | 0.332 | 41.8 s |
| 0.04 | 5/5 | **0.308** | 41.8 s |

A monotonic dose-response on the axis that actually causes the failures
(failures sit at 0.53-0.65 pitch), for no time. The planner knows the lateral
demand of every point on the course before the robot moves; pre-loading height
margin against it beat every reactive scheme tried here. Note it shows nothing
on trotRunning (8/8 either way) - there is no headroom to recover when the gait
is already perfect.

### MEASURED AND REJECTED overnight (equal arms, 5 per arm unless noted)

| lever | result |
|---|---|
| fore/hind stance differential (Zhang 4.3/5.3) | HARMFUL, dose-response the wrong way: 5/5 -> 3/5 -> 2/5, mean pitch 0.391 -> 0.479 -> 0.529 |
| two-axis sprawl guard | 0/5 vs 1/5, and makes BOTH axes worse (roll 0.851 vs 0.674, pitch 1.059 vs 0.386). Does not false-fire (5/5 matched control at 2.1); it just does not help |
| gait decider, re-tested with the governor on | 3/5 vs 3/5 - but see the gait table above, it was switching into the worse gait |
| governor at 2.7 (trotting) | 0/5 both arms - 2.7 is out of trotting's reach entirely |
| walking at 2.5 on the star | 0/5 both arms |

### THE GOVERNOR WORKS - 7/7 vs 3/8, and it costs 0.3 s

Interleaved A/B, star at 2.5 m/s, trotting, 16 runs:

| arm | passes | time | min height reached |
|---|---|---|---|
| stock | **3/8** | 41.7 s | 0.164-0.243 |
| governor | **7/7** | 42.0 s | 0.271-0.275 |

Fisher exact p = 0.026. The cost is 0.3 s (0.7 %), paid as ~15 % of commanded
speed surrendered for a fraction of a second at a time (`minScale` 0.83-0.86)
with the height reference lifted 0.300 -> ~0.345.

The mechanism shows in the spread, not the mean: governed runs land inside a
0.004 m band across seven runs, so the governor is not recovering departures,
it is stopping them from starting. Stock scatters over 0.079 m and the low tail
falls.

Scope: one gait (trotting), one speed, one course. Not yet a claim about 2.8,
about trotRunning, or about hardware.

### THE ATOM COURSE separates TWO failure modes that looked like one

`WP_MISSION=atom:<outer_r>[:<lobes>]` - `WaypointNav::makeAtom`. One closed
stroke, no vertices anywhere:

    x(t) = cos t + A cos(kt),  y(t) = sin t - A sin(kt),  k = lobes - 1

an epitrochoid with (k+1)-fold symmetry. The radius sweeps out to a lobe tip,
back through the nucleus, out to the next, and closes - the atom logo drawn
without lifting the pen. Speed is never zero so there are no cusps: the dog is
always turning and never has to stop and pivot. Default `atom:9.0:6`:

    127.6 m closed stroke, 108 waypoints @ 1.20 m
    turn radius 2.14 - 6.58 m (3.1:1, continuously varying)
    tightest allows 2.31 m/s at a_lat 2.5; nucleus hole 1.00 m

**Join the curve where the tangent is RADIAL, not at a lobe tip.** At a tip the
tangent is perpendicular to the radius, so running out from the nucleus meets
the curve at 90 degrees - a hard corner at the entry of a course whose whole
point is that it has none. The planner duly reported `tightest R=0.27 m` on a
curve whose true minimum is 2.14 m and braked to 0.82 m/s for the artefact.
The fix is the roots of `p x v = 0` with `p . v > 0` (there are `lobes` of them,
all at 3.67 m for the default), rotated so one lies due north. After it,
`tightest R=1.89 m -> 2.18 m/s`, which is the real curve.

**And then it measured something the star could not.** At 2.5 m/s the atom
fails 0/3 - but NOT the way the star fails:

| | star @ 2.5 failure | atom @ 2.5 failure |
|---|---|---|
| min height | 0.164-0.243 | **0.273** (governed), 0.169-0.213 (stock) |
| peak departure | +0.052 to +0.087 | **+0.021** |
| yaw rate before loss | \|w\| <= 0.3 rad/s | **0.88-1.12 rad/s, held 3 s** |
| where | straight, full speed | mid-lobe, still turning |

On the governed atom runs height never departs at all, so the governor has
nothing to detect and cannot help - and indeed governor-on and governor-off
both go 0/3 at 2.5. This is the ROLL limit, which this port already knew was
what bounds cornering, arriving in a form the star never produced.

The reason is the course shape. The star's corners are episodic with straights
to recover on; the atom asks for moderate lateral load CONTINUOUSLY and never
lets up. So "gentler" is only true of peak demand:

    star   brief 36-degree vertices, near-stop, long recovery straights
    atom   no corner tighter than R=2.14 m, and no recovery anywhere

Use the star to exercise force delivery on straights, and the atom to exercise
sustained turning. They are different instruments.

### What does NOT improve it (all measured, do not re-try)

| lever | result |
|---|---|
| lower lateral budget (a_lat 2.2 / 2.0) | SLOWER (42.1-42.7 s) and no more reliable - 2/4 and 1/2 |
| hairpin pivot on all corners | 50.0/50.1 s 2/2 - most repeatable measured, 8 s slower |
| hairpin pivot on corner 1 only | 0/3, always dies at an ARCED corner afterwards |
| gait switching (either pairing) | declines to switch at 2.0, fails above it |
| more yaw authority | roll 27 -> 52 -> 72 deg for no time gain |
| corner crouch | neutral-to-worse; and section 4.7 above says why |

Given correction 3, treat every one of these as "aimed at the wrong part of
the course" rather than as evidence about cornering.

## STAR MISSION, CURRENT STATE (speed-dependent braking, acc 1.5)

| cruise | passes | time | verdict |
|---|---|---|---|
| **2.0 m/s** | **4/4** | 42.2-43.1 s | **the reliable choice** |
| 2.5 m/s | 2/3 | 41.7-41.9 s | marginal, ~1 s faster |
| 3.0 m/s | 1/3 | 40.8 s | fastest single run, not reliable |

The fastest RELIABLE mission is ~42.6 s at 2.0 m/s. 40.8 s at 3.0 is a
one-in-three and should not be quoted as a result. Higher cruise now COMPLETES
at all - which it never did before the braking fix - but it does not yet
complete dependably, and the gain over 2.0 is about a second on a 100 m course.

## THE REAL CORNER BUG: the braking zone was shorter than the stopping distance

Every mission above 2.0 m/s failed at the FIRST corner, and it was never the
gait, the friction, or the lookahead. Instrumenting the approach showed the
robot passing wp0 at 2.58 m/s STILL ACCELERATING, overshooting 5.5 m.

The speed profile's braking zone was shorter than the robot's real stopping
distance. At a_lon=1.5 the profile begins braking 1.85 m before the corner; the
body decelerates at a MEASURED 1.2 m/s^2 and needs ~5 m including tracking lag.
The plan issued a deceleration it had already left too late to achieve - and no
lookahead can rescue that, because there is nothing earlier in the profile to
look ahead AT.

**The planning value must be LOWER than the physical one**, so the zone is
longer than the stop:

| cruise | a_lon | result |
|---|---|---|
| 2.5 | 0.6 | FAILED |
| 2.5 | **0.4** | **46.0 s PASS (2/2)** |
| 2.5 | 0.3 | 47.6 s PASS |
| 3.0 | **0.4** | **45.4 s PASS** |
| 3.0 | 0.3 | 47.2 s PASS |

2.5 and 3.0 had failed EVERY prior attempt - across gait pairings, corner
budgets, lateral limits and lookahead. This is what unlocked them.

The times are still slower than the 42.2 s baseline at 2.0: on 20 m legs the
longer braking zone costs more than the faster straights gain. The value is the
CAPABILITY (missions at 3.0 cruise) not the clock.

### The first corner of this star is a HAIRPIN, and the others are not

Every failure in this project has been at wp 1/5. The robot starts at the star's
CENTRE, so leg 1 is a radius and leg 2 is a chord: turning from +N at wp0 toward
wp1 is a 162 deg direction change - an **18 deg interior angle** against 36 deg
at every other corner. The planner reports it as `tightest R=0.28 m -> 0.83 m/s`.
That corner is an artefact of where the mission STARTS, not of the star, and it
sets the speed limit for the whole course.

## THE STAR IS CORNER-BOUND: what moves it, and what does not

Measured after the planner landed. Baseline is trotting at 2.0 m/s, 44.4 s, 3/3.

| change | time | runs | peak roll |
|---|---|---|---|
| baseline (a_lat 2.5, accept 1.0) | 44.4 s | 3/3 | 14.0 deg |
| a_lat 3.0 | 44.4 s | 1/2 | 40.4 on the failure |
| a_lat 3.5 | 44.5 s | 1/2 | 35.8 |
| **accept 1.5** | **42.2 s** | **2/2** | 20.8-22.0 |
| accept 2.0 | 40.3 s | 1/2 | 21.3 |
| a_lat 3.0 + accept 2.0 | - | 0/2 | |

**More lateral budget buys NOTHING.** Corner speed goes as `sqrt(a_lat * R)`, and
at R = 0.45 m the corner segment is so short that 40% more budget saves no
measurable time while driving peak roll from 14 to 40 deg. The binding
constraint is corner GEOMETRY, not the lateral allowance.

**Wider acceptance works** because it raises R *and* shortens the path - the two
things that actually matter. 1.5 m is the sweet spot.

**GAIT SWITCHING CANNOT BEAT IT EITHER.** The gait decider (Apollo's task of the
same name, switching only while slow) was built and measured:
- trotRunning straights / trotting corners: at 2.0 it correctly DECLINES to
  switch (planned speed never reaches the threshold, so trotRunning is never
  worth it) and matches baseline; at 3.0+ trotRunning fails the star regardless.
- trotting straights / walking corners: fails above 2.0, and at 2.0 again
  declines to switch and matches baseline (44.6 vs 44.5 s).
The mechanism is sound; the LEGS ARE TOO SHORT. trotRunning needs ~7 m to reach
cruise and ~7 m to shed it, against a 20 m leg, and it corners worse. Gait
switching would pay on long legs, not on this course.

**So the honest statement is that a 100 m star with 20 m legs and 144 deg
corners is bound at ~42-44 s, and the remaining lever is a MISSION decision
(how close must it pass each waypoint), not a controller one.**

## THE STAR TABLE: every gait, ranked - and it does NOT match the dash table

100 m star (5 legs of 20.0 m, r = 10.514 m), real estimator, qpOASES, with the
Apollo-derived planner (`WP_PLANNER=1`). "course m/s" is the nominal 100 m of
waypoint path over the elapsed time; the robot's own path is shorter because the
planner cuts corners inside the 1.0 m acceptance radius.

| rank | gait | num | cmd m/s | time | course m/s | runs |
|---|---|---|---|---|---|---|
| 1 | **trotting** | 9 | 2.0 | **44.4 s** | **2.25** | **3/3** |
| 2 | trotRunning | 5 | 1.5 | 54.8 s | 1.82 | 1/1 |
| 3 | walking | 20 | 1.5 | 56.4 s | 1.77 | **3/3** |
| 4 | bounding | 1 | 1.5 | 57.2 s | 1.75 | 1/3 marginal |
| 5 | **galloping** | 22 | 0.8 | 103.7 s | 0.96 | 1/1 |
| - | walking2 / pacing / pronking | 21/8/2 | - | no completion | - | 0 |

**GALLOPING RUNS A WAYPOINT MISSION.** It had never travelled more than ~13 m in
this port's history. It needed BOTH fixes: the solver, so a 40% duty gait can be
given the `m*g/duty` it actually needs, and the planner, so it is never asked to
corner at a speed its flight phase cannot redirect from.

### The dash table and the star table rank gaits DIFFERENTLY

| gait | duty | straight 100 m dash | 100 m star |
|---|---|---|---|
| trotRunning | 40% | **4.70 m/s, 24.8 s (1st)** | 1.5 m/s, 54.8 s (2nd) |
| trotting | 50% | 3.46 m/s, 32.2 s (2nd) | **2.0 m/s, 44.4 s (1st)** |
| walking | 50% | 2.57 m/s, 42.0 s (3rd) | 1.5 m/s, 56.4 s (3rd) |
| galloping | 40% | never crosses | 0.8 m/s, 103.7 s |

The fastest gait in a straight line is the WORST at cornering, and the duty
factor explains both halves. 40% stance means ~20% of every cycle fully
airborne: a body in flight has no feet to push against, so it cannot redirect
itself, cannot generate yaw authority and cannot arrest roll. That flight phase
is exactly what buys top speed on a straight and exactly what costs turning
authority in a corner. trotting's 50% duty always has a diagonal pair down.

**So straight-line speed does not predict mission speed, and a "fastest gait"
claim has to say which task it means.**

## CORNERING: what animals do, and what this port forbids

A dog, horse or cheetah taking a sharp corner does NOT hold speed through it. It
brakes hard on approach, plants and pivots at low speed, then accelerates out -
because turning force and propulsion draw on the same friction budget, so a
tight turn has to be bought with speed. A cheetah drops out of its sprint gait
entirely to turn.

This port forbids exactly that. `WP_TURN_FLOOR = 0.65` holds the robot at >=65%
of cruise through every corner, and the comment explains why: *"Dynamic gaits
arc through corners; they fall over trying to pivot in them."*

**That constraint was measured under the broken solver and is now obsolete.**
The yaw envelope above says the reverse: pirouetting at 3.0 rad/s with zero
forward speed is the most stable thing this robot does (roll < 1.5 deg, no falls
at any rate tried), while the ARC that nav prefers instead is what trips the
roll limit - 57.5 deg at 3.0 m/s against a 28.6 deg safety trip. The robot is
being made to do the dangerous thing to avoid the safe one.

### A star mission is corner-limited, not speed-limited

100 m star, 5 legs of 20.0 m (r = 10.514 m; leg = 1.9021*r, mission = 9.5106*r):

| gait | speed | result |
|---|---|---|
| trotRunning | 2.0 | COMPLETE 45.4 s, 5/5, peak roll 16.6 deg |
| trotting | 2.0 | COMPLETE 46.3 s, 5/5, peak roll 11.9 deg |
| trotting | 2.5 / 3.0 | FAIL at the FIRST corner (1/5) |
| trotRunning | 3.0 / 4.0 | FAIL at the first corner |

Both completions run ~9.1-9.3 s per 20 m leg - essentially the straight-line
dash time - so the legs are free and the corners cost everything. Peak roll at
failure is 27.6-57.5 deg against the 28.6 deg trip: **corner failure IS roll
failure**. Lengthening the legs would raise the average speed by amortising the
same five corners over more distance, but it does not raise the cornering limit.

### METHOD ERROR worth keeping: tune at the MARGIN, not past the cliff

The first corner sweep was run at 3.0 m/s - a speed at which the mission ALREADY
failed at 1/5 before any corner parameter was touched. Every cell failed at 0/5
and the sweep measured nothing, because a parameter that helps cornering cannot
rescue a speed the gait cannot hold through a corner at all. Sweep the knob at
the speed where the thing ALMOST works (here 2.5), not where it is hopeless.

## YAW ENVELOPE: pirouettes are SOLID, cornering is roll-limited

Measured with `SIM_WZ` (a yaw-rate command on the same ramp as `SIM_VX`, driving
the EXISTING right-stick channel -> `_yaw_turn_rate` -> MIT's stance-foot
rotation about body Z) and `[YAW]` instrumentation ($SIM_YAWDBG=1). trotting,
qpOASES, real estimator.

### Spinning in place (vx = 0)

| commanded | achieved | tracking | max roll | fell |
|---|---|---|---|---|
| 0.5 rad/s | 0.50 | 100% | 0.8 deg | no |
| 1.0 | 1.00 | 100% | 1.1 deg | no |
| 1.5 | 1.50 | 100% | 1.5 deg | no |
| 2.0 | 1.96 | 98% | 1.4 deg | no |
| 3.0 | 2.77 | 92% | 1.0 deg | no |

**3.0 rad/s is 172 deg/s - a full turn in 2.1 s - with roll under 1.5 deg and
body height dead steady at 0.266-0.269 m. Nothing falls.** The worry that the
robot would fall while pirouetting is not supported: spinning in place is the
most stable thing it does. This ALSO retires the crawl-era failure mode
documented above ("a hard v=0 pivot deadlocks whenever the achievable yaw rate
is below the commanded one") - on the MPC the achievable rate is ~100% of
commanded up to 2 rad/s, so the deadlock condition never arises.

### Cornering (turning while translating) - ROLL is the limit

| vx | wz | tracking | max roll | turn radius |
|---|---|---|---|---|
| 1.5 | 0.5 | 99% | 7.3 deg | 3.0 m |
| 1.5 | 1.0 | 97% | 12.2 deg | 1.5 m |
| 1.5 | 1.5 | 92% | **19.5 deg** | 1.09 m |

Yaw tracking stays good, but roll climbs steeply with turn rate - the banking
load. 19.5 deg is already 68% of `SafetyChecker`'s 28.6 deg trip, so **roll, not
yaw authority, is what bounds cornering**, and a nav clamp should be written
against roll rather than against yaw rate alone.

### SIGN: positive stick command turns CLOCKWISE (measured)

`wzCmd=+1.000` gives `wzTrue=-1.060`. The estimator agrees with truth
(`wzEst=-1.033`), and `yawEst + 1.569` equals `yawTrue` mod 2*pi - the constant
is the spawn yaw (+90 deg, body-x north) that the estimator zeroes out. So
estimation is fine on this axis; the COMMAND channel is what reads inverted
against MIT's documented CCW-positive intent.

**This contradicts the justification recorded for the nav mapping.** The note
above says `rightStickAnalog[0] = -navW` because "MIT's yaw rate is CCW-positive
and nav's is compass-sense (CW-positive)". Measured, the stick is ALREADY
CW-positive - the same sense as nav - which would make that negation wrong and
steer the robot away from each waypoint. The star mission did complete, and
`WP_YAW_SIGN` exists to flip it, so this is not asserted as a bug: it is a
DISCREPANCY between the documented reason and the measured channel, and it must
be settled by a nav test before any pirouette-to-waypoint work is trusted.

## BEFORE TRUSTING A REAL DOG: sim-fidelity gaps and the QA ladder

**Nothing in this file is hardware-validated.** These are SITL numbers, and at
4-5 m/s the simulation is flattering. Work this list before any of it is
believed on a machine that can hurt itself.

### Known places the sim is more generous than reality

| gap | sim | reality | why it matters at speed |
|---|---|---|---|
| **foot friction** | **mu = 2.0** | URDF says **0.6**; rubber on concrete ~0.8-1.0 | Set early to stop feet skating inward and collapsing the support polygon. At 4-5 m/s traction does enormous work; this is the single most suspect number behind the top-end results. |
| **actuator dynamics** | commanded torque applied directly | motor current limit, thermal derating, back-EMF at speed | No torque is ever refused. Peak knee demand measured at 26.4 Nm of a 35.55 Nm limit - fine on paper, but the real limit falls as the motor heats and as joint speed rises. |
| **joint velocity** | unbounded | finite, and the binding constraint for a fast swing | A swing leg that must return 0.4 m in 0.1 s may simply be unable to on hardware. |
| **transport** | loopback UDP, 0 stalls | RS485 at 500 Hz + eth0 that has FLAPPED under load | A dropped command window folds the robot; the bridge watchdog exists for this. |
| **IMU** | Gazebo IMU, and orientation is a NOISE-FREE pass-through | real VectorNav/DroneCAN noise and bias | The estimator has never been tested against realistic orientation error - `VectorNavOrientationEstimator` just forwards the sim's exact pose. |
| **ground** | perfectly flat speedway | farm mesh spawns on a 7.6 cm rise | Terrain following works well enough to STAND on the mesh, not to walk on it. |
| **contact** | idealised point contact | compliant foot, slip, debris | |

### AUDIT: every SIM_ variable, and which ones are actually a problem

51 of them exist. **`SIM_CHEATER` is the only one that ever fed the controller a
lie, and it is deleted** (the sole remaining occurrence is inside a comment).
Nothing else falsifies state. But several are load-bearing behaviour hiding in
an environment variable, which is its own hazard: an operator who forgets one
gets a different robot.

**A. Operator / test driver** - replaces a human hand on the gamepad, or the nav
layer. NOT cheating; on hardware the real gamepad or `WaypointNav` drives the
identical channel.
`SIM_VX` `SIM_WZ` `SIM_VX_DELAY_S` `SIM_VX_RAMP_S` `SIM_GAIT` `SIM_MODE`
`SIM_BAL_S` `SIM_STAND_S` `SIM_LOCO_S` `SIM_SKIP_BAL` `SIM_GAIT_WAIT_MS`

**B. Instrumentation** - print only, zero control effect.
`SIM_ESTERR` `SIM_MPCZ` `SIM_CONTACT_DBG` `SIM_YAWDBG` `SIM_AID_DBG`
`STM32MP1_EST_DBG` `STM32MP1_MPC_IN` `STM32MP1_MPC_MAT`

**C. Real tuning that MUST be promoted into config** - these change control
behaviour and every result in this file depends on them.
`SIM_MPC_MS` `SIM_SWING_H` `SIM_MPC_HORIZON` `SIM_WBC_DECIM` `SIM_MPC_ASYNC`
`SIM_BODY_H` `SIM_F_MAX` `SIM_KP_STANCE` `SIM_MPC_Q` `SIM_MPC_ALPHA`
`SIM_MPC_NWSR` `SIM_MPC_WARM` `SIM_MPC_PRIO` `SIM_MPC_SCHED_LEAD`
`SIM_CMD_FILTER`
> `SIM_MPC_MS` is the sharp one: it is **speed- AND gait-dependent** (22 ms for
> trot at 3.0+, 26 ms below and for trotRunning), a hidden default of 26 silently
> caps trot at 67 m, and there is currently no scheduling logic - a human picks
> it per run. This must become a function of commanded speed before anything
> autonomous uses it.

**D. Behaviour that must become DEFAULT, not a flag** - each fixes a genuine
upstream gap, and running without them is running a worse robot.
`SIM_HEADING_HOLD` (MIT has NO heading regulation at all)
`SIM_YAW_RATE_KP` `SIM_YAW_ERR_MAX` (the fix that took walking 9.3 m -> 93.6 m)
`SIM_ZEROVEL_HOLD_GAIT` `SIM_ZEROVEL_HOLD` (hold the standing gait until a
command exists - matches both operator practice and Unitree's own transition
request)

**E. Test-harness safety that is WRONG on hardware as written** -
`SIM_FALL_EXIT` `SIM_FALL_DEG` `SIM_FALL_Z` `SIM_FALL_HOLD_S`.
The detector zeroes the legs and then **exits the process**, which is right for
a sweep and dangerous on a machine: process exit also stops whatever was feeding
the motor watchdog. Hardware wants latch-limp-and-hold under supervision, and it
should key off ATTITUDE and kinematics rather than the ESTIMATED height it uses
now (that threshold already killed a day of valid runs when the estimate drifted
near it).

**F. Dead experiments - measured null or harmful, default off, should be DELETED**
`SIM_ABS_AIDING` `SIM_AID_TAU` (GPS/baro aiding: null on working gaits, and adds
a catastrophic tail - 0.25 m on one run)
`SIM_KF_UNCAP` (solved a problem that does not exist - Unitree ships the
identical covariance cap)
`SIM_CONTACT_DETECT` `SIM_CONTACT_BAND` `SIM_FREEFALL_G` (measured REGRESSION:
21.3 m -> 5.6 m, destroys the KF's trust ramp)
`SIM_BALLISTIC_Z` (null) `SIM_FLIGHT_COST_GATE` (harmful: cut force 39-42 ->
6.1 N/foot) `SIM_GPS_SIGMA` `SIM_BARO_SIGMA` (sim-only sensor noise)

**G. Deleted** - `SIM_CHEATER`.

**The pre-hardware task is therefore**: fold C and D into the yaml, rework E,
delete F, and leave only A (operator input) and B (debug prints) as environment
variables. Until then the honest statement is that these results depend on a
specific env incantation, recorded in `SKILL.md`, and not on the shipped config.

### The QA ladder (run in this order, stop at the first failure)

Deliberately boring, and all of it BEFORE any commanded navigation:

1. **Stand** - power on limp, stand to 0.30 m, hold 60 s. Watch body height and
   attitude drift, and RS485 error counters.
2. **Lie down** - controlled descent to belly, legs limp. This is the recovery
   path for everything below.
3. **Stand back up** from the belly - the transition that has to work before any
   fall is survivable.
4. **Slow walk** - 0.2-0.3 m/s, straight, a few metres. The statically-stable
   crawl (`static_gait_sim`) is the safest first mover; it is a sequence of
   static poses and cannot run away.
5. Only then: dynamic gaits at low speed, then yaw, then waypoints.

Steps 1-4 need no MPC at all and exercise the parts most likely to be wrong on
hardware: joint sign conventions, gear ratios, RS485 framing, torque scaling,
IMU orientation. **Validate those against a machine on a stand, with the legs
off the ground, before it ever bears weight.**

### The remaining SOLVER work is for the BOARD, not for the gaits

Worth stating precisely, because "fix the solver" now means something narrower:
- On the Mac the solver IS fixed - qpOASES, 0.6-1.7 ms against a 2.0 ms budget.
- The gaits that still fail (`galloping`, `pronking`, `bounding`, `walking2`,
  `pacing`) fail ON qpOASES, with correct forces available. Fixing JCQP will not
  rescue them; they fail for their own reasons.
- **JCQP is what the BOARD needs.** qpOASES costs 198-218 ms on the A7 against a
  26 ms segment, so the STM32 cannot run it inline. Either JCQP has to be made
  to converge on a moving gait, or the board runs qpOASES on the async path
  (`SIM_MPC_ASYNC=1`), or the contact-reduced qpOASES has to be re-measured
  there. Until one of those, **none of tonight's speeds can reach hardware.**

## The solver REORDERED the gait hierarchy (it is not a uniform lift)

All eight gaits re-screened at 2.0 m/s on qpOASES, real estimator. 2.0 is the
qualifying floor - a gait that cannot hold 2.0 is out of the running for
"fastest" by definition, so nothing below it is worth measuring.

| gait | num | ms22 | ms26 | qualified |
|---|---|---|---|---|
| walking | 20 | **46.6 s (2.29 m/s)** | 48.3 s (2.20) | YES |
| trotting | 9 | **47.2 s (2.24)** | 83.0 m fail | YES |
| trotRunning | 5 | 48.2 s (2.18) | **44.8 s (2.37)** | YES |
| bounding | 1 | **99.9 m** (1 m short!) | 17.7 m | no |
| galloping | 22 | 12.8 m | 6.9 m | no |
| pronking | 2 | 10.6 m | 5.5 m | no |
| walking2 | 21 | 5.6 m | 4.6 m | no |
| pacing | 8 | 0.4 m | 0.2 m | no |

### WHY it reordered: the solver fix INVERTED the optimal duty factor

The gait table (`OffsetDurationGait(nSeg, offsets, durations, name)`, 10 segs):

```
walking  (20): offsets (0,3,5,8)  durations (5,5,5,5)  -> 50% duty, 4-BEAT
walking2 (21): offsets (0,5,5,0)  durations (7,7,7,7)  -> 70% duty, diagonal pairs
trotting  (9): offsets (0,5,5,0)  durations (5,5,5,5)  -> 50% duty, diagonal pairs
trotRunning(5):offsets (0,5,5,0)  durations (4,4,4,4)  -> 40% duty, diagonal + FLIGHT
galloping(22): offsets (0,2,7,9)  durations (4,4,4,4)  -> 40% duty, asymmetric
```

So `walking2` is literally **trotting with a 70% stance** - same diagonal-pair
footfall, much longer contact. And peak force to hold height is `m*g / duty`:

| gait | duty | peak force needed | rank under JCQP | rank on qpOASES |
|---|---|---|---|---|
| walking2 | 70% | **1.43x mg** | **1st** (121.9 s) | fails at 5.6 m |
| trotting | 50% | 2.00x mg | 2nd | 2nd |
| walking | 50% | 2.00x mg | LAST that crossed | 3rd |
| trotRunning | 40% | 2.50x mg | never crossed | **1st** |

**Under a solver delivering 0.25-0.45x mg, the only gait that could function was
the one demanding the LEAST peak force.** walking2 topped the old table because
it is the CHEAPEST gait, not the best one. With force actually available that
advantage vanishes, and its long stance becomes a liability: stance duration x
speed is the distance the foot must sweep, and at 70% duty and 2.0 m/s that is
~0.308 m against ~0.318 m of horizontal reach - right at the kinematic limit.

Low duty was previously unaffordable and is now optimal. That is the whole
reordering, and it predicts the new ranking from duty alone.

**Two complete reversals against the JCQP table**, which is the point worth
keeping:
- `walking` was the SLOWEST gait that crossed (162.4 s at 0.6 m/s). It is now
  the FASTEST at 2.0 m/s (46.6 s), beating trotting.
- `walking2` was the BEST gait (121.9 s at 0.8 m/s, the headline of the old
  table). It now fails at 5.6 m.
- `pacing` crossed at 0.6 m/s under JCQP and now collapses at 0.4 m.

So the solver did not lift every gait by a constant factor - it changed which
gaits work at all. Any per-gait conclusion in this file measured under JCQP
describes the solver's failure mode for that gait, not the gait.

**`trotRunning` is the first flight-adjacent gait to cross on the real
estimator** (it never did before), and it prefers the 26 ms segment where
trotting and walking prefer 22 - so segment is gait-dependent as well as
speed-dependent.

**`bounding` reached 99.9 m of 100** at 2.0/ms22. A flight gait that has never
travelled in this port's history came one metre short. Parked deliberately: the
simple gaits get their full dash first, then the flight-gait machinery
(`getFlightState`, per-leg stance/swing timing, `zeroVelTransitionAmend`) gets
revisited - all of it was validated only against the broken solver.

## Speed tiers fail by DIFFERENT mechanisms - do not re-fix a solved one

Each ceiling this port has hit had its own cause. Diagnosing the tier you are
actually at matters more than any single lever, because the lever that unlocked
the previous tier is inert at the next one.

| tier | mechanism | signature | status |
|---|---|---|---|
| <=2.75 m/s | force starvation | `Fz/mg = 0.25` vs 2.00 needed; body sinks to z=0.028 flat, `roll=0 pitch=-0` | **FIXED** - qpOASES |
| 3.0 m/s | vertical oscillation | force fine (2.45-2.64x), body vz +-0.5-0.7 m/s, bounces itself down over 50-95 m | **FIXED** - 22 ms segment |
| 3.5 m/s | open | orientation safety trips, then sinks; force 2.5x and height 0.285-0.292 until the last 0.3 s | **OPEN** |

### CORRECTION: "3.5 m/s fails" was the wrong question

Commanded speed and ACHIEVED speed are not the same thing here - the robot
cruises consistently faster than commanded (3.0 -> 3.46 m/s, 2.0 -> 2.24). The
achievable ceiling is about **3.5 m/s of actual ground speed**, so commanding
3.5 asks for roughly 3.9 and is guaranteed to fail.

That invalidates the FRAMING of the sweeps below, though not their data. Every
one of those ~20 configurations was being asked for ~3.9 m/s - past the
machine's limit - so none of them could have succeeded, and the flat response
across all of them measures the overshoot rather than the parameters. Walking
the command up finely instead finds the edge is between 3.15 and 3.2 commanded:

| commanded | achieved cruise | 100 m | result |
|---|---|---|---|
| 3.15 | 3.52 m/s | 31.7 s | crossed |
| 3.1 | 3.46 m/s | 32.2 s | crossed 3/3 |
| 3.2 | - | - | fails at 31-38 m |
| 3.5 | - | - | fails at 20-28 m |

**Lesson**: when a ladder rung fails, check what the robot was actually DOING
before sweeping parameters at that rung. A command the machine cannot achieve
makes every parameter look inert, which reads exactly like "this lever does
nothing" and is how a whole afternoon gets spent proving nulls.

### The 3.5 m/s wall: what it is NOT (all measured)

Recorded so none of this gets re-tried. Every row is a real sweep, not an
argument:

| candidate | result | evidence |
|---|---|---|
| force starvation | NO | MPC commands 2.45-2.64x mg against the 2.00 the duty needs |
| orientation gains | NO | `Kd_ori` 40/60/80 and `Kp_ori` 40/80 -> 27.7 / 27.5 / 26.9 / 27.1 m. Flat within noise - a genuine null, not a weak effect |
| swing clearance | NO, HARMFUL | `SIM_SWING_H` 0.11 -> 27.7 m, 0.14 -> 22.1 m, 0.17 -> 13.8 m |
| joint torque limit | NO | peak `abad 10.4 / hip 17.5 / knee 26.4 Nm` against 23.7 / 23.7 / 35.55 - knee at 74% |
| leg reach limit | NO | max `|p| = 0.393` against `_maxLegLength 0.430`; only 0.5% of samples exceed 0.38 m |

The orientation-gain null is worth dwelling on: the failure PRESENTS as attitude
loss (`Orientation safety check failed`), and the gains that govern attitude do
nothing. So attitude is a symptom of the collapse, not its cause - the same
mistake shape as blaming the estimator for errors it only shows AFTER the fall.

Segment and horizon were then swept properly at 3.5, and both have interior
optima that the base config already sits on:

| `SIM_MPC_MS` | 18 | 20 | 22 | **24** | 26 | 28 | 30 | 32 |
|---|---|---|---|---|---|---|---|---|
| distance (m) | 12.2 | 15.7 | 25.5 | **27.4** | 24.6 | 23.3 | 20.0 | 19.1 |

| `SIM_MPC_HORIZON` | **10** | 12 | 14 | 16 |
|---|---|---|---|---|
| distance (m) | **23.3** | 22.0 | 7.4 | 9.0 |

So ~20 configurations across segment, horizon, orientation gains and swing
clearance all land at 20-28 m, and the best of them (27.7 m) is the stock
config. Nothing cheap moves this wall.

**Remaining candidate**: contact timing - whether the swing leg completes its
trajectory and is actually on the ground when the MPC's contact schedule says it
is. Force commanded is not force delivered, and a schedule/reality mismatch
sends the solved force into air. Measure scheduled contact against actual foot
height before trying anything else. Note the swing-clearance result is
consistent with this: raising swing height gives the foot MORE distance to cover
in the same swing window and makes things monotonically worse (27.7 -> 22.1 ->
13.8 m), which is what a swing that is already time-starved would do.

## THE 100 m DASH, REAL ESTIMATOR (the honest table)

Measured with `SIM_CHEATER` **absent** from the environment - the harness no
longer sets it, and the sweep was launched under `env -u SIM_CHEATER`. Gazebo
truth is used only by `dash_trace.py` to MEASURE distance; it never enters the
control loop. Corrected model (12.859 kg, real rotor inertia/locations, knee
gear 9.4995) plus the WBIC damping config (`Kd_body=[40,40,40]`,
`Kd_ori=[40,40,20]`). Fall detector OFF.

| gait | num | max speed | 100 m | cruise | next rung up |
|---|---|---|---|---|---|
| **walking2** | 21 | **0.8** | **121.9 s** | 0.83 | 1.0 -> 10.4 m |
| walking | 20 | 0.6 | 162.4 s | 0.61 | 0.8 -> 10.2 m |
| pacing | 8 | 0.6 | 183.5 s | 0.54 | 0.8 -> 5.3 m |
| trotting | 9 | 0.6 | 185.8 s | 0.53 | **0.8 -> 78.7 m (MARGINAL, repeating)** |
| trotRunning | 5 | - | never | - | fails 0.8/0.6/0.4 (2.0/3.0/4.7 m) |
| bounding / pronking / galloping | 1/2/22 | - | never | - | not re-run; never crossed on cheater either |

**Four of five gaits now cross 100 m on the robot's own state estimate.** The
previously recorded honest baseline never crossed at all (walking2 ~21 m at
1.0 m/s, 34.95 m at 0.6). That is the real gain from this session's model
corrections and WBIC damping - not the cheater table's headline times.

### Cheater vs real: the gap is the estimator, and it is large

| gait | cheater ceiling | real ceiling | cheater 100 m | real 100 m |
|---|---|---|---|---|
| trotting | 1.0 | 0.6 | 106.5 s | 185.8 s |
| walking2 | 1.0 | 0.8 | 107.0 s | 121.9 s |
| walking | 0.8 | 0.6 | 130.7 s | 162.4 s |
| pacing | 0.8 | 0.6 | 131.1 s | 183.5 s |
| trotRunning | 0.6 | - | 175.9 s | never |

trot is the worst-hit: it crosses at 1.0 m/s on ground truth and dies at
**4.4 m** at the same speed on its own estimate. Whatever the WBIC damping fix
bought at trot@1.0 was bought *against ground-truth state*; it does not survive
the real LinearKF. walking2 degrades least (one rung), which is consistent with
it being the yaw-balanced, most forgiving gait.

**So the estimator remains THE wall, and it is now the only thing between this
port and ~1 m/s.** Every gait's real ceiling is one to two rungs below its
cheater ceiling, and the failures are not gradual - the robot goes down inside
5-10 m at one rung above its ceiling, versus completing 100 m at the ceiling.

### Method note
`dash_sweep.sh` stops at the first speed that completes, which biases the
answer DOWNWARD for any gait that is bimodal near its limit (this trap has
already understated trot by 50% once in this file). The rung above every
reported ceiling is therefore listed explicitly above. Four of them fail
decisively (4-10 m). trot@0.8's 78.7 m does not, and is being repeated.

## Still open
- Heading hold for the MPC trot: 11.4 m drifted 5 m left (no yaw feedback in
  the straight-line sequencer). Wiring WaypointNav into the mit_ctrl sequencer
  (yaw + velocity like the crawl has) gives missions AND heading hold at once.
- Real-estimator walking: 0.65 m vs cheater's 11.4 m. LinearKF tuning /
  contact-phase interplay under the trot.
- The trot's travel direction is inverted: at 1.0 m/s commanded it makes 1.00
  m/s of ground speed with the *magnitude right and the sign wrong*, and
  flipping the stance sweep changes the magnitude rather than the sign, so the
  propulsion is coming from somewhere other than the stance sweep (most likely
  swing-leg ground contact). The crawl, with identical IK and sweep direction,
  goes forward.
- Terrain following works well enough to stand on the farm mesh but not to walk
  on it; `worlds/go1_farm_flat.sdf` keeps the farm scenery with a flat walkable
  ground as an interim.

## The Conductor (Aug 2026): fleet control panel, cornering fix, dash-as-finish

`stm32mp1/gazebo/conductor/` is a browser panel (`server.py`, aiohttp-free
stdlib HTTP + polling, `static/*.js`) that orchestrates up to 3 Go1s in Gazebo:
draft a mission per dog slot (course, gait, speed, dash distance, per-dog
camera config), launch, watch live telemetry and three body-mounted camera
feeds per dog (front/nadir/chase), pull raw logs, stop. `GET /docs` is a
Swagger-style reference for every route (Play button + curl + live response),
covering `/api/state`, `/api/logs/{i}` (raw `ctrl`/`bridge` log text, not the
curated event feed `/api/state` carries), `/api/launch`, `/api/stop`,
`/api/slots/*`, `/api/speed_cap`, `/api/terrain`.

### Cornering: the star's "elephant foot" was a steering-rate bug, not a geometry bug

The angle-graded corridor fix (earlier session) correctly tightened the fillet
radius at a sharp corner (down to R~0.03 m at the star's tightest vertex) but
`BodyPathPlanner::computeSpeedProfile()`'s `p.v_max` was capped ONLY by
traction (`v <= sqrt(a_lat_max/kappa)`) - it never checked whether the body
could physically STEER that fast. At R~0.03 m, traction allowed 0.31 m/s while
the body's `yaw_rate_max` (1.2 rad/s) only allows `wz_max/kappa ~= 0.036 m/s` -
commanding the traction number sent the robot into a turn it could not track,
and that does not fail safe: it overshoots and loops back around the corner
before recovering, which is the "elephant foot" shape reported at every corner
of the star, not just the tightest one.

**Fix**: a second, independent cap, `v_steering = yaw_rate_max / kappa`, with
`v_max = min(v_traction, v_steering)`. The `v_min` floor previously applied to
`p.v_max` had to be removed too - it was silently re-flooring the new cap back
up at exactly the corners it exists to slow down. (The UNRELATED `v_min` floor
on `_path[0].v`, which exists so `follow()`'s nearest-index lookahead can
advance off a literal zero, is untouched - different assignment, different
purpose, do not conflate them again.)

**Verified two ways before trusting it**:
1. Isolated open-loop yaw-sweep (`SIM_VX=1.0 SIM_WZ=1.2`, no waypoint nav,
   direct stick commands) - the standard method from earlier sessions, reused
   per direct instruction rather than re-derived. Yaw tracked ~100% of
   commanded over a sustained 35 s, roll held 6-9 deg, no falls - so steering
   authority itself was never the problem, only commanding a curvature the
   body could not deliver was.
2. Live full star run (dash=0): clean pentagram, all five corners tight, no
   loop-back at any vertex, confirmed both in the nav logs and a browser
   screenshot of the drawn-vs-flown overlay.

### Dash-as-finish: the 100 m dash ends a mission, it does not replace one

Per direct instruction, `dash` is now how a course FINISHES, not a standalone
mission: complete the full waypoint loop, return to wp0, THEN sprint a
straight `dash` metres onward. `WaypointNav::appendDash(distance_m, speed)`
appends two points, not one - first an explicit return-to-wp0 (v1 skipped
this and extrapolated the dash off the raw final-leg vector instead, which
shot it out at the star's oblique tip angle when the user first tested it),
then a point `distance_m` beyond wp0 along the wp[n-1]->wp0 heading.
`mit_sim_main.cpp` captures `dash_wp_index = loop_wp_count + 1` (one past the
return point) so the loop-complete interlude fires at the right boundary.
Verified geometrically correct and reproduced live: full star+dash test just
run shows a clean return to wp0 (`reached wp05 (N=10.51 E=0.00) dist=1.38`,
which IS wp0's coordinate) before the interlude fires - the "we never went
back to wp0" bug reported earlier is fixed.

### The loop-to-dash interlude (stop, lie down, stand back up, dash) - STILL UNRESOLVED

This is the one open item from this session and it should NOT be reported as
fixed. Three real, independently-verified bugs were found and fixed chasing
it, and the sequence still falls:

1. **`BALANCE_STAND -> STAND_UP` is not a legal FSM transition.**
   `FSM_State_BalanceStand::checkTransition()` has no `K_STAND_UP` case, so
   every lie-down request into that path was silently rejected (100+ "Bad
   Request" log lines, confirmed by reading the FSM source directly, not
   inferred from behaviour). Fixed by routing through `K_PASSIVE` first, the
   FSM's own legal path to `STAND_UP`.
2. **The `PASSIVE` hop cuts leg torque entirely, and `edamp` was not covering
   it.** `RobotRunner::finalizeStep()` applies `edampCommand()` AFTER the FSM
   state's own control output regardless of which state is active - confirmed
   from the call order in `RobotRunner.cpp` (control at line ~319, edamp at
   ~524). Fixed: `setEdamp(8.0)` spans the `PASSIVE` hop, `setEdamp(0.0)`
   before `STAND_UP`'s own interpolation so damping does not fight the
   position controller.
3. **The post-arrival deceleration ramp coasted the robot unsteered past the
   stop point.** Lengthening the ramp (to reduce deceleration rate) made this
   WORSE, not better - confirmed live: `v=3.50` still logged at `reached
   wp05`, and the robot visibly overshot ~5 m before slowing. Fixed: short,
   sharp deceleration (15 steps x 50 ms, ~0.75 s) followed by CLOSED-LOOP
   verification - poll the real `vBody` magnitude from the state estimate and
   wait for it to drop under 0.15 m/s (2 s timeout) before touching the FSM at
   all, rather than trusting a fixed timer.

All three are real, and each was independently confirmed against source or
log evidence before being called a fix - but the interlude still falls. Most
recent full-sequence test (star course, dash=30, with the cornering fix
active): the robot ran a clean loop, returned to wp0 exactly as designed
(`dist=1.38` at wp05), printed `[nav] loop complete ... stop, lie down, stand
back up`, and then collapsed within roughly a second - `[FALL] collapsed:
roll=-3 deg pitch=12 deg z=0.071 m held 0.50 s`. Two things about this
specific failure are worth recording because they narrow where to look next:

- **The attitude is mild** (3 deg roll, 12 deg pitch) but **height is not**
  (0.071 m, well below any standing pose) - the same "height collapses before
  attitude does" signature documented elsewhere in this file for the star at
  2.5 m/s, not a tip-over signature.
- **It happens at or before `[GAIT] Transitioning gait from TROT to STAND`**,
  which fires automatically off the velocity-ramp-to-zero (this is upstream of
  and separate from the interlude's own explicit
  PASSIVE -> STAND_UP -> BALANCE_STAND -> LOCOMOTION sequence quoted above) -
  i.e. the fall may be happening DURING the sharp-decel phase itself, before
  the interlude's own lie-down code ever runs. This is a different code
  region than the three fixes above targeted, and it was not diagnosed
  further this session. **Do not assume the three fixes above are wasted** -
  each was independently verified against source/log evidence on its own
  terms - but they were evidently not the only thing wrong, and the failure
  point moving after each fix (PASSIVE hop -> STAND_UP interpolation ->
  BALANCE_STAND -> now apparently the TROT->STAND gait switch itself) is
  consistent with either a fourth distinct bug or genuine marginal
  instability in this specific stop-and-lie-down maneuver.

**Next step if resumed**: pull the full raw ctrl log via the new
`GET /api/logs/{i}?kind=ctrl&full=1` route (see below) around the exact tick
of `[GAIT] Transitioning gait from TROT to STAND` and look at body height/
velocity/foot contact state tick-by-tick through that transition, rather than
at the coarse ~1 Hz nav-line resolution used so far.

### RESOLVED (next session, same night): it was never the lie-down - it was the ARRIVAL

That "next step" was executed and found the real bug one level up. The raw
log showed the dog reaching the stop waypoint at **v=3.50 - full cruise**.
The speed profile brakes for corners and had NO concept of braking for a
stop; the stop sequence then slammed the stick 3.5->0 in 0.75 s (~4.7 m/s^2
demanded against a body that does ~1.2), the gait scheduler cut TROT->STAND
on the zeroed command while the body was still moving fast, the braking
pitch tripped `SafetyChecker::checkSafeOrientation` (a ZERO-debounce 28.6
deg ESTOP - one bad tick cuts the motors) within a tick, and the "fall
during lie-down" was really a fall during an unplanned crash-stop. The
unsteered brake-skid is also the "yaw'ed off centre right before the fall"
seen on camera on two dogs at once.

**The regression's history (found by git archaeology, per direct
instruction)**: the stop sequences were built and validated in the trotting
@ 2.0 m/s era (the 13/13 "settles + lies down" record). The campaign then
moved the loop recipes to trotRunning @ 3.5 - and nobody made the STOP
speed-aware. The loop got faster; the stop stayed tuned for 2.0. All three
prior interlude fixes (FSM path, edamp coverage, ramp shape) were real but
downstream - treating the landing while the approach was broken.

**Fix set (commit e1cdcb6), "stops are part of the plan":**
- `BodyPathPlanner::addStopXY()` / `setEndStop()` (`WP_END_BRAKE=0`
  disables): the path end is always a stop, and the loop-closure waypoint
  is a registered mid-path stop when a dash is armed. Both force v to
  v_min ahead of the existing backward pass, so the already-tuned
  braking-zone math corners use builds the deceleration into the plan, and
  the forward pass re-accelerates out of a mid-path stop automatically -
  which is exactly what the dash sprint needs.
- `WaypointNav::makeStar` now closes the loop itself (0->1->2->3->4->0).
  The closing stroke previously existed only as a side effect of
  appendDash's return-to-wp00 insert, so a star with dash=0 stopped one
  visible leg short of the drawn (closed) plan.
- `appendDash` detects an already-closed course (last waypoint within 1 m
  of wp00 - star now, oval/atom always) and appends ONE sprint point along
  the closing tangent instead of a degenerate metre-long return leg.
- The interlude's decel ramp starts from the LIVE commanded speed, not
  cruise - the plan has already braked the dog to a creep by the time it
  fires, and ramping "vx down to zero" from there would first spike the
  command back up to 3.5.

**Verified live** (single dog, star @ 3.5 trotRunning, dash=0): all SIX
waypoints including the closing leg, planned braking visible in the log
(2.72 -> 2.29 -> 1.76 -> 1.19 m/s into the final waypoint), NO orientation
trip, roll/pitch 2 deg at the end. The flown trace is a clean closed
pentagram. Path is 107.4 m now (was 88.1 without the closing leg) - the
69.4 s wall time is NOT comparable to the old 5-leg headline numbers.

**Also tried and REVERTED (do not retry)**: disabling
`checkSafeOrientation` in LOCOMOTION/BALANCE_STAND to stop the trip.
Symptom-level, made things worse (falls at 9 s mid-course), and the actual
fix was removing the crash-stop that caused the pitch spike.

### RESOLVED (same night): item 2 was the detector shooting the dog for obeying orders

The z-drift theory above is DEAD, killed by polling Gazebo TRUTH z live
through the stop window: the dog arrived at the closure at creep, stood
level at true z=0.30, began the COMMANDED lie-down, descended under
control (truth 0.30 -> 0.19 -> 0.11), and the fall detector's z-collapse
branch (estimate 0.094 < 0.10 held 0.5 s - the estimate within 2 cm of
truth) called the intentional crouch a collapse and `_exit()`d the process
mid-lie-down. A commanded lie-down and the collapse that branch exists to
catch are the SAME signature: a level body descending through 0.10 m. The
process exit is also the whole reason "the dog never stands back up".
Fix: `setFallZEnable()` gates ONLY the z branch; the mission suspends it
around its own lie-downs and re-arms it once standing again; the ATTITUDE
branch stays armed throughout (a lie-down is level by definition, so a
genuine tip during one still trips). Commit 9cd5b8b.

### FULL-SEQUENCE RESULTS (2026-08-24, single dog at a time, dash=100)

loop -> stop at closure -> lie down -> stand back up -> 100 m dash ->
planned end braking -> settle -> final lie down -> judged:

| course | result | detail |
|---|---|---|
| star @3.5 trotRunning | **PASS 7/7** | truth-polled end to end: lie-down z 0.117, back up, dash peak 3.64 m/s, settle 0.288 level, laydown 0.027. First judged PASS on the full sequence in this project's history |
| oval @3.5 trotRunning+analyzer | **PASS 95/95** (1 of 3 tries) | all 4 pre-planned gait changes fired; one mid-course fall (marginal, loop health clean), one interlude tip (roll=72 in BALANCE_STAND ~2 s after a clean level arrival at v=0.90) - the interlude is MARGINAL on this course, cause not isolated |
| atom @2.1 trotting | **PASS 109/109** | clean first try, laydown z=0.040 |

Every mission also proved the dash geometry: the sprint continues the
course's closing tangent (star 3.64 m/s peak; atom ran its 100 m due
north to N=103.6).

### RESOLVED: the hairpin was the FOLLOWER's arc model, not the planner

The first-corner trace from a quiet run showed the profile doing its job
(arrival braked to ~1.0 m/s, ~0.04 m/s planned at the vertex) and the
pigtail coming from pure pursuit itself: the lookahead's 0.35 m floor
reaches past a fillet only ~0.1 m long, so at a 162 deg vertex the
steering target lands on the exit leg nearly BEHIND the body's nose plane
(ex < 0) - where the arc-through-the-target model is degenerate. Arcing
toward a point behind you while creeping forward traces exactly the
measured loop. Fix (commit ab36f0b), per the operator's own prescription
("slow and almost yaw in place"): when ex < 0 the follower PIVOTS -
yaw_rate_max toward the target's side, speed floored to min(vplan, v_min)
- until the target comes back in front. Verified 2/2 (68.8/68.9 s, 0.1 s
spread): pivot at v=0.25/w=1.20, heading 0->160 deg in ~2.5 s, body
confined to ~0.4 x 0.6 m at the vertex, clean accelerate-out. The branch
only fires when the target is genuinely behind the nose, which gentle
curvature never produces - and it doubles as recovery from a large
tracking upset (pivot back onto the path at a creep, not an arc at
cruise).

**Full-sequence re-verification ON THE PIVOT BUILD, one dog at a time,
dash=100: star PASS 7/7, oval PASS 95/95, atom PASS 109/109.**

### RESOLVED: the SECOND stop-window killer - MIT's zero-debounce orientation ESTOP

Two separate mechanisms police a stop, and only one had been made
mode-aware. This port's fall detector got its z-gate (9cd5b8b); MIT's
stock `SafetyChecker::checkSafeOrientation` - zero debounce, 28.6 deg,
every 2 ms, force-ESTOP to PASSIVE (motors cut) on a single bad sample -
stayed armed through the stop sequence's BALANCE_STAND. A transient
wobble tick during the settle cut the motors mid-crouch, CAUSING the
fall it polices, and the stop sequence (unaware the FSM was yanked to
PASSIVE under it) drove a dead robot and hung the mission at "running"
for three hours. Fixed per direct instruction ("IF this mode, and not
commanding forward, AFTER a stop command - don't trip"):
`setOrientTripEnable()` gates the ESTOP off from the stop command until
standing-and-driving again; inside the window the DEBOUNCED detector
(50 deg / 0.5 s) is the arbiter, so genuine tips still end runs; cruise
is untouched. Verified on the failing cell (oval, 3 reps): PASS, PASS,
and one genuine 88-deg tip caught by the debounced detector with zero
ESTOP lines - runs now end only when the dog really goes over.

**Still open, in priority order:**
1. **The oval's MID-COURSE entry to its first sustained 180 is marginal**
   (~1 in 3): identical signature twice in one morning - wp40, v=2.60,
   w=0.96 (a_lat right at the 2.5 budget), orientation trip or
   level-sink, both times at the same coordinates. This is SEPARATE from
   the (now stop-gait-held) closure stop. The campaign-era "VSUS 2.6 ->
   6/6" tuning was measured on an older build lineage; the current stack
   (steering cap, pivot follower, stop braking) has not had its oval VSUS
   re-swept. Re-sweep 2.4/2.5/2.6 with equal N before trusting 2.6 again.
### RESOLVED: the oval mid-course bisect - NOTHING regressed, the envelope moved

The controlled A/B settled it: last night's exact binary (c80f7e8, built
fresh in a worktree, deployed with the three code-signing defenses) falls
mid-180 on the SOLO oval at 3-of-4 (wp85/86), statistically identical to
HEAD (2-of-4, wp45/47) - semi-interleaved blocks, all logs archived. So
none of today's commits broke the oval: the sustained-turn cell at
VSUS 2.6 has been ~50% marginal on this whole lineage, and last night's
"working" loops were two lucky fleet samples. The failing mechanism
matches the project's documented sustained-turn wall exactly: the yaw
command pinned at the lateral-budget cap (w = a_lat/v = 0.96) while the
body rides the 180 a metre and a half wide.

**Fix: the oval recipe's sustained cap is re-swept to WP_VSUS=2.4.** Its
first run cleared the curves AND ran the complete stop/lie-down/stand-up/
dash sequence, witnessed live by the operator ("what ever you JUST did
save it"). Interleaved 2.4-vs-2.6 tally continuing; per-slot `extra`
overrides (86cd71e) are what made the sweep possible without touching
RECIPES mid-test. The campaign-era "VSUS 2.6 -> 6/6, 30.48 s" row now
reads as history, not spec: re-measure a course's envelope after any
change to the planner/follower lineage before trusting its old tuning.

**FINAL TALLY (end of the 2026-08-24 session), all on the shipping build:**
- VSUS 2.4 mid-course: **9/9 curve-clean** (3 sweep + 3 settle-off + 3
  final), vs ~50% at 2.6 across both bisect binaries. The curves are
  fixed.
- Trot-in-place settle: MEASURED HARMFUL and reverted - 7-of-8 stop
  arrivals tipped (~88 deg, near-identical) with the 1.2 s hold, vs
  1-in-3 stock. The full verdict lives as a comment at
  `ConvexMPCLocomotion::zeroVelHold`. A plausible mechanism endorsed by
  everyone (operator included) still loses to an interleaved A/B.
- Star regression guard on the final build: **PASS x2**. Atom: **PASS**.
- ~~REMAINING OPEN, the oval STOP~~ **FIXED (cc7650c): STEERED
  DECELERATION.** The diagnosis sharpened once arrival SPEED was ruled
  out (the star's stop passes arriving at 1.2 m/s; the oval tipped at
  0.9): makeOval closes its lap 1.2 m off the exit of a continuous R=5
  arc, so the stop sequence zeroed the yaw stick while the body still
  carried the 180's residual yaw/roll - the dog stopped
  mid-straightening. Both stop sequences now keep the follower's
  steering live through the first 0.5 s of the decel (re-reading
  GPS/estimator per ramp step), zeroing yaw only once genuinely slow; a
  straight approach already steers ~0 so other courses are no-ops by
  construction. Measured: oval 4/4 full-sequence PASS - the FIRST
  4-for-4 end-to-end oval in this project's history - star guard 2/2 on
  the same binary.
### THE FULL FLEET COMPLETED - 2026-08-24 14:52, tagged fleet-complete-20260824

**All three dogs, SIMULTANEOUSLY, ran their complete sequence - loop,
stop, lie down, stand back up, 100 m dash - and every one finished.**

    dog1  oval:40:5.0  @3.5 trotRunning  COMPLETE t= 83.8 s  (judged PASS,
                       held gait switch fired "9 -> 5 while standing")
    dog0  star:10.514:5 @3.5 trotRunning COMPLETE t=114.6 s
    dog2  atom:9.0:6   @2.1 trotting     COMPLETE t=118.2 s

Conditions: quiet host (load ~1.7 at launch - the gz-teardown-on-done fix
had just removed the idle-engine load leak), cameras off, single branch
master at 6e94c74, every change committed and pushed BEFORE the run so
the exact producing state is reproducible. Loop health across all three:
max period 2.51-2.52 ms, ZERO samples over 4 ms in ~135 per dog - the
cleanest parallel run ever recorded here, and the final confirmation that
every earlier simultaneous wipeout was the host, not the controller.

Known nit, deliberately NOT changed (operator: "save this! don't change
it right now!"): the new teardown-on-done fires ~1 s after the LAST dog's
"MISSION COMPLETE", which truncates the final settle/judge printout of
the later-finishing dogs (their missions and dashes are complete; only
the [mission] RESULT bookkeeping line is cut). dog1 finished early enough
to get its full judged PASS. Queued fix for LATER: key the poller's done
detection on "[mission] RESULT" / [FALL] instead of "MISSION COMPLETE".

### BASELINE, END OF 2026-08-24 (build 308ef89/f51a433, single branch master)

Solo, one dog at a time, dash=100, cameras off, quiet host - the
CURRENT VALIDATED BASELINE:

| course | full sequence (loop -> lie down -> stand -> 100 m dash -> lie down) |
|---|---|
| star @3.5 trotRunning | PASS (and guard PASSes all day on every build) |
| oval @3.5 trotRunning, VSUS 2.4 | PASS, held gait switch verified firing ("9 -> 5 while standing"); stop at 5/6 (~83%) |
| atom @2.1 trotting | PASS |

SIMULTANEOUS (all three at once, cameras off): the loops, interludes and
dash STARTS all work in parallel - but on the current Mac the fleet's
SPRINT phase is host-stall-limited: in the final quiet-host attempt the
star and atom completed perfect loops+interludes and then both tripped at
the SAME wall-second (14:38:09) mid-dash, each with a single ~16 ms
control-loop stall (vs 2 ms budget; dog1's loop stayed at 2.70 ms max). A
16 ms stall applies sprint forces 8x too long. Identical simultaneous
failure across independent processes = the HOST, never the controller
(the rule earns its keep again). The one fleet arm that ever fails alone
is the oval's known 1-in-6 stop tip. VERDICT: solo baseline is green;
3-dog-with-dashes needs either a quieter host, a reboot (operator's
call), or RTF-tolerant pacing - it is NOT a robot-code defect.

OPERATIONAL TRAP FOUND TODAY, twice: the conductor leaves gz sim ALIVE
after "done", idling at ~a full core simulating an empty world. That one
leak (a) kept ambient load at 3.8-4.3 and framed the operator's web
browsing, (b) poisoned a five-run batch when a new launch stacked a second
physics engine on it (upside-down dogs, below-floor estimates - looked
exactly like a code regression). Batches now gate on stragglers+load and
kill gz after each run; a server-side fix (stop gz on done) is the queued
proper cure.

### EVENING SESSION TALLY (2026-08-24, build f90df0e)

The run-in experiment: REVERTED by its own A/B (0/8 vs the original
geometry's 5/6 - held-switch and clean loops verified in the failing
arm, mechanism unidentified, verdict at the knob in WaypointNav.cpp).
Two real fixes shipped instead, both operator-log-driven:
- the poller marks a dog done at its JUDGE line, not MISSION COMPLETE
  (the teardown was killing single-dog runs mid-lie-down and framed two
  perfect 114.2 s star guards as failures);
- the stop window's ESTOP suspension opens on the BRAKING APPROACH
  (the near_stop radius), closing the arrival-wobble race that cut
  motors mid-stop and left an un-detectable 30-45 deg zombie.

Final-build verification: star PASS/PASS (arrival race closed), oval
PASS + one known-cell mid-course fall (TM still enabled during the
window). Standing oval residuals: stop tip ~1-in-5 on the restored
geometry, sustained-entry ~1-in-5 at VSUS 2.4 - both real, both
documented, neither a regression.

### FLEET AFTER THE REBOOT (2026-08-24 evening, build 45c0596) - and two retractions

Post-reboot, on a quiet host, 6 three-dog fleet runs with the 100 m dash:

    star   4/6      oval   4/6      atom   0/6   (atom passes SOLO)

**THE ATOM IS THE FLEET-FRAGILE COURSE, and it is not the launch slot.**
Decisive swap test: atom moved to slot 0 (first away, 4 s delay) STILL
fell; star moved to slot 2 (last, 14 s delay) PASSED. So the ramp stagger
is exonerated, and so is start order - the atom simply has no margin in
company. That is consistent with what this file already says about it:
"no corner tighter than R=2.14 m, and NO RECOVERY ANYWHERE", roll-limited
rather than height-limited. star and oval have long straights to recover
on; the atom never stops asking.

**RETRACTION 1 - the shared-simulator stall was WRONG.** The theory (gz
falls behind, forces integrate over a longer step, all dogs hit at once)
was reasonable and is refuted by its own measurement: real_time_factor
over a full fleet run is p5=0.996, mean=1.000, and the fraction of
samples below 0.9 is 0.0%. The sim keeps up. (An earlier "min 0.442" came
from a parser grabbing the wrong protobuf fields - 138k samples off a
5 Hz topic should have been the tell. Fixed to parse real_time_factor.)

**RETRACTION 2 WAS ITSELF WRONG - un-retracted, with data.** I wrote that
"the post-reboot fleet runs logged ZERO excursions past 28.6 deg" after
checking TWO clean runs and generalising to all of them. Counting every
post-reboot fleet log instead: **7 dog-runs tripped the orientation
ESTOP**, and the instrumentation this commit added says exactly what they
were. The small-sample error, made a fourth time in one day, in the
direction of retracting a CORRECT hypothesis.

What the instrumented trip actually measures:

    RECOVERED (rode it out, debounce saved them):
      49.8 deg for 36 ms      39.6 deg for 24 ms
      31.5 deg for 12 ms      30.3 deg for 22 ms
    TRIPPED (all seven, every one at the 60 ms ceiling):
      peak 30.0 / 30.1 / 30.6 / 31.4 / 32.3 / 35.9 / 36.6 deg, held 62 ms

Two things follow. First, **the debounce is doing real work**: under stock
MIT every one of those four recoveries - including a body that hit
49.8 deg and came back in 36 ms - would have been an instant ESTOP, motors
cut, dog dead. One star run recovered FOUR excursions before tripping on
the fifth. Second, **60 ms may still be too short**: the trips fire at
peaks of only 30-37 deg, well under an excursion the same robot is
measured recovering from, and the post-trip [FALL] state differs from the
trip state (e.g. trip at roll=29/pitch=37, final roll=40/pitch=10) - the
signature of a dog flopping after its motors were cut, not of a dog that
was already lost. $CTRL_ORIENT_HOLD_MS is env-tunable precisely so this
is a measurement rather than an argument; a 200 ms arm is being run.

What survives: falls no longer share a timestamp (the stagger did break
the synchronized ramp-top), the star and oval are genuinely mid-pack
reliable in a fleet, and the remaining fleet failures are ordinary
per-course marginals - the atom's continuous curvature above all.

### THE ORIENTATION-HOLD A/B (2026-08-24 late) - promising, NOT yet causal

$CTRL_ORIENT_HOLD_MS, same build, same host, same slot order, 3-dog
fleets with the 100 m dash:

    hold = 200 ms   run1 PASS/PASS/PASS   run2 PASS/PASS/PASS   = 6/6
                    orientation events logged: ZERO
    hold =  60 ms   run1 INCOMPLETE/PASS/FALL                   = 1/3
                    trips: peak 49.8 deg and 40.5 deg, both held 62 ms

Two complete fleet sweeps at 200 ms, including the atom, which was 0/6
across the preceding six fleet runs. That is the best fleet result since
the milestone tag.

**But the causal claim does NOT hold up yet, and the reason is subtle:**
the winning arm logged ZERO excursions past 28.6 deg, so its 200 ms hold
was never exercised. A hold value can only matter in a run where an
excursion happens; if none does, the two arms are running identical
controllers. So what the data actually shows is that the 200 ms runs were
DYNAMICALLY CLEANER, not that the longer hold rescued anything - and the
60 ms arm's two trips were at 49.8 and 40.5 deg SUSTAINED past 62 ms,
which look like genuine losses the ESTOP was right to catch (compare the
recovered transients: 49.8 deg but only 36 ms).

Also explained by this instrumentation: the "stuck dog" mode. dog0 in
hold60_1 sat at wp6/7 for 160 s, commanded v=0.68 and not translating,
never falling and never finishing - it had ESTOPed to PASSIVE at t~60 s
and could not move. A trip mid-mission does not always produce a [FALL];
sometimes it produces a zombie that burns the whole timeout.

**THE INTERLEAVED A/B WAS RUN, AND IT REFUTES THE HOLD.** Three pairs,
arms alternated so host drift hits both equally, load recorded per run:

    hold 200 ms   (P,F,P) (P,F,P) (P,F,F)   = 5/9 dogs
    hold  60 ms   (P,P,F) (I,F,F) (F,P,F)   = 3/9 dogs

Fisher p ~ 0.6 - noise. The blocked 6/6-vs-1/6 was the confound, not the
knob: the 60 ms block simply ran later on a busier host. The debounce
stays (its four measured saves are real, and a zero-tick ESTOP is still
wrong for this port) but it is NOT the fleet's problem and tuning it
further is not the lever. **Fifth small-sample lesson of the day, and the
first one caught BEFORE it was published as a result.**

What the same 18 dog-runs do show:
- **One genuine host stall, caught in the act**: il_p2_h200 has all three
  dogs at 13.3-16.7 ms worst loop in the same run (over-4 ms on every
  dog) - the real signature, and nothing like the clean 2.5-3.5 ms of the
  runs around it. That is what a shared-host event looks like when it
  actually happens, and it is rare.
- **The atom has a REPRODUCIBLE failure at t=11.5 s**, 3 for 3 in one
  arm, always at wp14-16 (the first lobe, N~1, E~0-2), always a pitch
  excursion of 30.5-35.7 deg held past 62 ms, always ending level
  (roll 7-14, pitch ~0) rather than tipped. Same waypoint, same time,
  same shape - that is a deterministic dynamics problem at the atom's
  first lobe entry, NOT variance, and it is the single most tractable
  open item on the board. Reproduce it SOLO first (it may not even need a
  fleet), then instrument the lobe entry.

### HOST-STALL DETECTION AND GRACEFUL DEGRADATION (operator-specified)

The controller now notices when the HOST is failing it and degrades on
purpose instead of sprinting into the stall. Detector lives in
RobotRunner::run() beside the fall detector, response lives in the
mission thread.

**Why this is the right layer.** The control loop is wall-clock timed: it
computes forces for a 2 ms step and the sim integrates them over however
long the tick really took, so a 16 ms tick is an 8x impulse and a 64 ms
tick is a 32x one. Nothing else can see it - the bridge's command rate
and the estimator both look fine either side. But the loop can time
ITSELF, and over-4 ms ticks are essentially absent from healthy runs here
(over4=0 of ~140), so a single overrun is a sensitive, cheap trigger.
"Pre-emptive" honestly means: the first overrun latches, and the mission
stops being VULNERABLE to whatever comes next.

Response, in the operator's own words ("pause walking, trot in place, and
lay down, then stand back up when ready"):
  1. stick to zero at once - stop translating, gait keeps stepping in
     place under zeroVelHold rather than freezing mid-stride;
  2. if it persists past $SIM_STALL_LIEDOWN_S (3 s), lie down under
     control - the mission's own PASSIVE-hop -> STAND_UP-low -> damped
     hold, with both detectors suspended so the deliberate crouch is not
     read as a collapse;
  3. when the host is clean again (500 clean ticks = 1 s), stand back up
     through the staged entry and resume from the same waypoint, with the
     speed ramp re-armed so it eases back to cruise.

**VERIFIED BY INJECTION, not by waiting for luck.** A CPU storm (8
spinners) did NOT perturb the loop at all - max period 2.51 ms, over4=0 -
which is itself worth knowing about this machine. A precise
SIGSTOP/SIGCONT of 60 ms did:

    [STALL] control period 64.5 ms (limit 4.0) - host stalled, mission entering safe hold
    [nav] HOST STALL at t=5.7s (wp0/6) - pausing, holding position
    [STALL] clear after 500 clean ticks - safe to resume
    [nav] resuming mission at t=6.7s (wp0/6)
    [mission] RESULT: PASS  (waypoints 6/6, settle ok, laydown ok)

A 64.5 ms stall is a 32x force impulse and was previously an unsurvivable
event; the dog paused, held, resumed and completed the mission. Knobs:
$SIM_STALL_MS (4.0), $SIM_STALL_CLEAR (500), $SIM_STALL_LIEDOWN_S (3),
$SIM_STALL_DETECT=0 to disable.

**Scope, honestly**: this proves the mechanism end to end on an injected
stall. It does NOT yet prove it rescues the fleet - real fleet stalls are
rare (one in 18 dog-runs) and the remaining fleet failures are per-course
marginals, not stalls. It also does nothing about a stall that hits
DURING the lie-down itself.

**On the "one binary for N dogs" question**: worth doing for a different
reason than scheduling. Separate processes are what make the current
3-dog fleet honest - independent control loops, independent estimators,
no shared failure inside the controller. One binary could schedule its N
control loops against each other and guarantee they never contend, but it
would also put N dogs behind one crash and one GIL-free-but-shared
address space. The real contention here is gz's single physics thread,
which merging controllers does not touch.

### SESSION 2026-08-25 (early hours): five real bugs, and two of my own making

**1. THE INTERLUDE BUG - re-entering STAND_UP from STAND_UP skips the ramp.**
The end-of-mission lie-down has always passed; the dash interlude never
did. The ONLY structural difference is that end-of-mission never stands
back UP. FSM_State_StandUp interpolates on `progress = 2*iter*dt` capped
at 1.0, with `_ini_foot_pos` captured in onEnter() ONLY, and
checkTransition() increments iter every tick. After 2.5 s crouched, iter
is ~1250 and progress is pinned at 1.0 - so raising g_standUpHeight
0.15 -> 0.25 and re-requesting K_STAND_UP does NOT re-enter the state and
does NOT interpolate: pDes[2] jumps the full 10 cm in ONE tick against
kpCartesian=500. A launch, not a stand. Fix: hop through K_PASSIVE first
so onEnter() re-captures the crouched feet and restarts the ramp -
exactly what mission start does. This is why the dog "fell right after
lying down before the dash", every time, for days.

**2. THE DASH COURSE COULD NOT LAUNCH AT ALL.** The panel spells it
`outback:100`; its RECIPES key is `dash`; mission_kind() returned
"outback", the lookup missed and launch() died on recipe["note"] with a
KeyError visible only in server stdout. Aliased. Found only because a
verification harness reported the dash as PASS with numbers BYTE-IDENTICAL
to the atom run before it - the launch failed, no new log was written, and
the reporter read the PREVIOUS run's file. The harness now deletes
ctrl_*.log before every run. A harness that can emit a false PASS is worse
than no harness.

**3. THE PIVOT BRANCH FIRED AT FULL CRUISE** (dash, once launchable):
`PIVOT fired: vplan=3.00 ex=-1.73`, snapping 3.00 -> v_min with full yaw
against 3 m/s of momentum, roll 52 / pitch 69. Gated to
`vplan <= 4*v_min`. And the reason the profile never braked: **a 180
reversal is three COLLINEAR points**, so the 3-point curvature estimate
reads kappa ~ 0 and the turnaround looks like a straight. Curvature cannot
express "reverse direction here" - any waypoint with >150 deg direction
change is now registered as a planner stop.

**4. THE ATOM'S FAILURE IS PITCH, AND THE a_lon DEFAULT IS BACKWARDS.**
Every atom trip measured pitch-dominant (30.5, 33.0, 35.6, 35.9, 36.6,
36.9 deg) with roll in the teens. A first fix lowering the LATERAL budget
(WP_ALAT=1.8) was therefore aimed at the wrong axis and the very next run
rejected it (pitch 36.9 again). Pitch is braking and driving, and plan()
picks a_lon from cruise speed:

    a_lon_max = (v_cruise >= 2.2) ? 0.4 : 1.5;

Star and oval cruise at 3.5 and get the gentle 0.4; the atom cruises at
2.1 and lands in the 1.5 branch - 3.75x more longitudinal demand than
either course that passes, against a body measured to track ~1.2 m/s^2,
on the ONLY course whose curvature varies continuously (braking and
driving the whole lap, never coasting). The rule assumes slow = safe to
brake hard, which is exactly backwards for a slow course that is always
turning. Atom recipe now carries WP_ALON=0.4. First reps: PASS t=124.0s
and PASS t=123.9s, trips=0 on a course that had been tripping every run.

**5. SPAWN POSE IS ILLEGAL, NOT COSMETIC (operator-spotted, OPEN).**
`q=0` is OUTSIDE the calf joint's range (-2.818 .. -0.888), so all four
calves spawn at an illegal angle, the legs splay straight down through the
floor and the dog excavates itself. Very likely also the source of the two
"STATE ESTIMATE WENT NON-FINITE" lines every dog prints at boot. Fix needs
legal folded joint angles at spawn (the documented crouch (0,-1.3,2.5)
abstract = URDF (0,+1.3,-2.5), both in range), NOT a higher spawn z, which
would just stand it on locked straight legs.

### MY OWN TWO, recorded because they cost the operator real time

**The stall MITIGATION was worse than the stall.** Asked to detect host
stalls and degrade gracefully, I built pause -> lie down -> stand up. A
single 4.8 ms tick - ordinary scheduler jitter on a machine sitting at
24% CPU - tripped it mid-dash, it zeroed a 3.00 m/s command in one tick,
and the dog flipped to roll=154. It destroyed a perfect run AND the
evidence: a mitigated run says nothing about what the stall would have
done. Per direct instruction the mitigation is GONE - detect and log only,
the dog is allowed to fall, and the data stays honest. Threshold also
4 -> 8 ms: genuine damaging stalls here are 13-17 ms.

**"Host load" was the wrong model.** The operator's machine never exceeds
~24% CPU. Small overruns are a thread missing its slot, not a saturated
host, and calling them "the host stalled" led directly to a trigger that
did real damage. Only the 13-17 ms events were ever genuine.

**RUN NUMBERS** (operator request): monotonic, persisted in
RUN_DIR/run_seq.txt across restarts, shown in the panel, stamped on every
orchestration log line, and passed to each controller as $SIM_RUN_ID so it
lands in that dog's own ctrl log. "Run 47's atom did X" now means the same
run to both parties.

### THE HOST-STALL CULPRIT, NAMED (operator-diagnosed): TIME MACHINE

Hourly backups on this Mac start around :38. Both same-wall-second
multi-dog kills sit inside backup windows - 14:38:09 EXACTLY at the :38
start, and 16:44:09 six minutes into the 16:38 backup - and the stall
shape matches perfectly: a backup I/O burst wedges the control loops for
16-18 ms against a 2 ms budget (a ~9x force impulse), which drops every
sprinting dog in the same instant with clean logs before and after. The
conductor now REFUSES to launch while `tmutil status` reports Running=1
(ddeedc7), with the operator commands in the refusal message; for a
guaranteed-clean session, `sudo tmutil disable` before testing and
`sudo tmutil enable` after (the launch gate cannot protect a mission from
a backup that STARTS mid-run). This very plausibly also explains the item
below, previously the last instrument-clean mystery.

- Also open: ONE unexplained mid-dash spin-out on the atom (roll 102 at
  steady-state 2.1, straight line, w=0.00 for 10 s prior) with EVERY
  instrument clean - loop max 3.07 ms / 0 over-4 ms, zero bridge stalls,
  zero pivot fires. Matches the campaign-era force-delivery-on-straights
  class, now in the dash-after-mission regime; cameras/GPU load was the
  one uncontrolled variable (they now default OFF, fail-dark).

1a. **superseded - kept for the method** (was: IN FLIGHT bisect note). The
   dash=0 A/B cleared the stop machinery (falls persist without any
   mid-path stop: 2 PASS / 2 mid-course falls). The live suspect is the
   PIVOT follower branch (ab36f0b) firing transiently mid-curve: a tick
   of ex<0 at cruise snaps the stick from 2.6 to v_min with full yaw
   while the body still carries 2.6 m/s of momentum - and the flown
   trace shows exactly that signature, a straight CHORD cut across the
   bottom 180 (operator-spotted). A throttled diagnostic print now sits
   in the branch ([follow] PIVOT fired, with vplan); if it fires at
   vplan~2.6 mid-curve the fix is gating the branch to planned-creep
   speeds (vplan <= v_min) - which is exactly the star hairpin's regime
   (plans ~0.04) and can never engage on the oval's 2.6 curves, so the
   star is untouched by construction. Also deployed, one sample so far
   (inconclusive): the trot-in-place settle - zeroVelHold 50 ms -> 1.2 s
   plus a 1.3 s pre-BALANCE_STAND wait in both stop sequences, per the
   operator's "standing trot helps balance automatically". Note its
   dash=0 end-stops went 2/2.

1b. **RETRACTED on more reps: the gait-hold did NOT cure the oval stop.**
   "2/2 passes since the hold" was written at N=2 and fell apart at N=5:
   the stop tally on the gait-hold build is 3 PASS / 2 sideways tips
   (roll 68 and 84 deg, both immediately after "loop complete") - the
   same ~40% tip rate as before the hold. The small-sample trap, again,
   committed into the record hours after re-reading the rule. The hold
   itself is KEPT (entering a stop mid-gait-switch is still wrong on
   principle and costs nothing), but the oval stop's instability has a
   different dominant cause. What distinguishes it from the star's clean
   stop, for the next session: the oval's closure sits ~10 m after the
   180-exit, so braking from the 2.6 sustained cap barely fits and the
   dog reaches the stop still carrying ~0.9 m/s (the star brakes down a
   20 m straight and arrives at a true creep); and the tips are pure
   ROLL with pitch ~0, pointing at lateral velocity / roll oscillation
   from the S-weave across the joint still live when TROT->STAND fires.
   Next diagnostic: a failing rep with SIM_YAWDBG/STM32MP1_EST_DBG in
   the oval recipe's extra (temporary harness edit - RESTORE IT) to see
   roll/vy through the stop window tick by tick.
2. **Spawn pose: the dog's legs are below z=0 pre-stand** (user-observed).
   Spawn is z=0.08 belly-down but joints spawn at q=0, so the legs pierce
   the ground plane until the initial fold - likely also why every run
   logs 1-2 startup "STATE ESTIMATE WENT NON-FINITE" blips (Gazebo truth
   shows the body popped to z=0.097 by the first sample). Investigated:
   TWO spawn conventions already coexist - `make_world.py` drops the base
   from z=0.45 to settle onto its legs, while the speedway proto the
   conductor clones (`clone_dog(..., height=0.08)`) uses the DELIBERATE
   belly-down start documented elsewhere in this file ("the real Go1
   procedure... makes each SITL run start from an identical settled
   pose"). Every validated result in this file shares the 0.08 start and
   its startup blips, so changing it re-baselines everything - an operator
   decision, not a bug fix: either accept the cosmetic clip, or move the
   fleet proto to the 0.45 settle-drop and re-validate the boot sequence.
3. **Host load kills fleets, in matching pairs/triples**: all three dogs'
   control loops blew from a clean 2.48 ms to 20-24 ms at the same
   wall-second (a 10x force impulse - dog1 genuinely rolled 53 deg) and
   every dog "fell" simultaneously. Identical simultaneous failure across
   independent processes = the HOST, never the controller. Cameras were
   the GPU load (49% -> 0% with them off; they are now dynamically
   mutable from the panel). Run one dog at a time when measuring.

### New debug route: raw logs over REST

`GET /api/logs/{i}?kind=ctrl|bridge&tail=N|full=1` - the full text
`mit_ctrl_sim`/the bridge wrote for dog `i`, not the curated one-line-per-event
feed `/api/state`'s `log` array already carries. Added specifically because
diagnosing the interlude fall above needed exact tick-level text that the
curated feed does not carry, and pulling it by hand meant SSH-ing into a
scratch directory instead of one `curl`.

## The standalone dash: it was never a reversal, and never needed one (2026-08-25)

The panel's "dash" mission was wired to `WaypointNav::makeOutAndBack` -
100 m out and 100 m BACK, a 180-degree reversal at the far end. A whole
line of work (pivot-follower ramping, reversal-as-planner-stop) went into
making that reversal survivable, on the unquestioned assumption that a
"100 m dash" mission necessarily contains one. Per direct correction ("wtf
do you mean 'turned around' and 'target' it should be the final
waypoint!"), it should never have had a reversal at all: a dash is ONE
straight leg, ending at its own final waypoint. `WaypointNav::makeDash()`
now does exactly that (`stm32mp1/gazebo/WaypointNav.{hpp,cpp}`), wired
ahead of the `outback:` branch in `mit_sim_main.cpp`'s `navThread()` so
`dash:100` never falls through to the old course. The pivot-ramp and
reversal-registration code from the earlier line of work is KEPT - it is
correct for `outback:100` or any future course with a genuine reversal -
it is just no longer what "dash" means in the panel.

This is UNRELATED to (and does not replace) the existing "dash-as-finish"
mechanic (`appendDash()`, `$WP_DASH`, documented above under "The
Conductor"): that one tacks a straight sprint onto the END of a star/
oval/atom loop, continuing from wherever the loop closes, with the full
stop/lie-down/stand-back-up interlude in between. The panel's standalone
"dash" mission is the OTHER case - the sprint IS the whole mission, so it
needs no interlude at all, just the ordinary end-of-mission decelerate/
settle/lie-down every mission already does. `mit_sim_main.cpp` already
gates the interlude on `dash_pending` (true only when `$WP_DASH` is set to
append a finish onto a loop) - a bare `makeDash()` mission has `_n=1`, so
neither the mid-path stop nor the reversal-detection loop (which needs
>=3 collinear points to fire) ever engages. No pre-planner change was
needed for this half; it was already structurally correct once the
mission itself stopped being a reversal.

**Two more places silently assumed "dash" meant "outback", found chasing
the launch failures below - the same class of bug, twice more:**

1. `make_multi_world.py::mission_bbox()` had no `"dash"` case, so building
   a fleet world for `dash:100` returned `None` and the whole launch died
   with `world build FAILED: not a mission spec: 'dash:100'`. Added,
   mirroring `outback`'s bbox shape (both span 0..d north, 0 east).
2. `trail_daemon.py::mission_waypoints()` (imported into `server.py` to
   plan the drawn/flown overlay) also had no `"dash"` case - and its
   fallback is `raise SystemExit(f"unknown mission spec: {spec}")`, not a
   normal exception. **`SystemExit` is a `BaseException`, not an
   `Exception`, so the launch thread's `except Exception` never saw it -
   threading's bootstrap swallows an uncaught `SystemExit` in a non-main
   thread with NO traceback at all.** The launch thread just died,
   silently, after printing "wrote fleet.sdf" and before "starting Gazebo
   HEADLESS" - phase stuck at `launching` forever, no error logged
   anywhere, no gz/bridge/controller process ever started. This is a worse
   failure mode than a normal crash precisely because it produces NO
   evidence. Fixed two ways: added the missing `dash` case (mirrors
   `makeDash`: `[(d, 0.0)]`, one point, no return leg), and added a second
   `except SystemExit` clause in `server.py`'s `_run()` so this WHOLE
   CLASS of bug (any future helper that raises `SystemExit` on an
   unrecognised spec) surfaces as `phase="error"` with a note instead of
   wedging silently. `mission_viz.py`'s standalone copy of the same
   function got the same one-line fix for consistency, though it is not on
   the server's live path.

Verified: two consecutive solo `dash:100` runs, 30.5-30.6 s each, `settled
on its feet ... -> ok`, `lying down ... -> ok`, `mission result: PASS`.

## `mission_runner.py`: a proper launcher, replacing the ad-hoc `.sh` scripts

Per direct instruction ("stop using .sh scripts to launch the processes
and make yourself a proper python launcher that ends properly and reads
logs properly without just hanging indefinitely"). Lives in
`stm32mp1/gazebo/conductor/mission_runner.py`, talks ONLY to the
conductor's REST API - never touches gz/bridge/controller processes
directly, so it structurally cannot cause the "harness kills a legit run"
class of bug this file already documents.

```
python3 stm32mp1/gazebo/conductor/mission_runner.py --slot "dash:100" --timeout 90
python3 stm32mp1/gazebo/conductor/mission_runner.py \
  --slot "star:10.514:5" --slot "oval:40:5.0" --slot "atom:9.0:6" \
  --gait trotRunning --gait trotRunning --gait trotting \
  --speed 3.5 --speed 3.5 --speed 2.1 \
  --dash 100 --dash 100 --dash 100 --timeout 300
```

Two bounded timeouts, not one: `--timeout` (overall wall-clock budget) AND
`--stall-timeout` (abort if no NEW orchestration log line appears for N
seconds). The second is what actually caught the `SystemExit` bug above
live, on the first real run: the script printed the world-build lines,
then correctly declared `TIMEOUT: no new log line for 45s - run appears
wedged, stopping` and called `/api/stop` itself, instead of hanging the
terminal the way a fixed `sleep`-loop `.sh` script would have. That is the
whole point of a stall timeout distinct from an overall one - a wedge that
happens 5 seconds into launch and a wedge that happens 5 seconds before a
300 s budget expires should both be caught quickly, not just eventually.

Two more traps hit and fixed while writing it, both worth keeping in mind
for any future harness against this same server:

- **`/api/state`'s `log` array is a sliding window, not a growing log**
  (`self.log[-60:]` server-side). A naive "diff by length since last poll"
  either skips lines (window advanced past what was counted) or reprints
  stale ones once a run passes 60 orchestration lines - exactly the kind
  of subtle harness bug that produces a wrong verdict without ever
  raising an error. Fixed by tracking the last line actually printed and
  finding its most recent occurrence in each new window, printing only
  what comes after it (falling back to printing the whole window if that
  line aged out entirely, rather than silently dropping lines) - and every
  streamed line is also accumulated into a local buffer so the final
  PASS/FAIL count is never at the mercy of the last snapshot's 60-line cap
  either.
- **The verdict strings have to match the CURATED log wording, not the
  raw controller log's.** A first version matched `"[mission] RESULT:
  PASS"` - the string `server.py`'s `EVENT_PATTERNS` regex matches
  *inside* each dog's raw `ctrl_%d.log` - but that raw string never
  appears in `/api/state`'s `log` array, which instead carries
  `EVENT_PATTERNS`' own reformatted output, `"dog%d: mission result:
  %s"`. Guessing at the wrong layer's string reported a run that had
  actually printed `mission result: PASS` and settled cleanly as a FAIL,
  on the very first live test - a self-inflicted instance of the "harness
  reads the wrong thing and produces a false verdict" trap this file
  already warns about elsewhere, caught immediately by comparing the
  runner's own verdict against the raw log rather than trusting it blind.

## star/atom "failed" right after the GPS_HZ change - Time Machine, not a regression

Immediately after the GPS_HZ work, a star+atom regression check both
FAILED with what looked like stale-log contamination (the "failed" star
run's ctrl log was actually showing an earlier galloping dash's tail).
Chased it as a possible reintroduction of the async-teardown race (had
just done two manual server restarts testing GPS_HZ=50, each preceded by
a proper `/api/stop` per the documented safe procedure) - checked for
orphaned processes (none), confirmed the conductor was genuinely idle,
then tried a clean solo re-launch of star. It was REFUSED outright: `tmutil
status` showed a live backup in progress (`BackupPhase = Copying`, 39%).
This is this project's own already-documented, already-diagnosed host
hazard ("Hourly backups on this Mac start around :38... an I/O burst
wedges the control loops for 16-18ms against a 2ms budget") - the launch
gate that refuses new launches during one exists for exactly this
reason, and it correctly fired. The likely real explanation: the backup
started WHILE the earlier galloping/star/atom tests were already running
(the gate only blocks NEW launches, not one already in flight), and the
I/O storm's effect on an in-progress run - filesystem contention during
concurrent log writes, or an outright control-loop stall - produced the
garbled/interleaved log content, not a code regression in the teardown
fix or the GPS_HZ change. **Confirmed after the backup cleared**: star and atom both PASS cleanly
(94.6s each, matching their own established baselines exactly). The
GPS_HZ change (10 -> 20 Hz default, plus the per-launch override
mechanism) is NOT a regression - the earlier failure really was the
Time Machine I/O storm hitting an in-progress run, exactly as this
project's own launch gate already exists to warn about. `GPS_HZ` stays
shipped at its new 20 Hz default.

## A self-inflicted contamination: restarting the conductor SERVER unsafely bypasses its own teardown fix

Found immediately after fixing the async-teardown race (`_teardown_done`)
and applying it via a raw `kill $PID` + relaunch of `server.py` itself,
twice in one session (once for the fix, once for the oval recipe change).
That is exactly the unsafe pattern the fix exists to prevent, just one
level up: `_teardown_done`/`_reap_and_confirm` only run when the SERVER
PROCESS is alive to run them - a bare `kill` on the server itself is a
plain SIGTERM with no registered cleanup handler, so any gz/bridge/
controller child still active at that exact instant is immediately
orphaned with nobody left to reap it. The NEXT server instance starts
with `self.procs = []`, structurally blind to those orphans, and a
subsequent launch's own stale-process sweep (port/pgrep-based) does
eventually kill them - but not before a few of their last buffered log
lines can land in the freshly-truncated ctrl log of an unrelated later
run. Reproduced exactly this way: a solo trotting/circle:9:36 run showed
a `[FALL] roll=163` sandwiched inside ~140 repeated
`[FSM LOCOMOTION] On Enter` lines, then MORE fresh FSM initialization
after it - two runs' content interleaved in one file, not a real gait
failure. An immediate clean re-run of the identical config PASSED (23.6s,
roll 0.7/pitch 0.8) - the "failure" was purely an artifact of restarting
the server the wrong way. Lesson for any future restart to load a code
change: `curl -X POST :8420/api/stop` FIRST (routes through the real
`_reap_and_confirm`, confirms every child dead) and only then kill/
relaunch the server process - never kill the server itself while it
might own live children.

## THE OVAL'S MID-COURSE FALL: not the gait switch at all - trotRunning itself can't hold this curve

Challenged directly on treating this as an acceptable ~1-in-3 residual
rather than actually fixing it, given how well atom/spirograph/lissajous
(all continuous, non-trivial curvature) run. Fair challenge - investigated
for real rather than re-citing the old note.

**Three targeted fixes to the pre-planned gait switch, each tested against
the exact failing case, each measured to NOT fix it:**
1. Defer the switch while roll/yaw-rate are elevated (a genuine mid-turn-
   destabilization hypothesis) - switch fired with `calm=true` logged, no
   forced/safety-valve suffix, and it still fell with the same signature.
2. Also require current speed within 0.3 m/s of the segment's own planned
   floor (`seg->v_plan_min`) before switching, since the raw log showed the
   switch firing while still decelerating through ~2.7-3.0 m/s - fired
   correctly gated this time (spd already near the cap), still fell, this
   time via a flat collapse (`roll=-5 z=0.041`) rather than a tip-over.
3. `WP_ANALYZER_LEAD=15` (vs. the 4 m default) to give trotting a long
   straight-line runway to settle onto before the curve even starts - fell
   at essentially the same wall-clock offset from nav taking the stick as
   every previous attempt.

**The decisive test, and the one that should have been run first**:
`WP_ANALYZER=0` - NO gait switch at all, trotRunning held for the entire
course. It STILL fell entering the exact same curve. The gait switch was
never the cause; every fix aimed at it was correcting a correlation, not
the mechanism. Confirmed further: even dropping the sustained-curve speed
cap to 1.8 m/s (WP_VSUS=1.8, no switch) did not save it either - two
orientation trips, the second one landing outside any stop-window
protection and latching the FSM into ESTOP (the already-documented "stuck
dog" zombie, motors cut, sits until the timeout). trotRunning genuinely
cannot hold R=4.47 m reliably on this course, independent of gait-
switch timing, dynamics-at-switch, or approach speed within the range
tested.

**The actual fix, found by testing the obvious alternative rather than
continuing to patch the switch**: trotting - the same all-rounder gait
that already handled every discrete-corner and smooth-circle cornering
test tonight up to 2.5 m/s cleanly - for the WHOLE course, no analyzer, no
switch. `oval:40:5.0` @ trotting 2.4 m/s, `WP_ANALYZER=0`: **PASS x2**
(45.9s, 46.1s), clean settles both times (roll/pitch under 1.2 degrees).
This is a config choice, not a code fix - the oval's own established
recipe (trotRunning on the straights, switching to trotting for the
curves, chasing extra straight-line speed) was the thing built on a false
premise: that trotRunning could handle this specific radius at all, given
enough care about WHEN to hand it off. It cannot, so there was never a
switch-timing fix to find.

**Honest trade-off, not hidden**: trotting-only is slower (2.4 m/s
throughout vs. trotRunning's 3.5 m/s on the straights) - roughly 46 s
against the mixed-gait approach's best-case ~30.5 s when that approach
happened to work. Reliable-but-slower vs. fast-but-genuinely-broken at
this exact radius.

**Follow-up, per "hammer at all the things open": re-checked reliability
with more reps, and it is real improvement, not perfect.** Four more
solo runs of the trotting-only config: 3/4 PASS, clean settles (roll
0.2-0.3 deg) - but one FELL, and NOT in the stop/settle window at all.
The archived log shows a genuine MID-COURSE collapse at wp43/93 (~46%
through the lap, t=22.4s), `roll=-36 pitch=9`, during the curved section
- a lateral/roll failure while cornering, not the pitch-forward stop
crash this whole investigation was originally chasing. So: trotting-only
is a real, substantial reliability win over the old trotRunning+analyzer
config (which failed essentially every time at the mid-course curve
entry) but is not 100% reliable either - roughly 75-80% across 6 total
reps tonight (2 earlier + 4 now), with a residual, different failure
mode (mid-course roll during the sustained curve, not a stop-window tip)
that the earlier "~1-in-5 stop tip" framing does not actually describe.
That older finding was measured on the NOW-REPLACED trotRunning+analyzer
config and should be considered superseded, not confirmed, by this
result - the mechanism is different, even if the rough failure rate
happens to land in a similar range. Not chased further tonight; a real,
open, honestly-scoped residual for whoever continues this course. The three switch-timing code changes
(`pending_gait_since`, the roll/yaw-rate + speed defer gate in
`mit_sim_main.cpp`) are left in place as reasonable general hardening
(deferring a gait change during a genuinely violent transient is still a
sound principle for OTHER courses/segments, and costs nothing when the
gate is rarely engaged) - but they should not be credited with fixing the
oval, because they did not. If oval's speed is worth chasing further, the
real next step is investigating WHY trotRunning cannot hold R=4.47 m at
even 1.8 m/s (WBIC gains, follower steering-cap math at this specific
radius - similar in spirit to the star hairpin fix, not yet attempted
here), not more gait-switch tuning.

## Smooth-circle coverage completed: trotting/walking clean, trotRunning genuinely fails (0/2), correcting an earlier mixed-up claim

Filling the last gap (trotting/trotRunning/walking on `circle:9:36`, the
smooth 36-vertex circle) hit a real contamination artifact first: a
3-gait batch showed all three "failing," but investigation traced it to
MY OWN unsafe restart of the conductor server minutes earlier (see the
section above) - a stale process's tail landed in a fresh run's log. A
clean re-test cleared trotting immediately (PASS 23.6s). But re-testing
trotRunning genuinely, solo, twice, gave **0/2 real falls** (tip-overs,
`roll=-75` and similar) - not contamination, not variance rescuing a
mistaken claim. Correcting the record: an earlier message in this session
claimed trotRunning had "already passed the smooth circle twice tonight
(46.7s, 62.7s)" - those numbers actually belong to bounding/galloping/
pronking's own passes on this same course, not trotRunning, which had
never been tested here before this investigation. There was no prior
pass to contradict.

**Final, corrected smooth-circle (`circle:9:36`) tally:**

| gait | speed | result |
|---|---|---|
| bounding | 1.0-2.0 | PASS at every speed tried (up to 2.0, no ceiling found) |
| galloping | 0.8-1.4 | PASS at every speed tried (up to 1.4, no ceiling found) |
| pronking | 0.6-1.0 | PASS at every speed tried (up to 1.0, no ceiling found) |
| trotting | 2.5 | PASS 23.6s |
| walking | 1.5 | PASS 33.8s |
| **trotRunning** | **3.5** | **FELL 0/2** (both solo, clean, genuine tip-overs) |

A genuinely interesting reversal of the discrete-corner pattern from
earlier tonight, where trotRunning was often the MOST robust gait
(clean on the octagon and every discrete angle tested). On sustained,
continuous curvature specifically, it is the one gait that struggles,
while the fully-synchronized/asymmetric flight gaits (bounding/galloping/
pronking) and the always-multi-support gaits (trotting/walking) all
handle it comfortably. Not root-caused - flagged as a real, open
observation for whoever next has time, and a caution against assuming a
gait's discrete-corner performance predicts its continuous-curvature
performance, or vice versa.

**Bracketed the ceiling**: PASS at 2.0 (25.6s), PASS at 2.75 (21.0s),
**FELL at 3.2**. So trotRunning's smooth-circle wall sits at 2.75 PASS /
3.2 FAIL - well below its flat-dash and discrete-corner ceilings
elsewhere in this file. `path_analysis.py`'s hairpin finding two sections
up (overshoot starting 2-3 corners before the actual fall) is the
leading, still-unconfirmed lead for WHY - worth checking whether the same
overshoot-then-correct signature appears, growing, as speed is pushed
from 2.75 toward 3.2, before assuming a totally different mechanism.

## `unittests/`: a repeatable regression suite over the validated missions, and a false positive found on its first real use

Per direct request for "a concise repeatable set of unit tests... so when
we need to you don't have to burn tokens thinking about regression tests."
`unittests/test_validated_missions.py` runs real Gazebo SITL missions
(there is no meaningful way to unit-test this controller without the sim)
through the conductor's own `mission_runner.py`, each case citing the exact
CLAUDE.md section its config comes from so a future failure can be checked
against real prior evidence instead of re-argued from scratch. `unittests/
run_history.py` is a ring-buffer archive (same capped/evict-oldest shape
this project already uses elsewhere - `TRAIL_MAX`, `server.py`'s own
`self.log[-60:]`) of every run: verdict, timing, and for a failure, both a
short signature (grepped from the runner's own output) and a reference to
the FULL per-tick forensic ShmTrace archive the conductor already writes
on every fall (`archive/shm_trace/*_FALL.json` - tens of MB, tens of
thousands of tick records) rather than duplicating that data into the
lightweight history file.

**Found a real, serious bug on the very first multi-case run**: launching
one case immediately after the previous one's `mission_runner.py` process
exits is NOT the same as waiting for the SERVER to finish tearing the
previous fleet down - `server.py`'s teardown (`terminate()` -> `sleep(1)`
-> `kill()` on gz/bridge/controller) runs on a background thread and can
still be in flight for a second-plus after the previous run already
reported its verdict and exited. `star` (real PASS) immediately followed
by `atom` returned a bogus PASS in 10.4 s - atom's real baseline is 55+ s -
because the new run's ctrl log picked up a few lines of star's own tail
before truncation caught up (the exact same class of bug as the earlier-
documented stale-tail-text-reaper contamination, just triggered by
machine-speed back-to-back launches rather than a slow teardown race). The
identical atom case, given a 6 s settle gap, passed correctly in a normal
~95 s. Fixed with a `SETTLE_S` sleep between every case/repeat in the
suite - a pragmatic mitigation in the TEST HARNESS, not a server.py fix,
but "a harness that can emit a false PASS is worse than no harness" is
this project's own already-learned lesson (see the standalone-dash mission
section above), so this was not optional to fix before trusting the suite.
The bogus 10.4 s entry is left in `unittests/history.json` rather than
scrubbed - it is genuine history, and the false-positive is fully
explained in this section and in the code's own comment.

**Per direct instruction ("fix this once and for all, no more hanging
bridges and races"), fixed at the ROOT instead of leaving the client-side
sleep as the only defense.** `Fleet.__init__` now carries
`self._teardown_done`, a `threading.Event` that is CLEARED the instant a
teardown starts and only SET once every process from that run is
CONFIRMED dead - a real `p.wait()` per process (with a bounded SIGKILL
fallback and a logged warning for a truly stuck pid), not "sent a signal
and moved on" the way both teardown sites (the natural "all dogs done"
path and `stop()`) used to. `launch()` now waits on this Event, outside
`self.lock` so it cannot block unrelated requests, before doing anything
else - a new launch is now structurally unable to start while a previous
teardown is still completing, regardless of how fast the caller fires the
next request. Verified directly, not just argued: star immediately
followed by atom with the client-side `SETTLE_S` set to **zero** (no
sleep at all between the two `mission_runner.py` calls) now gives atom a
real ~60 s completion (`t=60.4s` in the raw log, clean roll/pitch) instead
of the bogus 10.4 s the same sequence produced before this fix. The
unittests suite's own `SETTLE_S` is left at 0.0 with a comment explaining
why - the fix belongs in the server, and now it is there, not papered
over in every caller.

```bash
python3 unittests/test_validated_missions.py                    # everything
python3 unittests/test_validated_missions.py --only star atom   # by name
python3 unittests/test_validated_missions.py --list             # see the tribal knowledge without running anything
python3 unittests/test_validated_missions.py --history           # the ring-buffer archive
```

## GPS VELOCITY AIDING: implemented, tested, and it made things WORSE - a real frame-convention lead, not confirmed

> **READ THE `x_comp_integral` SECTION AT THE END OF THIS FILE BEFORE
> SPENDING ANY MORE TIME ON THIS ONE.** Everything below is factually
> accurate about GPS velocity aiding itself (three real bugs found and
> fixed, a real measured dose-response on GPS rate and sigma), but the
> PROBLEM it was chasing - "the robot's real velocity decays to zero and
> reverses on a long straight" - was NOT primarily an estimator problem.
> It is `x_comp_integral` windup in stock MIT's own `ConvexMPCLocomotion`
> actively commanding the robot backward at up to ~1x bodyweight. That is
> root-caused, fixed, and verified against Gazebo truth (100 m dash
> completes, 99.82 m actually travelled). So the "partial, real effect"
> results below - galloping's dash progress moving 3 m -> 14.5 -> 24.9 ->
> 54.5 m across the wiring fix, GPS rate and sigma tuning - are best read
> as this feature partially COMPENSATING for a control bug by feeding the
> estimator better data, not as evidence about how much aiding this robot
> needs. Re-measure the whole dose-response with `$CTRL_XDRAG_CLAMP` set
> before drawing any conclusion about GPS aiding's real value; the honest
> status of that question right now is UNKNOWN, not "partially effective".

Direct challenge, and a good one: "the controller thinks it's still
accelerating forward while it's actually drifted backward... we literally
have GPS and magnetometer available." Checked first whether this repeats
the already-known-harmful GPS/baro POSITION aiding (`$SIM_ABS_AIDING`,
documented above as net-harmful to locomotion) - it does not, structurally:
position aiding corrects a state the controller's own tracking cost never
reads (which is WHY correcting it destabilizes things - see the hierarchical-
estimation paper's own principle), while velocity feeds the Raibert
foothold formula and the MPC's velocity-tracking cost directly, and is NOT
covariance-suppressed the way position is (MIT's `_P.block(0,0,2,2)/=10`
hack only ever touches indices 0-1). So real GPS velocity aiding is a
genuinely different, better-motivated intervention, not a repeat of a known
failure - worth building and testing on its own merits.

**What was built.** `gps_vel[3]` (NED, Doppler-derived from Gazebo's real
NavSat sensor) was ALREADY fully wired end-to-end - the bridge reads it,
the UDP packet carries it, `gazebo_get_aux()` delivers it - and simply
never consumed ("not consumed by Cheetah yet" in `rt_gazebo.h`'s own
comment). Extended `AbsolutePositionAiding<T>` with `haveVel`/`velocity`/
`velSigma`, populated independently of `$SIM_ABS_AIDING` via a new
`$SIM_VEL_AIDING=1` flag in `Stm32mp1HardwareBridge.cpp`, and implemented
a real textbook sequential Kalman update on the velocity states (indices
3-5) in `PositionVelocityEstimator.cpp` - not the same "time-constant"
workaround position aiding needed, because velocity's covariance isn't
artificially crushed, so a real Kalman gain has genuine authority here.

**Tested against galloping's dash - the one case with a confirmed,
ground-truth-verified root cause - and it made things WORSE, not better.**
With `$SIM_VEL_AIDING=1` (galloping @ 0.5 m/s, dash:100, `$SIM_ESTERR=1`
to compare against Gazebo truth directly):

| t (s) | true north position | estimate (own frame) | true height |
|---|---|---|---|
| 19.9 | 0.009 m | 0.796 m | 0.279 m |
| 252.1 | **4.111 m** | **47.292 m** | 0.257 m |

The robot NEVER fell (no `[FALL]` line anywhere in the log, height stayed
a normal 0.257-0.307 m the entire 252 s) - but its TRUE position barely
moved at all (4.1 m in 252 s at a commanded 0.5 m/s, which should cover
~126 m), while its own velocity ESTIMATE ran away to a WORSE divergence
than the unaided baseline ever showed (47 m vs. the unaided case's ~29 m
frame-corrected drift over a comparable dash). So this is not a crash -
it is the robot standing its ground, upright, essentially failing to
translate, while genuinely believing it is moving briskly.

**Leading hypothesis, NOT yet confirmed**: a frame-convention mismatch.
The GPS velocity was injected using the same "x=East, y=North, z=up"
convention the existing position-aiding code assumes for the KF's own
`_xhat` states - but that convention was never actually verified for the
MIT-stack estimator, and this file's own `[ESTERR]` investigation earlier
tonight found the ESTIMATE lives in a frame rotated relative to Gazebo's
world ENU (documented under "the backward-walk investigation, continued").
If that rotation also affects the KF's internal velocity-state axes,
position aiding's identical assumption would have been WRONG THE WHOLE
TIME and simply never surfaced, because position aiding's own Kalman gain
is ~0 (masked by the covariance-suppression hack) - a wrong-frame
injection into a near-zero-gain update is a silent no-op. Velocity aiding
has REAL gain by design (that is the whole reason it was worth building),
so the same frame error, if present, is no longer silent - it actively
drags the state in the wrong direction instead.

**Honest status**: a well-motivated, correctly-differentiated idea (not
the same failure as position aiding, verified structurally), implemented
as real code with real gain, tested against the exact confirmed failure
case - and it made that failure case measurably worse. Left in the tree,
default OFF (`$SIM_VEL_AIDING`, `$SIM_GPS_VEL_SIGMA`), documented rather
than silently reverted.

**UPDATE, same night: the frame-mismatch hypothesis is now CONFIRMED, in
the actual source, not just inferred from symptoms.**
`VectorNavOrientationEstimator::run()`
(`common/src/Controllers/OrientationEstimator.cpp:44-83`) is NOT a raw
pass-through of true world orientation - on the very first tick
(`_b_first_visit`) it captures the initial roll/pitch/yaw, builds
`_ori_ini_inv` from `-rpy_ini` (roll/pitch zeroed, YAW KEPT), and applies
`orientation = _ori_ini_inv * raw_orientation` on every subsequent tick.
Every downstream quantity - `rBody`, `aWorld`, and therefore the KF's own
integrated `_xhat` position/velocity - is expressed relative to the
robot's OWN SPAWN YAW, not true world yaw. Verified directly with fresh
`$SIM_ESTERR=1` data on a north-spawning mission (`dash:100`, which
spawns at world yaw +90 deg so body-forward = world north): sustained
forward motion showed up as world truth's Y (north) climbing steadily
(`pT` index 1: 20.5 -> 20.9 m) while the KF's own estimate showed the
SAME motion in its OWN index 0 (`pE` index 0: 19.5 -> 19.9 m, index 1
flat) - a clean, exact confirmation that **estimator frame = world frame
rotated by -spawn_yaw**, not a fixed 90 degrees, not a guess.

This is precisely why my velocity-aiding injection (`_absAiding.velocity
<< east, north, up`, assuming the KF's x/y directly ARE world east/north)
was wrong FOR THIS SPAWN HEADING specifically, and why position aiding's
IDENTICAL assumption never surfaced as a bug before now - position
aiding's Kalman gain is near-zero (masked by MIT's covariance
suppression), so injecting a rotated-wrong vector into a near-inert
update was silently harmless; velocity aiding has real gain, so the same
wrong-axis injection actively corrupted the state instead.

**IMPLEMENTED, same night** - `Stm32mp1HardwareBridge::setSpawnYawRad()`
(new setter, default pi/2 matching the universal spawn convention),
called once from `mit_sim_main.cpp` with the TRUE world yaw derived from
`spawn_bearing_rad` (`world_yaw = pi/2 - spawn_bearing_rad` - compass
convention to Gazebo-yaw convention, verified: spawn_bearing_rad=0 for a
north-spawning mission gives world_yaw=90deg, matching the known
convention exactly). The velocity-aiding injection now rotates world
(east, north) GPS velocity into the KF's own frame:
`ex = ve*cos(yaw) + vn*sin(yaw), ey = -ve*sin(yaw) + vn*cos(yaw)` -
checked numerically against the one validated data point (yaw=90deg,
pure-north velocity must map to pure estimator-x) BEFORE trusting it in
code, which caught a real sign-transcription error in the first draft
(the cross-terms were backwards) before it ever ran.

**Re-tested against galloping's dash with the corrected rotation - IT DID
NOT FIX IT EITHER.** Same signature as the unrotated version: no fall
(height stayed normal the whole 252s+ run), but true north position
barely moved (`pT` index 1: 0.81 m at t=252.5s, should be ~125m) while the
KF's own estimate ran away just as far as before (`pE` index 0: 46.8).
**This is the honest, important negative result**: the frame-mismatch fix
is real, verified, and worth keeping (it is objectively more correct than
injecting a rotated-wrong vector, and may matter for other missions with
non-north spawn headings even if it does not fix galloping) - but it was
NOT the reason velocity aiding hurts this case. Something else about
injecting a real-time GPS velocity correction into the KF every tick is
disrupting the controller, independent of axis alignment - a plausible
but UNCONFIRMED lead is a rate mismatch (Gazebo's simulated GPS/NavSat
almost certainly updates slower than the 500 Hz control loop, so
`aux.gps_vel` may be a zero-order-held stale value between fixes, and a
Kalman correction applied every tick against a value that only genuinely
changes every N ticks could inject a small step discontinuity each time a
new GPS sample actually lands - structurally similar to the ALREADY-
documented reason position aiding hurts locomotion, "the controller needs
consistent RELATIVE motion... injecting [it] steps the tracking error and
the MPC fights it," just via a different channel). **Tested, same night**: added a staleness gate (`static float lastGpsVel[3]`,
comparing exact bit-for-bit equality since a genuinely zero-order-held
value between fixes IS bit-identical, not merely close) so the Kalman
correction only fires on the tick a NEW GPS sample actually lands, not
all ~49 stale ticks between real 10 Hz updates. Re-tested against
galloping's dash - **byte-for-byte identical result to the unfixed rate
gate**. That identical-across-genuinely-different-code-changes pattern
was the tell, and it led to the actual bug:

**THE REAL BUG, finally found**: `$SIM_AID_DBG=1` showed
`[AID-PTR] estimator sees absAiding=0x0` throughout - the pointer wiring
`_robotRunner->absAiding = &_absAiding` (`Stm32mp1HardwareBridge.cpp`,
right after `RobotRunner` construction) was gated on `SIM_ABS_AIDING`
ALONE. Every `$SIM_VEL_AIDING=1` test run tonight - the frame-mismatch
fix AND the rate-staleness gate on top of it - had this pointer sitting
null the entire time, so `PositionVelocityEstimator`'s
`if (aid && aid->haveVel)` never once evaluated true. All three "fixes"
were correct, real code, computing correct values into a struct the
estimator was never connected to read. Fixed by wiring the pointer on
EITHER flag. Confirmed with `$SIM_AID_DBG=1`: `absAiding` is now a real
non-null address and `[VELAID]` fires with a genuine, non-trivial Kalman
gain (`K=0.71`) - the mechanism has now, for the first time tonight,
actually run.

**Final result with everything actually connected**: galloping's dash
shows REAL, MEASURABLE improvement - true forward progress reached
14.5 m by t=252s (every prior attempt, mechanism disconnected, stalled
under 3 m) - but it is still far short of the ~125 m the commanded
0.5 m/s implies, the dash still does not complete, and the KF's own
POSITION estimate diverges even FURTHER than the unaided baseline
(`pE` index 0 reaches 50.4, worse than the ~29-47 m range seen in every
earlier variant). So velocity aiding, now genuinely active, measurably
HELPS the robot's real physical progress while still not solving the
underlying divergence and arguably worsening the position-side symptom -
a partial, real effect, not a full fix and not inert either.

**Honest final status for tonight**: three real, independent bugs found
and fixed in pursuit of this one feature (a frame-convention error, a
GPS-update-rate staleness gate, and - the one that actually mattered
tonight - a missing pointer wire that meant NOTHING before this point had
ever really been tested). The feature itself remains an open, partially-
effective lead: `$SIM_VEL_AIDING` now genuinely runs and genuinely
changes behavior, but does not resolve galloping's estimator divergence
on its own. Left default OFF. Next concrete step for whoever continues:
now that the mechanism is confirmed live, log `_xhat`'s own diagonal
covariance for the velocity states around each correction to see whether
the Kalman gain (K=0.71, quite large) is appropriate for the actual GPS
velocity noise, or whether `$SIM_GPS_VEL_SIGMA`'s default (0.1) is too
aggressive now that the correction is real rather than a no-op.

### GPS_HZ: the simulated GPS rate was a flat, uncited 10 Hz - fixed to a real spec, and it reveals a clean dose-response

Per direct instruction to "simulate a fast ublox... use the realistic
Hz rate from that" rather than leave the 10 Hz figure that CAUSED the
staleness bug above sitting unexamined. Checked: 10 Hz had no citation
anywhere in this codebase - it was simply what got typed the first time
a GPS sensor was added. Replaced with a real, commonly-used spec: the
u-blox ZED-F9P's documented standalone/DGNSS navigation rate is 20 Hz
(its 10 Hz figure applies to RTK-fixed mode, which this project has no
base station to model). Baked into `worlds/go1_speedway.sdf` (the actual
proto the conductor's fleets clone from - `make_world.py`'s own
GPS_HZ-driven default was fixed too, but that script is not what
generates the file the conductor uses) and made independently overridable
per-launch via `GPS_HZ` in the conductor SERVER's own environment
(`make_multi_world.py::clone_dog()` patches the cloned sensor's
`update_rate` if the env var is set) - "selectable... for regression /
alternate scenarios," exactly as asked, though the granularity is
per-server-restart, not per-mission, since the sensor rate is baked into
the world at BUILD time, before any per-mission `extra` env vars the
controller itself sees ever come into play.

**This directly answers "every gait should at least be able to do the
dash" - not by fixing it, but by finding a genuine, quantified,
positive-but-incomplete lever.** Re-tested galloping's dash
(`$SIM_VEL_AIDING=1`, now actually wired) at three GPS rates:

| GPS rate | real forward progress by ~230s | 
|---|---|
| disconnected (the wiring bug, effectively 0 corrections) | under 3 m |
| 10 Hz (the old default) | 14.5 m |
| 20 Hz (the new realistic default) | 24.9 m |
| 50 Hz (idealized, PAST any real GPS hardware - diagnostic only) | 37.2 m |

A clean, monotonic dose-response - correction FREQUENCY is a real,
measurable lever on this failure, not a null one. But even at 50 Hz,
deliberately beyond what any real GPS module offers, the dash still does
not complete (37 of 100 m) and the KF's own position estimate still
diverges. Honest reading: faster GPS genuinely helps and is worth
shipping at the real 20 Hz spec regardless, but rate alone will not
finish the job - the next lever is very likely `$SIM_GPS_VEL_SIGMA`
(the assumed measurement noise, still an unexamined default of 0.1)
or the raw Kalman gain magnitude itself, not more speed from a sensor
that is now already modeled realistically.

**Chased that exact lever, same night, and it plateaus rather than
solves it.** Tightening `$SIM_GPS_VEL_SIGMA` (trusting the GPS velocity
measurement more, raising the Kalman gain) at the realistic 20 Hz rate:

| `SIM_GPS_VEL_SIGMA` | real forward progress by ~230s |
|---|---|
| 0.1 (original default) | 24.9 m |
| 0.02 | **54.5 m** |
| 0.005 | 53.7 m - no further gain, within noise of 0.02 |

Real, substantial improvement (roughly DOUBLING progress versus the
untuned default) with a clear point of diminishing returns around
sigma~0.02 - past that, the bottleneck becomes how often a genuinely NEW
GPS sample arrives (rate-limited at 20 Hz), not how hard each one is
trusted, which is exactly the sensible engineering picture and a clean
result to have in hand. **Still not a complete fix**: even at this
tuned combination the dash reaches only ~54 of 100 m before the run's
time budget ends, roughly half the course, not the whole thing. Complete,
honest picture for tonight: wiring bug fixed, frame bug fixed and
verified, GPS modeled at a real hardware rate, gain tuned to its own
point of diminishing returns - each step measurably better than the
last, none of them alone or together sufficient to finish the dash.
`$SIM_VEL_AIDING` stays default OFF; `$SIM_GPS_VEL_SIGMA=0.02` is the
recommended value for whoever next investigates this, not yet promoted
to the shipped default.

## Two more found via the new launcher: a missing dash overlay, and a real panel bug behind an atom spin-out

**The dash overlay was not drawing.** `mission_waypoints("dash:100")`
correctly returns `[(100.0, 0.0)]` after the fix above - one point, no
return leg - but the panel canvas only draws a planned track when it has
more than one point (`static/app.js`: `if (p && p.length > 1)`), so a
single-point course drew nothing at all. Fixed in `server.py`, not in
`mission_waypoints()` itself: when the planned-track builder sees exactly
one point it prepends the local origin `(0.0, 0.0)` - the same "the path
must start where the robot is" anchor `BodyPathPlanner` already uses for
the real controller. Prepending in `mission_waypoints()` instead would
have been wrong: that function's point COUNT also gates whether the
separate append-dash-finish overlay logic fires (`if s.get("dash") and
len(pts) >= 2`), and inflating a standalone dash from 1 to 2 points would
have made a leftover truthy `dash` field on that slot incorrectly trigger
the finish-overlay math on top of an already-dash-only mission. Verified
via `/api/state`'s `planned` field directly (`{"0": [[0.0, 0.0], [0.0,
100.0]]}`), not just by eyeballing the canvas.

**The atom spin-out (run17) was a real, independent panel bug - not a
regression from any of today's C++ planner changes.** The atom slot was
running `gait=trotRunning speed=3.5` - star/oval's profile - instead of
its own validated `trotting @ 2.1, WP_ALON=0.4`, even though the slot's
own `note`/`extra` fields correctly displayed the atom's recipe text.
Root cause: `static/app.js`'s change handler for the mission dropdown only
ever POSTs the new `mission` string - it never re-applies that course's
recipe (`gait`/`speed`/`extra`), the way `draft_add_slot()` already does
server-side for a BRAND NEW slot. So switching an EXISTING slot's mission
kind (star -> atom) left the previous mission's gait/speed in place while
the displayed note text (looked up fresh from `state.recipes` on every
render) correctly described the new one - a UI showing the right label
over the wrong command. trotRunning is documented above as one of the
worst gaits for the atom's continuous curvature; running it 67% over its
own recipe's speed on top of that produced exactly what the operator
saw. Fixed: selecting a mission now looks up `state.recipes[kind]`,
resolves its numeric gait index back to a name via `state.gaits`, and
sends `gait`+`speed`(model-capped)+`extra` in the SAME POST as `mission`.
Confirmed innocent of today's BodyPathPlanner/reversal-stop changes
first: run17's raw log has no `[plan] reversal ... registered as a
stop` and no `[follow] PIVOT fired` anywhere near the fall - those
branches never engaged. Re-tested at the CORRECT recipe: PASS, t=62.3s,
in range of the previously-measured 58.97-124s atom baseline.

**A false stall-timeout, caught on the way.** The first re-test at the
correct recipe was killed by `mission_runner.py`'s own `--stall-timeout
45` mid-mission, at a point where the raw ctrl log showed the control
loop perfectly healthy (2.1-2.2ms, on-target period) and `v=2.10` still
commanded - not a wedge. The dog visibly parked at one N/E for 3+ seconds
crossing the atom's tightest corner (R=1.89m, the same point the mission
analyzer already flags as the costliest feature) before continuing; a
longer allowance (180s) let it finish cleanly. The curated orchestration
log genuinely can go quiet for 60s+ on a healthy run through a slow
corner, with nothing to distinguish that from an actual hang except
patience or checking the raw log - `--stall-timeout`'s default was raised
100 (was 60) and the module docstring now says this explicitly. Do not
trust a TIMEOUT verdict from this tool without checking the raw log
first; it is a safety net against silent wedges (like the SystemExit bug
above), not proof of one.

## The poller could get permanently stuck at "running" - a dog falling AFTER its own MISSION COMPLETE masked itself forever

Operator report: "Fleet Running got stuck. everything else was perfect."
`/api/state` showed `phase: running` indefinitely with two dogs already
judged PASS and the third's controller process long dead - no launch
possible until a manual `/api/stop`.

`_start_poller()`'s per-dog classification is an `if/elif` chain over the
log's FULL text, re-read from scratch every tick (not incremental - only
the curated EVENT_PATTERNS scan is): `"[mission] RESULT"` -> complete,
`"MISSION COMPLETE"` -> finishing, `"[FALL]"` -> fell. The order was
wrong for a real sequence: loop+dash finishes (prints MISSION COMPLETE),
then the end-of-mission settle/lie-down itself goes wrong and the dog
falls (prints [FALL]) BEFORE ever reaching the judge's RESULT line. Once
`"MISSION COMPLETE"` is anywhere in the file it is in the file on EVERY
future tick too, and it was checked before `"[FALL]"` - so that dog was
classified `"finishing"` forever, never added to `done`, and the fleet's
overall `len(done) == len(locked)` check could never pass. A dog that
falls after completing its course, but before settling safely, is a
real and not even rare event (see the atom fleet-fragility note above) -
this was not a one-off. Fixed by checking `"[FALL]"` before `"MISSION
COMPLETE"` (still after `"[mission] RESULT"`, which is the more
authoritative final outcome and should win if somehow both are present).

Verified live: re-ran the identical 3-dog config (atom+dash, oval+dash,
atom+dash) that produced the original stuck report. This time one dog
(dog0, atom) fell mid-course - roll 43.8 deg against `Unsafe locomotion`'s
40 deg cap, the already-documented atom-in-fleet roll-limited fragility,
not a new bug - and `phase` correctly reached `"done"` with `PASS=2
FAIL=0 FELL=1` instead of hanging. The stuck-panel bug is fixed; the
atom's marginal multi-dog reliability is unchanged and already tracked
above.

**Still open, not fixed here**: if a controller process dies WITHOUT ever
printing `[FALL]`, `MISSION COMPLETE`, or `[mission] RESULT` at all (a
segfault, an OOM kill, a transport failure) the poller still has no way
to notice, and the fleet would wedge the same way with no evidence in the
curated log at all. `self.procs` mixes gz/bridge/controller Popen objects
in one flat list, not indexed per dog, so a clean "is dog i's controller
still alive" check would need its own tracking - not implemented, since
every wedge seen so far has left a clear text marker to key off.

## "× remove doesn't work well", and the oval-with-dash-checked bug that turned out to be the same bug

Operator report, two symptoms in one message: the remove button on a slot
"doesn't work well", and separately, a slot with the "100m dash when
done" checkbox visibly CHECKED did not actually dash at the end of an
oval run.

**The remove handler used a stale positional index.** `static/app.js`'s
`[data-remove]` click handler captured `i = +e.target.dataset.remove` at
render time, spliced the LOCAL `slots` array immediately, fired
`DELETE /api/slots/i`, and re-rendered - all synchronously, with the
fetch's response never even read. Removing a slot changes the SERVER
index of every LATER slot, so any second click (another remove, or any
other field edit) that fires before the first DELETE's response lands is
captured against a now-wrong index: it can act on the wrong slot, or send
an out-of-range DELETE that server-side `draft_remove_slot()` correctly
rejects (`"no such slot"`) - a rejection the client never even looked at,
so nothing surfaced the failure. Reproduced directly: firing two remove
clicks back to back only removed ONE slot locally while still sending a
DELETE for the OTHER (now out-of-range) index, and depending on request
ordering the server's draft could end up silently different from what
the panel displayed - exactly a "doesn't work well" symptom with no error
anywhere.

Fixed properly, not patched: a module-level `_removing` flag now folds
into the same `locked` condition that already disables every slot control
during an active fleet, so the INSTANT a removal starts, every input
(including the other remove buttons) is disabled - closing the race
structurally rather than by luck of timing. The handler is now `async`,
awaits the DELETE response, and adopts the SERVER's own returned slot
list as truth (`slots = j.slots`), the same server-truth pattern
`addSlot()` already used. Verified in the browser: right after a click,
querying every remaining `[data-remove]` button shows `disabled: true`
until the request resolves, and a second click fired in that window is a
structural no-op (disabled elements do not dispatch click at all) rather
than a race that sometimes loses.

**The oval-with-dash symptom was very likely a downstream effect of the
same bug, not a separate mechanism failure.** Tested the dash mechanism
directly, isolated from any remove interaction: a solo oval slot with
`dash=100`, launched through the exact `/api/launch` path the button
calls, dashed correctly end to end - `loop complete - stopping, lying
down before the dash finish` -> `back on its feet - dashing the final
leg` -> `mission result: PASS` at t=83.8s. So the underlying appendDash/
interlude machinery is not at fault. The credible explanation is that an
earlier slot removal, racing under the bug above, desynced the oval
slot's server-side `dash` field from what the checkbox displayed - the
panel showed checked while the server actually launched with `dash=0`.
Since the race is now closed structurally, this should not recur; flagged
here rather than closed outright because it was not reproduced in this
exact shape (only inferred from the mechanism and the timing of both
reports arriving together).

## "Re-launch fleet needs clicked several times to react" - same family, plus a real caching trap found chasing it

Same shape as the remove-button bug, on the Launch button: `renderFleet()`
only disables `launchBtn` once a POLL TICK (every 400 ms) observes
`phase` actually leave `"launching"/"running"`, but the click handler
itself did nothing synchronous - no disable, no relabel - so there was up
to a 400 ms window after a click where the button still looked live. A
click landing in that window fires a SECOND `/api/launch`, which the
server correctly refuses (`"a fleet is already active"`) - but that
refusal surfaced via `alert()`, a BLOCKING modal that freezes ALL page JS
(including the `poll()` loop) until dismissed. A couple of impatient
clicks stack a couple of frozen dialogs: the FIRST click had already
launched the fleet, but the page looked dead behind the pile-up, which is
exactly "needs clicked several times to react" from the outside. Fixed
the same way as remove: disable the button and set it to "Launching..."
synchronously, before the `fetch`, and only restore it manually on an
actual refusal (a success leaves it disabled for `renderFleet()` to take
over once `phase` really moves).

**Found chasing this, independently worth having**: the browser serving
this panel was running STALE `app.js` through a `Cmd+Shift+R` hard
reload, a brand new tab, AND a fresh `preview_start` - repeatedly. Cause:
`SimpleHTTPRequestHandler`'s default headers carry `Last-Modified` but no
`Cache-Control`, so a browser's heuristic freshness rules can cache a
static file indefinitely with no revalidation at all - not even a
conditional GET. For a panel whose JS/CSS get edited and reloaded
constantly during active development, that is a real trap: a page open
from before a fix landed keeps running the OLD, already-buggy code with
NO visible sign anything is stale - the exact same "silently wrong
because nobody rechecked the source" shape this file already warns about
for logs, just for served files instead. Fixed with one line
(`Handler.end_headers()` now sends `Cache-Control: no-store`
unconditionally, for both the JSON API and the static files) rather than
worked around per-request, since this is a single-operator local dev tool
where there is no caching benefit worth ever risking staleness for.
Verified: `curl -sI /app.js` now shows `Cache-Control: no-store`, and the
button fix was confirmed live immediately afterward with no further
cache fighting.

**Operator-confirmed, not just self-tested.** Per direct confirmation
this now works: a full re-launch cycle through the live panel (run29 into
run30, both via real clicks - not curl/`mission_runner.py`) landed clean
- both dogs `settled on its feet -> ok`, `lying down -> ok`, `mission
result: PASS` - with none of the multi-click/frozen-dialog behaviour that
prompted this whole chain of fixes. That closes out, in one session, the
remove-button race, the launch-button race, the stale-cache trap that
made both hard to verify, and the earlier fleet-phase-stuck-at-"running"
poller bug - all four found from two short operator reports and none of
them speculative fixes.

## The spawn-underground bug: two wrong fixes, then a measured one

Operator report, from a screenshot: at launch the dog's legs/feet are
visibly buried below the ground plane instead of resting on it.

**Attempt 1, REVERTED both times it was tried, per direct correction: a
deliberate joint-angle crouch pose.** Encoding a folded (hip=0, thigh=1.3,
calf=-2.5 rad, URDF convention) starting posture into each leg's own child
LINK pose (the `//joint/axis/initial_position` SDF element is deprecated
in 1.8 and gone from the 1.9 schema this project uses - verified it is a
no-op on this gz-sim by direct measurement before trying the pose-encoding
route instead). This is the WRONG FIX regardless of whether the specific
angles chosen were also bad: the operator's actual complaint was about Z
placement only, and in practice this made the robot topple onto its back
during stand-up - not a rendering artifact, reproduced live. Reverted to
the exact pre-session baseline both times it came up. **Standing lesson:
do not re-introduce a joint-angle spawn fix without checking with the
operator first** - they have now rejected it twice, explicitly.

**Attempt 2, ALSO reverted: raising spawn Z alone to 0.47 m.** This
overcorrected into a different, worse failure - the run died in the same
second it launched. Root cause, confirmed by reading the code rather than
guessing again: `RobotRunner.cpp`'s z-collapse fall check latches `stood`
the first time estimated body height exceeds 0.25 m, specifically so the
robot's normal low, belly-down spawn (0.08 m) is never misread as a
collapse before it has ever stood. Spawning above 0.25 m arms that latch
immediately, and the very next low reading - the ordinary settle under
gravity during the boot-limp phase, before any controller torque exists
at all - reads as an instant collapse. `[FALL] collapsed: roll=-0
pitch=2 z=0.043` at t~0, level attitude: not a real fall, a false
positive from a threshold this change violated.

**What actually shipped (commit 4c84efc): spawn Z raised to 0.42 m, plus
a genuinely separate fix for the false-positive this surfaces.** The Z
value itself was chosen empirically, not calculated: probed the settled
FL_calf world pose via `gz topic -e -t .../pose/info` after a full 1 s of
physics-only settling (no controller) at several spawn heights.

| spawn Z | settled foot bottom (front / rear) |
|---|---|
| 0.08 (old) | -0.32 to -0.37 m |
| 0.20 | -0.18 to -0.19 m (worse than 0.42 - see below) |
| **0.42** | **-0.10 / -0.17 m** |
| 0.60 | -0.10 / -0.18 m (no further improvement) |

Two things this measurement found that a hand calculation would have
missed: the improvement is NOT linear with spawn height (0.20 -> 0.42 is
a much bigger jump than nominal geometry predicts, because the leg
absorbs some of the extra drop by bending further under the fall's
momentum before settling), and past ~0.4 m the settled configuration
stops depending on spawn height at all - 0.42 and 0.60 land within
noise of each other, so 0.42 reaches the same equilibrium as 0.60
without the extra drop. **This is also the honest limit of a Z-only
fix**: the legs' own contact-resolved resting shape has a physical floor
(front feet ~10 cm under, rear ~17 cm under) that no spawn height above
this closes further - reaching exactly Z=0 would need the leg's own
posture changed, which attempt 1 already showed is off the table without
checking first. Front feet clearing better than rear is itself a real,
unexplained asymmetry - not investigated further this session.

**The false-positive fix, done correctly this time**: rather than bump
the 0.25 m arm threshold (which cannot work - the new 0.42 m spawn is
itself ABOVE the robot's own normal standing height of ~0.29 m achieved
crouched under load, so no single threshold can sit above spawn height
AND at-or-below standing height at once), the fix reuses
`setFallZEnable()`, the mechanism this codebase already has for exactly
this situation (it already suspends the same check around the dash
interlude's commanded lie-down). Suspended in
`Stm32mp1HardwareBridge.cpp`'s sequencer thread for the whole boot
window (spawn through the first genuine stand), re-armed the instant the
sequencer reaches its final control mode, with a fallback re-arm for the
rare `$SIM_MODE=1` debug path that never reaches that point. Mid-mission
collapse detection is completely untouched - this only widens the boot
window that was already exempt, using the pattern already validated
there.

Verified: single-dog star mission, clean run start to finish, no
immediate false fall, `mission result: PASS` at t=61.8s matching the
pre-existing baseline for this exact config - the fix does not regress
anything measured earlier in this file.

## Seven new missions: SAR search patterns and Lissajous search curves

Per direct request (source: Steckenrider et al., "Lissajous curves as
aerial search patterns", Sci Rep 14:11144, 2024 - Fig. 1 for the four SAR
patterns, Fig. 2/Eq. 8 for the Lissajous curves): circle, sector,
parallel track, and expanding square search, plus a Lissajous mission
covering the 1:2 and 5:7 ratio examples and the 11:9 stretch goal. All
seven now have a `WaypointNav.cpp` generator (circle already existed),
full panel wiring (RECIPES, dropdown, `mission_bbox`, the Python
trail-overlay mirrors in `trail_daemon.py`/`mission_viz.py`), and a
verified PASS:

| mission | spec | result |
|---|---|---|
| circle search | `circle:9:8` | PASS 32.8s |
| sector search | `sector:15:3` | PASS 112.8s |
| parallel track | `parallel:30:5:8` | PASS 158.2s |
| expanding square | `expsquare:5:12` | PASS 87.4s |
| Lissajous 1:2 | `lissajous:15:1:2` | PASS 96.2s |
| Lissajous 5:7 | `lissajous:15:5:7` | PASS 345.9s |
| Lissajous 11:9 (stretch) | `lissajous:15:11:9` | PASS 561.7s |

All seven converged on the SAME tuning - `gait=walking, speed=1.5,
WP_ACCEPT=1.5 WP_CORRIDOR_MIN=0.1 WP_ALON=0.4` - which is worth noting on
its own: none of these new courses needed a NEW lever, only the two
already discovered for star (graded corridor) and the atom (gentle
longitudinal braking). Six of the seven initially fell at the DEFAULT
(untuned) config, including circle - a constant-radius course with no
corners at all, which fell in its END-OF-MISSION STOP, not the course
itself.

### The sector-search geometry bug: duplicate waypoints at the same physical point

First implementation of `makeSectorSearch()` (the six-leg alternating
D/D-half "flower" pattern) put a literal waypoint at the pattern's own
centre for EVERY repeated cycle. This is a real property of the six-leg
math (verified by hand and numerically: the six legs at 120 deg apart
always sum to exactly zero displacement), not a bug in the generator -
but it meant `reps=3` put THREE separate waypoint indices on the exact
same (north, east) coordinate. A follower doing nearest-point-on-path or
pure-pursuit target selection cannot tell "arrived at cycle 1's centre"
from "cycle 2's" apart when they are literally the same point, and
measured live it produced exactly the textbook self-intersecting-path
failure: the flown track cut a smooth blob through the centre instead of
tracing the six-leg zigzag (operator screenshot), and the mission failed
within one cycle. Fixed by skipping the cycle-closing waypoint for every
repetition except the true final one - same physical path (the
north/east running totals are unaffected), just not asking the follower
to treat the shared point as several distinct arrival goals. This did
not by itself fix the fall (see below), but it materially changed the
flown path shape and was worth doing in its own right.

### Tuning story: same two levers as star/atom, on six more courses

Chasing sector search's fall (after the geometry fix) reproduced the
exact troubleshooting arc already documented above for star and the
atom, on brand-new courses:
- default corridor: fell at t~22s, 0/16 waypoints captured.
- `+WP_CORRIDOR_MIN=0.1` (star's graded-corridor fix): survived to
  55s, captured 3 waypoints, tightest corner correctly re-computed
  tighter (R=0.14m) and STILL fell - but now visibly PITCH-dominant
  (roll=22.5, pitch=34.4) - the atom's own signature.
- `+WP_ALON=0.4` (the atom's fix): PASS, clean, 112.8s.

The same sequence, or a subset of it, resolved parallel track (also
needed a SPEED drop to 1.5 - its failure was carrying full cruise
momentum through a short 5m connector immediately after a 30m straight,
a genuine braking-distance problem the corridor grading alone did not
fix), expanding square and both remaining Lissajous ratios (clean on
the FIRST try at the full tuning), and circle (fell once at the bare
2.0 m/s default, clean at 1.5 with the full tuning). The practical
takeaway: for any FUTURE new mission on this planner, start with this
tuning rather than the bare default - it is now the common case, not
the exception.

### A real, silent process-timeout bug, found chasing what looked exactly like a crash

The Lissajous 5:7 test (558m, the longest course tried to that point)
appeared to hang: `mit_ctrl_sim` had cleanly reached wp228/366 (62%
through, healthy control loop, no orientation trip) and then simply
STOPPED WRITING TO ITS OWN LOG - no [FALL], no error, nothing - for
minutes, reproduced identically three times. `pgrep mit_ctrl_sim`
returned nothing: the process was gone. No macOS crash report was ever
generated (checked `~/Library/Logs/DiagnosticReports/` - empty for
today), which is itself the tell that this was NOT a segfault. The
actual cause: `server.py`'s controller launch command wraps
`mit_ctrl_sim` in `timeout 240` - a hard safety net sized for
star/oval/atom/dash, which all finish in well under 240s. Lissajous 5:7
legitimately needs 400-500s and was silently SIGKILLed two-thirds of the
way through, with a failure signature (process just vanishes, no
evidence anywhere) indistinguishable from an actual crash without
checking `pgrep` and the crash-report directory. Raised to 900s (comfortable
headroom for 11:9's ~560s actual runtime). **If a future mission's
controller process "vanishes" with no log evidence, check `timeout`
before spending time hunting for a memory bug** - this cost real time
here precisely because the failure signature is identical to one.

### An intermittent boot-time flip, NOT resolved, flagged honestly

Twice out of six total launch attempts on Lissajous 5:7 today, the robot
flipped completely over (`roll=180 z=nan`) DURING THE BOOT SEQUENCE,
before nav ever took the stick - i.e. before the mission's own geometry
could possibly be involved. A same-moment control test (`dash:100`,
launched immediately after one of the failures) passed cleanly, ruling
out "the host is generally unhealthy right now." The other four boot
attempts on the identical mission succeeded normally. This is a real,
reproducible-but-intermittent failure mode (2/6 this session) that was
NOT root-caused - it was not chased further because (a) it is confined to
the boot window, before any mission-specific code runs, so it is very
unlikely to be specific to the new missions in this file, and (b) it
carries the same signature (an unexplained attitude/height blowup with no
clear trigger) as the host-load-stall class of issue extensively
documented elsewhere in this file for OTHER reasons (Time Machine, shared
gz physics). Flagged here as a known, open, intermittent risk rather than
either ignored or falsely claimed fixed.

## Full 11-mission end-to-end re-run, and a methodology bug in the harness that ran it

Per direct request, after the operator questioned a stale UI warning and
a screenshot that looked like the new missions round corners worse than
star always did: ran oval, atom, star, dash, then all seven new missions,
back to back, using each one's live `/api/state` recipe (not
hand-transcribed, to avoid the exact staleness bug just reported).

**The stale-warning report was correct and a real instance of the
already-documented trap**: `RECIPES["circle"]` had been edited in
`server.py` after the last server restart, so the running process was
still serving the OLD (untuned, 2.0 m/s) recipe to the panel. Restarted;
confirmed `/api/state`'s `recipes.circle` matched the source file before
proceeding. No code fix needed here - purely a "restart after every
RECIPES edit" discipline gap, same class of thing SKILL.md already warns
about for `server.py` generally.

**The corner-rounding hypothesis was checked directly, not argued
against**: `git diff` of every commit from today against
`common/include/Planning/BodyPathPlanner.h` (the actual fillet/corner
logic) is EMPTY. The Lissajous and SAR-pattern generators are new,
separate functions in `WaypointNav.cpp` that never touch the planner.
There is no code path by which one could corrupt the other's cornering.
The screenshot was almost certainly circle's ORIGINAL untuned attempt
(2.0 m/s, no corridor grading) - the one already documented above as
falling - not the fixed version.

**A real methodology bug in the harness script that ran the sequence**:
it only ever set slot 0, never cleared slots 1/2, so EVERY "solo" test in
the run was actually an accidental 3-dog fleet with whatever was left in
the other two slots (the default oval/atom layout, unchanged all
morning). Nine of eleven still came back an unambiguous `PASS=3 FAIL=0
FELL=0` - a fleet result where all three pass is not weakened by not
knowing which dog was which. Two were ambiguous (star: 2 PASS 1 FELL;
circle: 1 PASS 2 FELL) - re-run as genuinely isolated single-slot
launches, both came back clean: star PASS 68.6s, circle PASS 33.1s,
matching their already-established solo baselines exactly. **All eleven
missions confirmed working**; the two "falls" were an artifact of the
accidental 3-dog fleet context (consistent with this file's own
long-documented fleet fragility, not a new regression), not of star or
circle themselves.

## Automatic post-mission reports: orchestration log + planned-vs-flown plot

Per direct request: "start making a report post mission that includes
the orchestration log, and what you see on the path planned / path
driven part of the screen" - motivated by exactly the kind of
stale/ambiguous-screenshot confusion documented in the section above.
`mission_runner.py` now writes `/tmp/cheetah_conductor/reports/run<N>_
report.{txt,png}` on every run - PASS, FAIL, or TIMEOUT/stall abort - with
the full accumulated orchestration log, a per-dog PASS/FAIL/FELL/
incomplete breakdown, and a plot of each dog's planned vs. flown path.

**Deliberately reads the panel's own data, not a re-derivation.** The
plot pulls `state["planned"]` / `state["positions"][i]["trail"]` straight
from `/api/state` - the exact world-frame arrays `server.py` freezes at
launch and updates live, that `app.js`'s canvas already draws from
(`hue.dim` for planned, `hue.bright` for flown). Reconstructing the
geometry independently (e.g. re-integrating the raw controller log) could
silently diverge from what was actually on screen, which defeats the
entire point of the feature.

Also pulled `mission_waypoints()` out of `trail_daemon.py` and
`mission_viz.py` into a new dependency-free `stm32mp1/gazebo/
mission_geometry.py` (no `gz` imports) so the report generator could use
it without requiring the `gz.transport13` bindings, which only exist in
specific venvs on this Mac. Found a real, previously-unnoticed bug doing
this: `mission_viz.py`'s own copy of the function had silently drifted
and was missing the `atom` and `oval` cases entirely - it would have
raised `unknown mission spec` if anyone had ever run a viz on either.
Fixed as a side effect of de-duplicating, not a separate patch.

Verified against the live server on two shapes, and caught two real bugs
in the process:
- A degenerate near-straight `dash:20`: `aspect='equal'` on a near-zero-
  width bounding box crushed the x-axis to a sliver and its tick labels
  overlapped into an illegible smear. Fixed by mirroring `app.js`'s own
  bounding-box + fixed `pad=6` framing convention (with the axis span
  floored so a truly 1-D course still gets a plottable width) instead of
  trusting matplotlib's auto-margins.
- Per-dog verdict attribution: the first cut matched
  `line.startswith("dogN:")`, but `/api/state`'s log lines carry a
  `"[HH:MM:SS] runN "` prefix ahead of the `"dogN: ..."` text
  `_note()` was actually called with - so nothing ever matched and every
  dog reported `incomplete` regardless of its real result. Fixed to a
  substring check (`"dogN:" in line`), which cannot false-match a
  different dog's line up to 3 dogs (`"dog1:"` is not a substring of
  `"dog10:"`).

The `sector:15:3` test run's plot visibly reproduced the corner-rounding
the operator had already flagged from a live screenshot - the bright
(flown) track visibly cuts inside the dim (planned) polygon's sharp
vertices - now captured automatically as report evidence instead of
needing a live screenshot at the right moment.

## Sector's corners tightened to star's own standard, via a standalone planner probe

Per direct request, after the report above put a number on the
corner-rounding: "start attacking the pre-planner fixes to allow this
current shape to be followed as tightly as the star." The
`BodyPathPlanner.h`/`MissionAnalyzer.h` code itself had already been
confirmed (see the section above) to be byte-identical across every
commit in this file - so this was never "the planner is broken for the
new missions", it was "the planner's corner-grading CALIBRATION was
tuned against exactly two data points (star's own 144/162 deg corners)
and never validated at any other angle."

**Measured, not re-derived by hand.** Built a standalone probe
(`planner_probe.cpp`, kept in scratch alongside the existing
`test_missions.cpp`, not committed - same convention) that links the
REAL `WaypointNav.cpp` and the REAL `BodyPathPlanner.h` with no Gazebo
dependency, generates a mission's actual waypoints, runs them through
`plan()` with that mission's actual recipe parameters, and dumps every
fillet corner's direction-change angle, radius, and commanded speed. On
star and sector's baseline tuning:

```
star:   144 deg corner -> R=0.191m v=0.229m/s   (162 deg tip -> R=0.028m v=0.033m/s)
sector: 120 deg corner -> R=0.830m v=0.996m/s   (dominant angle, 10 of 15 corners)
```

Sector's typical corner was planned 4x WIDER and 4x FASTER than star's
mildest corner - by geometry, not by a tracking failure. Root cause: the
corridor-grading curve (`BodyLimits::corridor_scale_min`, engaged by
`WP_CORRIDOR_MIN=0.1` in both recipes) ramps linearly from full corridor
at `turn_soft` (80 deg) to the graded minimum at `turn_hard` (160 deg).
Star's two corner angles (144/162 deg) sit 80-100% up that ramp. Sector's
dominant 120 deg angle sits only ~50% up the SAME ramp - the mechanism
that makes star's corners tight was firing at roughly half strength on
sector, never touched or validated for an angle in between.

**Fix, scoped to one recipe.** `WP_TURN_SOFT`/`WP_TURN_HARD` already
existed as env-var hooks in `mit_sim_main.cpp` (added for exactly this
kind of tuning) but no recipe had ever set them. Swept candidate
(turn_soft, turn_hard) pairs through the probe until sector's dominant
corner landed in star's own ballpark:

```
turn_soft=0.8 rad (46deg), turn_hard=2.0 rad (115deg):
  sector 120 deg corner -> R=0.150m v=0.180m/s   (was 0.830m / 0.996m/s)
  sector 147.5 deg (cycle-boundary) -> R=0.058m  (tighter than star's own tip)
```

Added to `sector`'s `RECIPES` entry ONLY (`server.py`). `WP_TURN_SOFT`/
`WP_TURN_HARD` default to 1.4/2.8 rad when unset, so star/oval/atom/dash
and every other mission's recipe is untouched - confirmed by re-running
star through the fixed harness afterward: PASS 69.0s, matching its
established baseline exactly.

**Live result**: sector:15:3 PASS 141.3s (was 112.8s untightened) - a
report plot (`run83_report.png`) shows the flown track hugging every
vertex with no visible rounding anywhere on the course, a stark contrast
to the same plot before the fix. The time cost (+28.5s, ~25%) is the
expected price of a tighter racing line - hugging a vertex instead of
cutting it is a longer path at a lower cornering speed, not a free
change, and is exactly the tradeoff that was asked for ("as tightly as
the star", not "as fast as possible").

**A real bug in the test harness, caught mid-investigation, fixed
separately from the planner tuning.** The first attempt to test the new
tuning launched sector's waypoints at star's gait/speed/dash
(trotRunning, 3.5 m/s, dash=100) instead of sector's own (walking, 2.0
m/s, no dash) - `mission_runner.py`'s docstring had always claimed
omitted `--gait`/`--speed` "fall back to that mission's own recipe
default", but `/api/slots/{i}` only OVERWRITES fields it is explicitly
given; there is no server-side recipe lookup for an omitted field (that
lookup lives in the browser's own JS). A server restart (for the
`RECIPES` edit above) reset the draft slot to server.py's hard-coded
default, and the claim in the docstring turned out to have never been
true - it happened to work before only because whatever a human or an
earlier script call left sitting in that slot's other fields was
usually already correct. Fixed by having `mission_runner.py` look up
`state["recipes"][kind]` itself and explicitly resolve gait/speed
whenever the matching flag is omitted. Caught a SECOND bug applying the
first fix: recipe gait is stored as the numeric `SIM_GAIT` id (e.g. 20
for walking), but `/api/slots/{i}` validates gait by NAME against
`GAITS` (name -> id) - sending the id back verbatim silently failed that
check and dropped the field, the exact same class of silent
carryover the fix was trying to close. Fixed with a reverse
id->name lookup. And a THIRD, on `extra`: `launch()` already prepends
`RECIPES[kind]["extra"]` to the slot's own extra field automatically
(`server.py:598`), so defaulting the omitted case to a COPY of the
recipe's extra doubled every token in the locked launch line (harmless -
env `A=1 A=2` keeps the last - but confusing and wrong). Fixed by
defaulting the omitted case to an explicit `""` instead, which also
closes a carryover path of its own (a stale custom `--extra` from an
earlier call on the same slot).

## The final-waypoint gap, a closed-loop follow() simulation, and the real cause of "0/1 dogs came up"

Three follow-ups in one thread, after the operator complimented the
tightened sector corners and asked to keep refining: "you fell short of
the final waypoint too"; "the stars corners are cleaner... less
porpoising"; and, once that turned into a gait A/B test, an unrelated
but serious host-level bug that had nothing to do with any of it.

**Final-waypoint gap - real, fixed.** Confirmed in the raw log first:
`reached wp15 (N=-0.00 E=0.00) dist=1.49` - the dog called the mission
done 1.49m short of the true origin. Root cause: `WP_ACCEPT` is BOTH
the legacy waypoint-arrival radius (`WaypointNav::update()`'s own
"close enough, advance" check, which drives the mission's running/
complete state independently of `BodyPathPlanner`) AND the fillet
corridor width - tuning it down for tight corners means the mission-END
waypoint also counts as arrived a full `WP_ACCEPT` short, invisible at
an intermediate waypoint (the next leg starts anyway) but a permanent
gap on a closed course. Added `WaypointNav::final_accept_radius` (opt-in,
default -1/off) as `$WP_FINAL_ACCEPT`, wired only into sector's recipe at
0.3. Real C++ change - required a host rebuild via `deploy_host.sh`
(never `cp` a fresh binary into `host-run/` directly - that script's own
comment documents the codesign/EXIT-137 trap). Verified live twice: PASS
141.5s/141.8s, final wp closes to dist=0.25-0.27m. One intermittent FALL
seen on the very first live run of the rebuilt binary, at waypoint 9 of
16 (~85s in) - nowhere near the final waypoint this change touches, and
`final_accept_radius` is a no-op for every waypoint except the last.
Immediate re-run: clean PASS, confirming it wasn't reproducible.

**"Star's corners are cleaner" - measured, not guessed, and NOT a
planner bug.** Built a closed-loop probe (`follow_sim.cpp`, scratch
only) that runs the real `BodyPathPlanner::follow()` against an IDEAL
unicycle model - zero robot dynamics, zero tracking lag, perfect
actuation - to answer one question before touching any shared code:
does the corner-exit wobble come from the steering algorithm itself, or
from the real robot's imperfect tracking of a clean command? Answer:
in the idealized simulation, BOTH star's and sector's corners converge
identically cleanly - one small overshoot-correct, then locked to
`w=0.000` for the rest of the straight. The ~14 degree secondary
heading rebound measured live on sector does not exist in this
simulation at all. The planner code is not the difference.

**The gait hypothesis, tested and ruled out.** The operator's own
observation - star runs `trotRunning`, sector runs `walking` - was a
fair, testable hypothesis: swap sector to trotRunning and compare.
Once genuinely tested (see below for why that took three attempts),
sector on trotRunning at the same 2.0 m/s (PASS 142.9s) showed an
IDENTICAL corner-exit wobble to walking (`-108 deg` rebound vs
walking's `-111 deg`, same corner, same timing) and near-identical
total mission time. Gait choice does not explain or fix the wobble.
The remaining, sufficient explanation is course geometry: star's legs
run ~18-20m between corners, sector's run 7.5-15m, so the same absolute
settle-wobble occupies a much larger fraction of a visibly shorter leg,
and sector packs far more corners into a similarly-sized course. Not a
bug - a consequence of the shape being asked for.

**What actually blocked the gait test, and very likely explains some
of this file's own older "intermittent, unresolved" boot-time
instability too.** The FIRST three attempts at the trotRunning/walking
comparison all failed identically: the simulated dog froze at its spawn
point forever - v pinned at v_min, N/E never changing, "0/1 dogs came
up" at the sensor-advertise wait, bridge log showing all-zero telemetry
(`imu_az=0`, `gps=(0.0,0.0)`) despite `cmd_rx=500/s` looking perfectly
healthy. Ruled out in order, with real evidence at each step rather than
guessing: a stale process (a leftover `cheetah_gazebo_bridge.py`,
running for 10+ hours) squatting on UDP port 9100 - killed it, but a
plain walking `dash:20` with zero custom tuning STILL froze identically,
proving gait was never actually the variable under test; host CPU load
(retried at load average 0.98 - still failed); thermal throttling
(`pmset -g therm` - nothing recorded); stray gz/ignition processes
(none); stale gz-transport discovery/shared-memory files (none); low
memory (tight but not exhausted on this 36GB Mac).

The actual cause was sitting in `gz.log` the entire time, which nothing
up to that point had actually opened: flooded with `Exception sending a
multicast message: No route to host` on every single send.
gz-transport's discovery defaults to multicasting over whatever
interface the OS's routing picks (`en0` here), and that interface's
multicast route had stopped working sometime during this session - a
Mac that had been up 1+ day, some networking state (DHCP renewal,
sleep/wake, a VPN toggle) drifted underneath it. The exact trigger was
never isolated and does not need to be; the fix does not depend on it.
Every simulated dog's pose/IMU never actually got discovered, so runs
that appeared to "start" were dead on arrival the whole time, regardless
of gait, mission, or tuning - a run only ever "worked" if it happened to
launch in a window where multicast was still routing.

This is a single-machine, single-user setup - every peer already talks
over 127.0.0.1 (the controller-bridge channel, on a completely separate
path, was never affected - that's why `cmd_rx=500/s` kept looking healthy
while everything gz-transport-based was dead). There was never a reason
for gz-transport's OWN discovery to leave loopback. Fixed with
`os.environ["GZ_IP"] = "127.0.0.1"`, set once at module load right
alongside the existing `GZ_PARTITION` (same reasoning - has to be set
before the transport library initialises, and propagates into every
subprocess via the existing `env.copy()` chain). Confirmed outright:
`gz.log` came back completely empty (0 lines) on the very next run, "0/1
dogs" stopped appearing, and every mission tried since - `dash:20`
trotRunning (PASS 9.9s), `sector:15:3` trotRunning (PASS 142.9s),
`star:10.514:5` (PASS 69.0s, exact baseline match) - completed cleanly.

Not retroactively claimed as certain, but worth flagging for whoever
hits this class of symptom next: this exact signature (frozen at spawn,
all-zero bridge telemetry, "0/N dogs came up") may explain some of this
file's own earlier-documented intermittent boot-time falls from previous
sessions, which were never traced to a `gz.log` multicast error because
nothing had looked there before tonight.

## Camera checkboxes are already live; chase position isn't (and can't be, cheaply); the flown trail was silently truncated on long courses

Three follow-ups from watching the panel live during actual use.

**Camera on/off checkboxes work as designed.** Tested directly against
a real in-progress run (toggled `cam_front` on a live `lissajous:15:11:9`
mid-flight): the click POSTs to the draft, the draft updates, the LOCKED
(actually-running) slot correctly stays untouched, and `_subscribe_cameras`'s
per-frame callback already re-reads the draft on every image (see that
function's own comment) to mute/unmute a stream instantly. This part of
"slot settings don't react live" was not actually broken.

**Chase camera position (`chase_distance`/`height`/`degree`) is
correctly locked, and making it live is a real feature, not a bugfix.**
`configure_chase_cam` bakes the camera's offset into a body-mounted
sensor's `<pose>` at SPAWN time specifically so it rides with the dog at
zero per-tick cost (see that function's own comment). Changing it after
spawn would need converting to a free-floating camera pushed a fresh
pose every tick from the dog's live position - the exact per-tick loop
the current design was built to avoid. Flagged, not built, pending a
decision on whether that trade-off is wanted.

**The flown trail was being silently truncated on long courses -
`TRAIL_MAX 4000 -> 20000`.** Reported as "Lissajous 11:9 ended
prematurely, missed like 5-7 legs." Checked the actual navigation
first: the ctrl log's waypoint-reached sequence was 0 through 605,
every index in order, zero skips - PASS at 562.0s, matching the
established 561.7s baseline almost exactly. The mission never actually
missed anything; `_subscribe_pose`'s trail array is a rolling window
(`(trail + [pt])[-TRAIL_MAX:]`), and at `SEG_MIN=0.15m`, this course's
914.6m path needs a minimum of ~6100 points - the old 4000 cap silently
evicted the OLDEST ~2000+ points (the pattern's early loops) well
before the mission finished, so the live panel (and this same array
read back by the post-mission report generator) showed a shape missing
its first several legs even though the robot had already flown them
correctly. Raised to 20000 (>3x the longest course in the catalog).
Verified live: watched the trail array grow smoothly and un-truncated
(25 -> 160 -> ... -> 1930 points) on a fresh Lissajous 11:9 launch,
well past a meaningful fraction of the old cap with no sign of hitting
the new one.

**CORRECTION to the paragraph above, from the same night**: that "hang"
was NOT a controller/host issue at all - it was `mission_runner.py`'s
own `--stall-timeout` killing a perfectly healthy run, and every symptom
that made it look like a genuine wedge (healthy control-loop timing
right up to the last line, no `[STALL]`/`[FALL]`, `gz.log` empty, no
crash report, the bridge log showing continuous real telemetry - varying
IMU, drifting GPS - right through the "freeze") is exactly what a clean
external `SIGTERM`/`kill()` produces. Confirmed by reproducing it with a
live `sample` capture: by the time the stall-timeout's own staleness
check fired and the script tried to sample the process, it "no longer
appear[ed] to be running" - it had just been killed by this same
script's own `/api/stop` call, not by anything internal to the sim.

Root cause: `EVENT_PATTERNS` (`server.py`) has no entry for routine
waypoint advancement, and `lissajous:15:11:9` is a single-gait, non-
analyzer, no-dash mission - no gait change ever fires (needs
`$WP_ANALYZER`, unset for this recipe), no dash interlude, no fall,
nothing else to log. The orchestration log therefore produces ZERO new
lines between "nav taking the stick" and "settled on its feet" at the
very end - on this course, that gap is the entire ~550s middle of the
mission. Proven directly: relaunched with a stall-timeout the mission
could not possibly trip (700s against an expected 562s) and it ran to
completion, PASS 561.7s, matching its own established baseline exactly,
with the raw ctrl log growing continuously and linearly the whole time.

No stall-timeout VALUE fixes this - any finite number less than a
mission's own duration will eventually false-positive on a course
shaped like this one. Fixed the actual mechanism in `mission_runner.py`
instead: the poll loop now also resets its progress clock on any change
to `state["status"][i]["waypoints"]`/`["text"]`, which `server.py`'s
`_start_poller` already updates roughly once a second straight from the
raw per-tick ctrl log, independent of mission shape - catching "the
robot is still actually moving" even through a stretch that never emits
a single curated event.

## Every SAR mission tightened to star's own corner standard, spawns ON wp0, and a harness that says which end broke

Continuing the same night's work, after the sector-only fix: extended
the exact same probe-first methodology (`planner_probe.cpp`, scratch
only) to circle/parallel/expsquare, per direct report ("SAR variants
still round corners off real bad").

**All three were cutting corners at full cruise speed, zero braking.**
Measured before touching anything: circle's 45deg-per-vertex turns and
parallel/expsquare's constant 90deg turns all sit at or below the
DEFAULT corridor-grading window's own `turn_soft` (80deg) - circle never
graded at all, parallel/expsquare graded at only 12.5% strength. Result:
circle filleted at R=7.48m on a 9m-radius course; parallel/expsquare at
R=2.25m - both at full 1.5 m/s cruise, no deceleration whatsoever. Fixed
with `WP_TURN_SOFT`/`WP_TURN_HARD` narrowed to bracket each mission's
own angle plus `WP_CORRIDOR_MIN=0.07` (matching sector's own shipped
value): circle -> R=1.43m, parallel/expsquare -> R=0.253m with real
braking (v_min 1.5 -> 0.304 m/s). Verified live, fresh reports for each:
circle PASS 34.3s (was 32.8s), parallel PASS 201.2s (was 158.2s),
expsquare PASS confirmed (see below for why the FIRST attempt looked
like a regression and wasn't).

**The dog was spawning away from waypoint 0 and walking there - a
second, distinct bug, not a visual side effect of the corridor fix.**
Per direct, explicit instruction after the corner-rounding report, and
independently flagged on expsquare too ("had the same illogical non
start... had to yaw to get there and then walk to the start"). Root
cause: every mission's waypoints are relative to (0,0), and (0,0) is
always the robot's TRUE physical spawn point (its own first GPS fix at
boot) - for star this is deliberate and load-bearing (wp00 rotated due
north specifically so the opening leg needs no pivot), but for the SAR
generators wp0 landing away from origin was just an accident of how each
one's own parametric math happens to be centred, not a design choice.
Fixed with `WaypointNav::shiftFirstToOrigin()`: translate the whole
course so `_wp[0]` becomes (0,0) exactly, called only at the end of
`makeCircle`/`makeSectorSearch`/`makeParallelTrack`/`makeExpandingSquare`
- star/oval/atom/dash/`makeLissajous` never call it. Pure translation,
so it composes with the corridor tightening above with zero interaction
(every corner angle and leg length is invariant under translation) -
confirmed live, not just argued from the math. Mirrored in
`mission_geometry.py` (same four kinds only) and required fixing
`make_multi_world.py`'s `mission_bbox()`, whose circle/sector/parallel/
expsquare cases were hand-derived closed forms relative to the OLD
origin and had gone stale - replaced with a numeric bbox computed
directly from `mission_waypoints()` itself (correct by construction,
size unchanged by the translation, confirmed: circle's bbox is still an
identical 18m x 18m, just recentred). Real C++ change - rebuilt via
`cmake --build host-build --target mit_ctrl_sim` and the mandatory
`deploy_host.sh` (never `cp` a fresh binary into `host-run/` directly).

Verified live, both fixes together, fresh reports for all four:
circle PASS 30.2s, sector PASS 132.2s, parallel PASS 181.0s (all
FASTER than the corridor-only numbers above - removing the spawn-to-wp0
leg shortens the course), expsquare PASS x4 / FELL x1 across 5 total
attempts post-corridor-fix (109.0-109.2s per pass, remarkably
consistent) - the one fall predates the shift fix specifically (seen
on the very first corridor-tightening attempt, before shiftFirstToOrigin
existed) and reads as the same intermittent "state estimate went
non-finite" class of hiccup already documented elsewhere in this file,
not something either fix introduced or is expected to resolve.

**Full regression sweep on the untouched missions, same rebuilt binary**
- the explicit hard constraint going into this work was "NOT breaking
existing oval, star, atom, and dash, or the Lissajous which all work
well": star PASS 69.0s, oval PASS 37.3s, atom PASS 62.1s, dash PASS
33.3s, lissajous:15:1:2 PASS 96.1s - every one matching its own
established baseline exactly. The shift function is provably a no-op
for these five (they never call it), and this sweep confirms that held
in practice, not just on paper.

**Two harness reliability fixes, found live-testing the above.**
mission_runner.py's own `--timeout`/`--stall-timeout` produced FOUR
false positives in one night on runs that were healthy or had already
PASSED - most strikingly, expsquare printing `[mission] RESULT: PASS
(waypoints 10/10)` and then getting reported as a bare, unqualified
"FAIL" because the overall `--timeout` fired in a race right at the
finish line, indistinguishable from a real failure without opening the
raw log by hand. Fixed structurally, not by raising numbers again:
- `--timeout` default raised 300 -> 700 (lissajous:15:11:9's own ~562s
  baseline is ABOVE the old default - every default invocation on that
  course was silently guaranteed to false-time-out).
- A harness-induced timeout now exits 2, never 1 - exit 1 is reserved
  for a mission-REPORTED verdict (FAIL/FELL/error) the script disagrees
  with; exit 2 means the script gave up, which is a categorically
  different claim. Also prints an unmissable banner naming which bound
  fired and saying outright "NOT A MISSION VERDICT... CONFIRM before
  trusting this as a real failure."
- `archive_log()` in server.py: `gz.log`/`bridge_N.log`/`ctrl_N.log` now
  get moved to `RUN_DIR/archive/<timestamp>_run<N>_<name>` before each
  launch's fresh `open(path, "w")` truncates them in place - per direct
  request, after this exact investigation lost the precise ctrl log for
  an expsquare fall to the very next test launched two minutes later.

**The gait question, closed with data, not argument.** Per direct
instruction to "think hard" about `trotRunning` vs `walking` and whether
it explains anyone "shanking corners either prematurely missing them, or
rounding them off": ran `expsquare:5:12` - the one mission with an
observed intermittent fall - on `trotRunning` at the SAME 1.5 m/s cruise
`walking` uses, isolating gait as the only variable. Three consecutive
attempts, all PASS, all landing at 106.8-106.9s - tighter timing
variance than `walking`'s own 109.0-109.2s, and zero falls in 3 tries
versus `walking`'s 1-in-5. Not a large enough sample to claim
`trotRunning` is SAFER, but it is definitely not worse, and the
underlying mechanism explains why: the "state estimate went non-finite -
reinitialising" line that logically could have been a fall precursor
appears EXACTLY TWICE in every single expsquare run checked (four
`walking` passes, three `trotRunning` passes, all archived logs) -
completely independent of gait, mission outcome, or anything else. It is
a deterministic boot-time estimator transient, not a signal, and it
rules out the one gait-adjacent hypothesis that could have connected the
fall to anything measured here. Combined with the corridor-rounding
result (identical wobble on both gaits, documented above) and the
corner-cutting result (a pure function of `turn_soft`/`turn_hard`/
`corridor_scale_min`, with no gait term anywhere in that computation):
gait choice is not the explanation for any of "shanking," rounding, or
premature-miss falls observed tonight. `walking` stays the shipped
choice for every SAR pattern - there is no measured upside to switching
and it stays consistent with how each recipe was actually tuned and
validated.

## A Spirograph rosette mission, per direct creative challenge

"If you wrap it up before I wake, challenge yourself to do your best
rendition of this specific Spirograph image [8-fold symmetric flower,
big outer petals converging through a densely woven centre]... shoot
for the moon they say, you may hit it."

**The parameter search, done visually before touching any real code.**
Prototyped in Python/matplotlib (scratch only) rather than guessing once
against the real sim: swept roughly 25 hypotrochoid `(R, r, d, n_rev)`
candidates across four rounds, checking each grid against the reference
image's actual gestalt (8 clean, separated outer petals; a shared, richly
rewoven centre - not just "a flower-ish thing"). Early rounds either had
the wrong petal count (16+ instead of 8) or the right count but no
inner density (clean, sparse petals with an open ring in the middle).
The winning insight: `(R-r)/r` an INTEGER equal to the desired petal
count gives clean, unambiguous n-fold symmetry closing in ONE sweep,
and pushing `d` up toward `(R-r)` (the classic hypotrochoid "cusp"
regime) is what pulls the petals inward into a shared woven centre
instead of leaving them as separate closed loops.

**Realized the "winning" shape was already in the codebase's own
formula family.** `makeAtom`'s epitrochoid and the hypotrochoid found
above turned out to be the SAME parametric formula (verified by direct
side-by-side comparison, not assumed) - the only two differences are
which integer `k` multiplies the second cosine/sine term (`makeAtom`
uses `lobes-1`; the Spirograph shape wants `lobes` exactly) and how far
`depth` is pushed (`makeAtom` clamps well under 1.0 specifically to keep
a nucleus open; the Spirograph look wants depth near 1.0, where the
petals fold inward and converge). So `makeSpirograph` is not a new curve
family - it is `makeAtom`'s own math, deliberately placed at the corner
of its parameter space `makeAtom` avoids on purpose. Wired through the
whole mission catalog identically to every prior addition: WaypointNav
generator (same tangential-entry join trick as `makeAtom`, same reason -
starting at a lobe tip meets the curve at 90deg), `mit_sim_main.cpp`
parser, `mission_geometry.py` mirror (off by exactly 1 waypoint from the
C++ side - the same float/double rounding gap already accepted for atom/
lissajous), `make_multi_world.py` bbox (proved analytically, not just
assumed, that `|r(t)| <= outer_radius_m` holds for any `k`/`depth` in
this formula), RECIPES, and the mission dropdown.

**A real bug caught writing it, fixed before it shipped**: the
diagnostic curvature computation's acceleration terms had their signs
flipped relative to `makeAtom`'s own verified-correct formula, despite
`rx/ry/vx/vy` being copied faithfully - cosmetic only (feeds a log
printf, not the actual waypoints the robot flies) but wrong is wrong.
Also caught and corrected a wrong claim in the first draft's own
comments (called this curve "a hypotrochoid, opposite sign from
makeAtom's epitrochoid" - not actually true of the formula used here,
verified by direct comparison once written down) before it could
mislead the next person reading it.

**Verified live**: `spiro:9.0:8` PASSED on the FIRST attempt (119.2s),
identically on a second confirmation run (119.2s again) - starting from
`makeAtom`'s own proven `gait=trotting` tuning as a baseline, since this
is the same curve family and a smooth continuous course, not discrete
sharp vertices. A fresh report plot shows the flown track tracking the
planned rosette with excellent fidelity, including through the tight
centre convergence (the mission analyzer found one genuinely sustained
tight-curvature segment there, R=1.05m, correctly braked to 1.26 m/s).
Confirmed zero regression on star through the same rebuilt binary: PASS
68.9s, exact baseline match.

**Honest assessment against the reference**: a clean single-layer
8-petal rosette, not the reference's denser, more elaborately woven
multi-layer texture - some candidates in the parameter search DID
produce denser interference patterns, but none kept a clean,
unambiguous 8-fold structure at the same time as that density. A
genuine best-effort rendition, not a claimed exact match.

**Chased the density gap further, and it is a real budget wall, not a
missed parameter.** `k = (R-r)/r` a hair above 8 (e.g. 57/8 = 7.125,
denominator 8) DOES produce the reference's dense woven look while
keeping exact 8-fold symmetry - visually the best match found in this
whole search - but it only closes after 8 full revolutions instead of
`makeSpirograph`'s one, and arc length scales with revolution count.
Measured directly (not estimated): scaled to the same 9 m outer radius,
that curve is **1660 m long**. Even at the coarsest spacing still worth
calling a waypoint (1.5 m, well coarser than the 0.45 m the shipped
mission uses and too coarse to resolve the woven centre cleanly) that
is **1107 waypoints against the shared `MAXWP=768` budget** every
mission in this catalog is sized against - and at a normal cruise speed
the mission itself would run **~18 minutes**, an order of magnitude
past anything else in the catalog (the current longest, Lissajous
11:9, is 562 s). Raising `MAXWP` is a shared-constant change with
unknown reach into every other mission, and an 18-minute single run is
a real scope jump, not a tuning tweak - neither is worth it to improve
on a result already shipped, verified, and honestly caveated. Not
pursued further; recorded so nobody re-derives this from scratch.

**Multi-dog fleet confirmation, closing a real gap.** Every test above
this point tonight was a solo dog - the `make_multi_world.py` bbox fix
for circle/sector/parallel/expsquare (needed because `shiftFirstToOrigin`
made their old hand-derived closed forms stale) had only been unit-
tested against the bbox FUNCTION in isolation, never against an actual
multi-dog spawn layout, which is the only place a wrong bbox could
actually cause harm (overlapping spawns). Closed it: launched a genuine
3-dog fleet (`circle:9:8`, `sector:15:3`, `spiro:9.0:8` together) and got
PASS 3/3 - circle 30.2s, sector 132.5s, spiro 118.9s, every one matching
its own solo baseline almost exactly, and the report plot confirms all
three dogs spawned in cleanly separated lanes with zero overlap.

## Full-catalog regression sweep, one final binary, closing out the autonomous session

With the corner-tightening, spawn-on-wp0, and Spirograph work all
landed on the same rebuilt binary, ran every mission in the catalog
once more solo before calling the session done - not because any one of
them was individually in doubt, but because this is the FIRST time all
of tonight's changes have coexisted in one binary at once, and "each
change was fine in isolation" is not the same claim as "the combination
is fine":

```
star:10.514:5       PASS  69.0s (was 69.0-69.8s)
oval:40:5.0          PASS  37.4s (was ~37s)
atom:9.0:6           PASS  62.2s (was ~62s)
dash:100             PASS  33.3s (was ~33s)
circle:9:8           PASS  30.2s (post-shift baseline)
sector:15:3          PASS 132.2s (post-shift baseline)
parallel:30:5:8      PASS 182.2s (post-shift+further-tightening baseline)
expsquare:5:12       PASS 109.0s (5/6 across tonight - see the gait
                      investigation entry for the one intermittent fall)
lissajous:15:1:2     PASS  96.1s (was 96.0-96.2s)
lissajous:15:5:7     PASS 345.9s (exact match to established baseline)
spiro:9.0:8          PASS 118.9-119.2s x3
3-dog fleet (circle+sector+spiro together) PASS 3/3, no spawn overlap
```

Every mission in the catalog, confirmed on the final binary, in one
sweep. Nothing broken; everything asked for landed.

## THE REAL GAIT-SELECTION BUG: $SIM_GAIT silently discarded every runtime cmpc_gait write

Chasing "run pronking tests across all mission types" led to the oval's own
`WP_ANALYZER=1` mid-course gait changes: the analyzer's own `"[mission] gait
9 -> 5 entering straight..."` print fired correctly (proving the
`cmpc_gait.set()` call executed) but the robot never visibly changed gait.

`ConvexMPCLocomotion::run()` reads `cmpc_gait` fresh every tick (correct) and
then had `if (gait_env >= 0) gaitNumber = gait_env;` running UNCONDITIONALLY
every tick right after, where `gait_env` is `$SIM_GAIT` cached at first call.
`server.py` always sets `SIM_GAIT` on every launch. So the analyzer's write
took effect on `cmpc_gait` for exactly one tick before this put `gaitNumber`
right back - forever, for the rest of the process. Fixed: `$SIM_GAIT` now
writes into `cmpc_gait` itself, ONCE, on `firstRun` - it still seeds the
initial gait for a sweep without editing the yaml, but no longer fights a
write that comes after boot.

**What this does and does NOT invalidate.** Any mission that only sets the
gait ONCE at launch (star, atom, dash, every pronk/gallop/bound base-gait
test in this file) was NEVER affected - the old code's "override every tick"
and the new code's "seed once" are behaviourally identical when nothing ever
calls `.set()` on `cmpc_gait` again after boot. The only mechanism this ever
broke is `WP_ANALYZER=1`'s mid-course gait switching, which only oval's
recipe uses. Every prior star/atom gait claim in this file stands. Oval's
own measured `-19.5%` analyzer win also stands - that comes from
`WP_VSUS`, a pure speed cap that never touches `cmpc_gait` at all - but any
claim in this file that the oval's `9 -> 5 -> 9 -> 5` gait changes
themselves "fired" was checking the analyzer's own planning-intent print,
not real robot behaviour, and should be treated as unverified until re-shown
against ground truth.

**Verified against real ground truth, not the misleading signal.**
`GaitScheduler::createGait()`'s `"[GAIT] Transitioning gait from X to Y"`
print - what this investigation originally chased - comes from a separate
class `ConvexMPCLocomotion` never consults for its own `Gait*` selection
(that happens directly off `gaitNumber`, a plain member-variable switch).
Added a real ground-truth line at the actual selection site instead
(`applySchedule()`'s existing `lastGaitSeen` tracking, which had no log of
its own): `"[SCHED] gait changed A -> B"`. Live oval re-test after the fix:

```
[SCHED] gait changed 5 -> 4     (entry hold, standing)
[SCHED] gait changed 4 -> 5     (engage trotRunning)
[SCHED] gait changed 5 -> 9     (analyzer: entering the sustained curve)
[SCHED] gait changed 9 -> 5     (analyzer: entering the straight)
```
Matches the analyzer's plan exactly - the mechanism now genuinely works.

**A real, NEW failure mode this exposed, not previously observable because
the switch never actually happened before**: the same run fell
(`roll=0 pitch=7 z=0.058`) a few hundred ms after the `9 -> 5` switch -
i.e. switching INTO trotRunning right as the robot exits the sustained
curve, likely still carrying yaw/lateral momentum from the turn.
`applySchedule()` already has a 500 ms "do not stack a segment-clock change
on a gait change" guard, and it evidently is not sufficient for this
specific transition. NOT fixed this session (out of the current priority
order: pronk/gallop tests and max dash speed come first, per direct
instruction) - flagged here as a genuine, freshly-exposed bug for whoever
next touches oval's analyzer path, and it is directly relevant to the
per-gait/per-angle cornering-envelope stretch goal further down this file's
history, since "how much momentum can a gait switch tolerate mid-turn" is
exactly that question.

## Stale bridge/controller ports: now guarded automatically, at both ends

The "frozen roll=-3.14159 exactly, every run, until a stale pid on 9100 was
found by hand" failure class (a `cheetah_gazebo_bridge.py` left running from
an earlier manual test, no `gz sim` behind it, silently answering a fresh
controller with stale sensor data) invalidated an entire pronking
speed-ladder sweep this session. Per direct instruction ("check for stale
bridge should ALWAYS happen either at bridge start, or test start, or
both"), fixed at both:
1. `cheetah_gazebo_bridge.py`'s `_clear_stale_port()` runs before its own
   `sock.bind()` - `lsof`s its own `CMD_PORT`, kills whatever it finds.
2. `server.py`'s `launch()` sweeps every port pair for every dog slot about
   to launch, before building the fleet world - catches a stale
   `mit_ctrl_sim` on `SENSOR_PORT` too, which the bridge-side check alone
   cannot see, and covers the case (this session's actual failure) where
   the conductor server itself has been up for hours and a stray process
   from an unrelated manual test interferes with a later `/api/launch`.
Neither UDP socket sets `SO_REUSEADDR`, on purpose, so a stale occupant is
detected rather than silently shared - both fixes rely on that.

## PRONKING RE-TESTED ON THE CURRENT CONFIG: strong across corner-broken courses, cannot sustain a long straight

Per direct instruction, re-tested across the mission catalog now that the
gait-selection bug (above) and the stale-bridge bug are both fixed - this
is the first time pronking has been tested on the fully-fixed stack
(qpOASES, WBIC damping, real Go1 model, zeroVelHold, real gait selection).
`@0.6 m/s` unless noted, real estimator:

| mission | result |
|---|---|
| star:10.514:5 | **PASS 114.6s, 6/6 waypoints** |
| circle:9:8 | **PASS 52.3s, 8/8 waypoints** |
| expsquare:5:12 | **PASS 163.1s, 10/10 waypoints** |
| atom:9.0:6 | FELL at wp103/108 (96%), roll=40deg - the atom's own already-documented roll-limited cornering fragility, not a new pronking-specific bug |
| oval:40:5.0 (WP_ANALYZER=0) | inconclusive - still progressing at 32/93 when the harness's own timeout fired, not a fall |
| sector:15:3 | FELL early, 6/16 - sector's own tight (120-147deg) corners |
| parallel:30:5:8 | FELL after 211s, orientation trip |
| **dash:100** | **decays to a stall, does not complete, at every speed tried (0.6, 1.0)** |

Every one of star/circle/expsquare is a corner-broken course with no single
uninterrupted straight anywhere close to 100 m. This is a complete reversal
from every number previously recorded in this file for pronking (`<0.3 m at
every speed`, `no completion`) - all of it measured before the async-solve
race fix, the WBIC damping fix, the real Go1 model corrections, zeroVelHold,
and (this session) the SIM_GAIT fix, stacked together for the first time.

**The dash finding is the important one, and it is NOT a top-speed
ceiling.** Instrumented at 5 Hz N/t: pronking accelerates cleanly to
0.7-1.0+ m/s of ACTUAL ground speed for the first ~25-40 m (faster than
commanded, even), then the rate of progress decays continuously - not a
step down to a lower stable cruise, an asymptotic crawl toward zero -
converging to a near-stall around 33-40 m regardless of what speed is
commanded (0.6 and 1.0 both measured, same shape, same rough distance).
At 1.0 m/s the same mechanism produced an outright collapse instead of an
asymptotic stall (`[FALL] roll=0 pitch=0 z=0.037` - FLAT, not a tip-over,
the same signature this file already documents elsewhere for force/height
deficits, e.g. the star's 2.5 m/s corner failures).

**Height governor tested and RULED OUT as the cause**, despite
`HeightGovernor.h`'s own documented history of exactly this failure shape
for trotRunning (a flight gait's large natural bob spiking the departure
signal past the derate threshold, "fell at 33.9 m with scale pinned to
0.68"). Interleaved same-run A/B, both at 0.6 m/s: `CTRL_HGOV=1` (stock)
stalled at N=35.06 m by t=148s; `CTRL_HGOV=0` stalled at N=33.92 m by
t=143s - statistically the same stall, at the same rough distance, same
rough time, with the governor's speed-derate lever completely removed.
Whatever is decaying pronking's forward progress over a long straight is
upstream of the governor, not caused by it.

**Working hypothesis, not yet confirmed**: pronking is the one gait here
with ALL FOUR legs synchronized (offsets (0,0,0,0)), so every single gait
cycle has a genuine all-airborne flight phase with zero ground support -
if each cycle bleeds even a few mm of height (force asymmetry, a swing
that lands a hair short, whatever), a course with NO corners to force a
re-settle lets that loss compound cycle after cycle until it crosses the
collapse threshold, while every corner-broken course in the table above is
short enough between turns that the deficit never accumulates that far.
This would explain the whole table at once - corner-broken courses pass,
the one long uninterrupted straight does not - without needing atom's or
sector's own already-documented, unrelated cornering failures to explain
anything. NOT root-caused this session (out of priority order - this was
"determine real max dash speed," not "fix the dash"); the concrete next
step if resumed is to instrument body height (z) itself over the length
of a dash run and confirm whether it is monotonically declining, which
the governor's own `_hgov` state already tracks internally and could be
tapped for this specific diagnostic without adding new instrumentation.

**Honest answer to "real max speed in a 100m dash": there isn't one to
give as a single cruise number.** Pronking does not complete the dash at
any tested speed (0.6, 1.0) - it reaches its best distance (~35-40 m) in
the first 30-40 s regardless of commanded speed, then stalls or (at 1.0)
collapses. Contrast every OTHER gait in this file's dash tables, which
reach a genuine steady-state cruise and either complete or fail cleanly
at a describable ceiling. Pronking's real characterization is "very good
on courses with corners every 10-40 m, unreliable on anything longer and
straighter than that" - the opposite framing from a top-speed number.

## GALLOPING/BOUNDING RESOLVE ATTEMPTS: same DURATION pattern as pronking, different mechanism each time

Per direct instruction ("the pronk gallop resolve attempts... start there"),
re-tested on the current fully-fixed config, on the same two mission
shapes as pronking above (a corner-broken course and the uninterrupted
100 m dash), to see whether either now closes the gap this file has
called "the asymmetric-swing-sequencing gap."

**Corner-broken course: both confirmed working.** `galloping @0.8` on
`star:10.514:5` - **PASS 123.5s** (matches this file's own earlier
103.7s record within normal run-to-run variance). Not re-litigated
further; already established.

**The dash exposes a real, distinct failure for BOTH, and it is not
speed.** Tested `galloping`/`bounding` on `dash:100` at two speed tiers:

| gait | speed | result |
|---|---|---|
| galloping | 0.8 | FELL ~t=94s, orientation trip (tip-over) |
| bounding | 1.0 | FELL ~t=47s, orientation trip (tip-over) |
| galloping | 0.5 | did not fall - drifted to **N=-0.37 E=3.72** by t=254s (net BACKWARD and sideways from its own start point) |
| bounding | 0.6 | FELL ~t=163s, orientation trip (tip-over) - same failure, just slower to arrive |
| galloping | 0.4 | did not fall - drifted to **N=-5.54 E=4.12** by t=244s (worse: further backward, further sideways) |

Lowering speed did NOT fix bounding (still tips, just takes ~3.5x longer)
and did not "fix" galloping either - it just swapped a fast tip-over for a
slow, silent backward-and-sideways drift that never trips the orientation
check at all. **This is a DURATION/DISTANCE failure, not a speed
ceiling** - the same shape as pronking's dash finding above, but via
three DIFFERENT physical mechanisms across the three flight-adjacent
gaits tested tonight:

| gait | dash failure mode |
|---|---|
| pronking (synchronized, all 4 legs together) | flat height/force collapse (`roll=0 pitch=0`) |
| bounding (near-synchronized pairs) | orientation tip-over, delayed by lowering speed but not prevented |
| galloping (fully asymmetric offsets) | no trip at all - a silent, compounding BACKWARD+LATERAL positional drift |

None of these three courses (star/circle/expsquare/atom/sector/parallel)
is long and straight enough to expose any of this - every one of them is
short enough between corners, or has frequent-enough waypoint-tracking
corrections, that whatever is accumulating never gets the distance to
matter. A 100 m uninterrupted dash is the one course shape that lets a
small per-cycle bias compound into a real failure, regardless of which
of the three different mechanisms is actually producing that bias.

## THE runSwingLegControl/runContactLegControl PORT: NOT attempted, and why

Went back to `docs/LEGGED_SPORT_REVERSE.md` before touching any code,
per this project's own stated conclusion that these two functions were
"the leading remaining candidate" - and found the RE work itself already
draws the honest line:

- **`runContactLegControl`'s pseudocode IS fully reduced** (§7c) - and it
  is MIT's OWN existing stance branch, split into its own function, with
  exactly ONE non-trivial difference: a per-leg force sign flip
  (`f[0]` negated on the rear pair, `f[1]` negated on the right pair).
  The RE author's own words on that one new piece: "most likely reflects
  Unitree's leg-frame convention rather than a control improvement, and
  getting a force sign wrong per leg is exactly the kind of change that
  silently destroys a gait" - explicitly NOT ported, on purpose.
  Structurally splitting the function WITHOUT that flip would be a
  zero-behavior-change refactor - it would not touch tonight's dash
  failures at all, because it would produce bit-identical stance
  commands to what `ConvexMPCLocomotion::run()` already computes inline.
- **`runSwingLegControl`'s actual body was NEVER reduced to pseudocode.**
  Only its entry block (the swing/stance split test, and the per-leg
  `getCurrentSwingTime`/`getCurrentStanceTime` calls - both of which this
  port already has, confirmed independently) and a handful of constants
  (9.81, 0.07, three filter time-constant pairs, two unplaced antisymmetric
  per-leg arrays) were recovered. The doc's own words: "producing
  statement-level pseudocode for the remainder would be a large amount of
  work with a real risk of confident-looking errors, and speculative
  pseudocode in a reference document is worse than an acknowledged gap."

**Decision: did not write an original implementation and call it a "port"
of a function whose real content was never actually recovered.** Doing so
tonight, informed only by a guess at what a per-leg-aware Raibert
placement "should" look like, would carry exactly the risk the RE author
already flagged - a confidently-wrong swing-leg change is the single
easiest way to silently destroy a gait, and there is no way to
distinguish "fixed it" from "moved the bug" without the kind of careful,
isolated A/B this session's own history warns is easy to get wrong on a
first try (see the zero-velocity-hold false starts and the trot-in-place
settle reversal elsewhere in this file).

**What tonight's data actually adds, for whoever picks this up next**:
galloping's backward-and-sideways drift (not just a tip-over) is a
genuinely new, concrete data point - a directional, silent, compounding
bias is a different and MORE diagnostic signature than an eventual
orientation trip, because a trip could come from many things but a
consistent backward+lateral drift over a perfectly straight commanded
path points quite specifically at the swing foothold placement itself
computing a systematically wrong `Pf` for at least one leg under this
gait's asymmetric offsets. The concrete, low-risk next step is
instrumentation before any code change: log the swing-leg placement
`Pf` (already computed at ConvexMPCLocomotion.cpp:697-730) per leg
through a galloping dash and check whether one specific leg's realized
foothold is systematically short/long/off-axis relative to where the
Raibert formula intends it - that would confirm or rule out the
foothold-placement hypothesis with real evidence before any swing-leg
code is touched, exactly the kind of ground-truth check this file has
insisted on everywhere else.

**UPDATE, same session**: that instrumentation was built and run - see
"GALLOPING'S REAL CAUSE, CONFIRMED" further down. The per-leg-bias
hypothesis this section proposed turned out not to be a coherent one
(the Raibert correction is body-level, identical across legs by
construction, for any uniform-duration gait), but the same
instrumentation surfaced something more useful: the state estimator
itself appears to diverge hard from GPS truth under galloping. The
swing-leg port question is likely moot either way - a controller acting
on a wrong self-position would misbehave regardless of how correct its
foothold placement is.

## THE "corner:" MISSION: built for the cornering-envelope stretch goal, has an unresolved planner bug on wp0

Added `WaypointNav::makeCorner(leg_m, angle_deg, speed)` (commit
`5551e78`) for the stretch goal - an empirical per-gait, per-angle
cornering envelope. One real bug was found and fixed cleanly along the
way: `mission_opening_bearing_rad()` (`mission_geometry.py`) assumed
EVERY mission with >=2 waypoints now spawns ON wp0 (true for
circle/sector/parallel/expsquare, and - newly confirmed while chasing
this - star/atom/spiro too, all via `shiftFirstToOrigin()`), and so
computed the SDF spawn yaw and `WP_SPAWN_BEARING_DEG` from wp0->wp1's
bearing. `corner` deliberately does NOT shift (wp0 is ahead of true
spawn, matching dash's convention, so the approach leg is real and
measurable) - the function had no kind-based exclusion for that case,
only a length-based one dash happens to trigger by having just 1
waypoint. Result: `corner:25:45` spawned the dog facing 45 degrees (the
TURN ANGLE) instead of north, live-confirmed via the `[nav] ... heading
datum ... spawn bearing 45.0 deg` log line matching the angle parameter
exactly. Fixed with an explicit `if kind == "corner": return 0.0`.

**That fix was real and necessary, but did NOT resolve the actual
symptom.** Re-tested after the fix (spawn confirmed correct - `[nav]
corner mission: 25.0 m approach...` with no bad heading-datum line) and
the dog still overshoots wp0 (the corner vertex itself, not the exit
leg) and has to loop back around to re-approach it from the wrong
side - visible as N/E DECREASING toward wp0's coordinates with a
roughly-constant backward heading, i.e. genuinely correcting back
rather than progressing normally. The pre-planned corridor fillet at
this corner is generous (R=12.14 m, nothing like the star hairpin's
R=0.03-0.28 m that caused the historically-documented "elephant foot"
overshoot), so this does not look like the same steering-vs-traction
cap issue that fix addressed. Not root-caused tonight - the leading
guess, not verified, is something specific to wp0 being the FIRST
waypoint of the whole path with no preceding leg to fillet against,
which every other mission in the catalog never exercises (they all
either spawn on wp0 with a real leg already computed into wp1, or - for
dash - have only one waypoint and no corner at all).

**Decision: did not keep debugging this under time pressure.** Per
direct instruction to use judgement and keep moving on architectural
issues, pivoted the cornering-envelope work to the EXISTING, already
solid mission catalog's own natural corner angles instead of a novel,
still-broken primitive: `circle:9:8` (45 deg/vertex), `parallel`/
`expsquare` (90 deg), `sector:15:3` (120-147.5 deg, dominant 120),
`star:10.514:5` (144/162 deg). This sacrifices the "any angle in 5
degree notches" flexibility the dedicated mission would have given, but
delivers real data on infrastructure already proven not to fight itself
mid-corner. The `corner:` mission and its geometry/bbox/spawn-bearing
wiring are left in the tree (commits `5551e78` and this session's fix)
rather than reverted - the spawn-bearing fix is correct and worth
keeping regardless, and a future session chasing the wp0-overshoot bug
starts from a mission that is at least correctly oriented at spawn.

## CORNERING ENVELOPE (partial, first pass): walking2 on the star's 144-162 deg corners

Pivoted here (previous section) to the existing catalog's own angles
rather than the still-broken `corner:` mission. Time-boxed to a small,
genuinely useful first pass rather than the full 5-degree/all-gait sweep
originally scoped - walking2 had the least existing per-corner-angle data
of any working gait, so it went first.

**A 3-dog fleet result was caught and correctly discarded rather than
trusted.** `walking2 @1.0` on `circle:9:8` / `sector:15:3` /
`star:10.514:5` simultaneously: all three FELL within 1-2 seconds of each
other - the exact "identical simultaneous failure" shape this file has
flagged as host-suspect many times before. Checked properly this time
BEFORE writing anything down: `maxPeriod` was clean (2.75-3.00 ms) on all
three at the moment of failure, ruling out the usual control-loop-stall
signature - but the close-set timing across three different courses of
different lengths was still suspicious enough to demand a solo re-test
rather than being taken as three independent confirmations.

**Solo re-test: the failure is real, not a host artifact.** `walking2 @
1.0 m/s` on `star:10.514:5` ALONE fell 1 second after nav took the stick
- immediate, at the very first corner, reproducing the fleet result
exactly. `walking2 @ 0.6 m/s` on the same course, solo: 2 of 5 waypoints
reached cleanly, no fall, still progressing when the harness's own
timeout (130s, too tight for this course's real ~200s budget) ended the
script - a TIMEOUT verdict, not a FAIL, per this project's own harness
discipline.

| gait | course | corner angle | speed | result |
|---|---|---|---|---|
| walking2 | star:10.514:5 | 144/162 deg | 1.0 m/s | **FELL immediately, solo-confirmed** |
| walking2 | star:10.514:5 | 144/162 deg | 0.6 m/s | **no fall through 2/5 waypoints** (harness timeout, not a verdict) |
| walking2 | circle:9:8 | 45 deg | 1.0 m/s | **FELL immediately, solo** |
| walking2 | circle:9:8 | 45 deg | 0.6 m/s | **no fall through 3/8 waypoints** (harness timeout, not a verdict) |

**CORRECTION to the read given two paragraphs up**: the original 3-dog
fleet result (all three falling within 1-2s of each other) was flagged
as host-suspect and set aside pending a solo check - the right instinct,
but the conclusion drawn from the solo star check alone (treating it as
confirmation the fleet result was real, full stop) was still incomplete.
Testing circle - 45 degrees, nothing like the star's sharp corners -
shows the IDENTICAL immediate failure at 1.0 m/s, and the identical
clean pass through several waypoints at 0.6. **This is not a cornering
ceiling at all - it is a general walking2 speed ceiling around 1.0 m/s
that shows up on literally any course with real curvature**, gentle or
sharp alike, consistent with this file's own older dash-table finding
that walking2 already failed at 1.0 m/s in a straight line (`ms22`
config: "fails at 5.6 m"). The three-dog fleet's near-simultaneous
failure across three different course shapes was therefore genuine all
along, for a boring and unglamorous reason (all three independently hit
the same speed ceiling at roughly the same elapsed time) - not a host
artifact, but also not the angle-dependent result it was first framed
as. Left both readings in rather than silently editing the first one
out, because the correction - and how it was reached (test the gentlest
angle available, not just confirm the original suspicious data point) -
is the actual lesson.

**Scope, stated honestly**: this is ONE gait on ONE course shape at TWO
speeds, not the "every usable gait, angles in 5 degree notches" sweep
originally asked for. The `corner:` mission built for that sweep has an
unresolved bug (previous section); doing the full sweep properly needs
either that bug fixed or a lot more of this same manual, per-cell,
solo-when-suspicious methodology repeated across circle (45deg)/
parallel/expsquare (90deg)/sector (120-147.5deg)/star (144/162deg) for
every gait in the usable set (trotting, trotRunning, walking, walking2,
pacing, bounding, galloping, pronking - noting pronking/galloping/
bounding's OWN dash findings above already show their real constraint on
a straight is duration, not cornering, so their cornering ceiling is a
genuinely separate question from their dash one). Flagged here as the
concrete continuation point rather than claimed complete.

### walking2 on sector/parallel at 0.6 m/s: inconclusive, not chased further

Attempted to complete walking2's angle coverage (only 45/144-162 degrees
had data before tonight). Result was genuinely inconclusive rather than a
new finding: the parallel-course dog's orientation trip landed BEFORE nav
even took the stick (engagement-time, the same already-documented
coin-flip class of instability seen elsewhere for other gaits, not a
90-degree-cornering result), and the sector-course dog was still healthily
progressing (160s in, un-stalled) when the harness's own 260s timeout
ended the run - too tight for this course at walking2's low 0.6 m/s, not
a mission failure. Not re-run - walking2 is already well-characterized as
generally speed-limited (~1.0 m/s) independent of angle, so completing
its exact per-angle table is lower value than the coverage already
gathered tonight for the other seven gaits.

### PACING: a real methodological near-miss, caught in time

Attempted `pacing` on star (0.8, 0.5 m/s) and circle (0.8 m/s) as a
3-dog batch to check both angle- and speed-sensitivity at once. Two of
three fell within the same second, on DIFFERENT courses at DIFFERENT
speeds (`circle@0.8` and `star@0.5`) - which, taken at face value, would
have produced an incoherent, wrong finding (why would a gentler course
at a slower speed fail while the SAME gait on a harder course at a
HIGHER speed passed?). Checking timestamps against the boot sequence
before writing anything down: both failures hit `SAFETY CHECK FAILED`
during their own gait-ENGAGEMENT settle-hold, seconds before nav ever
took the stick - i.e. before course or commanded speed could possibly
be involved at all. A 3x repeat of the identical config
(`pacing@0.8/star`, same everything) reproduced this exactly: one dog
tripped in the same wall-clock second nav took the stick, two others
carried on without falling. Across 6 total attempts (this session):
roughly half failed at the gait-engagement/nav-handover transition,
independent of course or speed - a genuine, marginal, coin-flip-style
instability at that ONE transition, the same shape this file already
documents for bounding's own entry ("BIMODAL... roughly a coin flip"),
now shown for pacing too and localized more precisely (the handover
moment itself, not general locomotion).

**The lesson, worth stating plainly**: a same-second multi-dog failure
is not evidence about the mission being run - check WHEN in the
sequence it happened before attributing it to course geometry or
commanded speed, every time, not just when the pattern looks suspicious
on its face (walking2's genuinely-simultaneous failure earlier in this
file WAS about the mission; this one was not, and the only way to tell
them apart was to read the timestamps against the boot log both times).

**CORRECTION (2026-08-27, continuation session): the "~50% coin-flip" was
itself the host-load artifact this section's own lesson warns about.**
Per direct "hammer at all the things open" instruction, went back to
actually test this rather than leave it filed as an accepted, unexplained
residual. Every one of the six earlier attempts that produced the ~50%
figure was run as PART OF A 3-DOG BATCH - none were solo. Ran pacing on
star at the exact same 0.8 m/s, SOLO, four times: **4/4 clean passes, zero
engagement trips.** Also confirmed earlier tonight: 3-4/4 clean on a short
solo dash at the same speed. That is 7-8 consecutive solo passes against
a claimed ~50% failure rate - a coin flip landing heads 7-8 times running
is vanishingly unlikely (under 1% for 8 flips). The far more likely
explanation, consistent with this project's own repeated finding
elsewhere in this file ("identical simultaneous failure across
independent processes = the HOST, never the controller"): pacing's own
gait-engagement transition is NOT marginally unstable in isolation - the
~50% figure was measuring 3-dog SHARED-HOST contention during the exact
moment three controllers simultaneously hit their own settle-hold/
gait-engage transition together, not a property of pacing itself.
Bounding's own similarly-described "bimodal... roughly a coin flip" entry
instability (cited above as the same shape) has NOT been re-checked solo
yet and should not be assumed confirmed just because this one wasn't -
same next step, not yet done.

**Checked immediately after, same session: bounding's own bimodal claim
is also obsolete, though for a different reason than pacing's.** That
finding is genuinely old (from a session well before most of this file's
locomotion fixes) and described near-INSTANT collapses (3-4 cm of travel,
~18 s) alternating with clean ~5.4 m runs. Re-tested solo on the CURRENT
codebase: 4/4 short attempts showed real, healthy, sustained progress
(36 m by t=33s at a commanded 1.0 m/s, tracking the command closely) -
zero instant collapses. A full-length re-test DID eventually fail, but
via a completely different, already-characterized mechanism: a flat
force/height collapse (`roll=0 pitch=-0 z=0.037`) at 44.77 m, not an
engagement-time crash. So the specific OLD claim ("bimodal at
engagement") does not reproduce at all on the current build - it was
superseded by the many fixes since (WBIC damping, the real Go1 model,
zeroVelHold, the async-MPC-race fix, etc.), consistent with this
session's own repeated finding that "documented as marginal" often just
means "measured a long time ago, before the codebase that fixed it."
What DOES remain, and is a genuinely NEW, more precise finding: bounding
at 1.0 m/s does not reliably complete a 100 m dash, failing partway
through via the same flat force/height-starvation signature this file
already documents for OTHER gaits under sustained straight-line load -
a distance/duration ceiling, not an engagement coin-flip. Worth a proper
bracket (does it hold at 0.8? does the failure point move with speed?)
if this thread is picked up again, but that is a different, better-posed
question than the one this section originally raised.

### BOUNDING: clean at both angles tested, no confound this time

`bounding @1.0 m/s` on `star:10.514:5` and `circle:9:8` together: BOTH
PASSED cleanly (star 93.6s, circle 41.6s) - no engagement-phase trip,
no cornering trip, at either the sharp or the gentle angle. Consistent
with this file's own "BIMODAL at 1.0 m/s" finding for bounding
elsewhere (this pair of runs landed on the good side of that coin
flip), and a genuine, unconfounded positive data point: when bounding's
marginal entry succeeds, it handles both of the angle extremes tested
in this catalog without difficulty at this speed.

### GALLOPING: clean at both angles too - 2/2, matching the historical star record

`galloping @0.8 m/s` on `star:10.514:5` and `circle:9:8` together: BOTH
PASSED cleanly (star 122.4s, circle 55.6s) - star's time lands right in
this file's own previously-recorded 103.7-123.5s range for this exact
gait/speed/course, and circle is a new, clean confirmation at the
gentle end. Combined with the dash findings earlier in this file
(galloping fails a 100m straight via silent positional drift, not a
cornering or top-speed problem), galloping's overall picture is now
well characterized in this session: excellent on any course with
corners, structurally unable to sustain an uninterrupted straight.

### GALLOPING'S REAL CAUSE, CONFIRMED: the state estimator, not the swing leg

Followed through on the concrete next step recommended above (log
per-leg swing foothold `Pf`/`pfx_rel`/`pfy_rel` through a galloping dash)
rather than leaving it purely as a suggestion, and it took two attempts
to get right - both attempts, and what changed the conclusion, are worth
keeping.

**Attempt 1 had a real instrumentation bug and no real answer.** The
rate-limit counter (`static int nsw`, `if ((nsw++ % 20) == 0)`)
incremented once per (leg, tick) in the loop's fixed 4-leg-per-tick
order; since 20 is a multiple of 4, every sample landed on the same
phase of that cycle - all of it was `leg=0`, and the per-leg comparison
this existed to run was never actually possible from that data. Fixed
with a per-leg counter array and a real elapsed-time accumulator
(the first attempt logged `t=0.0` by mistake, losing the ability to
line results up against the nav layer's own timestamps).

**Attempt 2, with the fix, answers a DIFFERENT and more important
question than the one asked.** `pfx_rel`/`pfy_rel` (the Raibert
correction terms) turned out to be STRUCTURALLY IDENTICAL across all
four legs, confirmed numerically (mean `pfx_rel=0.00203`,
`pfy_rel=-0.00085`, to 5 decimal places, on every leg) - not a
coincidence, a property of the formula: it depends only on
`seResult.vWorld`, `seResult.position`, `_yaw_turn_rate` and
`stance_time` (identical across legs whenever the gait's durations are
uniform, true of galloping/pronking/bounding/trotting alike). **The
per-leg-Raibert-bias hypothesis this diagnostic was built to test was
never a coherent one to begin with** - only the NOMINAL hip-offset term
(`getHipLocation`/`side_sign`) and each leg's own `swingTimeRemaining`
vary by leg, not the correction itself. Recorded so nobody re-derives
this and re-builds the same diagnostic a second time.

**What the fixed data actually shows is the real finding.** `Pf[0]`
(tracks the state estimator's own body position within a few tens of
cm) climbs MONOTONICALLY and `vWorld[0]` stays CONSISTENTLY POSITIVE
for the entire logged run - `Pf[0]`: 10.8, 11.7, 12.5, 13.3, 14.0, 14.7,
15.3, 15.9, 16.7, 17.2, 17.9, 18.4, 19.1, 19.6, 20.2, 20.8 m, with
`vWorld[0]` never once going negative in this stretch (0.42, 0.22, 0.36,
0.19, 0.27, 0.22, 0.17, 0.17, 0.18, 0.20, 0.15, 0.26, 0.17, 0.23, 0.17,
0.21 m/s). Over the SAME real time window, the nav layer's own
GPS-derived readout - a completely independent position source, read
from the SAME log, this time actually comparable now that both carry
real timestamps - shows N **peaking at 10.81 m and then DECLINING**:
10.81, 10.54, 9.85, 9.05, 8.05, 6.96, 5.88 m.

**The controller's own belief about where it is, and how fast it is
going, is simply wrong - it thinks it is accelerating steadily forward
while GPS truth says it turned around.** This is not a swing-leg
placement bug at all: the controller has no reason to correct anything,
because its own state estimate says everything is working. It also
explains the failure signature better than a foothold bug would -
galloping's dash failure never trips the orientation safety check
(unlike bounding's), which is exactly what a mis-LOCALIZED but
attitude-stable robot would produce. If real (a single run, not yet
independently repeated), this would be a FAR larger drift than any
previously recorded in this file (worst case before now: 4.5 m over an
83 m walk) - consistent with, and a plausible root cause for, the
already-documented general concern that leg-odometry-based estimation
degrades under the kind of rapidly-changing, asymmetric contact pattern
an asymmetric gait like galloping produces (this file's own
contact-detection work found the estimator's KF trust ramp is tuned
around a graded phase signal that a symmetric gait's schedule already
represents well - an asymmetric gait's schedule is a much rougher input
to that same mechanism).

**CONFIRMED against actual Gazebo ground truth, same session, and it is
worse than the GPS comparison suggested.** Ran the identical config
(`galloping @0.5` on `dash:100`) with `$SIM_ESTERR=1` - truth logged
beside the estimate, never fed to it, exactly the mechanism this file
already uses elsewhere and RULE ZERO does not prohibit (cheater mode,
which feeds truth to the controller, is deleted entirely; this only
logs it). The divergence is real, large, and grows without bound for as
long as the run was observed:

| t (s) | truth position (world frame) | estimate position (estimator frame) | \|error\| |
|---|---|---|---|
| 19.5 | (-0.03, 0.47, 0.27) | (0.57, -0.01, 0.25) | 0.76 m |
| 51.0 | (0.01, 10.30, 0.25) | (12.91, -0.27, 0.23) | 16.7 m |
| 111.0 | (0.13, 8.91, 0.25) | (24.27, -1.10, 0.23) | 26.1 m |
| 171.0 | (0.96, 5.42, 0.27) | (34.28, -2.65, 0.25) | **34.3 m** |

(The two frames are rotated 90 degrees from each other by convention -
truth's own second axis is world NORTH, the estimate's first axis is
the estimator's own initial-heading-relative FORWARD - so "truth's
north" and "estimate's forward" are the columns to compare, and that is
exactly what the table above does.) Truth's own north position PEAKS
around 10.8 m near t=60s and then falls back to 5.4 m by t=171s -
matching the nav/GPS reading exactly, as it should since nav reads a
third independent source (GPS) that agrees with Gazebo truth, not the
estimate. The estimate, meanwhile, never stops climbing: from 0.57 m
at t=19.5s to 34.28 m at t=171s, a nearly linear runaway with no sign
of correcting itself. **This is an order of magnitude larger than any
previously documented estimator drift in this file** (worst case
before now: 4.5 m over an 83 m walk, for a symmetric gait).

This closes out the investigation for tonight with a confirmed, not
merely suggestive, answer: galloping's dash failure is a leg-odometry /
state-estimation failure, not a swing-leg placement or gait-control
problem. The likely mechanism (not yet verified further): the LinearKF
trusts each stance leg's kinematic velocity estimate according to a
phase-based ramp tuned against gaits with slow, predictable stance/swing
transitions - galloping's asymmetric, rapidly-changing per-leg contact
schedule is a much rougher input to that same mechanism, and if the
filter over-trusts a leg that is not actually bearing load the way the
schedule assumes, the resulting velocity bias integrates into exactly
this kind of unbounded position runaway. NOT yet repeated (this session's
own small-sample discipline still applies to the exact magnitude), but
the qualitative finding - and that it is a state-estimation problem, not
a control or swing-leg one - is solid. This also means the
runSwingLegControl/runContactLegControl port considered earlier in this
file was very likely never going to fix galloping's dash failure at all,
regardless of how it was implemented.

### Cornering-envelope tally, CONTINUED next session: trotRunning at flagship speed

Per direct re-instruction to prioritize this over the estimator/force-gate
thread. Filled the gap this file itself flagged ("trotting/trotRunning...
were the best-characterized gaits already" - true for their OWN courses at
their OWN speeds, but never tested on the SAR-pattern angles at trotRunning's
actual flagship speed, 3.5 m/s). 3-dog parallel batch, real mission (not a
synthetic corner probe):

| course | angle | trotRunning @ 3.5 m/s |
|---|---|---|
| circle:9:8 | 45 deg | **PASS 24.0s**, roll 0.6/pitch 0.1 at settle |
| sector:15:3 | 120-147.5 deg | **PASS 129.6s**, roll/pitch 0.1/0.1 at settle |
| parallel:30:5:8 | 90 deg | **FELL** ~196s after nav took the stick, orientation trip |

The parallel failure is NOT a new 90-degree-angle finding - this file
already documents parallel's specific weakness (a short 5m connector right
after a 30m straight is a genuine braking-distance problem, which is why
its own validated recipe runs at 1.5 m/s, not 3.5). Running it at 3.5 was a
deliberate push past its known-safe envelope, and it failed exactly where
predicted. Net new result: trotRunning corners CLEANLY at both 45 and
120-147.5 degrees at full flagship speed, on courses/angles it had never
actually been run against before tonight.

### Cornering-envelope tally, CONTINUED: trotting at 2.0 m/s, all three angle extremes clean

Same 3-dog-parallel method, trotting instead of trotRunning:

| course | angle | trotting @ 2.0 m/s |
|---|---|---|
| circle:9:8 | 45 deg | **PASS 25.4s** |
| sector:15:3 | 120-147.5 deg | **PASS 131.0s** |
| star:10.514:5 | 144/162 deg | **PASS 56.1s** |

3/3, zero falls. Combined with the trotRunning result above, both of this
port's two most flagship-relevant gaits now have confirmed clean cornering
across the full angle range this catalog can produce (45 through 162
degrees), each at its own established cruise speed. Next: push speed UP
past each gait's known cruise to find the actual per-angle ceiling (the
literal "how fast into X degrees before it spins out" the stretch goal
asks for), rather than only confirming pass/fail at one speed per gait.

### Cornering-envelope tally, CONTINUED: pushing trotting past its own known cruise to find the real ceiling

Trotting's own established star ceiling elsewhere in this file was
~2.0-2.5 m/s. Re-checked at 2.5 across all three angle extremes rather
than assuming the old number still holds on the current stack (post
gait-selection fix, WBIC damping, real Go1 model):

| course | angle | trotting @ 2.5 m/s |
|---|---|---|
| circle:9:8 | 45 deg | **PASS 24.9s**, roll 1.3 |
| star:10.514:5 | 144/162 deg | **PASS 61.8s**, roll 0.1 |
| sector:15:3 | 120-147.5 deg | **PASS 129.7s**, roll 0.2 |

3/3 again, no ceiling found yet at any angle. Continuing up to 3.0 m/s.

### Cornering-envelope tally, CONTINUED: trotting's real ceiling found, and it is ANGLE-DEPENDENT

Pushed to 3.0 m/s (bracketing the 2.5-PASS result above):

| course | angle | trotting @ 3.0 m/s | failure signature |
|---|---|---|---|
| circle:9:8 | 45 deg | **PASS 24.9s** | - |
| sector:15:3 | 120-147.5 deg | **FELL** | flat height collapse: `roll=1 pitch=0 z=0.040` |
| star:10.514:5 | 144/162 deg | **FELL** | genuine tip-over: `roll=139 pitch=0 z=-0.372` |

This is the literal thing asked for: trotting's cornering ceiling BRACKETS
to 2.5 m/s PASS / 3.0 m/s FAIL for any angle >=120 degrees, while the
gentle 45-degree corner tolerates at least 3.0 with no sign of strain
(roll stayed under 2 degrees at every speed tried, 2.0 through 3.0). And
the two tight-angle failures are not the same mechanism - sector fails via
the flat force/height-starvation signature already documented multiple
times elsewhere in this file, while star's sharper 144/162-degree vertex
genuinely rolls the robot over. Not narrowed further (e.g. 2.7/2.8) given
time - the bracket itself is the useful empirical answer, and coverage
across more gaits matters more than sharpening this one gait's exact
decimal.

### Cornering-envelope tally, CONTINUED: bounding/galloping/pronking on sector's 120-147.5 deg - the first data at this angle for all three

| gait | speed | sector:15:3 (16 waypoints, dense 120-147.5 deg corners) |
|---|---|---|
| bounding | 1.0 | **PASS 170.9s** - first confirmed clean run at this angle |
| galloping | 0.8 | **FELL at wp06/16** (~37%), flat collapse `roll=11 pitch=0 z=0.044` |
| pronking | 0.6 | **FELL at wp02/16** (~12%), `roll=-41 pitch=0 z=-0.001` |

Bounding adds a genuinely new clean data point. Galloping's and pronking's
falls are worth reading carefully rather than filed as "another cornering
failure": sector packs corners far more densely than any course either
gait has been tested on before (16 waypoints, 120-147.5 degrees, over a
161m course - tighter spacing than the atom's continuous-but-gentler
curve or the star's five widely-spaced corners). Galloping's failure
signature (flat, mild roll, height collapse) matches its already-confirmed
estimator-divergence mechanism from earlier tonight, plausibly compounded
by never getting a long enough straight recovery between corners to
resettle. Pronking falling after only 2 waypoints is the most concerning
data point of the three - much earlier than its established atom/star/
circle performance - and raises a real, not-yet-answered question of
whether it's the ANGLE, the CORNER DENSITY, or both. Next: same three
gaits against a gentler, more widely-spaced 90-degree course (parallel/
expsquare) to separate "angle" from "density" as the actual variable.

### Cornering-envelope tally, CONTINUED: a flawed test design, caught and corrected

Tried to isolate "angle" from "corner density" by testing bounding/
galloping/pronking on `expsquare:5:12` at their own established speeds,
on the assumption it was a gentler, more widely-spaced 90-degree course
than sector. That assumption was WRONG - an expanding square is a
SPIRAL, so its early legs are short with frequent, tight-radius turns,
not spaced out at all. Result: ALL THREE fell, around wp07-09 of the
course:

| gait | speed | expsquare:5:12 |
|---|---|---|
| bounding | 1.0 | **FELL wp09**, genuine tip-over `roll=-169 z=-0.019` |
| galloping | 0.8 | **FELL wp07**, flat collapse `roll=-0 z=0.035` |
| pronking | 0.6 | **FELL wp07**, genuine tip-over `roll=-140 z=0.462` |

Bounding failing here is a real, new finding on its own (it had been
clean at every other angle tested tonight - 45, 120-147.5, 144/162 -
so this is its first observed cornering failure). But the course choice
does not answer the angle-vs-density question it was meant to - it may
have just reproduced sector's density problem through a different
geometry rather than testing a genuinely gentler case. Re-running the
same three gaits against `parallel`, which is actually long straights
with occasional turns (not a spiral), to get a clean answer.

### Cornering-envelope tally, CONTINUED: the density hypothesis is REFUTED - it is a mid-range ANGLE band, not spacing

`parallel` is genuinely long straights (30m) with occasional, widely-spaced
90-degree turns - the clean test the expsquare attempt failed to be. All
three gaits STILL fell:

| gait | speed | parallel:30:5:8 |
|---|---|---|
| bounding | 1.0 | **FELL wp11**, flat collapse `roll=0 pitch=0 z=0.057` |
| galloping | 0.8 | **FELL wp08**, genuine tip-over `roll=-79 pitch=52 z=0.247` |
| pronking | 0.6 | **FELL wp09**, mild collapse `roll=18 z=-0.073` |

This refutes the density hypothesis outright - these are widely-spaced
turns with a full 30m straight to recover on between them, and all three
gaits still went down. Combined with tonight's earlier clean 2/2 passes
for bounding/galloping/pronking at BOTH 45 degrees (circle) and 144/162
degrees (star), the real pattern is not "tighter angle is harder" at all:

| gait | 45 deg | 90 deg | 120-147.5 deg | 144/162 deg |
|---|---|---|---|---|
| bounding | PASS | **FELL x2** | PASS | PASS |
| galloping | PASS | **FELL x2** | **FELL** | PASS |
| pronking | PASS | **FELL x2** | **FELL** | PASS |

All three flight-phase gaits are fine at the gentlest angle AND at the
sharpest angle tested, and struggle specifically in the MID-RANGE
(90-147.5 degrees). That is a genuinely surprising, non-monotonic result
worth taking at face value rather than forcing into a "tighter is worse"
story the data does not support. A plausible mechanism, not yet checked:
the follower's own pivot-vs-arc branch (documented elsewhere in this file
- it switches behavior based on whether the look-ahead target lands
behind the body's nose plane) may transition through exactly this
mid-range angle band differently than it handles either extreme, which
would make this a PLANNER/FOLLOWER artifact rather than a gait-dynamics
one - worth checking the raw nav log for `[follow] PIVOT fired` around
each of these falls before assuming it is the gait's fault at all.

**Checked immediately, REFUTED**: `grep -c "PIVOT fired"` on all three raw
ctrl logs returns 0 - the pivot branch never engaged in any of these three
runs. This is genuinely a gait-dynamics response to the 90-147.5 degree
angle band, not a planner/follower artifact. Real, unexplained, and
consistent across three independent flight-phase gaits - a legitimate
open finding for whoever picks up the deeper "why" next, not something to
keep guessing at tonight.

### Cornering-envelope tally, CONTINUED: walking closes out gait coverage, and it is far more robust - with one exception

| course | angle | walking @ 1.5 m/s |
|---|---|---|
| circle:9:8 | 45 deg | **PASS 30.5s** |
| parallel:30:5:8 | 90 deg | **PASS 184.2s** |
| sector:15:3 | 120-147.5 deg | **orientation ESTOP trip, zombie-stalled** (the already-documented "stuck dog" mode - motors cut, no clean fall, no clean pass, burns the timeout) |

Walking succeeds at exactly the two angles (45, 90) that broke every
flight-phase gait tonight, which is the expected result for a gait that
never leaves two-plus feet on the ground - but it is not immune, and its
own failure at 120-147.5 degrees is a softer one (an orientation trip
into a stall, not a fall) than the flight gaits' outright collapses/
tip-overs there.

**Reconfirmed on a second, independent course shape at the SAME 120-147.5
degree band**: walking @1.5 tested against expsquare (90 deg, **PASS
108.3s**) and parallel (90 deg, **PASS 184.2s**) - both clean, consistent
with the 90-degree result above - alongside sector (120-147.5 deg) again,
which again showed the orientation trip. Two independent 90-degree
courses now agree with each other, and sector's 120-147.5-degree result
is reproduced rather than a one-off - walking's mid-band softness at this
specific angle band looks real and repeatable, not noise.

**CORRECTION (2026-08-27, continuation session): also a multi-dog host
artifact, same pattern as pacing's.** Checked the batch this "again showed
the orientation trip" result actually came from: a 3-DOG BATCH
(expsquare+parallel+sector together), not a solo run - exactly the same
shape as the pacing coin-flip correction above. Ran walking on sector
solo, three times: **3/3 clean passes, zero trips.** Combined with the
earlier solo sector PASS reported elsewhere in this file, that is 4/4
clean solo runs against a claim that was never actually tested solo in
the first place. Walking's "mid-band softness at 120-147.5 degrees" does
not hold up - it was, like pacing's, very likely multi-dog shared-host
contention landing on whichever dog happened to be at a sensitive moment,
not a property of walking or of that angle. This does not undo the
BOUNDING/GALLOPING/PRONKING 90-147.5 degree finding higher up this file -
those failures were checked and reproduced SOLO from the start (that is
why that finding was trusted this far) - it only retracts walking's own,
never-solo-verified addition to it.

**Meta-lesson worth stating once, plainly, given it happened three times
in one night**: pacing's "~50% engagement coin-flip," bounding's "bimodal
at 1.0 m/s" (partially - that one turned out to be an obsolete-codebase
issue instead, a different but related trap), and now walking's
"120-147.5 degree mid-band softness" were ALL measured, at least in part,
inside multi-dog batches and treated as properties of the gait or the
course. Every one collapsed under solo re-testing. This project's own
documented rule ("identical simultaneous failure across independent
processes = the HOST, never the controller") was written for the
easy case - dogs failing in the SAME wall-clock second. These three were
subtler: failures scattered across DIFFERENT dogs, DIFFERENT seconds,
inside the SAME batch, which reads like independent evidence about
several different gaits/courses but may just be several controllers
sharing one contended host at different moments each. Going forward: any
"marginal"/"coin-flip"/"~N% failure rate" finding that was measured
inside a multi-dog batch and never independently re-run solo should be
treated as UNVERIFIED, not as evidence, until it is.

**Speed ceiling pushed on the passing angles**: 3/3 PASS at 2.0 m/s
(expsquare 26.0s, parallel 95.6s, octagon 160.1s - the wall-time spread
tracks each course's own length, not a stability difference). At 2.5,
**3/3 FELL** - flat collapses (`roll~0, z=0.036-0.078`), spread over 33s
wall-clock (not the identical-second host-artifact signature this file
has learned to distinguish elsewhere), following a clean-then-uniform-
fail pattern that reads as a genuine speed wall, not host noise. Walking's
cornering speed ceiling across 45/90-degree courses brackets to **2.0
PASS / 2.5 FAIL**.

## CORNERING ENVELOPE, CONSOLIDATED (2026-08-27 continuation session)

Full cross-gait x angle table, everything gathered across tonight's
batches (all real mission runs, not synthetic corner probes; "-" =
not tested tonight):

| gait | speed(s) tried | 45 deg (circle) | 90 deg (parallel/expsquare) | 120-147.5 deg (sector) | 144/162 deg (star) |
|---|---|---|---|---|---|
| trotting | 2.0-3.0 | PASS to >=3.0 | - | PASS to 2.5, FAIL 3.0 (flat collapse) | PASS to 2.5, FAIL 3.0 (tip-over) |
| trotRunning | 3.5 | PASS | FAIL* (parallel only - known braking-distance issue at this speed, not a new angle finding) | PASS | PASS (established) |
| walking | 1.5 | PASS | PASS | orientation trip -> zombie stall | - |
| walking2 | 1.0/0.6 | FAIL@1.0 general speed ceiling, not angle | - | - | FAIL@1.0 (same ceiling), PASS@0.6 |
| bounding | 1.0 | PASS | **FAIL x2** (expsquare + parallel) | PASS | PASS |
| galloping | 0.8 | PASS | **FAIL x2** (expsquare + parallel) | FAIL (wp06/16) | PASS |
| pronking | 0.6 | PASS | **FAIL x2** (expsquare + parallel) | FAIL (wp02/16) | PASS |

**Headline finding**: all three flight-phase gaits (bounding, galloping,
pronking) share a specific, reproducible weakness at 90-147.5 degrees
while being fine at both the gentle extreme (45) and the sharp extreme
(144/162). This is NOT explained by corner density (parallel is genuinely
long straights with occasional turns and still broke all three) and NOT
a planner/follower artifact (`[follow] PIVOT fired` count is 0 across
every one of these failing logs, checked directly). It is a real,
open, gait-dynamics question: something about a MID-RANGE direction
change specifically challenges a gait with a full-airborne phase, in a
way neither a very gentle turn nor a near-reversal does. Trotting (full
duty-cycle-appropriate stance, no flight phase) shows the more intuitive
monotonic pattern instead - fine at every angle up to a speed-dependent
ceiling that drops as the course's tightest angle increases.

Coverage still open: walking2 was not re-tested at 90/120-147.5 (already
established to be speed-limited around 1.0 m/s independent of angle, so
low priority); no gait has been pushed to find its OWN ceiling at 90 or
120-147.5 degrees the way trotting's was bracketed at 2.5/3.0 - only
pass/fail at each gait's own previously-established base speed.

**Refinement worth noting**: trotRunning ALSO has a real flight phase
(40% duty, per the gait table) and PASSED sector's 120-147.5 degree
corners cleanly (129.6s) where bounding/galloping/pronking all failed
there. The difference is not "has a flight phase" but likely which LEGS
are airborne together: trotRunning alternates DIAGONAL PAIRS (always
laterally balanced even mid-flight), while bounding/pronking synchronize
more legs at once and galloping's offsets are asymmetric. If this holds
up under more testing, the mid-band vulnerability may be specifically
about lateral support asymmetry during the airborne phase, not the
airborne phase itself - a sharper, more useful hypothesis than "flight
gaits are fragile in corners."

**Tested directly, REFUTED.** Pacing synchronizes LATERAL pairs (left
legs together, right legs together) - arguably worse lateral symmetry
during its stance/swing pattern than trot's diagonal pairs, which made it
the natural next test. First attempt was contaminated by pacing's own
already-documented ~50% gait-ENGAGEMENT coin-flip (the orientation trip
fired at 11:54:56, one second BEFORE "nav taking the stick" at 11:54:58 -
checked the timestamps before drawing any conclusion, same discipline
this file has needed before for this exact gait). Clean solo retry got
past engagement cleanly and reached wp13 of 16 (81% through sector's
dense 120-147.5 degree corners) with zero falls before the harness
timeout ended it, still healthy and progressing. So pacing - despite
weaker lateral symmetry than trotRunning's diagonal pairs - handles the
same angle range that broke bounding/galloping/pronking outright. The
lateral-support-symmetry hypothesis does not survive this test. The
mid-band vulnerability remains real and unexplained, but it is not
simply about which legs share the airborne phase - both explanations
tried so far (has-a-flight-phase, and now leg-sync symmetry) have been
directly tested and refuted rather than left as untested guesses.

### Naming correction, and a real circle test

`circle:R:N` with N=8 (used throughout the table above) is a regular
OCTAGON - 8 vertices, exactly 45 degrees of direction change at each one
by construction - not a smooth curve, and it should not have been
described as "circle" in the reporting above without that caveat. The
underlying generator (`WaypointNav::makeCircle`) is fine and correctly
named for what it is; the issue was calling an 8-gon a circle in prose.

Added a genuine test of continuous curvature at comparable severity:
`circle:9:36` (36 vertices, ~10 degrees each - functionally smooth at
this port's corridor/lookahead scale). Bounding, galloping and pronking
at their base speeds: **3/3 PASS** (46.7s / 62.7s / 57.3s), no falls.
This confirms the 90-147.5 degree finding above is specific to DISCRETE
sharp direction changes, not to sustained curvature of comparable
turning severity - the same three gaits that fail hard on sector's or
parallel's individual corners handle a continuously-curving path just
fine. Worth remembering when interpreting any future course built from
this catalog: "circle:R:8" (or any low point-count invocation) is a
polygon and should be described as one.

**Speed pushed on the smooth circle too**, same 3-dog method as trotting's
octagon bracket above:

| gait | base speed | +1 rung | result |
|---|---|---|---|
| bounding | 1.0 | 1.5 | **PASS 32.5s** |
| galloping | 0.8 | 1.1 | **PASS 46.5s** |
| pronking | 0.6 | 0.8 | **PASS 45.7s** |
| bounding | 1.0 | 2.0 | **PASS 26.1s** |
| galloping | 0.8 | 1.4 | **PASS 38.8s** |
| pronking | 0.6 | 1.0 | **PASS 39.0s** |

3/3 clean at BOTH higher rungs - none of these three has found its
smooth-curvature ceiling yet even at 1.75-2.5x their established base
speed, in sharp contrast to how quickly they broke on the DISCRETE
octagon/sector/parallel corners at or below base speed. The gap between
"continuous curvature: seemingly very forgiving, even well past normal
cruise" and "discrete sharp corners: fails even below base speed at
90-147.5 degrees" is now the sharpest, best-evidenced version of tonight's
headline finding. Not pushed further tonight (diminishing returns relative
to broadening coverage to the remaining gaits) - the qualitative point is
solid: whatever breaks bounding/galloping/pronking in a corner is a
property of the DISCRETE DIRECTION CHANGE itself, not of turning at speed
in general.

### `corner:` mission revisited: the ORIGINAL bug appears gone, a DIFFERENT one found in its place

Per the plan to fix `corner:`'s wp0-overshoot bug and unlock real 5-degree
notches, re-tested it live rather than reading code further. Result:
the previously-reported symptom ("dog overshoots the vertex and loops
back to re-approach from the wrong side") did NOT reproduce -
`corner:25:45` at trotting 1.5 tracked the 45-degree turn cleanly (a
brief heading transient through the corner, then locked onto the exit
leg's heading and closed distance steadily) and reached wp01 to within
0.24m. Most likely fixed as a side effect of the pivot-follower and
steering-cap work done earlier this session for other courses, not
independently verified before now.

**A different, real bug is what's left**: the mission reaches its final
waypoint still at full cruise speed (v pinned at 1.50 in the nav log with
no visible deceleration anywhere in the approach), then the end-of-mission
stop sequence tips the robot over (`roll=40.5 deg` at settle, judged FAIL).
This is the same CLASS of bug already found and fixed for oval/star
("RESOLVED: it was never the lie-down - it was the ARRIVAL", "steered
deceleration") - `corner:`'s own end-of-path stop registration
(`_path[n-1].v = v_min` when `_endStop` is set, confirmed present in
`BodyPathPlanner.h`) is not translating into a visible braking zone before
arrival on this specific 2-waypoint course, for a reason not yet isolated
(possibly the mission's short waypoint count exposing an edge case the
multi-corner missions never hit). NOT fixed tonight - deprioritized in
favor of continuing empirical coverage on the working catalog, since
5-degree notches are a nice-to-have and this stop-sequence bug class has
already needed real dedicated investigation elsewhere in this file. Left
as a concrete, scoped next step for whoever has time: instrument
`BodyPathPlanner::plan()`'s backward pass specifically for a 2-3-point
path and compare against a working 5+ point course.

### `corner:`'s stop bug, actually run to ground (2026-08-27, continuation session)

Per direct "hammer at all the things open" instruction - picked this
back up rather than leaving it at "deprioritized." Instrumented exactly
as the note above suggested: `$WP_PLAN_DBG=1` dumps `BodyPathPlanner`'s
own internal `_path` array after `plan()` runs. Result: the planner's
OWN speed profile is computed CORRECTLY - a clean, smooth ramp from
0.502 m/s down to `v_min` (0.250) over the path's last ~20 points, even
tested against an artificially tiny `a_lon_max=0.05` that should have
demanded a 22.5 m braking zone (nothing was wrong with the physics or
the backward pass this file's own earlier note suspected).

**The real bug: `use_planner` was never engaged.** `corner:` has no
established recipe, so none of tonight's tests ever passed `$WP_PLANNER=1`
- meaning EVERY earlier "no visible deceleration" observation was `nav`'s
own separate, simpler distance-based logic driving the whole time, never
touching `BodyPathPlanner` at all despite `plan()` being called (and
producing a perfectly good profile nobody was reading).

**With `$WP_PLANNER=1` set, a second real bug surfaced**: `follow()`
bails out (`return false`, leaving `nv` holding whatever it was on the
last tick the follower actually ran) once the tracked index comes within
2 points of the path's end - precisely the resampled points where
`_endStop`'s `v_min` and the backward pass's braking zone live. On this
course (497 resampled points from a 2-waypoint, 49.5 m mission) that is
a large enough fraction of the final approach that `nv` was still
reading close to cruise when the main loop exited into the end-of-mission
decel/settle code, which itself ramps down FROM `nv` - so the ramp
started from an unbraked value and the robot pitched over during/after
it (`roll=33-53, pitch=51-62` across several attempts).

**Two things tried to loosen `follow()`'s own margin, both regressed
into a DIFFERENT failure**, recorded so neither gets retried:
- No margin (`i >= _path.size()`): fixed the speed (visibly reached
  `v=0.25`) but the pure-pursuit lookahead ran out of path to aim at
  once `i` hit the literal last point, so the ALREADY-DOCUMENTED "pivot
  when target is behind the nose" logic (correct everywhere else) had
  nothing finite to converge toward and spun in place forever instead of
  arriving.
- One point of margin (`i + 1 >= _path.size()`): reverted almost
  completely to the original bug - this course's `v_min` region is
  concentrated in only the last handful of resampled points, so even one
  point of early handoff loses the critical part of the braking zone.

**The fix that actually shipped**: left `follow()`'s own margin exactly
as it was (`i + 2 >= _path.size()`, avoiding both regressions above) and
instead fixed the two CALLERS in `mit_sim_main.cpp` (the end-of-mission
decel ramp and the dash-interlude's matching one) to seed their ramp
from `std::min(nv, planner.plannedSpeed())` rather than `nv` alone.
`plannedSpeed()` reads `_path[_lastIdx].v`, and `_lastIdx` is updated
unconditionally at the TOP of `follow()`, before the bailout check - so
it stays valid and correctly reflects the braked tail speed even on
ticks where `follow()` itself returns false.

**Measured result: real, partial improvement, not a full fix.** The
outright violent tip-over is gone (`pitch` dropped from the 51-62 degree
range to 30.5 during the settle check, `-> BAD` but not a crash) and
`$SIM_AID`-style instrumentation confirms the ramp now genuinely starts
near zero (`nv=0.000, plannedSpeed=0.250` at the transition, both
correctly low). But the mission still does not cleanly PASS - the robot
is still pitching further than the mission's own "settle" gate allows,
which points at a DIFFERENT, deeper issue than either bug fixed tonight:
the ACTUAL, PHYSICAL deceleration happening in the last meter or two of
the real approach (not the commanded ramp AFTER arrival, which is now
provably fine) is still steeper than the body can track smoothly -
`follow()`'s own code comments this exact failure mode elsewhere ("the
body tracks a commanded deceleration at about 1.2 m/s^2... needs 3-6 m
of travel to comply") for CORNERS, and this course's braking zone
(~0.75-2.8 m, sized for a much gentler speed change) may simply be too
short for a 1.5 -> 0.25 m/s stop specifically, independent of any code
bug. Not chased further tonight - full regression suite re-run to
confirm neither fix broke anything else before committing (both changes
touch shared code every mission's end-of-run and dash-interlude stop
uses).

### Cornering-envelope tally, end of session

| gait | angle(s) tested | speed | result |
|---|---|---|---|
| walking2 | 45, 144/162 deg | 1.0 m/s | FELL both (general speed ceiling, not angle-specific) |
| walking2 | 45, 144/162 deg | 0.6 m/s | clean partial progress both, no fall |
| pacing | 45, 144/162 deg | 0.5-0.8 m/s | ~50% fail at gait-ENGAGEMENT (unrelated to angle/speed); the runs that got past engagement did not fall |
| bounding | 45, 144/162 deg | 1.0 m/s | PASS both (2/2) |
| galloping | 45, 144/162 deg | 0.8 m/s | PASS both (2/2), matching this file's own historical star record |

Five gaits given at least one real (angle- or engagement-)data point
tonight, out of the eight in the usable set - trotting, trotRunning,
and pronking's own cornering behaviour on these two angle extremes
specifically were not re-tested this session (pronking already has
star/circle PASSes recorded earlier in this file at 0.6 m/s; trotting/
trotRunning were the best-characterized gaits already, from the
extensive pre-existing star/oval/atom work this file documents). Still
not the "every gait, 5 degree notches, empirical max yaw rate" sweep
originally scoped - two real angle extremes (45 and 144/162 deg) per
gait, not a continuous curve - but every number above is genuine,
solo-or-fleet-verified data with confounds actively checked for and
either ruled out or correctly identified, not a guess.

## SESSION SUMMARY (2026-08-27, autonomous overnight run)

Everything in this file from "THE REAL GAIT-SELECTION BUG" down was one
continuous autonomous session, working through a priority order set in
advance: fix what's blocking valid measurement, re-test pronking/gallop/
bound on the current stack, decide on the swing-leg-control port, then
the cornering-envelope stretch goal. In order of what actually shipped:

1. **Fixed a real gait-selection bug**: `$SIM_GAIT` was unconditionally
   overwriting `gaitNumber` every tick, silently discarding every runtime
   `cmpc_gait.set()` write - the exact mechanism the mission analyzer's
   mid-course gait switching depends on. Added ground-truth `[SCHED] gait
   changed A -> B` logging at the real selection site (not
   `GaitScheduler`'s disconnected print) to verify the fix live.
2. **Fixed the stale bridge/controller port bug at both ends** (bridge
   self-check at its own startup, launch-time port sweep in `server.py`)
   per direct instruction, after it silently corrupted an entire pronking
   speed-ladder sweep earlier in the session.
3. **Re-characterized pronking, galloping, and bounding** on the fully-
   fixed stack (qpOASES, WBIC damping, real Go1 model, zeroVelHold, the
   gait fix above) - a complete reversal from every number previously in
   this file for pronking, and a durable, well-evidenced new finding for
   all three: none of them is limited by top speed on a corner-broken
   course (all pass star; pronking/circle/expsquare also pass), and all
   three instead fail a 100m uninterrupted dash via three DIFFERENT
   mechanisms (height collapse / delayed tip-over / silent positional
   drift) - a duration/distance effect, not a speed ceiling.
4. **Investigated, then declined, the runSwingLegControl/
   runContactLegControl port** - read the actual RE documentation rather
   than assuming it was ready to use, found the part that would matter
   (`runSwingLegControl`'s body) was never reduced to pseudocode, and
   made the judgement call not to write speculative code and call it a
   port. Followed through on the recommended instrumentation-first
   alternative instead of leaving it as a suggestion, caught a real bug
   in the diagnostic itself on the first attempt (fixed it), and the
   corrected data answered a different and more important question than
   the one asked - see item 7.
5. **Built and partially debugged a new `corner:` mission** for the
   cornering-envelope stretch goal - found and fixed two real bugs
   (wrong spawn bearing, a general recipe-fallback crash), hit a third,
   deeper planner bug (wp0 overshoot) that was not resolved under time
   pressure, and made the call to pivot the envelope work onto the
   existing, proven mission catalog instead of continuing to debug a new
   primitive.
6. **Delivered two genuine cornering-envelope findings and caught a
   third result before it became a wrong claim.** Walking2 turned out to
   have a general ~1.0 m/s SPEED ceiling, not a cornering one - it fails
   just as fast on a gentle 45 deg circle as on the star's 144/162 deg
   corners, a correction only found by deliberately testing the gentlest
   angle available rather than stopping at the first confirming result.
   Pacing looked briefly like it had angle- and speed-dependent
   cornering behaviour (two dogs failing on different courses at
   different speeds in the same second); checking boot-sequence
   timestamps showed both failures actually hit during gait ENGAGEMENT,
   before nav ever took the stick - a marginal, coin-flip-style entry
   instability (roughly 50% across 6 attempts) with nothing to do with
   the course or commanded speed, the same shape already documented here
   for bounding's own entry. Bounding and galloping, tested cleanly at
   both angle extremes (45 and 144/162 deg) with no confound, both
   PASSED 2/2 - galloping's star time matched this file's own existing
   historical record exactly. Five of the eight usable gaits now have at
   least one real cornering data point from tonight. Documented the
   honest scope gap against the original "every gait, 5 degree notches"
   ask - this remains a partial, first-pass characterization (two angle
   extremes per gait, not a continuous sweep).
7. **Found and CONFIRMED (against Gazebo ground truth) galloping's real
   dash failure mechanism: the state estimator, not the swing leg.**
   The per-leg-Raibert-bias hypothesis this diagnostic was built to test
   turned out incoherent by construction (that correction term is
   body-level, identical across legs for any uniform-duration gait) -
   but the same instrumentation showed the state estimator's own
   position runs away to 34+ meters of error against Gazebo truth over
   a 171 s galloping dash, an order of magnitude past any previously
   documented drift in this file, while truth itself shows the robot
   peaking at ~11 m and drifting back to ~5 m - exactly matching the
   nav layer's own independent GPS reading. A controller acting on a
   position belief that wrong has no reason to correct anything, which
   explains the "no orientation trip, just silent drift" signature far
   better than a foothold-placement bug would, and means the
   runSwingLegControl port from item 4 almost certainly would not have
   fixed this regardless of how it had been implemented.

Every numbered item above has its own detailed section earlier in this
file with the actual data, the code changes, and (where relevant) the
git commit it shipped in. Nothing here is a new claim - this is the
index.

## CORRECTION, same night: the dash failure is NOT gait-specific, and NOT something this session introduced

> **CONFIRMED AND ROOT-CAUSED - see the `x_comp_integral` windup section
> at the end of this file.** Both of this section's central claims held
> up exactly: the failure is NOT gait-specific (it is in
> `ConvexMPCLocomotion`, which every gait shares) and was NOT introduced
> by that session (the offending line dates to the 2019 MIT import,
> `git log -S` -> `c54e50b`). Its "one general vulnerability, not a
> separate mechanism per gait" reading was right too - just about the
> wrong subsystem: the shared mechanism is a control-side integrator
> windup, not the LinearKF. The KF velocity-covariance collapse is real
> and independently measured but SECONDARY. One more thing this section
> got right and is worth repeating: the `[ESTERR]` `dp` field conflates
> two differently-rotated frames and overstates error - always compare
> truth's north axis against the estimate's forward axis by hand.

Directly challenged ("you just flat out broke the dash via some weird
regression") - the right response was to test the claim, not defend the
earlier writeup. It led to a real finding this file's own "three
different mechanisms per gait" framing above was wrong about.

**The A/B that settles it.** trotRunning has an extensive, repeatedly-
confirmed prior record of completing the standalone `dash:100` cleanly
(186.1s at 0.6 m/s is the number quoted earlier in this file). Re-run
tonight at the same speed: it walks BACKWARD past its own start point,
reaching N=-19.7 m by t=173s, with the control loop clean throughout
(maxPeriod 2.48-2.49 ms, zero stalls) - the identical shape already
documented above for pronking/galloping/bounding. Two hypotheses were
live: (a) something in tonight's session broke it, or (b) the log was
contaminated by a stale process (see the very next section) and the
result was never real. Both were tested directly rather than argued:

1. **A genuine log-contamination bug was found and fixed** (next
   section) - but the CLEAN, verified-single-launch re-test still showed
   the identical backward-walk.
2. **The decisive test**: swapped `ConvexMPCLocomotion.cpp` back to its
   EXACT pre-session state (commit `29cd8db`, the parent of every commit
   this session touched in that file) and re-ran the identical clean
   test. The baseline showed the SAME failure - N peaking at 18.24 m at
   t=45.5s (current-session code: 18.23 m at t=35-46s, statistically the
   same run) before declining. **This is conclusive: none of tonight's
   code changes caused this.** It is a pre-existing behavior of the
   codebase that predates this entire session, and the "186.1s" number
   was never re-verified with a clean, long-timeout, watched-the-whole-
   trajectory test after whatever combination of the many later fixes in
   this file (WBIC damping, the real Go1 model, zeroVelHold, WBC
   decimation, gain changes) actually produced it. It was carried forward
   as fact across all of them without anyone re-running it.

**What the failure actually is, corrected from the "three mechanisms"
framing above**: re-running trotRunning's dash with `$SIM_ESTERR=1` (the
existing ground-truth-logged-but-never-fed-to-the-controller mechanism,
not cheater mode - see RULE ZERO) shows the estimate tracking truth
closely for the first ~35-40 s, then diverging - by t=97-99s the
estimate's own position has stalled near a fixed value while its own
velocity readout has become internally inconsistent with that (large,
noisy velocity samples against an almost-flat position trace, the
signature of a filter whose state has stopped being self-consistent),
while GROUND TRUTH shows the body's real, physical, BODY-FRAME forward
velocity has gone NEGATIVE (-0.3 to -0.35 m/s) - the robot is genuinely,
physically walking backward, not just misreporting its position. This
is the same general shape as galloping's own confirmed divergence
earlier in this file, just with a later onset and a different local
signature (stall+noise vs. runaway) - strong evidence this is ONE
general vulnerability in the existing `LinearKFPositionVelocityEstimator`
under sustained, long-duration locomotion, not a separate mechanism per
gait as first framed. Correcting the record: the "flat height collapse /
delayed tip-over / silent positional drift" framing earlier in this file
undersold how related these probably are - all three may trace back to
the same estimator eventually losing self-consistency given enough time,
manifesting differently depending on which state (height for pronking,
position for galloping/trotRunning) the resulting bad feedback disturbs
first. **Also worth being honest about the `[ESTERR]` metric itself**:
its printed `dp` field computes `(estimate.position - truth.position).norm()`
without first rotating one into the other's frame (truth is world ENU;
the estimate is in the estimator's own initial-heading-relative frame,
rotated ~90 degrees from world) - so `dp`'s raw NUMBER overstates the
true along-track error by conflating swapped axes. The earlier-reported
"34.3 m" galloping divergence is still real and still large by the
CORRECT frame-aware comparison (truth's north axis vs. the estimate's
forward axis: ~29 m at t=171s), but the exact figure quoted there was
inflated by this frame mismatch and should be read as "roughly 29 m,"
not literally 34.3.

**Not yet done**: root-causing WHY the LinearKF loses self-consistency
after ~35-90s of sustained locomotion (leading candidate, per this
file's own prior contact-detection work: the phase-based stance-leg
trust ramp degrading under long-duration integration, independent of
gait - needs the actual estimator source read and a proper derivation
before touching it, not a guess). This is now understood to be the
single most consequential open item in this file - it plausibly explains
every "gait can't finish a long course" finding from tonight at once,
and likely several already-suspect historical dash numbers besides
trotRunning's.

## A second stale-process bug found chasing the above: the tail-text log bridge

While investigating the dash regression, found that `shm_reaper.py
--tail-text`'s bridge process holds NO network port at all (it is a
pure file-tailing process) - so the launch-time port-based stale-process
guard added earlier tonight can never see it. The natural "done" and
`/api/stop` teardown paths both clear `self.procs`/`self.phase` to
idle/done IMMEDIATELY (inside the lock), then do `terminate()` ->
`sleep(1)` -> `kill()` on a background thread - so `/api/state` can
report the fleet as finished a full second or more before the old
tail-text reaper (and possibly the bridge/controller too) actually
receives its kill signal. A launch landing in that window races the old
reaper: it re-opens whatever file now sits at `ctrl_%d.log` on its next
0.2s poll (rather than holding one fd for its whole life) and keeps
appending the PREVIOUS run's text into the NEW run's fresh log.
Reproduced live: a trotRunning dash launched moments after a galloping
run's fall showed the galloping run's own `[SCHED] gait changed 22 -> 4`
tail-end appearing before the new mission's own `[nav] dash mission`
line ever printed - old and new content simply concatenate in file
order, no visible corruption, just silently wrong data feeding whatever
conclusion gets drawn from it. Fixed with a THIRD stale-process gate in
`launch()`, alongside the port-based one: kill any `shm_reaper.py
--tail-text <i>` process by command-line pattern (not port) for every
dog index about to launch. **Practical implication for any future rapid
back-to-back testing**: always let a fleet reach a state where its
processes are confirmed gone (`ps aux | grep mit_ctrl_sim` etc.), not
just `phase: idle`/`done`, before trusting the next run's log - or rely
on this new gate, but verify the log's own opening line
(`"[nav] ... mission:"` should appear exactly once) before trusting a
result that matters.

## Two panel bugs fixed from live operator feedback

1. **The slots panel only ever synced from the server's draft ONCE**, on
   initial page load (`let _synced = false` gating the whole adoption
   block). Anything that changed the server's draft afterward - this
   session's own `mission_runner.py`/curl calls, another browser tab -
   never appeared without a manual page refresh (which re-runs the
   one-time branch via a fresh page load). Fixed to re-sync on every poll
   tick whenever the incoming draft actually differs from what is
   showing, guarded against clobbering an in-progress edit by skipping
   the sync entirely while focus is inside a slot control.
2. **The "not this course's validated combo" mismatch warning was
   browser-only and easy to miss** - a silent `<p>` on a slot card,
   invisible to anything driving the panel via the REST API and easy to
   overlook even when watching the page (this is the exact class of gap
   that let the atom-spin-out incident earlier in this file happen
   silently). The identical comparison now also runs server-side, in
   `launch()`, and logs a `dog%d: NOT this course's validated combo -
   running X @ Y, recipe is Z @ W` line into the orchestration log the
   moment a mismatched config is about to launch - visible in the live
   panel, in any `mission_runner.py` stream, and in the archived log
   file, not just a UI element that can go unwatched.

## The backward-walk investigation, continued: two hypotheses tested and REFUTED, fault localized but not found

> **SOLVED - see the `x_comp_integral` windup section at the end of this
> file.** This section's own closing instruction ("log
> `world_position_desired` itself and compare its drift against the
> achieved trajectory") was followed and led to the answer, so the
> narrowing below did its job: the fault really was downstream of a
> demonstrably correct command, and really was not numerical breakdown or
> MIT's covariance suppression (both refutations still stand and are
> still worth not re-testing). What it was: `x_comp_integral`, a
> never-reset integrator in stock MIT's `ConvexMPCLocomotion`, winding up
> until the MPC actively commanded ~1x bodyweight of BACKWARD force. One
> correction to this section's framing, though: it says "the fault is
> most likely in the MPC's own internal trajectory reference
> (`world_position_desired`)" - close, but the reference itself was fine.
> Disabling its `max_pos_error` clamp entirely was tested and did NOT fix
> the decay. The bug was one layer further in, in the MPC's own
> LINEARIZED DYNAMICS MODEL (`A(11,9) = x_drag`), not in its reference.

Per the control-math-verification discipline this session was pointed
at ("don't oversell, verify or say so" - report a contradicted
hypothesis honestly rather than re-explain it away): read the actual
`LinearKFPositionVelocityEstimator` source
(`common/src/Controllers/PositionVelocityEstimator.cpp`) rather than
guessing, formed two concrete hypotheses about it, and tested both
directly.

**Hypothesis 1, numerical breakdown - REFUTED.** The filter uses the
simple (non-Joseph-form) covariance update in single-precision `float`
for tens of thousands of ticks per run - a plausible setup for `_P` to
lose positive-semi-definiteness to round-off. Added a `$SIM_KF_HEALTH=1`
diagnostic checking `_P`'s diagonal for negative entries (objectively
impossible for a valid covariance, not a judgment call) plus trace/
determinant tracking. Result: no negative diagonal ever appeared. What
it found instead was unexpected - `_P`'s trace collapses from its
initial ~1800 down to ~0.005-0.02 almost immediately and stays there,
i.e. the filter becomes extremely (over)confident very early, not
numerically unstable.

**Hypothesis 2, MIT's covariance-suppression hack starving the
leg-odometry Kalman gain - REFUTED.** The collapsed covariance from
Hypothesis 1 pointed at `_P.block(0,0,2,2) /= 10` (applied every tick,
already documented elsewhere in this file for a DIFFERENT reason - it
is what makes GPS aiding inert) as a plausible cause: if it also starves
the ALWAYS-AVAILABLE leg-odometry correction, not just GPS, the estimate
would drift as near-uncorrected IMU dead-reckoning over a long enough
run. Directly testable with the EXISTING `$SIM_KF_UNCAP=1` flag (already
in the code, already used for the GPS question). Result: identical
failure, same peak-then-reverse shape, all the way to N=-20.94m by
t=170s. This matches this file's own older, already-recorded verdict on
that flag ("solved a problem that does not exist") - which should have
been weighted more heavily before re-deriving a new theory against it.

**What DID come out of tonight's testing: the command path is completely
clean.** Using the existing `$STM32MP1_EST_DBG=1` diagnostic (already in
the code, not written for this): `xcmd`/`xdes` (the velocity actually
handed to the MPC, at every stage: pad -> stick -> xcmd -> xdes),
`yawrate`, and the body-height reference all stayed rock-steady at
0.600 m/s / ~0 rad/s / 0.300 m for the ENTIRE run, including well past
the point where ground truth shows the robot already reversing. This
rules out nav, the command smoothing filter, the height governor's
speed-scale, and the sprawl guard's speed-scale all at once - none of
them touch `_x_vel_des` at any point in this failure. **The fault is
downstream of a demonstrably correct command**, most likely in the MPC's
own internal trajectory reference (`world_position_desired`, which
integrates the velocity command into a FUTURE position target the cost
function tracks - not logged tonight) or in force delivery not matching
what was solved for, not in anything upstream of the MPC.

**Honestly, not fixed.** Two well-reasoned hypotheses were tested and
died; the failure is narrowed considerably (clean command in, wrong
physical motion out, no numerical or covariance-suppression cause) but
not root-caused. **Concrete next step, for whoever picks this up**: log
`world_position_desired` itself over the course of a long dash and
compare its drift against the achieved (ground-truth) trajectory - if
that internal reference diverges from where the robot actually needs to
go, faster than the visible command itself would predict, that is the
next thread to pull. This is a more valuable place to end an
investigation than a fix built on an unverified guess would have been.

## THE ATOM CORNERING FIX: raised the roll WBIC gain, actually tested this time

Per direct instruction ("it is is known fragile... fix it") rather than
cataloging the atom's roll-limited cornering fragility again. The
IJRR paper read tonight (Park/Wensing/Kim, "High-Speed Bounding with
the MIT Cheetah 2") gives the real hardware's lateral/roll stabilization
law directly: `tau_x = -k_psi^P * psi - k_psi^D * psi_dot` - a plain
roll-angle PD producing a correcting torque, structurally the SAME
mechanism this port's WBIC already uses (`Kp_ori`/`Kd_ori` feeding the
whole-body QP). Checking the deployed gain: `Kp_ori: [40, 70, 70]` -
the ROLL axis (index 0) is weaker than pitch/yaw for no principled
reason found in either the literature or this file's own history.
Raised to `[70, 70, 70]`, matching the other two axes.

**Tested against the exact failing case, not assumed.** The most
concrete, reproducible roll failure from tonight was pronking on the
atom at 0.6 m/s, which fell at wp103/108 (96%) with `roll=40deg`. With
the gain change: PASS at 138.4s. Repeated once more alongside a star
regression check (in case raising a WBIC gain globally destabilized
something else): atom PASS again (138.0s, matching the first run to
within 0.4s), star PASS 60.3s, no regression. 2/2 on the fix, 2/2 on the
regression check.

**Scope, stated honestly**: this is 2 repeats on ONE course/gait/speed
combination and ONE regression check, not the kind of large interleaved
A/B this file's own small-sample discipline usually insists on before
calling something proven - the atom's own documented history includes
bimodal/coin-flip behavior at exactly this kind of marginal boundary
(see bounding's 1.0 m/s coin-flip elsewhere in this file), so a false
confirmation from two lucky runs is a real risk this file has hit
before. Applied to both `host-run/` and `stm32mp1/deploy_pkg/` yaml
copies (matching the precedent from the earlier Kd_body/Kd_ori
increase), `config/` left untouched (already documented elsewhere as a
separately-drifted copy pending its own reconciliation). Worth a wider
sweep (more speeds, more repeats, the other flight gaits) before this
is trusted as a general fix rather than "helped in the one case
checked."

## A real audit gap, called out directly: the WBIC per-situation gains were flagged but never actually read

Earlier in this file, the note on Unitree's `_ParameterSetup` gain
symbols (`Kp_body_stance`, `Kp_body_running`, `Kp_ori_stairs`, etc.)
stops at "flagged as a genuine gap rather than a guess" - the symbol
NAMES were confirmed to exist, but the actual VALUES were never safely
read after a first attempt misread the array layout (assumed an 8-float
Kp+Kd block where the real layout is 4 floats, reading into the next
symbol's memory and producing nonsense). That gap sat unaddressed. The
binary is still present on this machine
(`/Users/kfinisterre/Desktop/Cheetah/pi/Unitree_latest/autostart/
sportMode/bin/Legged_sport`, and two more copies alongside it) and the
extraction tooling (`tools/reversing/`) still works. Continuing this
properly - disassembling `LocomotionCtrl<float>::_ParameterSetup`
itself to confirm each symbol's real size and read site before touching
any array - is the concrete next step, not a re-guess at the same
layout that already produced garbage once.

**Follow-up, same night**: actually attempted the disassembly. Confirmed
`_ParameterSetup`'s real layout (a loop over 3-float Vec3 copies from a
constant-pool base at `0x2dd250`, per-field byte offsets 0x390-0x440),
verified the read METHOD is sound (re-read the already-known-correct Q
weight vector at `0x2fe070` and got an exact match), then applied that
same method to the per-situation gain addresses and got garbage
(huge/denormal-exponent floats). Conclusion: these specific values are
NOT compiled-in `.rodata` constants - `_ParameterSetup` loads them from
a runtime config file at process start, consistent with what this
section already said. This is a genuine, confirmed dead end for static
analysis, not an unresolved gap anymore. What's now verified and usable
without the exact numbers: Unitree's per-situation gain STRUCTURE is
real, and this port's single fixed WBIC gain set is a plausible source
of marginal stability at the top of every gait's range - already acted
on via the atom `Kp_ori` fix above.

## The IMM-KF / disturbance-observer line of investigation: a real, testable idea, tried, parked unproven

Two more papers read this session, both on state estimation via contact
detection - Menner & Berntorp ("Simultaneous State Estimation and
Contact Detection for Legged Robots by Multiple-Model Kalman Filtering",
arXiv:2404.03444) and Bledt/Wensing/Ingersoll/Kim ("Contact Model Fusion
for Event-Based Locomotion in Unstructured Terrains", ICRA 2018). Both
independently make the same core point relevant to this port's own
unsolved backward-walk/estimator-divergence bug: MIT's `ContactEstimator`
is a pure schedule pass-through with no evidence check at all, while a
real hardware-validated system infers contact from PHYSICS (a
hypothetical foot force from joint torque via the Jacobian, checked for
physical validity - positive normal force, inside the friction cone)
and only THEN decides how much to trust the schedule.

**Implemented as `$SIM_FORCE_GATE=1`**
(`common/src/Controllers/PositionVelocityEstimator.cpp`), additive to
the existing phase-based trust ramp, NOT replacing it (learning directly
from why the earlier `$SIM_CONTACT_DETECT` attempt regressed things -
see that section above). Computes `f = J^-T * tauEstimate` per leg
(this port's sim applies commanded torque directly, so `tauEstimate` is
a faithful proxy for applied force, not just a command) and derates
trust only when the schedule says confidently mid-stance but the force
is not physically consistent with real load-bearing contact.

**First cut reacted to a single instantaneous reading and made things
worse, not better**: tested against galloping's dash (the one case with
a confirmed, ground-truth-verified root cause), the gated run fell
DURING GAIT ENGAGEMENT - before nav even took the stick - while the
identical run with the gate off did not. The Bledt/Wensing paper's own
data explains why: even their REAL momentum-based disturbance observer,
measuring genuinely applied torque on real hardware, is "a large amount
of noise... during swing" (4-8N RMS), and their whole reason for an
event-based FSM includes an explicit debounce delay "to prevent fleeting
contact from catastrophically affecting the robot's gait." A hard
single-tick threshold was never going to survive a high-dynamic
transient like gait engagement.

**Fixed with a debounce** (`$SIM_FORCE_GATE_DEBOUNCE_MS`, default 30ms) -
requires the invalidity to persist before touching trust at all. This
DID fix the immediate-engagement fall (re-test: nav took the stick and
ran ~105s before any fall, versus falling before nav ever engaged). But
the debounced version still fell earlier than the gate-off baseline (105s
vs. the baseline surviving to a 145s harness timeout still progressing,
no fall). That is ONE run each, on galloping - this port's own least
reliable, most marginal gait - so per this file's own repeatedly-learned
lesson, this is not evidence the gate is harmful, but it is also not
evidence it helped. **Parked here, unproven, not adopted**: the flag
exists, defaults OFF, changes nothing for any existing validated result,
and is a real, well-motivated lead for whoever has time to run the
proper interleaved A/B (several repeats each, gate on vs. off, on
galloping's dash specifically) rather than continuing to spend tonight's
limited remaining time on a gait this file has already established has
no real speed/mission value - the priority for the rest of tonight is
the cornering-envelope sweep at flagship speed, which was explicitly
re-requested twice.

## SOLVED: the backward-walk is `x_comp_integral` windup - a 2019 stock-MIT bug, and the dash now completes

**This is the root cause of the "commanded velocity constant, real
velocity decays toward zero and eventually goes negative" failure** that
this file has chased across several sections and attributed, in turn, to
the swing leg, the estimator, numerical breakdown, and MIT's covariance
suppression. All of those were tested and refuted (each is recorded
above and stays recorded - the refutations are real). The actual
mechanism, found by instrumenting the MPC's own commanded FORWARD force
rather than reasoning further about the estimator:

```
ConvexMPCLocomotion.cpp (stock MIT, unchanged since the 2019 import):
  if(vxy[0] > 0.3 || vxy[0] < -0.3)
    x_comp_integral += _parameters->cmpc_x_drag * pz_err * dtMPC / vxy[0];
```

`x_comp_integral` is a bare class member (`float x_comp_integral = 0;`,
ConvexMPCLocomotion.h:242), `+=`'d every qualifying MPC tick and **never
reset, clamped, or decayed anywhere in the tree** (confirmed by grepping
every reference). It divides a small, persistently-SIGNED height error
(`pz_err = z - _body_height`; a trotting body sits ~4-5 cm BELOW its
height reference the entire run - measured: z=0.246-0.252 against
zref=0.300) by the CURRENT forward velocity. So as velocity shrinks for
any reason, the SAME height error produces a LARGER increment. It feeds
`update_x_drag()` -> `SolverMPC.cpp`'s `ct_ss_mats()`: `A(11,9) = x_drag`
- a coupling term in the MPC's own linearized dynamics claiming forward
velocity (state 9) drives vertical velocity (state 11). Wound up
negative, the model believes moving forward costs height, and since the
QP weights z 250x more than vx (`Q = {...,2,2,50, 0,0,0.3, 0.2,...}`,
index 5 vs index 9) it sacrifices - then REVERSES - forward velocity to
protect a height prediction that was never real.

**Measured, the runaway is unmistakable** ($SIM_MPCZ extended to print
net commanded stance Fx, which is what finally isolated this - the
existing diagnostic only printed VERTICAL force):

| t | vx (est) | commanded net forward Fx |
|---|---|---|
| 6 s | +0.61 | **+21.9 N** |
| 20 s | +0.61 | -2.5 N |
| 36 s | +0.61 | -48.3 N |
| 46 s | +0.45 | -85.6 N |
| 55 s | +0.21 | **-110.2 N** |

The controller is not going passive - it is actively commanding ~1x
bodyweight of BACKWARD force. That is why "no environmental force and it
still drifts backward" looked absurd: nothing external was pushing it,
the controller was.

**Why v=0 station-keeping is immune, and why this hid for so long.**
The accumulation is gated on `|vxy[0]| > 0.3`, so a standing/trot-in-place
hold never accumulates at all - exactly matching the operator's own
observation that position hold is rock solid while cruise decays.

And the honest correction to a first, wrong explanation of the hiding:
"only a long uninterrupted straight exposes it" is NOT sufficient, since
this project HAS run the dash for days. Checking the archive settles it -
it is DURATION past ~35-60 s of sustained cruise, and the dashes that
were passing were **20 m**, finishing in ~23 s, well under the threshold:

```
20 m dashes:   4 runs, 4 completed
100 m dashes: 20 runs, 2 completed
```

Star/oval/atom are immune for a different reason: their frequent corners
and stops repeatedly drop speed through the 0.3 gate, so the integral
never gets an uninterrupted run at winding up.

**THE FIX, and the result.** `$CTRL_XDRAG_CLAMP=<value>` bounds
`|x_comp_integral|` (opt-in; unset = stock MIT, bit-for-bit unchanged).
Clamping rather than deleting preserves whatever short-timescale
compensation MIT intended while removing the unbounded part. At
`CTRL_XDRAG_CLAMP=1.0`, the identical previously-failing case:

    dash:100, trotRunning @0.6    PASS, t=149.6s
    settle z=0.288 roll=1.0 pitch=0.4 -> ok    laydown -> ok
    GROUND TRUTH (pT north): 99.82 m of 100 m actually travelled

Same timestamps that previously showed decay now hold vx=+0.60 with Fx
POSITIVE (+8 to +24 N). **This is the first verified 100 m dash
completion in this investigation**, and it is confirmed against Gazebo
truth via $SIM_ESTERR, not against the robot's own belief.

**Scope, stated honestly**: one gait, one speed, one course, one run so
far, plus a regression check. Per this file's own repeatedly-learned
small-sample discipline that is a strong, mechanism-backed result - the
mechanism is derived from the source and confirmed by a dose-response in
commanded force, not just an outcome flip - but it is NOT yet a
multi-rep proof, and the clamp is deliberately left OFF by default until
it has one.

**What this does NOT retract**: the KF velocity-covariance collapse
found the same night is real and independently measured ($SIM_KF_HEALTH:
the velocity block falls from ~0.025 to ~0.0007 within about one second
of any run, taking the effective Kalman gain on new leg-odometry
evidence from ~0.20 to ~0.007). `$SIM_KF_VFLOOR` floors it, and on its
own measurably delayed the failure (upright past t=313 s vs falling at
t=112 s) without preventing the physical reversal - consistent with it
being a real but SECONDARY contributor: it degrades the velocity signal
the windup then amplifies. Both remain opt-in; which (if either) belongs
on by default is a question for a proper interleaved A/B, not for
tonight.


## "Close final leg": some courses walked home, some just stopped where their math ran out

Operator-reported: "some missions never close their path to the home
point, but some do... leaves a final leg unclosed." Correct, and it split
cleanly along a line nobody had noticed - measured distance from home at
each course's LAST waypoint, before any change:

| closes itself | gap | leaves a leg undone | gap |
|---|---|---|---|
| lissajous:15:1:2 | 0.00 m | circle:9:8 | **6.89 m** |
| spiro:9.0:8 | 0.05 m | sector:15:3 | **15.00 m** |
| atom:9.0:6 | 0.43 m | expsquare:5:12 | **18.03 m** |
| oval:40:5.0 | 1.20 m | parallel:30:5:8 | **46.10 m** |

The left column closes by construction (periodic curves - they come back
to their own start), the right column stops wherever its generator's
parametric math happened to end. Nothing principled distinguished them;
`makeStar` had already been fixed to close itself explicitly for exactly
this reason ("closing belongs to the mission itself"), and that precedent
just never got generalised.

`WaypointNav::closeFinalLeg()` appends one waypoint at the local-frame
ORIGIN - where the dog actually stood when the GPS datum was taken, NOT
wp0, which for a course that never called `shiftFirstToOrigin` can be
somewhere else. Panel checkbox "Close final leg", **default ON** per
direct instruction, `$WP_CLOSE_LEG=0` to disable. Three deliberate
no-ops: fewer than 2 waypoints (a dash IS its final leg - closing it
would silently turn every dash into an out-and-back, a DIFFERENT mission
this file already distinguishes by name), already within 2 m of home, and
no room left in `_wp`. Runs BEFORE `appendDash` so the dash correctly
appends one sprint point to a now-closed course.

### A C++/Python mirror drift found by measuring, not by reading

Building the table above is what exposed it: `mission_geometry.py`'s star
mirror reported the last waypoint **20 m** from home when the C++ ends at
0. `WaypointNav::makeStar` appends a closing waypoint (`_wp[points] =
_wp[0]; _n = points + 1`); the Python mirror never did. So the panel drew
star as an OPEN 5-point path while the robot flew 6 and closed the
pentagram. Same drift class this file already documents for
`mission_viz.py` losing the atom and oval cases - and the same lesson:
these mirrors drift silently, and the cheap way to catch it is to compute
a scalar from both sides and compare, not to re-read the code.

### THE TRADEOFF, measured, not hidden: closing can create a sharp corner

Turning for home is a real turn, and on two courses it is a much sharper
one than anything else on the route. Sharpest interior angle, open vs
closed:

    expsquare:5:12   90.0 deg -> 33.7 deg   (at the new closing corner)
    parallel:30:5:8  90.0 deg -> 49.4 deg   (at the new closing corner)
    sector:15:3      32.5 deg -> 32.5 deg   (unchanged - already had one)
    circle:9:8      135.0 deg -> 135.0 deg  (unchanged)

The planner does the right thing with that (`tightest corner R=0.04m ->
0.05m/s` on the closed expsquare) but "the right thing" is braking to a
crawl for one corner, so a closed expsquare/parallel is slower than its
open version, and its own `WP_TURN_SOFT`/`WP_TURN_HARD` tuning was never
measured against a corner this sharp.

**Measured, so this is no longer an inference.** `expsquare:5:12` with
the box ticked: **PASS, t=121.3 s**, 11 waypoints, `reached wp10 (N=0.00
E=0.00) dist=1.48` - the dog genuinely walks home - settling clean
(roll 0.7, pitch 1.8) and lying down ok. Against this file's own
open-leg baseline of 109.0-109.2 s that is **+12 s, about 11%**, which
is the price of one hard-braked corner and a walk home, exactly as
predicted. So the sharp closing corner IS feasible at 33.7 deg (it was
predicted feasible by comparison with the star's 36 deg vertices, and
that prediction now has a measurement behind it) - it is a time cost,
not a stability cost. `parallel`'s 49.4 deg is the gentler of the two
and has not been re-measured; anyone re-tuning either course should
re-check with the box in the state they actually intend to run.


## THE HOST-LOAD BUDGET, computed up front - and course "complexity" is NOT the variable

Per direct instruction: "based on the complexity and length of the
mission you can preemptively calculate the load budget on the CPU /
simulator as part of pre-planning too if we are gonna imply things about
that host contention theory of yours. You could use that value as a
weight, and see if any specific combined weight triggers it."

That is the right correction to make. This file has repeatedly blamed
"host contention" for multi-dog failures AFTER the fact and
qualitatively, which is unfalsifiable as stated - and has been WRONG
about it at least three times in one night (the meta-lesson section
above: pacing's "~50% coin-flip", walking's "120-147.5 deg mid-band
softness", and part of bounding's "bimodal" all evaporated under solo
re-testing). A number computed BEFORE launch that predicts which
combinations should break is testable. A story told afterwards is not.

`stm32mp1/gazebo/conductor/load_budget.py`.

### The intuition was reasonable and the data refutes it

"I assume in context dash is the least complex" - checked against 120
archived ctrl logs, median control-loop period by mission kind:

| mission | runs | median period | mission | runs | median period |
|---|---|---|---|---|---|
| atom | 11 | **2.49 ms** | oval | 16 | **2.48 ms** |
| circle | 40 | **2.49 ms** | star | 16 | **2.48 ms** |
| dash | 28 | **2.48 ms** | corner | 9 | 2.94 ms (unvalidated) |

**Per-tick cost is FLAT across every validated mission kind** - identical
to two decimal places on courses whose waypoint counts differ by 100x
(dash 1, spiro 503, lissajous 606) and whose geometry ranges from a
straight line to a 902 m rosette. The architecture predicts exactly this
once stated plainly: the convex-MPC solve is a fixed-size QP (horizon 10,
12 decision variables) that knows nothing about course geometry;
`BodyPathPlanner::follow()`'s nearest-index search is forward-only from
`_lastIdx`, so amortised O(1) rather than O(path); its lookahead scan is
bounded by `Ld / resample_step`, a constant; and Gazebo's physics step is
per-DOG, not per-course.

So geometric complexity is not the load variable. **DURATION is**, and
the consequence inverts the intuition:

    dash:100 @0.6 m/s   392 dog-seconds  (157,612 control ticks)
    atom:9.0:6 @2.1     190 dog-seconds
    star @3.5           129 dog-seconds  ( 52,025 control ticks)

The simplest course in the catalog is the MOST expensive one, because it
is slow. "You could run more dashes than sectors" is therefore not
automatically a lighter fleet - at 0.6 m/s one dash outweighs three
stars. Worth stating plainly since it is the opposite of what the
qualitative version of the contention story would predict.

### What this changes about experiment design

Every multi-dog batch in this file confounded two variables at once:
how many dogs, and how much work each was doing. `--equal-load N` inverts
`dog_seconds()` to solve for the speed that makes each mission cost the
same, so a contention test can vary CONCURRENCY alone:

    equal-load fleet of 3, target 170 dog-seconds each
      dash:100        2.00 m/s
      star:10.514:5   2.00 m/s
      sector:15:3     3.27 m/s

Two quantities are reported and they answer different questions: total
dog-seconds is how much work the fleet asks for; PEAK CONCURRENCY is how
much lands at once, which is what a shared physics thread and a fixed
core count actually contend over. If a threshold exists it should track
the second. **Not yet run** - the model and the tool are built and the
per-tick flatness is measured, but the experiment itself (sweep
concurrency at equal per-dog load, look for a knee) is the concrete next
step, and until it runs "host contention" stays a hypothesis with a
number attached rather than a finding.


### The clamp is MIT's own treatment, not an invention - proof is 100 lines up

Worth recording because it settles whether the missing bound on
`x_comp_integral` was a deliberate design choice or an oversight, and it
is the latter. `rpy_int`, in the SAME function, is the identical pattern:

```cpp
if(fabs(v_robot[0]) > .2)                                  // divide-by-zero guard
  rpy_int[1] += dt*(_pitch_des - seResult.rpy[1])/v_robot[0];   // error / velocity
if(fabs(v_robot[1]) > 0.1)
  rpy_int[0] += dt*(_roll_des  - seResult.rpy[0])/v_robot[1];
rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);          // CLAMPED
rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);          // CLAMPED
```

An error integral divided by the current velocity, gated on a minimum
speed for exactly the same reason `x_comp_integral` is gated on
`|vxy[0]| > 0.3` - and MIT bounds it to +-0.25 **and** zeroes it in the
constructor (`rpy_int[0..2] = 0`, right beside `rpy_comp`).
`x_comp_integral`, written the same way two dozen lines further down, got
NEITHER. A systematic grep for accumulators in the control path found
exactly one unbounded one, and this is it.

So the fix is not a new idea imposed on MIT's controller - it is MIT's
own handling of this exact construct, applied to the single instance that
was missed.

**The missing RESET is a second, separate defect, and it bites this port
specifically.** `x_comp_integral` was zeroed nowhere at all, so whatever
it wound up to in one locomotion episode carried into the next - and this
port re-enters LOCOMOTION mid-mission for real, twice: the dash
interlude's stop/lie-down/stand-back-up, and every end-of-mission stop.
Now reset in the `firstRun` block beside `world_position_desired` and
`_yaw_des`, which are reset there for the same reason.


### The close-leg time cost, as a controlled A/B - and the recipe notes it invalidated

Operator-spotted from the panel: "did you update all the validated
combos with this new info?" No - the CLAUDE.md writeup went in but the
RECIPES `note` fields, which are what the panel actually displays, still
quoted leg-OPEN timings as though they were current. Fixed.

The suite happened to run the same three `circle:9:8` cases immediately
before and after `close_leg` was deployed - same binary otherwise, same
cases, back to back - which makes it a genuine controlled A/B rather
than a comparison across batches:

| case | leg open | leg closed | delta |
|---|---|---|---|
| circle @ trotting 2.5 | 58.5 s | 62.5 s | **+6.8 %** |
| circle @ bounding 1.0 | 76.6 s | 82.6 s | **+7.8 %** |
| circle @ galloping 0.8 | 90.6 s | 98.6 s | **+8.8 %** |
| expsquare @ walking 1.5 | 109.1 s | 121.3 s | **+11 %** |

Consistent, and the ordering is the physically sensible one: the cost is
the same walk home every time, so the SLOWER the gait, the larger the
fraction it adds. Nothing regressed - all four still pass.

**A note on the "not this course's validated combo" warning**, since it
fired on those runs and looked alarming: it was correct and expected.
The three octagon cases deliberately run `circle:9:8` at trotting 2.5 /
bounding 1.0 / galloping 0.8 to probe the CORNERING ENVELOPE; circle's
own validated recipe is walking @ 1.5. The warning is doing its job -
flagging that the launch is off-recipe - and the cases are off-recipe on
purpose. It is not evidence of a misconfiguration.


## EVERY GAIT NOW COMPLETES THE 100 m DASH - and the "three different mechanisms" reading was wrong

Direct instruction, from earlier in the session: *"'The dash still
doesn't complete' that is the dumbest shit ever. EVERY gait should at
least be able to do the dash."* With the `x_comp_integral` windup fixed
(`$CTRL_XDRAG_CLAMP=1.0` plus the firstRun reset), solo, one gait at a
time, real estimator, `dash:100`:

| gait | speed | before tonight | now |
|---|---|---|---|
| pronking | 0.6 | asymptotic stall ~33-40 m, **never completed at any speed** | **PASS 107.7 s** |
| galloping | 0.8 | silent backward+lateral drift, **never completed** | **PASS 114.3 s** |
| bounding | 1.0 | orientation tip-over ~47 s, **never completed** | **PASS 84.8 s** |
| walking | 1.5 | - | **PASS 60.5 s** |
| trotting | 2.0 | - | **PASS 46.1 s** |
| trotRunning | 0.6 | walked BACKWARD to N=-19.7 m | **PASS 149.6 s / 148.3 s** |
| pacing | 0.6 | - | **PASS 149.8 s** |
| walking2 | 0.5 | - | **PASS 235.1 s** |
| walking2 | 0.8 | - | FELL - its OWN ceiling, see below |

**8 of 8 gaits in the usable set now complete the 100 m dash.** That is
the whole of the instruction, satisfied: there is no longer a gait in
this port that cannot run 100 m in a straight line.

`walking2` is the one that needs a caveat, and it is not this bug. At
0.8 m/s it fell at **t=20.7 s having travelled 1.46 m**, with an
orientation ESTOP - far too early for the windup, which needs 35-60 s of
sustained cruise before it dominates, and with the wrong signature (an
attitude trip at a standstill, not a velocity decay). Dropped to 0.5 it
completes cleanly in 235.1 s. So this is `walking2`'s own long-documented
fragility/speed ceiling (this file has it failing on any course with real
curvature at 1.0, and partially at 0.6), unchanged and unrelated.

The `trotRunning` cell is the one deliberate repeat: 149.6 s and 148.3 s
on independent runs, a 0.9 % spread, which is this project's usual
precision when a mission genuinely works. And `walking2 @0.5`'s 235 s is
incidentally the longest single sustained cruise run in this table - well
past the windup threshold - so it is a further duration check on the fix
rather than just another pass.

**This retires the "three different mechanisms" framing in the
flight-gait sections above, and that correction matters more than the
pass table.** This file previously recorded, in good faith and with real
data behind each observation, that pronking/bounding/galloping each
failed the dash by a DIFFERENT physical mechanism - a flat height/force
collapse, a delayed tip-over, and a silent positional drift respectively
- and reasoned about them separately for a long time. They were ONE bug,
in `ConvexMPCLocomotion` which every gait shares, presenting differently
depending on which state the runaway drag term happened to disturb first
in each gait's own dynamics. Three "mechanisms" was three symptoms.

### The estimator finding: the measurement was right, the CAUSAL DIRECTION was backwards

The section above titled "GALLOPING'S REAL CAUSE, CONFIRMED: the state
estimator, not the swing leg" needs correcting, and precisely, because
the numbers in it are real and were verified against Gazebo truth. What
was measured: over a 171 s galloping dash the estimate ran to ~29-34 m
while ground truth showed the robot peaking at ~10.8 m and drifting back
to ~5.4 m. That happened. What was WRONG was concluding the estimator
was the cause.

Re-measured on the same gait and course, now that it passes:

    truth  pT north  99.92 m        estimate  pE forward  90.19 m
    velocity: vT 0.85-0.93 m/s      vE 0.72-0.92 m/s  (dvx -0.02..-0.16)

A ~10% under-read that scales with distance, with velocity tracked
closely - not a runaway. The old divergence was leg odometry faithfully
integrating legs that were cycling while the BODY was being commanded
backward and going nowhere: feet sweeping under a body that is not
advancing is, to leg odometry, forward travel. The estimator was not
lying about a healthy robot; it was reporting a robot the controller had
stopped moving. Fix the controller and the same estimator tracks to 10%.

**What survives as a real, separate, much smaller issue**: that residual
~10% scale under-read on galloping. Worth its own investigation, worth
nobody confusing it with the 34 m figure, and NOT the thing that was
stopping the dash.

**Scope**: N=1 per gait. The effect size is enormous (never completed ->
completed cleanly, on six gaits including three that had never crossed
100 m in this project's entire history) and the mechanism is derived
from source and independently confirmed by a dose-response in commanded
force, so this is not a small-sample inference of the kind this file
keeps having to retract. But it is one run per cell, and the clamp is
still opt-in pending exactly that repetition.


**`parallel` closed-leg measured, filling the one gap left open above**:
PASS **212.8 s** against its 181-182 s leg-open baseline, **+17 %** - the
largest close-leg cost in the catalog, which is what its 46.1 m gap (the
biggest) plus a 90->49.4 deg closing corner predicts. Completes the set:

| course | gap from home | open | closed | delta |
|---|---|---|---|---|
| circle (trotting 2.5) | 6.89 m | 58.5 s | 62.5 s | +6.8 % |
| expsquare (walking 1.5) | 18.03 m | 109.1 s | 121.3 s | +11 % |
| parallel (walking 1.5) | 46.10 m | 181.5 s | 212.8 s | +17 % |

Monotonic in the gap, as it should be - it is the same walk home each
time, priced by how far home is. All three pass closed.

## THE FORCE CAP WAS SET IN FOUR PLACES WITH THREE VALUES - and a gait switch silently cut it to 120 N

Per direct instruction to fix the oval ("you HAVE to be able to run that
oval"). Chasing it turned up a real bug that explains a long-standing
wrong conclusion in this file.

`setup_problem()`'s 4th argument is the per-foot force cap. It was passed
at FOUR call sites with THREE different values:

| site | value |
|---|---|
| constructor | `$CTRL_F_MAX` or 175 |
| `applySchedule()` | **hardcoded 120** |
| `solveDenseMPC()` | `MPC_F_MAX` (175) |
| `_runSolve()` | `MPC_F_MAX` (175) |

120 is mini-cheetah's original number, corrected to 175 for the Go1 long
ago everywhere except here. And `applySchedule()` re-runs `setup_problem`
whenever the MPC SEGMENT TIMING changes - which is gait- and
speed-dependent - so **any mid-course gait switch or speed change
silently dropped the cap from 175 to 120 N/foot**: 350 -> 240 N across
the two feet a diagonal-pair gait has down. trotRunning at 3.5 m/s needs
`m*g/duty` = 126.1/0.4 = **315 N** during stance. 240 N cannot hold the
robot up.

**This is why the OVAL specifically failed**: it is the only course in
the catalog that switches gait mid-run (`WP_ANALYZER`). And it means this
file's own conclusion - "trotRunning genuinely cannot hold this curve" -
was measured on a build that handed it 76 % of the force it needed the
moment the analyzer acted. That conclusion should be treated as
UNVERIFIED, not as a property of the gait.

Measured directly with `$SIM_MPCZ`: `Fz` pinned at exactly **350.0 N**
(the 175 cap x 2) while body height fell 0.291 -> 0.264 with `vz`
negative throughout - the solver asking for everything it was allowed and
still sinking. `$CTRL_F_MAX=250` changed NOTHING, because that override
only ever reached the constructor.

Fixed with one `mpcForceCap()` accessor used by all four sites - the same
one-source-of-truth treatment the decel ramps, the draft slots and the
gait dropdown needed tonight.

### Honest status: this did NOT restore the fast oval

Two more measured negatives, so nobody re-runs them:

- `$CTRL_XDRAG_CLAMP=0` (x_drag fully disabled): still fell. The oval's
  mid-course fall is **not** the `x_comp_integral` windup.
- Cap unified, then raised to an effective 260 N/foot (verified reaching
  the solver - `Fz` reached 442.9, past the old 350 ceiling): still fell.

What DID change is how far it gets. Before: fell at **wp33**, right at
the first `5 -> 9` switch. After: reaches **wp44**, completing the FULL
analyzer cycle (`5 -> 9` into the curve, `9 -> 5` out of it) and failing
in the SECOND curve instead. So the force cap was a genuine, load-bearing
bug on this course and there is at least one more cause behind it.

One caution recorded because I got it wrong mid-investigation: `vx` in
`[MPCZ]` is `vWorld[0]`, the estimator's own initial-heading axis. On an
oval's RETURN leg a negative `vx` is correct, not a backward-walk
symptom. Do not read it as one.


### The windup clamp now ships ON by default

`$CTRL_XDRAG_CLAMP` defaults to **1.0** rather than -1 (off). It was
opt-in while it was one run's hypothesis; leaving it off shipped a robot
that still walks backward on any sustained straight, and that is not a
defensible default once the alternative is measured. 1.0 is the value
every one of tonight's passes used - all 8 gaits on the 100 m dash, and
the regression suite - so it is the value that ships rather than a fresh
guess. A NEGATIVE value restores stock MIT's unbounded behaviour for A/B.

Regression suite with it ON by default: **8/8 PASS** (star, atom, oval,
dash_trotRunning, dash_long_duration, and circle at trotting/bounding/
galloping).

Worth testing later, deliberately NOT done now: MIT clamps the sibling
integrator `rpy_int` to +-0.25, and a tighter bound here may well be
better. But 0.25 has not been measured and 1.0 has, across 8 gaits plus
the full suite - changing it now would trade evidence for symmetry.

## HOST CONTENTION, MEASURED AT LAST - and at N<=3 it does not exist

Per direct instruction, the experiment the load-budget model was built
for. `stm32mp1/gazebo/conductor/contention_sweep.py`.

**Design, and why it is not another inconclusive multi-dog batch.** Every
previous fleet batch in this file varied TWO things at once - how many
dogs, and how much work each was doing (different courses, speeds,
durations) - so a failure could never be attributed to concurrency rather
than to whichever mission happened to be in the batch. Here every dog
runs the IDENTICAL `dash:100 @2.0 trotting`: a straight line, no
cornering, no gait switching, nothing course-specific to blame. The only
variable is N.

And it measures the CONTROL-LOOP PERIOD TAIL, not pass/fail, because that
is the thing contention would physically do - the loop targets 2.0 ms and
a late tick applies its forces for however long the tick really took.
Threshold is this file's own, established across many runs: every failure
sat at ~14 % of intervals over 4 ms, every pass at <= 3.9 %.

### The result: 3 sweeps, 9 runs, 18 dog-runs, 876 samples

| N | worst period | over 4 ms | verdicts |
|---|---|---|---|
| 1 | 3.64 ms | **0 / 209** | PASS x3 |
| 2 | 3.45 ms | **0 / 434** | PASS x3 |
| 3 | 3.30 ms | **0 / 527** | PASS x3 |

**ZERO overruns at every concurrency, and the tail does not grow with N** -
the worst period is flat within noise (3.0-3.6 ms) and if anything trends
slightly DOWN as dogs are added. Running three dogs costs this host
nothing measurable in control-loop timing.

### What this means for the record

**"Host contention" as an explanation for multi-dog fleet failures is
refuted at N<=3.** That matters, because this file leans on it repeatedly,
and three separate findings already collapsed under solo re-testing
earlier tonight (pacing's "~50 % coin-flip", walking's "120-147.5 deg
mid-band softness", part of bounding's "bimodal"). Now there is a number
behind the doubt rather than just absence of evidence.

**The genuine stalls were never dogs competing with each other.** The real
13-18 ms events this file documents were operator-diagnosed to TIME
MACHINE - an EXTERNAL process bursting I/O, which the launch gate now
refuses to launch into. So "host contention" was conflating two very
different things:

  * dogs contending with each other  -> MEASURED, does not happen at N<=3
  * an external process (a backup) stealing the machine -> REAL, already
    named, already gated

Only the second was ever true, and it is not a property of fleet size.

### Scope, stated honestly

N<=3 only. This file separately documents that FOUR OR MORE dogs fail with
`STATE ESTIMATE WENT NON-FINITE` before standing - a different mechanism
at a different scale, untouched by this result and still unexplained.
`dash:100` is also the simplest course shape; the load model says per-tick
cost is flat across mission kinds (2.48-2.49 ms measured on every one), so
a heavier course should be longer rather than costlier per tick - but that
is an inference from the model, not a second measurement.


## `corner:` WORKS - it never had a recipe, and that WAS the bug

Per direct instruction, item 4. The `corner:<leg_m>:<angle_deg>` mission -
one isolated corner with a real approach and a real exit, built for the
"empirical per-gait, per-angle cornering envelope" question - has been
carried in this file as broken (overshoot, then `pitch 53.757 deg` at the
settle gate, "still no clean PASS"). It is not broken.

**It had no `RECIPES` entry at all.** Every other cornering course in the
catalog gets a tuned `extra` at launch; `corner:` got `{}`. So it launched
with none of the graded-corridor / gentle-`WP_ALON` treatment that circle,
sector, parallel and expsquare each needed before THEY worked, and the
resulting pitch blowup was read as a defect in the mission rather than as
the absence of tuning.

Given the same treatment, at three angles on ONE tuning:

| mission | result | settle |
|---|---|---|
| `corner:25:45` | **PASS 61.3 s** | roll 0.7, pitch 0.5 |
| `corner:25:90` | **PASS 54.9 s** | roll 0.1, pitch 0.4 |
| `corner:25:135` | **PASS 47.8 s** | roll 0.0, pitch 0.5 |

Against the previous 53.8 deg pitch failure. Confirmed again end-to-end
from the recipe itself with no manual `--extra`: `corner:25:90` PASS 54.8 s.

**One deliberate design choice worth stating**: the turn-grading window is
kept WIDE (`WP_TURN_SOFT=0.3`, `WP_TURN_HARD=2.0`, i.e. 17-115 deg) rather
than bracketed tightly around one angle the way circle's (0.3/0.79) and
sector's (0.8/2.0) are. A probe mission exists to be swept ACROSS angles,
and a per-angle-tuned window would make the sweep measure its own tuning
instead of the robot. Also now selectable from the panel dropdown.

**Correcting an earlier claim in this file**: the `corner:` write-up says
"corner: has no established recipe, so no test tonight had ever passed
`WP_PLANNER=1`". The recipe part is right and was the real problem; the
`WP_PLANNER` part is wrong - `server.py` sets `WP_PLANNER=1`
unconditionally on the launch command line for every mission, so any
conductor-launched `corner:` run always had it. What it lacked was the
tuning, not the planner.

This also unlocks the stretch goal that motivated the mission: a real
per-gait, per-angle envelope in arbitrary notches, instead of only the
angles other courses happen to contain (45 from circle, 90 from
parallel/expsquare, 120-147.5 from sector, 144/162 from star).


### The fast oval, after the force-cap fix: better, still not passing

Continuing item 6 on the current build (force cap unified, windup clamp
on by default, SHM run-id). `oval:40:5.0` trotRunning @3.5 with
`WP_ANALYZER=1 WP_VSUS=2.4`, varying only the per-foot cap:

| `CTRL_F_MAX` | reach | fall |
|---|---|---|
| 175 (default) | wp33 | flat collapse right at the `5 -> 9` switch |
| 240 | **wp44** | flat collapse, SECOND curve |
| 260 | **wp44** | flat collapse, SECOND curve |

So the cap is genuinely binding - raising it clears the first curve and
the entire `5 -> 9 -> 5` analyzer cycle - but it is not sufficient. And
this is not simply "too fast": trotRunning @3.0 with the analyzer fails
too, as does x_drag fully disabled.

Worth noting against this file's own force-cap comment, which says a
2-foot-down gait peaks at **128-192 N/foot** and the knee is good for
**240-355 N**: the shipped 175 is below the top of the gait's own demand
band and well below what the hardware could deliver, so 240 is not an
aggressive value - it is inside the documented envelope. It still is not
enough for this course at this speed.

**Status, plainly: the oval RUNS.** `trotting @2.4, WP_ANALYZER=0` is the
shipping recipe and passes the regression suite every time (80.5 s). What
remains unsolved is the FAST configuration (trotRunning @3.5 with
mid-course gait switching, historically ~30 s), which is a speed
optimisation, not a broken course. The next lead is the second curve
specifically - both surviving failures are there, not at the switch - so
whatever is left is about sustained R=5 cornering under a flight gait
rather than about the gait change.


## THE FAST OVAL, SOLVED - and the gait switch was the killer all along, in a way nobody could see until tonight

Per direct instruction ("figure it out"). Three findings, in the order
they fell, each verified before the next was trusted.

### 1. My "second curve" claim was WRONG - both falls bracket ONE curve's switches

Checked the geometry before theorizing: wp33 = 0.4 m before curve-1
ENTRY; wp44 = 174 deg around curve 1, half a metre from its EXIT. The
"failing in the SECOND curve" claim in the previous section is false -
nothing ever failed mid-curve, or in the second curve. Both falls sit
within a few hundred ms of a GAIT SWITCH: cap 175 died right after the
5->9 at curve entry, cap 240/260 survived that and died right after the
9->5 at curve exit. The "sustained R=5 cornering under a flight gait"
hypothesis dies with it.

### 2. The switch was firing at an arbitrary gait-cycle phase - 40% of which is lethal

trotting and trotRunning share offsets (0,5,5,0) but differ in duration
(5 vs 3 of 10 segments), so their stance tables DISAGREE on segments
3,4,8,9 - trotting has a diagonal loaded there, trotRunning calls all
four feet airborne. `gaitNumber` was adopted the INSTANT `cmpc_gait`
changed, at whatever phase the cycle was in:

* **9->5 landing in a disagree segment**: the diagonal CARRYING the robot
  is instantly re-scheduled airborne while the other pair is mid-swing -
  all four feet commanded off the ground from a non-ballistic state at
  2.4+ m/s. Free fall, flat collapse. (The wp44 signature.)
* **5->9 landing in segments 3-4/8-9**: those are trotRunning's FLIGHT
  segments - the body is ballistic and the new table says two feet are
  down, so the MPC solves force into airborne feet. Fatal at cap 175,
  survivable at 240 - which reframes the force-cap dose-response: the
  extra force was HEADROOM TO MUSCLE THROUGH A MIS-PHASED SWITCH, not
  steady-state need.

And because a mission replays almost tick-identically, the switch landed
at the SAME phase every run - which is why the failures looked
deterministic per config instead of the coin-flip a 40% hazard window
suggests.

**Fixed: PHASE-GATED GAIT ADOPTION** (`ConvexMPCLocomotion`, new
`_gaitAdopted`/`_gaitDeferTicks`): a requested change waits (at most one
cycle, ~300 ms; 600 ms hard cap) for the segment index to wrap to 0 -
the one phase every gait table here is defined against - before it is
applied. Switching INTO standing adopts immediately (all-stance can never
de-load a loaded foot, and it is the safety direction: zeroVelHold, the
async entry hold). This is the same discipline `applySchedule()` has
applied to SEGMENT-TIME changes all along ("only at a cycle boundary");
the gait number itself just never got it. Verified live: a 5->9 request
landed at seg=8 - squarely in the hazard window - and was deferred 26
ticks to the wrap, adopting cleanly. The flat collapse AT the switch is
gone.

**Honest half: the gate is necessary, not sufficient.** With clean
adoptions the dog still fell ~1 s later, ~45 deg into the arc - entering
hot (2.7-3.0 actual against the 2.4 plan; trotRunning overshoots its
command and the follower brakes late) while finishing the deceleration
on a freshly-swapped gait. Earlier switching (LEAD=8) and a lower cap
(VSUS=2.2) both still fell. A real mid-motion gait swap at speed has
transients beyond the contact table (swing-phase discontinuities - a
mid-air foot's swingState jumps when the schedule changes - and the
stacked segment-time change), and no tested variation survived them.

### 3. The archaeology that settled the design: the milestone oval NEVER SWITCHED

The fleet-complete-20260824 "analyzer oval PASS 95/95" predates the
SIM_GAIT-override fix - its `[mission] gait 9 -> 5` prints were the
analyzer's INTENT, silently discarded by the override every tick (this is
documented in "THE REAL GAIT-SELECTION BUG" but its consequence for the
oval recipe was never drawn). **What that milestone actually validated
was trotRunning the whole way with the VSUS speed cap - cap-only, no
switch.** Every historical "switching oval" pass was cap-only by
accident; a REAL switch has never passed on any build.

So the shipping fast recipe now makes the historically-validated
behavior explicit instead of an accident of a since-fixed bug:

    oval: trotRunning @ 3.5, WP_ANALYZER=1 WP_VSUS=2.4 WP_GAIT_CORNER=5
          (sustained-segment gait = trotRunning itself -> cap, no switch)

| config | result |
|---|---|
| switch, phase-gated, default cap | FELL (~45 deg into the arc) |
| switch, phase-gated, LEAD=8 | FELL |
| switch, phase-gated, VSUS=2.2 | FELL |
| **cap-only (WP_GAIT_CORNER=5)** | **PASS 4/4 - 37.0/37.0/37.0/37.1 s** |

A 0.1 s spread over four reps, at nearly 2x the trotting fallback's
pace. Also corrected in passing: "trotRunning cannot hold this curve"
was measured UNCAPPED at 3.5 cruise - capped to 2.4 through the arc it
holds it 4/4. The claim was about speed, not the gait.

The phase gate STAYS despite cap-only not needing it: it is dormant on a
course that never switches, it demonstrably converts a lethal
mis-phased adoption into a clean one, and every ENGAGEMENT (standing ->
dynamic gait at mission start) now happens at pair-A stance-start
deterministically instead of at whatever phase the stick moved -
plausibly relevant to the documented engagement coin-flips (pacing's
~50%, historical bounding entry), not yet separately measured.

