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

**A separate, NOT resolved issue turned up verifying the above**, and
is flagged honestly rather than folded into the trail-cap fix: that
same verification run hung at t=202s (wp 211/606) - `mit_ctrl_sim`
stopped producing any log output at all, with perfectly healthy
control-loop timing right up to the last line (`maxPeriod=2.48ms`
against an 8ms limit) and no `[STALL]`, no `[FALL]`, nothing. `gz.log`
was completely empty - ruling out the multicast bug fixed earlier
tonight (that failure mode fills `gz.log` with "No route to host" from
t=0; this one produced no gz-side output at all and got well into the
mission first). Not chased further tonight given the hour; a genuinely
new, silent, mid-run controller hang on the longest/densest course in
the catalog, distinct from every other failure mode documented above.
