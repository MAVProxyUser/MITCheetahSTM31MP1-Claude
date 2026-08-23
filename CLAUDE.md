# CLAUDE.md — STM32MP1 Cheetah port: rules, architecture, and traps

Read before changing the port. Companion to `SKILLS.md` (commands) and `README.md`.

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

Banking RAISES the height floor - 0.195 to 0.221 - which is the quantity that
separates passes from failures. Same speed, better margin. It is the first
change all session to move the actual failure mode rather than the cornering
geometry around it.

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

### THE OPEN LEAD: failures sink, and the force command is innocent

The 2.5 m/s failures are a COLLAPSE, not a topple - `roll=0 pitch=-0 z=0.058`,
flat and level, which is the force-starvation signature and NOT the cornering
signature every lever above was aimed at. That alone explains why six cornering
levers all did nothing: none of them touch the force budget.

But instrumenting the budget shows the force command is INNOCENT:

| run | result | min Fz/mg | min body z |
|---|---|---|---|
| r1 | PASS 41.6 s | 0.86 | 0.212 |
| r2 | FAIL 4/5 | 0.87 | **0.200** |
| r3 | FAIL 2/5 | 0.88 | **0.185** |
| r4 | PASS 41.6 s | 0.85 | 0.224 |
| r5 | PASS 41.6 s | 0.88 | 0.239 |

Commanded force is identical across passes and failures. HEIGHT is the
discriminator: failures dip to 0.185-0.200 where passes hold 0.212-0.239.

So the robot sinks lower on failing runs WHILE BEING COMMANDED THE SAME FORCE.
That is a delivery or transient problem - WBIC tracking, contact timing, a
corner transient - not a planning one, and it lives in a subsystem none of
today's work touched. **This is where the next session should start**, and it
should start by measuring achieved vs commanded foot force through a corner,
not by sweeping a seventh planner parameter.

### What does NOT improve it (all measured, do not re-try)

| lever | result |
|---|---|
| lower lateral budget (a_lat 2.2 / 2.0) | SLOWER (42.1-42.7 s) and no more reliable - 2/4 and 1/2 |
| hairpin pivot on all corners | 50.0/50.1 s 2/2 - most repeatable measured, 8 s slower |
| hairpin pivot on corner 1 only | 0/3, always dies at an ARCED corner afterwards |
| gait switching (either pairing) | declines to switch at 2.0, fails above it |
| more yaw authority | roll 27 -> 52 -> 72 deg for no time gain |

The failures at 2.5 are CONSISTENTLY at wp 3/5 - a specific corner, not random -
so the next honest step is to instrument that corner rather than sweep more
parameters. Six levers have now been swept against it without success.

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
specific env incantation, recorded in `SKILLS.md`, and not on the shipped config.

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
