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
