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

## Still open
- The trot's travel direction is inverted: at 1.0 m/s commanded it makes 1.00
  m/s of ground speed with the *magnitude right and the sign wrong*, and
  flipping the stance sweep changes the magnitude rather than the sign, so the
  propulsion is coming from somewhere other than the stance sweep (most likely
  swing-leg ground contact). The crawl, with identical IK and sweep direction,
  goes forward.
- Terrain following works well enough to stand on the farm mesh but not to walk
  on it; `worlds/go1_farm_flat.sdf` keeps the farm scenery with a flat walkable
  ground as an interim.
