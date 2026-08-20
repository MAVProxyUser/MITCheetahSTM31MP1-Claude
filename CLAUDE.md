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
