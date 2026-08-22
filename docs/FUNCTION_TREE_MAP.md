# Function-tree map: this port vs `Legged_sport` (Unitree's binary)

A systematic, subsystem-by-subsystem comparison of the call tree, built to check
for silent divergences rather than trust that "same class names" means "same
behavior." Status per entry: **1:1** (same signature/size class, behavior
consistent with source), **DIVERGES** (present both sides, meaningfully
different), **MISSING (ours)** (Unitree has it, we don't), **STUBBED
(Unitree)** (present but a trivial pass-through in their build).

Companion to `LEGGED_SPORT_REVERSE.md` (constants/pseudocode); this document
tracks structure - what calls what, and what doesn't exist on one side.

---

## Top-level control loop

| function | ours | Unitree | status |
|---|---|---|---|
| `RobotRunner::run()` | 500 Hz loop, calls `_stateEstimator->run()`, `setupStep()`, `_robot_ctrl->runController()`, `finalizeStep()` | `RobotRunner::run()` present, `initializeStateEstimator(bool)` present | **1:1** structurally |
| `ControlFSM::runFSM()` | `safetyPreCheck()` -> current state `run()` -> `checkTransition()` -> `safetyPostCheck()` | Same 4-step symbol present | **1:1** |
| `MIT_Controller::runController()` | thin wrapper around `ControlFSM::runFSM()` | present | **1:1** |
| `JPosInitializer` | present, used for crouch bring-up before `ControlFSM` even starts | **NO SYMBOL AT ALL** in the binary | **MISSING (Unitree)** - see below |

**`JPosInitializer` is absent from Unitree's binary entirely.** Not stubbed,
not renamed under a findable mangled name - no `JPosInitializer` string or
symbol anywhere. Two explanations, neither confirmed: (a) one of Unitree's 15
extra FSM states (`PreStand`, `StandDown`, `WallowStand`, `TorsoVertical`)
performs the same bring-up under a different name/architecture, or (b) the
real Go1's absolute joint encoders make a "find home by interpolating to a
known crouch" step unnecessary in a way this port's incremental encoders (or
this SITL) cannot skip. Not resolved - flagged as a genuine open question
rather than guessed at.

---

## FSM_State_Locomotion

| function | ours | Unitree | status |
|---|---|---|---|
| `run()` | present | `FSM_State_Locomotion<float>::run()` @ 0x166848 | **1:1** name/role |
| `checkTransition()` | present | @ 0x166a50 | **1:1** |
| `LocomotionControlStep()` | present | @ 0x1662c0 | **1:1** |
| `transition()` | present | @ 0x166898 | **1:1** |
| `onExit()` | present | @ 0x166108 | **1:1** |
| `locomotionSafe()` | gated to 0.24 m lateral limit (Go1-specific fix) | **`mov w0,#1; ret`** - 8 bytes, always true | **STUBBED (Unitree)** - documented in `LEGGED_SPORT_REVERSE.md` §7b |

---

## ConvexMPCLocomotion

| function | ours | Unitree | status |
|---|---|---|---|
| `_SetupCommand` | present | present | **1:1** |
| `initialize` | present | present | **1:1** |
| `recompute_timing` | present, **never called** (fixed dtMPC) | present | **DIVERGES** - Unitree can change gait period at runtime; this port has never exercised that path |
| `run` | present | present | **1:1** entry point |
| `solveDenseMPC` | present | present | **1:1** |
| `updateMPCIfNeeded` | present (async pipeline) | **absent** | **MISSING (Unitree)** - this port's own addition for the board's slow solve; not upstream MIT, not Unitree |
| `solveSparseMPC` / `initSparseMPC` | present, gated off (`cmpc_use_sparse=0`) | **absent** | matches - neither side uses the sparse path |
| `runSwingLegControl` | **absent as a function** - swing logic inline in `run()` | present, own function, 3056 bytes | **DIVERGES (structural)** - Unitree factored swing out; entry decoded (per-leg swing/stance time calls confirm the per-leg-duration port made here matches), body not fully reduced |
| `runContactLegControl` | **absent as a function** - stance logic inline in `run()` | present, own function, 656 bytes | **DIVERGES (structural)** - pseudocode in `LEGGED_SPORT_REVERSE.md` §7c |
| `trajPlanner` | **absent** - trajectory built inline in `run()` with a single 0.1 filter | present, own function, 2776 bytes, 3 complementary filter pairs (250ms-1s) | **DIVERGES** - not ported; `SIM_CMD_FILTER` knob added to test the concept, not the exact function |
| `zeroVelTransitionAmend` | ported as `zeroVelHold()` (different mechanism - see below) | present, 0xe41c0 | **DIVERGES (by design, verified)** - first port attempt (table-patching) was WRONG and measured to do nothing; the mechanism actually used here (hold standing gait) matches what the pseudocode shows Unitree's function does (raise a transition request), not a literal reimplementation |

---

## Gait (`OffsetDurationGait`)

| function | ours | Unitree | status |
|---|---|---|---|
| `getContactState` / `getSwingState` | present | present | **1:1** |
| `getMpcTable` | shifts contact schedule `+1` MPC step (MIT stock) | does NOT shift (`iter = i + _iteration`, no `+1`) | **DIVERGES, exposed as knob** - `SIM_MPC_SCHED_LEAD` (default 1 = MIT stock) |
| `getCurrentStanceTime`/`getCurrentSwingTime` | now per-leg (ported today) | per-leg (confirmed via vtable dispatch in `runSwingLegControl` entry) | **1:1** after this session's fix; was **DIVERGES** before it |
| `getFlightState` | ported this session | present @ 0xf7880 | **1:1** after porting |
| `debugPrint`, accessors | present | present, some (`getCurrentHybridMode`, `getItNxtHybMode`) are **stubs returning constants** in `OffsetDurationGait` (real logic lives in `ParkourGait`) | **STUBBED (Unitree, OffsetDurationGait only)** |

---

## State estimation

| function | ours | Unitree | status |
|---|---|---|---|
| `LinearKFPositionVelocityEstimator::run()` | present, 178 lines | present, 7864 bytes | **1:1 on the one mechanism checked** (covariance suppression, byte-identical structure) - the rest of the 7864 bytes not fully reduced |
| covariance cap (`_P.block(0,0,2,2) /= 10`) | present (this port's own diagnosis called it a bug this session, then corrected) | present, identical, confirmed by disassembly at 0x1c26a0-0x1c2778 | **1:1, verified** |
| `high_suspect_number` | 100 | 100 (confirmed immediate) | **1:1, verified** |
| `foot_process_noise_position` | 0.002 | 0.002 (confirmed immediate) | **1:1, verified** |
| `ContactEstimator::run()` | pass-through (MIT's own TODO, never addressed) | pass-through, **16 bytes**, confirmed | **1:1 (both stubbed)** - this closes the question of whether Unitree secretly implemented real contact detection; they did not, at least not here |
| `VectorNavOrientationEstimator::run()` | pass-through of `vectorNavData->quat` | not independently checked (same class name, same role expected) | orientation is a pass-through on BOTH sides by design - the finding this session was that OUR SITL feeds it noise-free ground truth, which is a SITL/SDF gap, not a code divergence |
| `CheaterOrientationEstimator`/`CheaterPositionVelocityEstimator` | present | present | **1:1** (both ship a cheat path for bring-up/debug) |

---

## WBC / WBIC

| function | ours | Unitree | status |
|---|---|---|---|
| `LocomotionCtrl::_ParameterSetup` | reads gains from yaml (`Kp_body`/`Kd_body`/`Kp_ori`/`Kd_ori`/`Kp_joint`/`Kd_joint`, ONE set) | reads from equivalent config mechanism, but the **symbol table alone shows far more variants**: `Kp_body_stance`, `Kp_body_running`, `Kp_body_stairs`, `Kp_ori_stairs`, `Kp_joint_stance`, `Kp_joint_swing`, `Kp_joint_swing_running` | **DIVERGES (structural, confirmed)** - Unitree tunes gains per-gait/per-phase; this port uses one fixed set for everything. Exact values NOT extracted with confidence (see next row) |
| gain value extraction | `Kd_body=40` after this session's tuning (measured, not decompiled) | raw `.rodata` read attempted; **retracted** - a first pass misread `Kp_ori` (4 floats) as an 8-float Kp+Kd block, spilling into `Kp_joint`'s memory; the region also interleaves 32-bit INTEGER gait-timing data with FLOAT gain data in nearby addresses (confirmed via `ldr w` vs `ldr s` in the same loop) | **NOT RESOLVED** - flagged as unsafe to pattern-match without full register-type tracing through `_ParameterSetup`'s ~328-line disassembly, not attempted at that depth this session |
| `_ContactTaskUpdate` | present (inline equivalent) | present, own function @ 0xb59e0 | **1:1** structurally |
| `SafetyChecker::checkPDesFoot/checkJointLimit/checkSafeOrientation/checkForceFeedForward` | present, all real implementations | present, all real implementations (152/696/352/608 bytes - none stubbed) | **1:1** - confirms `SafetyChecker` is genuinely active in Unitree's build even though the PER-STATE `locomotionSafe()` is stubbed; these are the general safety checks Unitree actually relies on |

---

## Swing trajectory

| function | ours | Unitree | status |
|---|---|---|---|
| `computeSwingTrajectoryBezier` | `(T phase, T swingTime)` - 2 args | `(float, float, int, float)` - 4 args | **DIVERGES, decoded this session** |

The two extra parameters are a **mode selector + associated float**, decoded
from the disassembly entry (`cbnz w1, ...` branch at 0x1ab9c4):
- `w1 == 0`: the standard Bezier path - what this port already replicates
- `w1 == 1`: calls `getSplineWaypoints(...)` then `malloc(56)` - an entirely
  different, **spline-based** trajectory generator this port does not have
- any other value: no-op

This is Unitree's terrain-adaptive swing mode (plausibly stairs/obstacles,
consistent with the `Kp_body_stairs`/`Kp_ori_stairs` WBC variants above - a
coherent "stairs mode" spans multiple subsystems). **Real capability gap**,
relevant to the terrain-following work already flagged open in this port, but
ruled out as the cause of the flat-ground speed ceiling: this SITL only ever
needs mode 0.

---

## LegController

| function | ours | Unitree | status |
|---|---|---|---|
| `updateData`/`updateCommand`/`zeroCommand`/`setEnabled` | present | present (with Unitree-SDK-specific overloads: `updateData(UNITREE_LEGGED_SDK::LowState const*)`, `updateCommand(UNITREE_LEGGED_SDK::LowCmd*)` alongside the MIT `TiBoardData`/`TiBoardCommand` ones) | **1:1** on the shared path; Unitree carries BOTH the original MIT hardware interface (Cheetah 3's TI boards) AND their own SDK - confirms they kept MIT's abstraction and added a backend, exactly the pattern this port also follows for its own hardware bridge |
| `ik`/`q2_ik`/`q3_ik`/`computeLegPosition`/`verifyIK` | present, MIT's stock kinematics | present, same names | **1:1** - no evidence of a changed kinematic convention; consistent with this session's earlier note that behavioral evidence (nothing walks backward, no kinematic sign flips observed) already supported this |

---

## The "hard 1.0" hypothesis: investigated, ruled out (this session)

Directly instrumented rather than guessed at. `_x_vel_des` (the filtered
velocity commanded to the MPC) was traced live during a failing trot@1.4 run:

```
xcmd=1.166 -> xdes=1.164
xcmd=1.178 -> xdes=1.176
xcmd=1.201 -> xdes=1.198
...
xcmd=1.247 -> xdes=1.244
[FALL] collapsed: roll=-0 deg pitch=1 deg z=0.060 m
```

`_x_vel_des` climbs smoothly and continuously through 1.16 -> 1.25 with **no
plateau, snap, or clamp of any kind**, and the robot falls mid-ramp, not at a
fixed ceiling. Static search (`grep` for `1.0f`/`1.f` near
`clamp`/`min`/`max`/`std::min`/`std::max` across the whole locomotion,
WBC, and command pipeline) found nothing either. Also checked and ruled out:
`DesiredStateCommand::deadband()` has a `(command/2)*(maxVal-minVal)` rescale
that WOULD 3x-amplify a raw m/s value if it were load-bearing - confirmed it
is NOT: `ConvexMPCLocomotion` reads `leftAnalogStick[1]` directly, bypassing
`deadband()`/`stateDes()` entirely.

**Conclusion: the ~1.0-1.25 m/s wall is genuine dynamic instability, not a
software ceiling.** The cluster of gaits failing in this band is consistent
with a real physical/control limit (as this port's earlier VHIPM/capture-point
analysis argued), not evidence of a bug.

## Summary: where this port and Unitree's code structurally diverge

1. **`JPosInitializer` doesn't exist in Unitree's build** - unresolved, real gap.
2. **Per-gait WBC gain switching** - Unitree has it, this port doesn't. Values
   not safely extractable without deeper disassembly than attempted so far.
   Raising `Kd_body`/`Kd_ori` uniformly (this session, measured 9-11x
   improvement for trot@1.0 specifically) is this port's OWN tune, not a
   decompiled Unitree value.
3. **Terrain-adaptive swing trajectory** (spline mode) - Unitree has it, this
   port doesn't. Not relevant to flat-ground speed; relevant to the
   already-open terrain-following gap.
4. **`trajPlanner`'s heavy reference filtering** - not ported; `SIM_CMD_FILTER`
   tests the general idea (measured no effect on pronking/galloping) but is
   not a faithful reimplementation of the actual 3-filter-pair structure.
5. **`zeroVelTransitionAmend`** - ported by mechanism (hold standing gait)
   rather than literally (this port's first literal attempt, patching the MPC
   table, was measured to do nothing and was reverted).

Everything else checked this session - `FSM_State_Locomotion`'s public
interface, `LegController`'s kinematics, `SafetyChecker`, the estimator
pass-throughs, the covariance cap, the gait table shift - is **1:1 or
verified-equivalent**, which is the useful negative result: this port's core
control loop is not silently diverging from MIT/Unitree's in some undiscovered
way. The remaining gaps are enumerated above, not hidden.
