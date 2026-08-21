# Reversing `Legged_sport`: Unitree's Go1 controller vs MIT Cheetah-Software

**Subject:** `/home/pi/Unitree_latest/autostart/sportMode/bin/Legged_sport`
(ELF 64-bit, ARM aarch64, PIE, 7,061,680 bytes, **not stripped, with DWARF
`debug_info`**, BuildID `f7a859eef749829e99a9a7b28e20a1b47414adf5`)

**Purpose.** Determine the provenance of the factory Go1 controller relative to
MIT Cheetah-Software (MIT licence, 2019 MIT Biomimetic Robotics Lab), and
recover the robot's real physical parameters so this port models the same
machine the factory controller models. Comparison of a shipped binary against
the open-source project it derives from, for licence-compliance and
interoperability purposes.

**Method.** Symbol table (`nm -C`, 9,739 symbols), embedded DWARF/source paths,
constant-pool recovery from `.rodata` via resolved `adrp`+`ldr` pairs, and
disassembly (`radare2` 6.0.8). Scripts under `tools/reversing/`.

---

## 1. Provenance: it is a direct fork, and the tree proves it

The binary carries its original build paths. Every one is MIT's directory
layout, verbatim:

```
/home/pi/src_legged_sport/common/include/Dynamics/Quadruped.h
/home/pi/src_legged_sport/common/include/Math/Interpolation.h
/home/pi/src_legged_sport/common/include/Math/Spline.h
/home/pi/src_legged_sport/common/src/ControlParameters/ControlParameters.cpp
/home/pi/src_legged_sport/common/src/Controllers/FootSwingTrajectory.cpp
/home/pi/src_legged_sport/common/src/Utilities/utilities.cpp
/home/pi/src_legged_sport/robot/src/RobotRunner.cpp
/home/pi/src_legged_sport/robot/src/SimulationBridge.cpp
/home/pi/src_legged_sport/user/MIT_Controller/Controllers/convexMPC/dance.cpp
/home/pi/src_legged_sport/third-party/qpOASES/src/{QProblem,Bounds,Constraints,...}.cpp
/home/pi/src_legged_sport/third-party/ParamHandler/src/{scanner,tag,nodebuilder,...}.cpp
```

MIT class names are present in bulk: `FSM_State` (623 symbols), `Quadruped`
(185), `ControlFSM` (181), `Gait` (123), `LegController` (117),
`BalanceController` (88), `WBIC` (66), `ConvexMPCLocomotion` (49),
`FootSwingTrajectory` (43), `OrientationEstimator` (37),
`DesiredStateCommand` (33), `WBC_Ctrl` (29), `LocomotionCtrl` (25),
`RobotRunner` (17), `ContactEstimator` (10), `SafetyChecker` (7).

The robot's filesystem also ships `reference_license/Cheetah-Software/LICENSE`
(MIT, 2019 MIT Biomimetic Robotics Lab).

**Conclusion: `Legged_sport` and this port are the same upstream codebase.**
That makes the factory controller a direct, authoritative reference for how MIT's
stack should be parameterised for a Go1 — which is what the rest of this document
uses it for.

---

## 2. Recovered physical parameters (the important part)

There is **no `buildGo1()`**. Unitree kept MIT's function name
`buildMiniCheetah<float>()` (`0xd5080`) and substituted Go1 numbers. Its
constant pool:

```
.rodata 0x2fdc10:   0.3762    0.0935    0.114     5.204
.rodata 0x2fdc20:  12.840     6.333     6.333    -9.4995
.rodata 0x2fdc30:   0.080     0.213     0.213     0.000
.rodata 0x2fdc40:   0.430     0.092     0.0985   24.000
```

Read against MIT's assignment order in `buildMiniCheetah`:

| field | Unitree value | this port had | status |
|---|---|---|---|
| `_bodyLength` | 0.3762 | 0.3762 | ✓ |
| `_bodyWidth` | 0.0935 | 0.0935 | ✓ |
| `_bodyHeight` | 0.114 | 0.114 | ✓ |
| `_bodyMass` (trunk) | 5.204 | 5.204 | ✓ |
| total mass | **12.84** | 13.101 (URDF sum) | see §2.3 |
| `_abadGearRatio` | 6.333 | 6.33 | ✓ |
| `_hipGearRatio` | 6.333 | 6.33 | ✓ |
| **`_kneeGearRatio`** | **9.4995** | **6.33** | **FIXED** |
| `_abadLinkLength` | 0.080 | 0.08 | ✓ |
| `_hipLinkLength` | 0.213 | 0.213 | ✓ |
| `_kneeLinkLength` | 0.213 | 0.213 | ✓ |
| **`_maxLegLength`** | **0.430** | **0.385** | **FIXED** |
| **`_batteryV`** | **24.0** | **21.6** | **FIXED** |
| `_motorTauMax` | 3.0 (imm) | 3.744 | see §2.2 |
| `_jointDamping` | 0.01 (imm) | 0.01 | ✓ |
| `_jointDryFriction` | 0.2 (imm) | 0.2 | ✓ |

### 2.1 The knee gear ratio — a wrong premise, now corrected

This port modelled all three joints at 6.33:1 and then invented a second field,
`Quadruped::_kneeMotorTauMax = 5.616`, to explain the knee's 35.55 N·m. The
comment justified it as *"the Go1 knee is 1.5x the hip at the SAME gear ratio,
which MIT's single-value struct could not express."*

**That premise was false.** The gear ratios are `6.333 / 6.333 / 9.4995`:

```
23.70 N·m / 6.333  = 3.7424 N·m at the motor
35.55 N·m / 9.4995 = 3.7423 N·m at the motor
```

Identical to four decimals: **one motor type drives all twelve joints**, and the
stronger knee is *gearing*, exactly as mini-cheetah does it with 6/6/9.33. MIT's
single `_motorTauMax` expressed this correctly all along.

`_kneeMotorTauMax` has been **removed** and `Quadruped.h` / `Quadruped.cpp`
restored to upstream MIT.

### 2.2 `_maxLegLength`

Unitree uses **0.430**; this port had derived **0.385** from the URDF knee limit
(the leg can never straighten, so reach is
`sqrt(l2² + l3² + 2·l2·l3·cos 0.888)`). The derivation is geometrically sound but
it is not what the factory controller uses, and this value **bounds foot
placement** — it feeds the swing planner and `SafetyChecker`'s
`maxPDes = L·sin(60°)`. Running 0.385 against a machine whose own controller
allows 0.430 is a self-imposed ~10% shorter stride, on the axis that sets speed.

### 2.3 Mass

The URDF sums to 13.101 kg across 38 links; Unitree's model uses 12.84 kg. The
difference (0.261 kg ≈ 0.0218 kg × 12) is consistent with the URDF counting the
twelve rotor links that MIT's model carries separately. Left as-is pending
confirmation; noted so it is not mistaken for an error later.

---

## 3. The MPC rigid-body model — our inertia was a guess, and wrong

`RobotState::set` (`0xfb9f0`) carries the single-rigid-body inertia diagonal:

```
Ixx = 0.136620   Iyy = 0.425578   Izz = 0.460359
```

This port had **scaled mini-cheetah's** `(.07, .26, .242)` by the 13.1/9 mass
ratio to get `(0.102, 0.379, 0.352)`:

| axis | ours (guess) | Unitree (real) | our error |
|---|---|---|---|
| Ixx (roll) | 0.102 | **0.13662** | **−25%** |
| Iyy (pitch) | 0.379 | **0.425578** | −11% |
| Izz (yaw) | 0.352 | **0.460359** | **−24%** |

An MPC that believes the body spins up a quarter more easily than it does will
under-command corrective moments — on precisely the two axes this port's gaits
fail on (roll collapse, heading drift). **Corrected.**

### MIT's cost weights are untouched

Scanning `.rodata` for 12-float cost vectors finds MIT's `Q` **verbatim**, twice
(`0x2fe070`, `0x2fe760`):

```
0.25 0.25 10   2 2 50   0 0 0.3   0.2 0.2 0.1
```

`alpha = 4e-5` (immediate in `solveDenseMPC`) also matches MIT exactly. Unitree
ships retuned variants for other modes — e.g. `0x2fe390`:
`0.5 0.5 10  20 20 15  0.1 0.1 1  0.5 0.5 0.5` (10× the position weight, lower
z) — but standard locomotion runs MIT's weights. **This port's use of MIT's Q is
correct.**

---

## 4. Joint limits: three different sets, and we used the wrong one

The binary embeds two limit sets, and the SDK header gives a third:

| source | abad | thigh | calf | what it is |
|---|---|---|---|---|
| binary `0x2fdbe0`-`0x2fdc00`, **= URDF** | ±0.863938 (±49.5°) | −0.6868..4.5012 | −2.8187..−0.8884 | mechanical |
| binary, second set | ±0.959898 (±55°) | −0.5759..2.8798 (−33..165°) | −2.6354..−0.9251 (−151..−53°) | operational clamp |
| `go1_const.h` (SDK) | ±1.047 (±60°) | −38..170° | −156..−48° | permissive SDK bound |

The second set is round numbers in degrees — deliberate software limits, not
mechanics. **This port had widened the simulated abad joint to ±1.047 in all four
worlds**, citing `go1_const.h`. That used a permissive *software* bound as a
*physical* stop, and it is wider than anything the factory controller allows.

**Corrected:** all 16 abad joints across the four worlds are now ±0.863938 rad,
matching both the URDF and Unitree's own dynamics model.

---

## 5. What Unitree built on top of MIT

### 5.1 FSM states — MIT ships 12, Unitree ships 27

MIT/this port: `Passive JointPD ImpedanceControl StandUp BalanceStand
Locomotion RecoveryStand Vision BackFlip FrontJump`.

Unitree adds: `Dance Gabriel Offline_Motion PreStand RollHoldMove S_Stake Space
SpaceStep StandDown TorsoVertical TurnOverMove TwoLegHop TwoLegStep WallowStand
ZeroTorque BackFlipWXX LocomotionVision`.

### 5.2 Locomotion variants

`ConvexMPCLocomotion` is joined by `ConvexMPCLocomotionStairs`,
`ConvexMPCLocomotionVision`, `ConvexMPCLocomotion_gabriel`,
`TwoLegStepLocomotion`, `Space_step`, `S_stake` — each with its own
`solveDenseMPC`. Gait classes: `OffsetDurationGait` (MIT's), plus `ParkourGait`,
`SpaceGait`, `UnitreeGait`, and a `P2PGaitPlanner`.

### 5.3 `ConvexMPCLocomotion` method set: what changed

| | MIT / this port | Legged_sport |
|---|---|---|
| shared | `_SetupCommand`, `initialize`, `recompute_timing`, `run`, `solveDenseMPC` | same |
| MIT-only | `updateMPCIfNeeded`, `solveSparseMPC`, `initSparseMPC` | **dropped** |
| Unitree-added | — | `runSwingLegControl`, `runContactLegControl`, `trajPlanner`, `zeroVelTransitionAmend` |

Unitree **split MIT's monolithic `run()`** into separate swing and stance
control paths, dropped the sparse-MPC path entirely (as this port also did), and
added an explicit trajectory planner plus a zero-velocity transition handler.

### 5.4 `OffsetDurationGait` gained flight-phase awareness

MIT's methods: `getContactState`, `getSwingState`, `getMpcTable`,
`setIterations`, `getCurrentStanceTime`, `getCurrentSwingTime`,
`getCurrentGaitPhase`, `debugPrint`.

Unitree **adds**: `getFlightState`, `getCurrentHybridMode`, `getItNxtHybMode`,
`getHoriz`, `getFliMacNH`, `getContactStateExpected`.

They explicitly model **flight state** and **hybrid modes**. This port's
bounding / pronking / galloping / trotRunning — exactly the flight-phase gaits —
collapse at gait engagement with velocity commanded to zero. That is a strong
hint the missing machinery is this, not gait tuning.

---

## 6. Pseudocode: `ConvexMPCLocomotion::zeroVelTransitionAmend`

`0xe41c0`, signature `(int* mpcTable, ControlFSMData<float>& data)`. Recovered
constants: velocity threshold `0.01` (`0x2665e0`), duty divisor `0.7`
(inline `0x3FE6666666666666`), offset `0.42` (`0x2fe1c8`).

```c
void ConvexMPCLocomotion::zeroVelTransitionAmend(int* mpcTable,
                                                 ControlFSMData<float>& data) {
    // this+0x1540 : bool  transition-enable flag
    // this+0x1574 : int   current gait id
    // this+0x20e8 : float v_des[0]        (x)
    // this+0x20ec : float v_des[1]        (y)
    // this+0x20dc : float stance-time / duty scratch (2 floats written together)
    // this+0x2194 : int   direction code  (-1 / 0 / +1)
    // this+0x2198 : int   transition counter
    // this+0x219c : int   transition phase
    // data +0x481 : bool  rescale-enable
    // data +0x482 : bool  allow-phase-advance
    // data +0x36a : bool  request-transition (output)

    if (transition_enabled && gait_id == 5)     // 5 == trotRunning in MIT's ids
        reset_transition_state();               // 0xe4318: zero a pair of floats

    // 1. Classify the commanded velocity into a direction code. The 0.01 m/s
    //    threshold is the "effectively zero" band.
    if (v_des[1] > 0 && v_des[0] < 0.01f)       direction = -1;   // lateral only
    else if (v_des[0] > 0 && v_des[1] < 0.01f)  direction = +1;   // forward only
    else                                        direction =  0;   // mixed / zero

    // 2. Renormalise the stance timing by the duty factor when asked. 0.7 is
    //    the duty of MIT's walking gait (7/10); 0.42 = 0.6 * 0.7.
    if (data.rescale_enable) {
        float t = this->stance_scratch / 0.7f - 0.42f;
        this->stance_scratch[0] = t;
        this->stance_scratch[1] = t;
    }

    // 3. Debounced transition state machine. The counter must exceed 1 before
    //    the phase is allowed to move, so a single tick of zero velocity does
    //    not trigger a gait transition.
    if (transition_counter > 1) {
        transition_counter++;
        if (transition_counter > 1) {
            transition_counter = direction;
            if (data.allow_phase_advance) {
                switch (transition_phase) {
                    case 1:  /* 0xe4370: amend mpcTable for phase 1 */ break;
                    case 2:  /* 0xe432c: amend mpcTable for phase 2 */ break;
                    default:
                        if (transition_phase > 1)
                            data.request_transition = true;
                }
            }
        }
    }
}
```

**Why it matters here.** MIT's stock `ConvexMPCLocomotion` has no equivalent: it
hands the gait's contact table to the MPC unmodified regardless of what the
velocity command is doing. Unitree found it necessary to *amend the contact
table* around zero-velocity transitions, with a debounce so a momentary zero
does not trigger one.

This port's measured failure mode is the same event: gaits collapse **at
locomotion entry, while the velocity command is still held at zero** — dead
level, body sinking, no orientation trip. Bounding, pronking and galloping do it
at 0.00 m travelled.

---

## 7. Corrections applied to this port

| file | change | source |
|---|---|---|
| `common/include/Dynamics/Go1.h` | `_kneeGearRatio` 6.33 → **9.4995** | `.rodata 0x2fdc20` |
| `common/include/Dynamics/Go1.h` | `_maxLegLength` 0.385 → **0.430** | `.rodata 0x2fdc40` |
| `common/include/Dynamics/Go1.h` | `_batteryV` 21.6 → **24.0** | `.rodata 0x2fdc40` |
| `common/include/Dynamics/Go1.h` | gear ratios 6.33 → **6.333** | `.rodata 0x2fdc20` |
| `common/include/Dynamics/Quadruped.h` | **removed** `_kneeMotorTauMax` (restored upstream MIT) | §2.1 |
| `common/src/Dynamics/Quadruped.cpp` | knee actuator uses `_motorTauMax` like the others | §2.1 |
| `.../convexMPC/RobotState.cpp` | inertia `(0.102,0.379,0.352)` → **`(0.13662,0.425578,0.460359)`** | `0xfb9f0` |
| `stm32mp1/gazebo/worlds/*.sdf` (×4) | abad limit ±1.047 → **±0.863938** | `0x2fdbe0`, URDF |

---

## 7a. `trajPlanner` and `runSwingLegControl` (constants recovered)

Not reduced to pseudocode, but their constants are informative.

### `trajPlanner` (0xe6a70): the reference is heavily filtered

Immediates include three complementary first-order pairs:

```
0.998 / 0.002      0.994 / 0.006      0.992 / 0.008
```

At 500 Hz those are **250 ms - 1 s** time constants. MIT hard-codes
`float filter(0.1)` for the operator command - about **20 ms**. So Unitree
smooths the reference it hands downstream 12-50x harder than MIT does.

Relevant here because the gaits still failing (pronking, galloping) die on the
transient at gait engagement, at a fixed ~18 s regardless of commanded speed,
rather than in steady state. Exposed as `SIM_CMD_FILTER` (default stays MIT's
0.1) so it can be measured rather than assumed.

Other clamps present: +-0.11, +-0.3, +-0.75, 1.5.

### `runSwingLegControl` (0xe7548): per-leg foot-placement bias

`9.81` confirms MIT's capture-point formula, and `0.07` matches the swing
height. The constant pool carries something MIT does not have:

```
0x2fe180:  -0.030   +0.030   +0.015   -0.015
0x2fe190:  -0.020   +0.020   +0.020   -0.020
```

Four-element, antisymmetric front-to-rear. MIT places the swing foot with a
single uniform lateral offset for every leg:

```cpp
Vec3<float> offset(0, side_sign[i] * .065, 0);
```

- no front/rear asymmetry and no per-leg trim. The Go1's mass is not evenly
distributed front-to-rear, so a deliberate per-leg stance bias is the sort of
thing a factory controller carries and a reference implementation does not.
NOT implemented here: the exact field these land in is unconfirmed, and foot
placement is not somewhere to change on a guess.

### `runContactLegControl` (0xe3f30): stance stiffness really is zero

Its pool includes an all-zero vector at `0x2fe0f8`, consistent with MIT's
`Kp_stance = 0*Kp`. **This rules out stance stiffness as the cause of this
port's collapses** - Unitree relies on MPC force for support exactly as MIT
does.

## 7b. Unitree DISABLED MIT's `locomotionSafe()`

`FSM_State_Locomotion<float>::locomotionSafe()` (0x166a48) is eight bytes:

```asm
mov  w0, #1        ; return true
ret
```

So is `FSM_State_Dance`'s. The factory controller does not run MIT's locomotion
safety check at all.

This is the same check this port had to fix twice: its lateral foot limit is
mini-cheetah's 0.18 m (the Go1 legitimately stands ~30% wider), and the line
carries an upstream typo, `std::fabs(p_leg[1] > 0.18)`, taking `fabs` of a
*bool*. Failing it sends the FSM to **RECOVERY_STAND, which folds all four
legs** - which is why every early "the MPC tumbles at gait start" note in
CLAUDE.md turned out to be this check firing. This port gated it to 0.24 m for
the Go1; Unitree simply returns true and relies on the orientation check in
`ControlFSM::safetyPreCheck` plus their own layer.

**Checked, and NOT the cause of this port's remaining failures**: the logs for
the collapsing runs (pronking, galloping, bounding) contain zero
`RECOVERY_STAND` transitions, so the 0.24 m gating is already permissive enough
and the check is not firing. Recorded because it is the factory's answer to a
check this port has twice had to work around, not because it explains the
current bug.

## 8. Open leads

- `trajPlanner`, `runSwingLegControl`, `runContactLegControl` — not yet reduced
  to pseudocode; the swing/stance split is the most likely place their gait
  robustness lives.
- `getFlightState` / `getCurrentHybridMode` — the flight-phase machinery our
  failing gaits appear to need.
- `0x2fdc40`'s `0.092` and `0.0985` are unidentified fields between
  `_maxLegLength` and `_batteryV`.
- The second (operational) joint-limit set is not enforced anywhere in this port;
  it belongs as a controller-side clamp, not a physics limit.
