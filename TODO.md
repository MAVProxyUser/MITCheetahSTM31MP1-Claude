# TODO.md — open backlog for the STM32MP1 Cheetah port

Everything here is a real, previously-identified gap pulled from `CLAUDE.md`'s
running history — not a wishlist. Each item was explicitly flagged as open,
unresolved, or "not pursued further" at the point it was written; none of them
were fixed afterward unless a later `CLAUDE.md` entry says so. When you close
one, update `CLAUDE.md` with the evidence and delete or check off the entry
here — this file should always reflect what is *actually* still open, not
accumulate history (that is what `CLAUDE.md` is for).

Grouped by what it would actually take to close it.

## Hardware validation (blocks trusting any sim number on a real dog)

- [ ] **`rt_unitree` RS485 driver has never touched real hardware.** Field
      scalings (T×256, W×128, Pos×16384/2π, K_P×2048, K_W×1024), FOC mode
      value, CRC word count, and per-joint gear(9.1)/sign/offset all need a
      bench validation once the fast RS485 adapter is available. See
      `CLAUDE.md`'s "Unitree RS485 (`rt_unitree`)" section.
- [ ] **Cheetah has no standalone DroneCAN participant.** The CAN IMU's
      compact stream is currently only kept alive by `fw_realposix`
      (ninjapilot.service) — for standalone hardware use, Cheetah needs its
      own participant that can keep the stream alive and tear down/restart an
      already-running one. See `CLAUDE.md`'s "CAN IMU (`rt_can_imu`)" /
      "KEY FINDING" note.
- [ ] **Sim-fidelity gaps, never closed, all flagger before trusting any
      SITL speed number on hardware:**
  - foot friction is μ=2.0 in every world; the URDF says 0.6, real
    rubber-on-concrete is ~0.8-1.0 — flagged as "the single most suspect
    number behind the top-end results."
  - no actuator dynamics (current limit, thermal derating, back-EMF at
    speed) — commanded torque is applied directly, never refused.
  - no joint velocity limit modeled.
  - orientation is a **noise-free pass-through** of Gazebo's exact simulated
    pose in every SITL run to date — `VectorNavOrientationEstimator` has
    never actually been tested against realistic IMU noise/bias.
  - See `CLAUDE.md`'s "BEFORE TRUSTING A REAL DOG" section for the full
    table and the QA ladder (stand → lie down → stand back up → slow walk →
    dynamic gaits) that should run before any of this is believed on a
    machine that can hurt itself.
- [ ] **The board's own solver path is unverified since the qpOASES fix.**
      qpOASES costs 198-218 ms on the A7 against a 26 ms MPC segment, so the
      Mac's "solver is fixed" result does not transfer as-is — the board
      needs either JCQP made to converge on a moving gait, or
      `SIM_MPC_ASYNC=1`, or a re-measured contact-reduced qpOASES, before any
      of the Mac's post-solver-fix speeds can reach hardware.

- [x] **Env-var consolidation, 2 of 3** (2026-08-29): the eight dead `SIM_`
      flags are deleted along with the code behind them, and every `CTRL_*`
      tuning value now resolves env > `ctrl_tuning.yaml` > code default.
      STILL OPEN: reworking the fall detector for hardware (latch-limp under
      supervision, keyed off attitude/kinematics, not `_exit()` on an
      estimated height). ISSUES OPEN-13.

- [x] **Time Machine I/O storms** — CLOSED 2026-08-30 as mitigated and
      accepted (ISSUES CLOSED was OPEN-16). The launch gate refuses to start
      during a backup; a backup that BEGINS mid-run is an OS scheduling fact
      the gate cannot cover, and `sudo tmutil disable` before a long session
      is the operator-side fix. Nothing left to build.
- [x] **Spiro dense-weave variant** — CLOSED 2026-08-30 as won't-do (ISSUES
      CLOSED was OPEN-18): 1660 m curve, ~1107 waypoints against MAXWP=768,
      ~18-minute run, to cosmetically improve a mission that already ships
      and passes. The arithmetic is recorded so it is not re-derived.

## Genuine unsolved mysteries (cause not isolated — do not re-guess without new evidence)

- [ ] **gz-transport discovery silently fails at launch** (`0/N dogs
      advertised sensors`) even on loopback with `GZ_IP=127.0.0.1` set —
      the world builds, gz starts, no sensor topic ever appears, and
      `gz.log` is EMPTY, which is itself the clue: the failure is silent.
      Mitigated (the launch now aborts cleanly, `mission_runner` exit 3,
      sweeps retry rather than record) but not root-caused. ISSUES OPEN-22.
- [ ] **The gz pose feed needed a recycle 3 times in ~20 launches** on
      2026-08-29 — worse than the ~25 previously documented. Harnesses
      recover, so this costs wall time rather than correctness, but the
      root fix (pose subscription in a per-run subprocess, so transport
      state dies with each run instead of accumulating) is the
      highest-value infrastructure item open. ISSUES OPEN-21.

- [ ] **The zero-debounce orientation ESTOP occasionally fires, and WHY it
      fires is still not isolated** — two distinct manifestations seen so
      far, and it is not yet known whether they share one cause:
      (a) a permanent, non-falling ZOMBIE (MIT's FSM force-cuts to PASSIVE,
      the dog stays upright issuing zero displacement, `[FALL]` never fires
      since nothing is actually toppling) — characterised on star/oval
      fleets (`CLAUDE.md`'s orientation-hold A/B — "dog0 sat at wp6/7 for
      160s") and reproduced on `lissajous:15:5:7` (2026-08-26, run162),
      tripping ~15s after nav handoff; (b) a genuine FAST FALL (roll racing
      past 50 deg within seconds) — reproduced twice on `sector:15:3` in a
      3-dog fleet mix (2026-08-26, runs 201/203), each time with clean
      control-loop timing (maxPeriod 2.7-2.96ms) ruling out a host/
      scheduling stall as the trigger. Solo sector was clean 2/2 in the
      same session, matching the ALREADY-DOCUMENTED "atom is fleet-fragile"
      precedent (`CLAUDE.md`) — a course solid alone, intermittently
      fragile only in a multi-dog context. The one thing already ruled out as a fix for
      case (a): debounce LENGTH (60ms vs 200ms Fisher p~0.6, pure noise on
      an interleaved A/B). `mission_runner.py`'s `find_zombies()`
      auto-diagnoses case (a) (ESTOP in the raw log + identical N/E across
      the last few `[nav]` lines) whenever a harness timeout fires - a
      detection aid, not a fix.
      **An ESTOP-recovery mechanism now exists** (`mit_sim_main.cpp`,
      `WP_ESTOP_RECOVER`, default on) that should directly close case (a):
      on a confirmed `isEstopped()`, wait up to 15s for roll/pitch to
      settle under 30 deg and hold there 1s, then run the same proven
      boot sequence (PASSIVE -> STAND_UP -> BALANCE_STAND -> LOCOMOTION)
      used at mission start, verify it worked, and resume nav from
      wherever it already was. Verified SAFE across 5 live runs (a normal
      mission unaffected, two genuine fast-fall cases where it correctly
      deferred to the existing fall detector rather than acting, and a
      clean 3-dog PASS with no added overhead) - but every ESTOP actually
      caught live during that testing turned out to be case (b), a genuine
      fall the mechanism is deliberately built NOT to act on. It has NOT
      yet been observed completing a full recovery cycle on an actual
      case-(a) zombie - verified safe, not yet verified successful.
      **A SHM-based per-tick trace + crash oracle now exists** to gather
      the fine-grained data needed to actually isolate case (b)'s cause
      (`stm32mp1/gazebo/ShmTrace.h` writer, `shm_reaper.py` reader/
      archiver): a lock-free single-writer ring buffer (65536 records,
      500 Hz, one named segment per dog instance) logs `t`, full rpy/
      omega/vBody/z, control period, per-leg contact, op_mode and a
      finite-state-estimate flag on every tick plus a tagged event on
      `STALL`/`FALL`/`recover_*`, cheap enough to run every tick with no
      syscalls or formatting on the hot path. `RobotRunner.cpp` and
      `mit_sim_main.cpp` both call `shmtrace::log()` at their existing
      STALL/FALL/recover-* print sites. The archiver ("even the launcher
      itself can reap") is wired into BOTH detection paths that already
      existed before this: `server.py`'s poller calls
      `shm_reaper.dump_snapshot(i, "FALL", run_id=...)` the instant its
      own `[FALL]`-in-raw-log check fires (case b), and
      `mission_runner.py`'s `harness_timeout()` calls it for every dog
      `find_zombies()` flags (case a) - so BOTH manifestations in this
      entry now get an automatic, timestamped JSON archive under
      `/tmp/cheetah_conductor/archive/shm_trace/` with the full trace
      leading up to the event, without anyone needing to have a
      `shm_reaper.py --watch` process already running. Verified
      end-to-end: a live healthy run's "tick" stream decodes with
      physically sensible values throughout (vx≈3 m/s matching commanded
      cruise, z≈0.27m matching stance height); a synthetic FALL-tagged
      trace round-trips correctly through both the manual
      `dump_snapshot()` call and the standalone `watch()` poll loop; and
      a real bug in `watch()`'s cross-run change-detection (write_seq
      alone cannot tell "a new process reused this instance number" from
      "the same process kept ticking" - it can land above OR below the
      previous run's count by coincidence, and macOS reports `st_ino=0`
      for every POSIX shm fd so that could not help either) was found and
      fixed by adding a `writer_pid` field to the wire format, stamped in
      `ensure_open()` and verified against the real running
      `mit_ctrl_sim` PID. **Not yet exercised against a genuine live
      case-(b) fast-fall** - two more 3-dog `sector:15:3` fleet attempts
      this session both came back a clean 3/3 PASS (consistent with the
      already-documented ~50% intermittent rate, not evidence the bug is
      gone) - so the tooling is verified and armed, but the actual root
      cause investigation this was built to unblock has not yet had a
      real specimen to examine. Next session: keep launching
      `sector:15:3` fleet mixes (and try `parallel:30:5:8`, an equally
      tightly-tuned SAR mission, as a second candidate) until one falls,
      then read the archived trace's per-tick roll/omega/period in the
      seconds before the FALL tag.
      **printf/std::cout is now fully retired from every file in the SITL
      hot path**: `RobotRunner.cpp`, `mit_sim_main.cpp`, `WaypointNav.cpp`,
      `Stm32mp1HardwareBridge.cpp`, `MissionAnalyzer.h`,
      `SafetyChecker.cpp`, and `FSM_State_Locomotion.cpp` - all routed
      through the SHM text ring + `shm_reaper.py`'s `tail_text_to_log()`
      bridge (see `ShmTrace.h`'s own header). Two of the FSM_State_
      Locomotion.cpp sites were a genuine, independently-found RT-safety
      bug, not just a style cleanup: `locomotionSafe()`'s five
      roll/pitch/leg-position/leg-velocity printfs had ZERO gating or
      rate-limiting, meaning a marginal safety violation could printf-
      block the control loop at 500 Hz for as long as the condition
      held - exactly the failure shape this session's own SHM work was
      built to eliminate elsewhere, just never caught here; and the
      "[CONTROL FSM] Bad Request: Cannot transition..." cout is the exact
      line CLAUDE.md already documents firing 100+ times in one real
      incident (the BALANCE_STAND->STAND_UP illegal-transition saga).
      Both are now safe to fire at any rate. Verified end-to-end after
      each stage (solo dash, solo star, 3-dog fleet - zero regressions,
      zero cross-contamination between dogs' bridges, and the two fixed
      messages confirmed silent on a clean run as expected).
      **Correction, per direct instruction: "out of scope" was the wrong
      boundary** - the right question is whether the code runs in the
      dog's actual control/state-machine path, not which directory it
      lives in. Extended the conversion accordingly to every file that
      genuinely executes there: `GaitScheduler.cpp`, `ConvexMPCLocomotion.cpp`,
      `SolverMPC.cpp`, `convexMPC_interface.cpp`, `RobotState.cpp` (+ the
      unused-but-compiled `VisionRobotState.cpp` sibling), `WBIC.cpp`, and
      the remaining `FSM_State_*`/`ControlFSM.cpp` files
      (`RecoveryStand`/`StandUp`/`Passive`/`BalanceStand`/base). First
      audited every `WBC_Ctrl/TaskSet/*` file (WBIC's own task list,
      ~90 combined printf/cout sites) and found them **100% commented
      out** - genuinely zero cost, nothing to convert. Also caught a real
      gap in the audit method itself: the initial grep only matched
      `std::cout`/`std::endl` literally, missing bare `cout`/`endl` reachable
      through a `using std::cout;` declaration - broadened the search and
      found two more real sites this way (`RobotState.cpp`/
      `VisionRobotState.cpp`'s own `print()`). `ConvexMPCLocomotion.cpp`'s
      per-tick `[SCHED]` envelope-clamp/segment-change lines and
      `SolverMPC.cpp`'s `"failed to solve!"`/`"BAD ERROR 1"` (inside the
      per-MPC-cycle solve path, not just boot) are the two clusters here
      with a real, if lower-frequency-than-locomotionSafe(), repeat-firing
      profile under sustained failure. Verified compiling cleanly at every
      stage; live-regression-tested (solo dash, solo star) with the
      GaitScheduler/FSM_State/ControlFSM subset already deployed. What's
      left uncompiled-in-a-normal-build (`K_DEBUG`/`K_PRINT_EVERYTHING`
      gated blocks, confirmed never defined anywhere in this build) was
      converted too, for when someone eventually defines them. The only
      thing NOT touched this pass: `rt_*` hardware transport files
      (`rt_gazebo.cpp` etc.), third-party `JCQP`/`qpOASES` solver internals,
      and the genuinely-unreachable `BackFlip`/`FrontJump`/
      `BalanceController` demo controllers' own per-tick bodies (their
      one-shot boot-time construction messages were left as printf) -
      flag for a future pass if any of those turn out to matter.
- [x] **Four or more dogs in one fleet always fail at boot** with `STATE
      ESTIMATE WENT NON-FINITE`. CLOSED 2026-08-28 as an accepted limitation
      by operator decision, with an automatic downgrade to 2 dogs and a
      warning when 3 give trouble (ISSUES CLOSED (was OPEN-4)). Note the
      SINGLE-dog version of that same message was a genuinely different bug
      and is now fixed: `VectorNavData` was never initialised, so the
      estimator read stack garbage at control iterations 0-1 — 38 blips over
      21 runs went to 0 (ISSUES CLOSED (was OPEN-6)). Whether that also
      moves the N>=4 case has NOT been re-tested.
- [x] **Four or more dogs**: CLOSED by decision 2026-08-29 - three is the
      cap, period (ISSUES CLOSED-55). Not root-caused, and no longer a
      question being asked.
- [ ] **The trotting 100 m dash fails 0/6 in a 3-dog fleet but passes
      reliably solo**, with clean loop health (p50 2.48-2.92ms, 0% over
      4ms) and RTF 1.005 in both cases. trotRunning and walking dashes are
      fine in parallel. Until isolated, confirm any dash result single-dog.
- [ ] **One unexplained mid-dash spin-out on the atom** — roll 102° at a
      steady 2.1 m/s straight line, `w=0.00` for 10s prior, every instrument
      clean (loop max 3.07ms, 0 over-4ms, zero bridge stalls, zero pivot
      fires). Cameras/GPU load was the one uncontrolled variable at the time
      (now default off) — not re-chased since.
- [ ] **The >3.1 m/s speed tier is explicitly marked OPEN.** ~20
      configurations (segment, horizon, orientation gains, swing clearance)
      all land at 20-28m and are ruled out. Remaining candidate, never
      measured: whether the swing leg completes its trajectory and is
      actually on the ground when the MPC's contact schedule says it is —
      "measure scheduled contact against actual foot height before trying
      anything else." (Partially addressed later by `[CONTACT]`
      instrumentation showing 0.0-0.1% airborne-during-stance at 3.0/3.5, but
      the wall itself was never broken.)
- [ ] **Oval mid-course stop tip (~1-in-5) and sustained-turn entry
      (~1-in-5 at VSUS 2.4)** are both real, both documented, and neither
      has a known fix — distinguishing feature vs. the star's clean stop:
      the oval's closure sits ~10m after its 180° exit, so the dog arrives
      still carrying ~0.9 m/s instead of the star's true creep, and the tips
      are pure roll (pitch ~0), pointing at lateral velocity/roll
      oscillation from the S-weave still live when TROT→STAND fires.

## Proposed follow-ups that were never built

- [ ] **Contact detection was found to regress the real estimator** (21.3m
      → 5.6m) because it overwrote the KF's graded stance-phase trust ramp
      with a two-level on/off signal. The suggested, never-attempted fix:
      use detection to *correct* the schedule's phase only when they
      disagree, rather than replacing the schedule outright.
- [ ] **The operational joint-limit clamp is not enforced anywhere in this
      port.** Three joint-limit sets exist (mechanical, an operational
      round-number clamp, and the permissive SDK bound); the worlds use the
      mechanical set, but the middle, controller-side operational clamp
      that real hardware presumably applies has never been added.
- [ ] **UPDATED, see CLAUDE.md "GALLOPING/BOUNDING RESOLVE ATTEMPTS" and
      "THE runSwingLegControl/runContactLegControl PORT" (2026-08-27):**
      pronking/galloping/bounding all now PASS corner-broken courses
      (star/circle/expsquare) on the current stack - the stale claim here
      that pronking "never crossed at any speed" predates the async-solve
      race fix, WBIC damping, the real Go1 model, zeroVelHold, and this
      session's SIM_GAIT fix, all stacked together for the first time.
      What's actually still open is a 100m-straight-dash-only failure, a
      different mechanism per gait (pronking: flat height collapse;
      bounding: delayed tip-over; galloping: silent backward+lateral
      drift, no trip at all) - a DURATION/DISTANCE effect, not a speed
      ceiling, invisible on any course with corners or waypoint
      corrections often enough to interrupt whatever is compounding.
      `runSwingLegControl`'s actual body was never reduced to pseudocode
      in `docs/LEGGED_SPORT_REVERSE.md` (only its entry block and unrelated
      constants were recovered) and `runContactLegControl`'s only
      non-trivial new content (a per-leg force sign flip) is explicitly
      flagged there as too risky to port on a guess - so no port was
      attempted. **UPDATE, same night: that instrumentation was built,
      run, and the per-leg-bias hypothesis was ruled out (the Raibert
      correction is body-level, identical across legs by construction) -
      but the same diagnostic found and CONFIRMED against Gazebo ground
      truth that galloping's real dash failure is the state estimator,
      not the swing leg or gait control at all.** Over a 171s galloping
      dash, the estimator's own position ran away to 34+ m of error
      against truth (truth itself peaks ~11m then settles back to ~5m,
      matching the nav layer's independent GPS reading exactly) - an
      order of magnitude past any previously documented drift in this
      file. See CLAUDE.md "GALLOPING'S REAL CAUSE, CONFIRMED" for the
      full data. This makes the runSwingLegControl port very likely moot
      for galloping specifically - a controller acting on a position
      belief that wrong would misbehave regardless. NOT yet: repeated
      (small-sample discipline still applies to the exact magnitude),
      root-caused at the KF level (leading guess: the phase-based
      stance-leg trust ramp is tuned against gaits with slow, predictable
      transitions and mishandles galloping's fast asymmetric schedule),
      or checked on bounding/pronking (bounding's dash failure is a
      tip-over, a different signature that may or may not share this
      cause; pronking's is a flat height collapse, also unchecked against
      this specific mechanism).

## Cosmetic / lower priority

- [ ] **Spawn-pose fix was Z-only, not the root cause.** The dog's joints
      still spawn at `q=0`, outside the calf joint's legal range
      (-2.818..-0.888 rad) — raising spawn Z to 0.42m and suspending the
      fall-detector's z-check during boot works around the visible
      "excavating itself" symptom, but the illegal joint angle itself was
      never corrected (two earlier attempts at a joint-angle fix were
      explicitly rejected by the operator — don't re-attempt without
      checking first). The settled-foot-depth asymmetry this left behind
      (front feet clear better than rear, ~10cm vs ~17cm under) was noted
      and not investigated.
- [ ] **The reactive height governor is validated on exactly one gait, one
      speed, one course** (trotting, 2.5 m/s, star). Not yet a claim about
      2.8 m/s, trotRunning, or any other course — needs its own sweep before
      being trusted more broadly.
- [ ] **Per-robot drift calibration was never pulled from a real dog.**
      Unitree measures `walk_x`/`walk_yaw`/`run_x`/`run_y`/`run_yaw` trim per
      physical unit rather than eliminating it; this port has no equivalent
      measurement to compare its own ~1.3 cm/s in-place crawl drift against.
