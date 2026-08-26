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

## Genuine unsolved mysteries (cause not isolated — do not re-guess without new evidence)

- [ ] **The zero-debounce orientation ESTOP occasionally fires right at nav
      handoff and produces a permanent zombie** — MIT's FSM force-cuts to
      PASSIVE, the dog never falls (so `[FALL]` never fires) and just sits
      frozen issuing zero displacement for the rest of the run, silently
      burning the full mission timeout. Previously characterised on star/oval
      fleets (`CLAUDE.md`'s orientation-hold A/B — "dog0 sat at wp6/7 for
      160s"); reproduced again in a randomized 3-dog fleet trial
      (2026-08-26, run162) on `lissajous:15:5:7`, tripping within ~15s of
      nav taking the stick rather than mid-mission. The one thing already
      ruled out as the fix: debounce LENGTH (60ms vs 200ms Fisher p~0.6,
      pure noise on an interleaved A/B) — the actual trigger for why the
      trip fires at all was never chased. `mission_runner.py`'s
      `find_zombies()` now auto-diagnoses this pattern (ESTOP in the raw
      log + identical N/E across the last few `[nav]` lines) whenever a
      harness timeout fires, so it no longer takes a manual log read to
      tell apart from a merely-slow mission — that's a detection aid, not a
      fix.
- [ ] **Four or more dogs in one fleet always fail at boot** with `STATE
      ESTIMATE WENT NON-FINITE`. Ruled out: real-time factor, loop
      starvation, sensor topic wiring, a startup race (readiness gate didn't
      fix it), settling time (20s didn't either).
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
- [ ] **Unitree's `runSwingLegControl`/`runContactLegControl` split (and
      `trajPlanner` proper) was only reversed structurally, never ported.**
      This is the leading remaining candidate for why pronking and
      galloping still fail (galloping now completes a mission at 0.8 m/s,
      but pronking has never crossed at any speed tried).

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
