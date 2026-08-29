# ISSUES.md — the official issue tracker

One line per issue where one line suffices; evidence lives in CLAUDE.md (the
archive), this file is the index of where we've been and what's left. Rules:

- **Numbers are permanent; status is said in plain words.** An issue is
  OPEN, IN PROGRESS, or CLOSED — no other vocabulary. When `OPEN-n`
  closes it moves to the CLOSED section titled `CLOSED (was OPEN-n)`,
  keeping its number forever — never renumber, never delete. The
  historical closed catalog is numbered `CLOSED-1` … `CLOSED-47`.
- **Open-item qualifiers** (why it's still open): `UNEXPLAINED` (real,
  reproduced, no root cause), `HARDWARE` (blocked on/scoped to the real
  machine), `PARKED` (built, unproven, default-off), `MITIGATED`
  (guarded, not eliminated), `DECISION` (operator's call).
- A closed entry keeps: symptom → root cause → fix → evidence. If it was
  ever *wrongly* diagnosed, the wrong turn stays in the entry — how a wrong
  turn was found is worth as much as the fix.

Last full validation: **19/19 suite PASS** (2026-08-28 ~05:15); last FAST
validation: **12/12** (2026-08-28 ~19:35, on the final build of the terrain
session - 5 cases on a fresh server + 7 with NOFEED recycling proving
itself in-line twice).

---

## OPEN

### In progress

- **OPEN-21 · The panel's gz pose feed degrades across accumulated
  in-process launches** — opened 2026-08-28, caught live by the new
  instruments mid-terrain-sweep: run747 (asphalt oval) lost ~40% of its
  trail (flown ratio 0.59 at xtrack 0.12 with wall time matching flat to
  0.1 s — a feed gap, not a dog gap; the DESYNC alarm fired and CLEARED
  around an ~8 s stall), then run748 (grass oval) had a fully dead feed:
  trail empty, gate demoted to INVALID — while bridge GPS shows the dog
  flew the complete oval (50.1×9.7 m range, 93 waypoints, 42.8 s). So
  the feed failed partial→total across two launches; a server restart
  (fresh gz-transport state) cleared it. Same family as the documented
  gz-transport discovery fragility, now measured degrading with
  accumulated Node create/destroy cycles inside one server process.
  Mitigations shipped: NOFEED verdict (near-empty trail + claimed
  completion ≠ INVALID — infrastructure, not a robot verdict; cross-check
  bridge GPS), a pose-feed heartbeat (silence >5 s logs POSE FEED
  STALLED), and a fresh-Node resubscribe self-heal. ESCALATED same
  evening: the self-heal is only TRANSIENT — resubscribe→RECOVERED
  cycles were observed, but the feed re-dies faster as launches
  accumulate, and by ~25 launches it stayed dead through six OPEN-7 reps
  (3× NOFEED) and an entire fast-suite run (11 of 12 cases demoted
  NOFEED while their controllers printed genuine PASSes — the suite was
  re-run on a fresh server for the real verdict). Root fix scoped, not
  yet built: either the server self-restarts at a safe idle point when
  the feed has been unhealthy, or (cleaner) the pose subscription moves
  to a per-run SUBPROCESS (trail_daemon-style) so transport state dies
  with each run instead of accumulating. Until then: restart the server
  before/between long campaigns; a NOFEED row is a re-run, never a
  verdict.

  **2026-08-28 night — the feed was corrupting the INSTRUMENTS, not just
  the trail, and that is now fixed.** Operator report: "I keep randomly
  seeing [DESYNC] ... sometimes I look up and the dog is moving and that
  message pops, other times it's just stopped dead." Both happen, and
  nothing on the panel could tell them apart, because EVERY world-motion
  instrument here — the drawn trail, the live DESYNC monitor, the
  post-run INVALID gate — read the SAME failing gz pose feed. When it
  goes quiet a healthy dog's displacement reads zero and the instruments
  accuse the robot. Measured, on runs that had already passed:
  - **run869** (`dash:100`): DESYNC fired twice, "world/GPS moving 0.00
    m/s" — while the BRIDGE GPS moved 37.4275 → 37.4284 lat (~100 m) and
    the mission passed 1/1 waypoints. The DESYNC/CLEARED/DESYNC/CLEARED
    alternation was the tell: a genuinely blocked dog does not recover
    and re-block every five seconds.
  - **run876** (`sector:15:3`): gated INVALID at "flew 43.1 m of a
    178.4 m plan" — while bridge GPS spanned 16.7 × 18.6 m, the correct
    box for a 15 m flower, with 17/17 waypoints and RESULT: PASS.
  - **run870 / run877** (octagon, parallel): same, at 0.0 m of trail.
  The message text also *named GPS it never read* — the monitor only ever
  differenced two pose samples.
  **Fixes (server.py):** bridge GPS is now the independent ARBITER for
  both instruments — it comes off the sim's NavSat over UDP and is
  untouched by gz-transport. DESYNC will not fire while the pose feed is
  stale (`_pose_last_t` older than the tick — two stale reads difference
  to zero, which is indistinguishable from a stopped dog), and when the
  window does look bad it checks GPS first: GPS moving ⇒ log "pose feed
  is LYING, not the dog" and suppress. The gate splits a short trail into
  NOFEED (GPS span ≳ the planned course's span ⇒ our infrastructure,
  re-run) versus INVALID (GPS agrees the body did not move ⇒ a robot
  result). Validated offline against the archived bridge logs before
  shipping: run876 25.0 m GPS span vs 24.8 m plan span → NOFEED, run870
  25.1 vs 25.5 → NOFEED; a stationary dog still reads INVALID.
  **And this explains the suite cascade**: INVALID does not trigger the
  harness's conductor recycle, NOFEED does — so misclassifying a feed
  failure as INVALID sent it down the path that never recovers, and every
  case after the feed died inherited it (the 2026-08-28 ~22:5x fast run
  went star INCONCLUSIVE→PASS on retry, then 8 PASS, then three "FAIL"s
  that were all this).

- **OPEN-7 · Terrain-aware planning: the GEOMETRY axis** — the friction
  axis is done and closed separately (CLOSED-49). What is left is the
  ground that has SHAPE rather than just grip: `rough` (±0.15 m short
  wavelength) and `rolling` (±0.35 m hills), where the gaits command every
  foot to a fixed depth below the body and the ground is not where that
  assumes.
  1. **Measure the envelope** — gait × speed on each geometry kind, and
     corner angle on each, through the ground-truth gate.
     `unittests/terrain_envelope.py` is built for exactly this (speed
     ladders on `dash:30`, angle cells on `corner:25:<angle>`, both
     resumable, NOFEED cells recycle the conductor rather than recording a
     fake failure).
  2. **Write the limitations down as limits.** Known going in: walking on
     rolling/rough is intermittent, and when it fails it fails SILENTLY —
     the estimator completes the course while the body does not (0/4 in
     the first matrix batch, then 2/2 solo; the flip was never isolated).
     That is why every terrain cell must come through the gate, and why
     the gate itself now arbitrates with bridge GPS (see OPEN-21).
  3. **Feed the measured ceiling to the pre-planner.** The mechanism is
     already shipped and inert: `BodyLimits::v_terrain_max` /
     `$WP_TERRAIN_VMAX`, deliberately UNSET on every kind because nothing
     has measured a ceiling to put there yet. Part 1 produces the number;
     encoding a guess ahead of it is the exact failure this whole program
     exists to avoid. DEM/heightmap sampling at waypoint-generation time
     (so the plan knows the ground PROFILE, not just a scalar cap) also
     remains.

- **OPEN-8 · The per-gait cornering envelope: the SPEED axis** — the
  angle axis is done and closed separately (CLOSED-48); what is left is
  the literal "how fast into X degrees before it lets go" question.
  Per-angle speed LADDERS per gait, on `corner:25:<angle>` probes so the
  angle is the only thing that varies and no other course's tuning rides
  along. The one bracket this project has ever measured — trotting 2.5
  PASS / 3.0 FAIL at ≥120° — PREDATES the `x_comp_integral` windup fix,
  so it is not evidence about the current build and has to be
  re-measured, not cited. Harness is built and resumable:
  `corner_sweep.py --gait <g>:<speed> --angles <list>` per rung, REFUSED/
  TIMEOUT cells self-retry, measured cells skip.
  **Method constraint, learned twice in this file**: run the ladder LOW to
  HIGH and measure EVERY rung — a stop-at-first-failure ladder has twice
  recorded a ceiling here that had to be retracted, because a marginal
  cell that fails one rung and passes the next is the normal case on this
  stack, not an anomaly.

### Hardware (nothing in this repo is hardware-validated)

- **OPEN-10 · Board backport: the solver on the A7** — `HARDWARE`. qpOASES
  costs 198-218 ms vs a 26 ms segment on the STM32MP1; needs the async path
  re-validated there, or JCQP made to converge on moving gaits, or the
  contact-reduced solve re-measured. None of the Mac speeds reach hardware
  until one of these lands.
- **OPEN-11 · Real GPS velocity fusion + DroneCAN participant** —
  `HARDWARE`. Velocity aiding (now default-on) is fed by the sim NavSat;
  the real dog needs the CAN GPS wired into the same path, and the
  compact-IMU stream still depends on ninjapilot keeping it alive — the
  standalone DroneCAN participant (keep-alive + stream restart) remains
  deferred.
- **OPEN-12 · RS485 bench validation** — `HARDWARE`. Field scalings, FOC
  mode value, CRC word count, per-joint gear/sign/offset — all unverified
  on a live motor; waiting on the fast RS485 adapter.
- **OPEN-13 · Pre-hardware env-var consolidation** — `HARDWARE`. Fold the
  load-bearing SIM_ vars into yaml, rework the fall detector for hardware
  (attitude/kinematics, latch-limp not process-exit), delete the dead
  flags. Scoped long ago in CLAUDE.md's SIM_ audit; untouched.
- **OPEN-14 · The QA ladder on the real machine** — `HARDWARE` umbrella.
  Stand → lie → stand → slow walk, legs off the ground first; validates
  joint signs, gearing, RS485 framing, torque scaling, IMU orientation
  before anything dynamic.
- **OPEN-15 · Real Go1: FR calf motor dead** — `HARDWARE`, physical.
  Diagnosed on the live dog (memory: go1-em-damp-stand-failure): EM_DAMP
  "stuck joint 3" = one leg's worth of joints failing to track because the
  FR knee motor returns all-zeros (0 °C vs 32-36 °C on the other 11;
  footForce[0] flat 0; hand-swing test: abad/hip track, knee dead). Fix is
  mechanical/electrical: reseat/replace the FR calf connector/cable.

### Mitigated / parked

- **OPEN-16 · Time Machine I/O storms** — `MITIGATED`. Launch gate refuses
  to start during a backup; a backup that *begins* mid-run can still kill
  a fleet. `sudo tmutil disable` before long sessions remains the real fix.
- **OPEN-17 · Parked experimental flags** — `PARKED`, all default-off, all
  documented at their sites: `SIM_FORCE_GATE` (contact validity, unproven
  A/B), `SIM_KF_VFLOOR` (velocity covariance floor — superseded in practice
  by aiding), `SIM_CONTACT_DETECT` (measured regression, kept for the
  correct-the-phase idea), `SIM_ABS_AIDING` position aiding (measured
  harmful for locomotion; nav already fuses GPS at its own layer).
- **OPEN-18 · Spiro dense-weave variant** — recorded won't-do: the 8-rev
  1660 m curve needs ~1100 waypoints against MAXWP=768 and an ~18-minute
  run for a cosmetic improvement.
- **OPEN-19 · Chase-cam lag halving** — nice-to-have. ~200-250 ms end to
  end (≈0.8 m rubber-band at sprint); halve the 100 ms follow tick and
  raise the 10 Hz camera rate if it ever matters, then re-measure GPU.

---

## CLOSED (symptom → cause → fix → evidence)

### Closed from the OPEN list

- **CLOSED-49 · Terrain-aware planning, FRICTION axis** (split out of
  OPEN-7, closed 2026-08-28 night, d9cde6e) — **characterised, documented,
  and wired into the pre-planner.**
  *Characterised*: TERRAIN.md Phase 1 (24-cell surface matrix) and
  Phase 1b (20-cell friction run) — nine selectable surface kinds from
  concrete μ0.90 to ice μ0.15, ground AND foot collisions patched on both
  sides of the contact pair so effective μ is unambiguous. The result:
  μ above a gait's demand line costs NO time, deviation scales with
  contact SOFTNESS not grip (rigid 0.08–0.14 m, mud 0.20 m, ice 0.47 m),
  and ice is the only surface that fails anything — at trot+oval, the
  highest lateral demand in the matrix.
  *Documented*: the deviation ladder and the per-kind table live in
  TERRAIN.md.
  *Wired*: `BodyLimits::mu_terrain` caps the lateral budget at
  `safety · μ · g` (safety 0.9) inside `plan()`, BEFORE any geometry is
  built, so corner speeds, braking zones and the analyzer's segment caps
  are all computed against the ground the conductor actually built.
  `server.py` passes `WP_TERRAIN_MU` from the same `terrain.py` entry that
  writes the SDF `<surface>` block and the foot collisions — ONE source of
  μ across ground, feet and plan. Verified live on ice:
  `[plan] terrain mu=0.15 caps lateral budget 2.50 -> 1.32 m/s^2`, PASS at
  ratio 1.03 (0.9·0.15·9.81 = 1.324). Unset = −1 = stock behaviour
  bit-for-bit, which is what keeps every validated flat result valid.
  **The rule is physics, not a fitted table, and it reproduces the data it
  was not fitted to**: at the 2.5 default budget the cap binds only below
  μ≈0.283 — i.e. ice alone — which is exactly the one surface Phase 1b
  measured failing. Fixed in passing: the `[plan]` summary printed the
  CALLER's pre-cap limits, contradicting the terrain line directly above
  it; it reads `planner.limits()` now.

- **CLOSED-48 · The per-gait cornering envelope, ANGLE axis** (split out
  of OPEN-8, closed 2026-08-28) — **question**: which corner angles break
  which gaits? **Answer: none of them, at base speed.**
  `unittests/corner_sweep.py` → `unittests/corner_envelope.csv`: 5 gaits
  (bounding 1.0, galloping 0.8, pronking 0.6, trotRunning 3.5, trotting
  2.5) × 10 angles (30–165°, 15° steps), solo `corner:25:<angle>` probes,
  close-leg off — **50/50 PASS, zero falls**. Wall time rises smoothly
  with angle (pronking 110.6 → 112.6 s): that is the planner braking
  harder for a sharper corner, a cost gradient, not a stability edge.
  **Consequence**: the planned 5°-notch refinement is MOOT at base speeds
  — there are no transitions left for a finer grid to bracket. Consistent
  with CLOSED (was OPEN-2): the older "flight gaits fail at 90–147.5°"
  finding was the `x_comp_integral` windup, not the angle. The speed axis
  is a different question and stays open as OPEN-8.

- **CLOSED (was OPEN-6) · Boot-time "state estimate went non-finite —
  reinitialising"** — closed 2026-08-28 night. **Symptom**: ~2/3 of every
  run printed 1–3 non-finite-estimate lines in the first few
  milliseconds, stable across three days (Aug 26 77/110, Aug 27 198/298,
  Aug 28 167/233); blip-free runs passed missions cleanly, so it was
  never fatal on its own and sat unexplained for exactly that reason.
  **Root cause**: `VectorNavData` (`common/include/SimUtilities/IMUTypes.h`)
  is a plain struct of Eigen members, and Eigen does NOT zero-initialise —
  nothing ever initialised it, so the estimator read STACK GARBAGE on
  control iterations 0 and 1, before the first sensor packet landed. Found
  by instrumenting the guard itself to name the offending field and tick
  (the item had stalled precisely because the old line said only THAT the
  estimate was non-finite): `NONFINITE-FIELDS (1) t=0.002 iter=0 bad:
  pos[0..2] vWorld[0..2] vBody[0..2] | quat=0.000 5.6e28 0.000 0.000
  pos=nan nan nan`. A quaternion of magnitude 5.6e28 goes through
  `quaternionToRotationMatrix` unnormalised, overflows, and the NaN
  reaches the KF's own state — which is what the guard was catching.
  **Fix, in two parts, and the second one is the interesting one**:
  (a) default-initialise `VectorNavData` (identity quat, zero rates);
  (b) `VectorNavData::valid`, raised by whichever backend actually
  delivers a packet (gazebo, CAN IMU, VectorNav), with
  `VectorNavOrientationEstimator` deferring its HEADING DATUM capture
  until then, and `_ori_ini_inv` explicitly identity-initialised.
  Part (b) exists because part (a) alone would have removed something
  that was doing real work by accident: the estimator captures its datum
  on first visit and keeps it for the whole run, and the old garbage was
  non-finite, so `RobotRunner`'s guard caught it and RE-CREATED the
  estimator — re-arming the capture until real data arrived. Clear the
  garbage without gating the datum and every run latches its heading
  reference to the default identity pose, i.e. the world frame instead of
  the spawn pose — the same defect shape as the star freeze when velocity
  aiding went default-on. **Evidence**: `unittests/boot_probe.py`, the
  same 21 cells (host load 0/4/8 spinners we control, terrain flat/mud/
  rough, gait, aiding on/off) run three times —

  | | blips | runs with a blip | non-PASS |
  |---|---|---|---|
  | baseline | 38 | **18/21** | 0/21 |
  | + default-init | 0 | **0/21** | 1/21 |
  | + datum gate | 0 | **0/21** | 0/21 |

  (`unittests/boot_blips_{baseline,defaultinit,datumgate}.csv`.) The
  symptom is gone, and the plan's own success criterion — "a factor that
  moves the rate, or a measured NULL" — was met in the strongest form:
  NONE of load, terrain, gait or aiding moved the rate (every cell sat at
  2–3 blips at `t=0.002 iter=0`), which is what said the cause was
  deterministic boot state rather than a scheduling or physics transient,
  and pointed the instrumentation at the right place. The one non-PASS in
  the middle column (a load-8 tip) did not recur and is N=1 — it is NOT
  claimed as evidence for part (b), which stands on its own correctness.

- **CLOSED (was OPEN-4) · Four-or-more dogs fail before standing** — closed
  2026-08-28 as an ACCEPTED LIMITATION by operator decision ("I think we can
  ignore open 4, and any time 3 dogs has trouble downgrade to 2 dogs and
  leave it there by warning the user"). The failure is real and was never
  root-caused (every dog hits `STATE ESTIMATE WENT NON-FINITE` before
  standing at N≥4, in both fleet architectures; RTF, loop starvation,
  sensor wiring, startup race and settling were all ruled out) — but N≥4
  has no operational value here: N≤3 is measured clean (contention refuted,
  0/876 ticks over 4 ms) and the panel caps at 3 slots by construction.
  **Mitigation shipped with the closure**, so fleet size self-limits
  instead of needing a human to remember: any 3-dog run in which a dog
  does not finish clean (fell / gated INVALID / no verdict) permanently
  drops the cap to 2 and says so — an orchestration-log line, a persistent
  banner on the panel, `fleet_cap`/`fleet_cap_reason` in `/api/state`, and
  a refusal (with the reason) when adding a third slot. It PERSISTS across
  server restarts (`RUN_DIR/fleet_cap.txt`) because "leave it there" means
  it must not quietly come back; only a deliberate `DELETE /api/fleet_cap`
  restores 3. A launch that still carries too many slots is TRIMMED with a
  loud log line rather than refused, so an automated caller cannot wedge.
  Verified: cap survives restart, third slot refused with the reason,
  restore works.

- **CLOSED (was OPEN-20) · The surface false-positive incident** — closed
  2026-08-28, same day, with every layer resolved and the story worth
  keeping whole. (1) Operator reported the dog "never stands, never
  moves" while the sim claimed navigation during the surface-terrain
  matrix. (2) My confirmation check was the actual false positive: it
  sampled AT the inter-cell boundary and read the NEXT cell's freshly
  spawned dog (gz truth x=2.64 = exactly the next slot's spawn offset),
  its 1-point new trail, and a finished run's post-lie-down GPS tail as
  a hallucinated run — then wrongly struck the whole matrix. (3) The
  per-run GPS-range forensic reinstated everything: every T1 dash swept
  exactly 30.1 m, every T2 octagon 17.8×17.6 m — real course-scale
  motion in every surface cell. (4) The permanent fix born from the
  scare, per the operator's own prescription ("checking the path actual
  vs path traveled trails should have told you"): mission_runner demotes
  any claimed PASS whose flown trail is <30% of the planned path to
  INVALID (exit 1). (5) On its FIRST day the gate caught four REAL
  hallucinated completions — walking on the rolling/rough GEOMETRY
  kinds, 0/4, belief finishing while the body goes nowhere — which is
  OPEN-7's mechanism, now measured (see OPEN-7). Final surface results:
  TERRAIN.md Phase 1 (19/20 real PASSes; ice fell at trot demand,
  exactly where the friction physics says it must). Lesson recorded:
  evidence sampled during a phase transition describes the transition,
  not the run — check the run window before reading state.

- **CLOSED (was OPEN-2) · Flight-gait 90–147.5° "mid-band" corner weakness** — closed
  2026-08-28: **it was the `x_comp_integral` windup**, not a property of
  the gaits or the angle. Retest on the current build (windup clamp +
  force-cap fix + velocity aiding), same course shape as the historical
  failures (`parallel:30:5:8`, close-leg off, no dash, solo, sequential):
  bounding @1.0 **PASS** (run638), galloping @0.8 **PASS** (run639),
  pronking @0.6 **PASS** (run640) — 3/3 where the 2026-08-27 record has
  FELL ×2 for each (parallel AND expsquare). Mechanism fits the original
  data exactly: parallel's 30 m straights gave the windup time to
  accumulate (the failures landed at wp07-11, i.e. minutes in), while the
  45° octagon's short legs and the star's hard-braked vertices kept
  dropping speed through the 0.3 m/s gate — which is why the two EXTREMES
  passed and the "mid-band" looked angle-shaped. The non-monotonic angle
  pattern was a course-length artifact. The 105-150° band gets re-measured
  at 15° resolution by the OPEN-8 envelope sweep (corner_envelope.csv); a
  failure there reopens this with fresh data.

- **CLOSED (was OPEN-5) · trotRunning's smooth-circle ceiling (2.75 PASS / 3.2
  FAIL)** — closed 2026-08-28: **the ceiling was the windup**, and it is
  gone. Current build, same course (`circle:9:36`, close-leg off, solo):
  3.2 **PASS ×2** (runs 644/645 — the historical FELL speed) and 3.5
  **PASS ×2** (runs 652/653 — trotRunning's own flagship speed, ~24 s
  laps). No anomaly remains to explain: a 36-gon lap is exactly the
  sustained-cruise shape the integrator needed (never brakes through the
  0.3 m/s gate), so the "ceiling" was accumulation time, not curvature.
  The old `path_analysis.py` hairpin-overshoot lead is moot — that
  overshoot-and-correct buildup was the dog fighting a growing backward
  force command, which also explains why it built over 2-3 corners
  rather than appearing at one.

- **CLOSED (was OPEN-9) · A course that rewards real gait switching** — closed
  2026-08-28, RUN, and answered in the negative — twice, at two radii.
  Experiment on `oval:40:2.5` (sustained R=2.5, the tightest oval yet):
  arm A cap-only (trotRunning 3.5, `WP_VSUS=2.2`, corner gait = itself)
  **PASS ×2, 37.8/37.9 s**; arm B real switching (`WP_GAIT_CORNER=9`, 4
  pre-planned 5↔9 changes firing per lap, verified in the stream) **PASS
  ×2, 37.8/37.7 s**; arm C trotting-only @2.4 **PASS ×2, 42.6/42.6 s**
  (runs 646-651). Conclusions: (1) the ANALYZER pays — both analyzer
  arms beat flat trotting by ~11%, reconfirming the oval thesis at a
  second radius; (2) the SWITCH buys nothing over capping — dead tie at
  R=5.0 (38.6 vs 38.2 s, prior session) and now R=2.5, because both arms
  corner at the same capped speed and capped trotRunning holds every
  sustained radius tested (my own pre-registered prediction that R=2.5
  would break it was WRONG — recorded per the rules). Structural read:
  switching can only beat capping where the corner gait corners FASTER
  than the fast gait can when speed-capped, and no such regime exists in
  this gait matrix on flat ground — tighter radii bind the STEERING cap
  (`wz≤1.2`), which slows both arms identically. Switching's remaining
  candidate value is duty-cycle/energy and terrain-class constraints
  (hardware-era questions), not lap time. The machinery stays validated
  and pinned by `oval_real_switch`.

- **CLOSED (was OPEN-3) · Atom-in-fleet fragility** — closed 2026-08-28: does not
  reproduce on the current build. Three consecutive 3-dog fleet reps
  (star+oval+atom, dash=100, recipe configs — the exact historical failing
  shape): **PASS=3 / PASS=3 / PASS=3** (runs 641-643), atom (dog2) PASS in
  all three, no Time Machine activity during any rep (`tmutil` checked per
  rep). Against the historical 0/6, three straight clean reps put the old
  failure rate away decisively. WHICH since-fixed bug was responsible is
  deliberately not asserted: the windup fits the longer-run falls but the
  documented t=11.5 s first-lobe falls poorly, and the failing era predates
  the force-cap fix, velocity aiding, and the `Kp_ori` roll gain too. What
  the tracker needs is settled: the current build's fleet atom is healthy,
  and contention was separately refuted at N≤3 (0/876 ticks over 4 ms).

- **CLOSED-47 · The dash interlude fell on the current build — two stacked,
  trace-proven bugs (found by an operator UI run; the suite was blind)** —
  closed 2026-08-28. (1) The phase gate's into-standing exemption
  ("all-stance can never de-load a loaded foot" — true, and HALF the
  hazard: all-stance also LOADS AIRBORNE feet) adopted the interlude's
  5→4 mid-FLIGHT — run599's SHM trace: contacts [0,0,0,0] on the tick
  before standing's schedule, dog ballistic at 0.8 m/s, feet slammed down
  mid-air, pitch −59°. Exemption removed; every adoption defers to seg 0.
  (2) With that fixed it STILL fell (0/3): `addStopXY`'s global-nearest
  scan resolved the loop-closure stop to **s=0** once `shiftFirstToOrigin`
  made the closure coincide with the path start — the closure got no
  braking and the dog arrived at its own stop at **vx=+2.98** (run602
  trace), a ~5 m/s² crash-stop. A regression from the shift era that
  star+dash had never been re-run across. Fix: brake every LOCAL MINIMUM
  of distance within 2 m, so a twice-visited coordinate brakes on both
  passes. After both: star+dash **3/3 PASS at 112.9-113.0 s**, matching
  the historical record. Suite gap closed: the star case's why always
  claimed the interlude but its config ran dash-less — now `dashes=[100]`.
  Lesson added to the process list: a case must exercise what its why
  claims.

- **CLOSED (was OPEN-1) · Spawn pose** — closed 2026-08-28, and the
  entry's own claims are the story. The "feet 10-17 cm under at settle"
  figure was FALSE — a frame-misread (gz pose/info reports link poses
  relative to their MODEL; the model's +0.073 was never added). Measured
  correctly (source: direct pose probe with composed frames, 2026-08-28):
  limp settle puts the belly pad flat, hips splayed to the ±0.864 stop,
  knees legally folded (−1.71/−2.35), feet within ±11 mm of the plane —
  which IS Unitree's own documented startup pose (source: operator-provided
  manual, wiki.cci.arts.ac.uk/books/robotics-lab/page/
  using-go1-edu-robot-dog-by-unitree). The physics was right all along.
  Residual artifact = the spawn-instant frames before the legs fold, plus
  ≤1 cm contact softness; fixed by spawn z 0.42→0.45 (straight-leg feet
  start at +0.02 instead of −0.01, settle equilibrium measured identical).
  ALSO corrected: this entry previously said the operator "rejected"
  fixes "twice" — what they actually rejected was my BROKEN implementation
  (it toppled the dog); recording that as rejection-of-the-approach was my
  error, called out by the operator, and is now covered by CLAUDE.md's
  "records of decisions and facts" rule.


### The two big control bugs (both stock MIT, 2019 import)

- **CLOSED-1 · The backward walk / "every gait fails a long dash"** — robots
  decayed to a stall or walked backward after ~35-60 s of cruise; three
  gaits had never crossed 100 m in project history. Cause:
  `x_comp_integral` windup — a never-reset, never-clamped integrator
  (error/velocity) feeding `A(11,9)=x_drag` in the MPC's own dynamics;
  commanded forward force measured +21.9 N → −110.2 N. MIT clamps AND
  zeroes the identical sibling construct (`rpy_int`) 100 lines up — an
  oversight, not a design choice. Fix: clamp (±1.0, `CTRL_XDRAG_CLAMP`,
  default ON) + reset in firstRun. Evidence: all 8 gaits complete the
  100 m dash (pronking/galloping/bounding for the first time ever);
  `dash_long_duration` pins it. Retired the wrong "three different
  mechanisms per gait" framing, and reversed the wrong "galloping's
  estimator is the cause" causal direction (leg odometry was faithfully
  reporting a robot the controller had stopped).
- **CLOSED-2 · Force cap set at 4 call sites with 3 values** — every mid-course
  gait/speed switch silently dropped the per-foot cap 175 → 120 N
  (mini-cheetah's number surviving in `applySchedule`): 240 N available vs
  the 315 N trotRunning needs at 3.5. Why the OVAL specifically failed
  (only course that switches). Fix: one `mpcForceCap()` accessor.
  Also invalidated "trotRunning cannot hold this curve" (measured at 76 %
  of needed force).

### Mid-motion gait switching (three transients deep)

- **CLOSED-3 · Phase-misaligned gait adoption** — `cmpc_gait` adopted the
  instant it changed; trot-pair stance tables disagree on 40 % of the
  cycle, so a 9→5 could command all four feet airborne mid-stride and a
  5→9 could schedule stance onto ballistic feet. Fix: phase-gated adoption
  at segment 0 (into-standing immediate). Missions replay
  near-tick-identically → the "deterministic" failures were one phase.
- **CLOSED-4 · Hot arc entry** — the plan reached the sustained cap exactly AT
  the arc; body lag + trotRunning overshoot meant 2.7-3.0 actual against a
  2.4 plan, max braking+turning at minimum margin. Fix: analyzer settle
  lead (`entry_settle_x·track_lag_s·v_cap` ≈ 5.8 m before the arc,
  `WP_ENTRY_SETTLE`). Cost ~1.2 s, by design.
- **CLOSED-5 · The clock teleport** — `applySchedule`'s segment-time change
  alters the divisor in `phase=(counter/iters)%10` mid-count, teleporting
  the segment index ~1 s after the adoption the gate had aligned. Fix:
  `_iterSegOffset` phase origin, rebased at every segment-time change.
  With CLOSED-3/CLOSED-4/CLOSED-5: first genuine switching passes in project history,
  3/3 at 38.6-38.7 s; suite case `oval_real_switch` pins all three.
  (Swing-continuity re-capture added alongside; measured insufficient
  alone, kept as free defense.)
- **CLOSED-6 · The fast oval itself** — shipped as cap-only trotRunning@3.5
  (`WP_GAIT_CORNER=5`), 4/4 + 3/3, ~38 s vs the 80 s trotting fallback.
  Archaeology: the milestone "switching oval" NEVER switched — the pre-fix
  SIM_GAIT override discarded the analyzer's writes; cap-only is what
  history actually validated.

### Estimation

- **CLOSED-7 · Galloping's ~10 % velocity/position under-read** — discriminated
  with `SIM_LEGVEL_DBG` (raw measurement recoverable from the logged
  blend): raw-meas/fused = 1.022, fused/truth = 0.917 → the raw odometry
  itself is low (slip/schedule mismatch under 40 % flight); the KF blend
  innocent. Fix: GPS velocity aiding **default-on** (σ=0.02) → 0.995;
  symmetric gaits unaffected (0.994). The feature was built for exactly
  this and mis-evaluated for a session while CLOSED-1 corrupted its tests.
- **CLOSED-8 · Aiding default's boot tip** — with aiding firing from tick 1,
  the whole stand/engage ran velocity corrections rotated for a NORTH
  spawn (`setSpawnYawRad` arrives via navThread much later); correct at
  bearing 0, 126° wrong at star's 162° — the dog tipped and ESTOPped at
  engagement. Fix: bridge constructor reads `WP_SPAWN_BEARING_DEG`,
  correct from tick 1. Star PASS 63.8 s.
- **CLOSED-9 · GPS velocity aiding's original "it made things worse" saga** —
  three real bugs found chasing it: estimator-frame rotation (spawn-yaw
  zeroed frame), 10 Hz staleness re-applied ~49×/sample, and the decisive
  one — the `absAiding` pointer only wired under `SIM_ABS_AIDING`, so
  every prior "test" ran disconnected. All fixed.
- **CLOSED-10 · KF velocity-covariance collapse** — measured (`SIM_KF_HEALTH`):
  P_vel collapses to ~0.001 within ~1 s (the filter's own algebraic steady
  state, not corruption), gain ~0.01. Contributing, not primary
  (`SIM_KF_VFLOOR` parked; CLOSED-1 was the real driver, aiding the real fix).
- **CLOSED-11 · dt-aware KF integration** — stalled ticks were integrated as
  2 ms; now uses measured dt (clamped 20×).
- **CLOSED-12 · `max_pos_error` clamp tested and exonerated** — disabling it
  entirely did not stop the decay (honest negative that redirected CLOSED-1's
  hunt from the reference to the dynamics model).
- **CLOSED-13 · SIM_KF_UNCAP wrong diagnosis retracted** — Unitree ships MIT's
  identical covariance cap; it never bound speed.
- **CLOSED-14 · SIM_CONTACT_DETECT regression** — replacing the graded trust
  ramp with a two-level signal cut walking2 21 m → 5.6 m; parked off.
- **CLOSED-15 · Cheater-mode contamination** — `SIM_CHEATER=0` still enabled it
  (getenv truthiness); entire "real-estimator" tables retracted and
  re-measured; flag deleted outright.

### Planner / navigation / missions

- **CLOSED-16 · Braking zone shorter than stopping distance** — plan a_lon must
  be LOWER than physical; unlocked 2.5-3.0 cruise on the star.
- **CLOSED-17 · Steering-rate cap** — corner v was traction-only; at R≈0.03 the
  body can't steer that fast → "elephant foot" loops. `v = min(v_traction,
  wz_max/κ)`.
- **CLOSED-18 · Hairpin pivot follower** — pure-pursuit target landing behind
  the nose plane at 162° vertices; pivot branch (gated to planned-creep
  after it fired at cruise on the dash).
- **CLOSED-19 · Stops are part of the plan** — `addStopXY`/`setEndStop`; the
  loop-closure and mission end brake in the profile instead of
  crash-stopping from cruise. Plus steered deceleration through the first
  0.5 s of every stop (fixed the oval's sideways stop tips), and the
  near-180-reversal-registered-as-stop rule (collinear points defeat
  curvature).
- **CLOSED-20 · The stop/lie-down/stand interlude chain** — illegal
  BALANCE_STAND→STAND_UP transition (route via PASSIVE), edamp coverage,
  re-entering STAND_UP skips its ramp (progress pinned at 1.0 — a launch,
  not a stand), fall-z gate suspended around commanded lie-downs,
  debounced orientation window replacing MIT's zero-debounce ESTOP during
  stop windows, ESTOP-recovery ladder.
- **CLOSED-21 · `corner:` mission "broken"** — it simply had NO recipe (every
  cornering course needs its graded-corridor tuning). One wide tuning:
  45/90/135 all PASS. (Also fixed en route: `mission_opening_bearing_rad`
  mis-yawing corner's spawn.) `WP_PLANNER=1` claim corrected — it was
  always set; tuning was what was missing.
- **CLOSED-22 · SAR/lissajous/spiro catalog** — seven new missions, each
  needing the same two levers (graded corridor + gentle a_lon); sector's
  duplicate-centre waypoints; per-angle turn-grading probes
  (`planner_probe.cpp`); `shiftFirstToOrigin`; `WP_FINAL_ACCEPT`; spiro =
  makeAtom's own formula at k=lobes, depth≈1.
- **CLOSED-23 · closeFinalLeg** — periodic curves end at home (0.00-1.20 m),
  generators' leftovers didn't (6.9/15/18/46.1 m); "Close final leg"
  default ON; measured cost +6.8 %→+17 % monotonic in gap; dash exempt
  (would become an out-and-back). appendDash branch interaction verified
  end-to-end (circle+dash: walks home, sprints exactly 100.00 m).
- **CLOSED-24 · Dash semantics** — standalone dash was wired to out-and-back
  (the reversal was never supposed to exist); `makeDash` = one straight
  leg. Dash-as-finish appends return-to-wp0 + sprint on the closing
  tangent. Dash slot defaults: dash=0, close_leg=off (kind_slot_defaults).
- **CLOSED-25 · Oval geometry/config history** — VSUS 2.6→2.4 re-sweep (bisect
  proved no regression, the cell was always marginal); trot-in-place
  settle measured harmful (7-of-8) and reverted; run-in experiment
  reverted by its own A/B (0/8).
- **CLOSED-26 · Analyzer/gait-decider foundations** — duration-not-severity
  regime classification; blame-the-turn cost attribution; sustained-curve
  speed envelope (curvature cannot express duration).

### Conductor / panel / infrastructure

- **CLOSED-27 · SIM_GAIT override discarded every runtime `cmpc_gait` write** —
  the analyzer's switches were phantom prints for their entire history.
  Ground-truth `[SCHED] gait changed` logging added at the real site.
- **CLOSED-28 · Async teardown race** — `_teardown_done` Event + real
  `p.wait()`; launches structurally cannot start during teardown (was
  producing bogus 10.4 s PASSes from contaminated logs).
- **CLOSED-29 · Stale-process contamination family** — port sweeps at bridge
  and launch; tail-text reaper kill by cmdline; **SHM ring replay false
  PASS** (a dead writer's ring replayed into a fresh log → run430
  "COMPLETE" in 9 s with run429's time) fixed by run-id-first staleness;
  `run_id` stamped into the SHM header itself (32-byte layout, asserts
  both sides) so SHM + conductor + logs + archives share ONE number.
  `[RUNID]` on every ctrl health line — added for exactly this, paid for
  itself the same evening.
- **CLOSED-30 · Suite integrity** — SETTLE_S false-PASS; verdicts matched at
  the wrong layer; `--stall-timeout` false positives (progress-watching,
  not bigger numbers); harness timeout = exit 2 ≠ verdict; **exit-2
  counted as PASS** (hid CLOSED-8's frozen star behind a 12/12) → retry once
  then FAIL; timeouts derived from geometry×speed instead of hand-picked
  (BASELINE_S flat table retired); full-catalog tier (19 cases, `--fast`
  quick gate).
- **CLOSED-31 · Harness rewrote the operator's draft** — every automated run
  left its config in the panel (the "not validated combo" warnings the
  operator kept seeing). mission_runner now launches via the explicit
  `slots` body; the draft belongs to the human. Cap-aware warning
  comparison (model ceiling) killed the last unclearable warning.
- **CLOSED-32 · Panel bug family** — remove-button stale-index race;
  launch-button alert pileup; `Cache-Control: no-store` (stale app.js
  through hard reloads); one-shot draft sync; mission-change recipe snap
  (the atom spin-out); gait dropdown hardcoded 5 of 8 gaits; default
  draft slots drifted from RECIPES (default oval launched the
  proven-broken config); poller stuck at "running" (`[FALL]` after
  MISSION COMPLETE; done keyed on the judge line); dog-0 undeletable +
  "delete all"; recipe notes updated with close-leg costs; octagon
  labelled honestly + smooth circle selectable.
- **CLOSED-33 · `SystemExit` swallowed in the launch thread** — unknown mission
  specs wedged launches silently; caught + surfaced as phase=error.
- **CLOSED-34 · GZ multicast off-host** — `GZ_IP=127.0.0.1`; the
  frozen-at-spawn "0/N dogs came up" class.
- **CLOSED-35 · TRAIL_MAX truncation; timeout-240 SIGKILL mid-lissajous;
  archive_log before truncation; reports (planned-vs-flown) per run.**
- **CLOSED-36 · Contention refuted at N≤3** — equal-load design (identical
  dash:100 ×N), 876 samples, zero ticks >4 ms at any N; the real 13-18 ms
  stalls were Time Machine. Load-budget model: per-tick cost is FLAT
  across mission kinds — DURATION is the load variable (dash@0.6 = 392
  dog-seconds vs star's 129).
- **CLOSED-37 · Chase cam** — live free-floating design existed (stale backlog
  entry); measured A/B: zero control-loop cost; lag characterized.

### Model / port foundations (the early wall of fixes)

- **CLOSED-38 · Eigen NEON alignment traps; JCQP AVX2→scalar/NEON; gcc-15
  Goldfarb gate; null LCM shim; qpOASES CMake flag clobber.**
- **CLOSED-39 · PeriodicTask free-ran on macOS** (500 Hz loop at 1.9 MHz) —
  absolute-deadline sleep.
- **CLOSED-40 · locomotionSafe 0.18 m lateral limit** (mini-cheetah's abad) —
  the original "MPC tumbles at gait start", plus the fabs(bool) typo.
- **CLOSED-41 · Gait numbers ≥10 collide with omni rewrite** — walking/
  walking2/galloping unreachable; moved to 20/21/22.
- **CLOSED-42 · Lateral capture point ~22× too weak** (stray ·dtMPC on y).
- **CLOSED-43 · Inline MPC solve 60-105 ms on the A7** — async worker →
  setup_problem data race (both solvers "correctly" returning zero force)
  → solver tuning (ρ=0.6/60, single precision, contact reduction 349→32
  ms) → **JCQP non-convergence under moving gaits** (¼ of required force;
  the fix that retired "why is it satisfied at z=0.204") → qpOASES on the
  Mac; WBC decimation caching; heading hold (upstream had NONE); walking's
  yaw-rate feedback (9.3→93.6 m); zeroVelHold; getFlightState port;
  entry-height ramp.
- **CLOSED-44 · Go1 model corrections vs Unitree's own binary** — knee gear
  9.4995 (not a second tau max), maxLegLength 0.430, real MPC inertia,
  rotor mass/inertia/locations (copy-paste from mini-cheetah), force cap
  175 (bodyweight ratio), mechanical joint limits (three limit sets,
  we'd used the wrong layer), WBIC Kd damping (trot@1.0 9-11×), atom
  Kp_ori roll 40→70.
- **CLOSED-45 · GamepadCommand uninitialised; block-buffered stdout losing
  logs; fall detector (z-threshold killed valid runs; process-exit
  semantics documented as hardware-wrong); stall "mitigation" worse than
  the stall (removed — detect and log only); GPS_HZ 10 (uncited) → 20
  (ZED-F9P) → 50 (NEO-M9V, datasheet-verified), selectable.**

### Real dog (separate thread from the sim work)

- **CLOSED-46 · "Wrong Model" red herring + EM_DAMP decode** — `sn[1]=5` print
  is cosmetic; the stand abort is FSM_State_StandUp's stuck-joint counter
  (trailing digit = COUNT of bad joints). LowState wire layout recovered
  and verified live. Root cause of the failed stand: OPEN-15 (dead FR
  calf motor — mechanical).

---

## Process lessons that keep earning their keep (from memory)

- Label every claim by its evidence; N<6 is not "fixed" (bit us ≥5×).
- A finding measured only inside a multi-dog batch is UNVERIFIED until
  solo-tested (three collapsed in one night).
- Regression cases must spend the DURATION a failure needs, not just cover
  the shape (the suite missed CLOSED-1 this way).
- A case must exercise what its why CLAIMS it exercises — the star case
  described the dash interlude for weeks while running dash-less, and two
  real bugs (CLOSED-47) lived in the gap until an operator UI click found them.
- One fact, one place: the decel ramps, draft slots, gait dropdown, force
  cap, and recipe notes all drifted as duplicated sources of truth.
- `pgrep -f` pollers match their own command line (86 min lost); prefer
  the job's own state.
- After raw-API testing, verify `/api/state` against recipes — don't wait
  for the panel warning (now moot: automation no longer touches the draft).
- The first full suite after a DEFAULT change is the test that matters
  most — and the suite must never count "no verdict" as "pass".
