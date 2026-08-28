# ISSUES.md — the official issue tracker

One line per issue where one line suffices; evidence lives in CLAUDE.md (the
archive), this file is the index of where we've been and what's left. Rules:

- **IDs are permanent.** `OPEN-n` moves to the CLOSED section with the same
  number when it closes (as `C-OPEN-n`) — never renumber, never delete.
- **Statuses**: `DECISION` (operator's call), `RETEST` (measured before a
  since-fixed bug; probably stale), `UNEXPLAINED` (real, reproduced, no root
  cause), `NOT-RUN` (infrastructure exists, experiment doesn't),
  `HARDWARE` (blocked on/scoped to the real machine), `PARKED` (built,
  unproven, default-off), `MITIGATED` (guarded, not eliminated).
- A closed entry keeps: symptom → root cause → fix → evidence. If it was
  ever *wrongly* diagnosed, the wrong turn stays in the entry — how a wrong
  turn was found is worth as much as the fix.

Last full validation: **19/19 suite PASS** (2026-08-28 ~05:15, every mission
at recipe defaults, velocity aiding default-on, all fixes active together).

---

## OPEN

### Real, reproduced, unexplained

- **OPEN-4 · Four-or-more dogs: `STATE ESTIMATE WENT NON-FINITE` before
  standing** — `UNEXPLAINED`. Every dog, both fleet architectures. Ruled
  out: RTF, loop starvation, sensor wiring, startup race, settling.
  Distinct from contention (refuted at N≤3).
- **OPEN-6 · Boot-time "state estimate went non-finite — reinitialising"
  ×2 every run** — `UNEXPLAINED`, minor. Deterministic, harmless-looking,
  never eliminated. Likely tied to OPEN-1's spawn clip.
- **OPEN-7 · Terrain: standing on the farm mesh works, walking doesn't** —
  `UNEXPLAINED`/unfinished. Contact-terminated stand improved it; every
  validated result is on flat ground. Proper fix needs touchdown-torque
  contact + a plane fit, per the old analysis.

### In progress

- **OPEN-8 · The per-gait cornering envelope** — angle axis ANSWERED
  2026-08-28; speed axis remains. First tranche complete
  (`unittests/corner_sweep.py` → `unittests/corner_envelope.csv`):
  5 gaits (bounding 1.0, galloping 0.8, pronking 0.6, trotRunning 3.5,
  trotting 2.5) × 10 angles (30–165°, 15° steps), solo `corner:25:<angle>`
  probes, close-leg off — **50/50 PASS, zero falls**. At its established
  base speed, NO angle in the range breaks ANY of the five gaits on the
  current build (consistent with C-OPEN-2: the old angle findings were the
  windup). Wall time rises smoothly with angle (e.g. pronking 110.6 →
  112.6 s), which is the planner braking harder for sharper corners —
  the cost gradient, not a stability edge. Consequence: there are no
  transitions for the planned 5°-notch refinement to bracket — the
  5°-resolution half of the stretch goal is moot at base speeds. What
  remains OPEN is the SPEED axis: per-angle speed ladders to find each
  gait's ceiling as a function of angle (the literal "how fast into X
  degrees" question — trotting's old 2.5 PASS / 3.0 FAIL at ≥120° is the
  only such bracket measured, and it predates the windup fix, so even
  that needs re-measuring). The harness is built for it: seed
  `corner_sweep.py --gait <g>:<speed> --angles <list>` per rung; REFUSED/
  TIMEOUT cells self-retry.

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

- **C-OPEN-2 · Flight-gait 90–147.5° "mid-band" corner weakness** — closed
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

- **C-OPEN-5 · trotRunning's smooth-circle ceiling (2.75 PASS / 3.2
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

- **C-OPEN-9 · A course that rewards real gait switching** — closed
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

- **C-OPEN-3 · Atom-in-fleet fragility** — closed 2026-08-28: does not
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

- **C-47 · The dash interlude fell on the current build — two stacked,
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

- **C-OPEN-1 · Spawn pose** — closed 2026-08-28, and the
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

- **C-1 · The backward walk / "every gait fails a long dash"** — robots
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
- **C-2 · Force cap set at 4 call sites with 3 values** — every mid-course
  gait/speed switch silently dropped the per-foot cap 175 → 120 N
  (mini-cheetah's number surviving in `applySchedule`): 240 N available vs
  the 315 N trotRunning needs at 3.5. Why the OVAL specifically failed
  (only course that switches). Fix: one `mpcForceCap()` accessor.
  Also invalidated "trotRunning cannot hold this curve" (measured at 76 %
  of needed force).

### Mid-motion gait switching (three transients deep)

- **C-3 · Phase-misaligned gait adoption** — `cmpc_gait` adopted the
  instant it changed; trot-pair stance tables disagree on 40 % of the
  cycle, so a 9→5 could command all four feet airborne mid-stride and a
  5→9 could schedule stance onto ballistic feet. Fix: phase-gated adoption
  at segment 0 (into-standing immediate). Missions replay
  near-tick-identically → the "deterministic" failures were one phase.
- **C-4 · Hot arc entry** — the plan reached the sustained cap exactly AT
  the arc; body lag + trotRunning overshoot meant 2.7-3.0 actual against a
  2.4 plan, max braking+turning at minimum margin. Fix: analyzer settle
  lead (`entry_settle_x·track_lag_s·v_cap` ≈ 5.8 m before the arc,
  `WP_ENTRY_SETTLE`). Cost ~1.2 s, by design.
- **C-5 · The clock teleport** — `applySchedule`'s segment-time change
  alters the divisor in `phase=(counter/iters)%10` mid-count, teleporting
  the segment index ~1 s after the adoption the gate had aligned. Fix:
  `_iterSegOffset` phase origin, rebased at every segment-time change.
  With C-3/C-4/C-5: first genuine switching passes in project history,
  3/3 at 38.6-38.7 s; suite case `oval_real_switch` pins all three.
  (Swing-continuity re-capture added alongside; measured insufficient
  alone, kept as free defense.)
- **C-6 · The fast oval itself** — shipped as cap-only trotRunning@3.5
  (`WP_GAIT_CORNER=5`), 4/4 + 3/3, ~38 s vs the 80 s trotting fallback.
  Archaeology: the milestone "switching oval" NEVER switched — the pre-fix
  SIM_GAIT override discarded the analyzer's writes; cap-only is what
  history actually validated.

### Estimation

- **C-7 · Galloping's ~10 % velocity/position under-read** — discriminated
  with `SIM_LEGVEL_DBG` (raw measurement recoverable from the logged
  blend): raw-meas/fused = 1.022, fused/truth = 0.917 → the raw odometry
  itself is low (slip/schedule mismatch under 40 % flight); the KF blend
  innocent. Fix: GPS velocity aiding **default-on** (σ=0.02) → 0.995;
  symmetric gaits unaffected (0.994). The feature was built for exactly
  this and mis-evaluated for a session while C-1 corrupted its tests.
- **C-8 · Aiding default's boot tip** — with aiding firing from tick 1,
  the whole stand/engage ran velocity corrections rotated for a NORTH
  spawn (`setSpawnYawRad` arrives via navThread much later); correct at
  bearing 0, 126° wrong at star's 162° — the dog tipped and ESTOPped at
  engagement. Fix: bridge constructor reads `WP_SPAWN_BEARING_DEG`,
  correct from tick 1. Star PASS 63.8 s.
- **C-9 · GPS velocity aiding's original "it made things worse" saga** —
  three real bugs found chasing it: estimator-frame rotation (spawn-yaw
  zeroed frame), 10 Hz staleness re-applied ~49×/sample, and the decisive
  one — the `absAiding` pointer only wired under `SIM_ABS_AIDING`, so
  every prior "test" ran disconnected. All fixed.
- **C-10 · KF velocity-covariance collapse** — measured (`SIM_KF_HEALTH`):
  P_vel collapses to ~0.001 within ~1 s (the filter's own algebraic steady
  state, not corruption), gain ~0.01. Contributing, not primary
  (`SIM_KF_VFLOOR` parked; C-1 was the real driver, aiding the real fix).
- **C-11 · dt-aware KF integration** — stalled ticks were integrated as
  2 ms; now uses measured dt (clamped 20×).
- **C-12 · `max_pos_error` clamp tested and exonerated** — disabling it
  entirely did not stop the decay (honest negative that redirected C-1's
  hunt from the reference to the dynamics model).
- **C-13 · SIM_KF_UNCAP wrong diagnosis retracted** — Unitree ships MIT's
  identical covariance cap; it never bound speed.
- **C-14 · SIM_CONTACT_DETECT regression** — replacing the graded trust
  ramp with a two-level signal cut walking2 21 m → 5.6 m; parked off.
- **C-15 · Cheater-mode contamination** — `SIM_CHEATER=0` still enabled it
  (getenv truthiness); entire "real-estimator" tables retracted and
  re-measured; flag deleted outright.

### Planner / navigation / missions

- **C-16 · Braking zone shorter than stopping distance** — plan a_lon must
  be LOWER than physical; unlocked 2.5-3.0 cruise on the star.
- **C-17 · Steering-rate cap** — corner v was traction-only; at R≈0.03 the
  body can't steer that fast → "elephant foot" loops. `v = min(v_traction,
  wz_max/κ)`.
- **C-18 · Hairpin pivot follower** — pure-pursuit target landing behind
  the nose plane at 162° vertices; pivot branch (gated to planned-creep
  after it fired at cruise on the dash).
- **C-19 · Stops are part of the plan** — `addStopXY`/`setEndStop`; the
  loop-closure and mission end brake in the profile instead of
  crash-stopping from cruise. Plus steered deceleration through the first
  0.5 s of every stop (fixed the oval's sideways stop tips), and the
  near-180-reversal-registered-as-stop rule (collinear points defeat
  curvature).
- **C-20 · The stop/lie-down/stand interlude chain** — illegal
  BALANCE_STAND→STAND_UP transition (route via PASSIVE), edamp coverage,
  re-entering STAND_UP skips its ramp (progress pinned at 1.0 — a launch,
  not a stand), fall-z gate suspended around commanded lie-downs,
  debounced orientation window replacing MIT's zero-debounce ESTOP during
  stop windows, ESTOP-recovery ladder.
- **C-21 · `corner:` mission "broken"** — it simply had NO recipe (every
  cornering course needs its graded-corridor tuning). One wide tuning:
  45/90/135 all PASS. (Also fixed en route: `mission_opening_bearing_rad`
  mis-yawing corner's spawn.) `WP_PLANNER=1` claim corrected — it was
  always set; tuning was what was missing.
- **C-22 · SAR/lissajous/spiro catalog** — seven new missions, each
  needing the same two levers (graded corridor + gentle a_lon); sector's
  duplicate-centre waypoints; per-angle turn-grading probes
  (`planner_probe.cpp`); `shiftFirstToOrigin`; `WP_FINAL_ACCEPT`; spiro =
  makeAtom's own formula at k=lobes, depth≈1.
- **C-23 · closeFinalLeg** — periodic curves end at home (0.00-1.20 m),
  generators' leftovers didn't (6.9/15/18/46.1 m); "Close final leg"
  default ON; measured cost +6.8 %→+17 % monotonic in gap; dash exempt
  (would become an out-and-back). appendDash branch interaction verified
  end-to-end (circle+dash: walks home, sprints exactly 100.00 m).
- **C-24 · Dash semantics** — standalone dash was wired to out-and-back
  (the reversal was never supposed to exist); `makeDash` = one straight
  leg. Dash-as-finish appends return-to-wp0 + sprint on the closing
  tangent. Dash slot defaults: dash=0, close_leg=off (kind_slot_defaults).
- **C-25 · Oval geometry/config history** — VSUS 2.6→2.4 re-sweep (bisect
  proved no regression, the cell was always marginal); trot-in-place
  settle measured harmful (7-of-8) and reverted; run-in experiment
  reverted by its own A/B (0/8).
- **C-26 · Analyzer/gait-decider foundations** — duration-not-severity
  regime classification; blame-the-turn cost attribution; sustained-curve
  speed envelope (curvature cannot express duration).

### Conductor / panel / infrastructure

- **C-27 · SIM_GAIT override discarded every runtime `cmpc_gait` write** —
  the analyzer's switches were phantom prints for their entire history.
  Ground-truth `[SCHED] gait changed` logging added at the real site.
- **C-28 · Async teardown race** — `_teardown_done` Event + real
  `p.wait()`; launches structurally cannot start during teardown (was
  producing bogus 10.4 s PASSes from contaminated logs).
- **C-29 · Stale-process contamination family** — port sweeps at bridge
  and launch; tail-text reaper kill by cmdline; **SHM ring replay false
  PASS** (a dead writer's ring replayed into a fresh log → run430
  "COMPLETE" in 9 s with run429's time) fixed by run-id-first staleness;
  `run_id` stamped into the SHM header itself (32-byte layout, asserts
  both sides) so SHM + conductor + logs + archives share ONE number.
  `[RUNID]` on every ctrl health line — added for exactly this, paid for
  itself the same evening.
- **C-30 · Suite integrity** — SETTLE_S false-PASS; verdicts matched at
  the wrong layer; `--stall-timeout` false positives (progress-watching,
  not bigger numbers); harness timeout = exit 2 ≠ verdict; **exit-2
  counted as PASS** (hid C-8's frozen star behind a 12/12) → retry once
  then FAIL; timeouts derived from geometry×speed instead of hand-picked
  (BASELINE_S flat table retired); full-catalog tier (19 cases, `--fast`
  quick gate).
- **C-31 · Harness rewrote the operator's draft** — every automated run
  left its config in the panel (the "not validated combo" warnings the
  operator kept seeing). mission_runner now launches via the explicit
  `slots` body; the draft belongs to the human. Cap-aware warning
  comparison (model ceiling) killed the last unclearable warning.
- **C-32 · Panel bug family** — remove-button stale-index race;
  launch-button alert pileup; `Cache-Control: no-store` (stale app.js
  through hard reloads); one-shot draft sync; mission-change recipe snap
  (the atom spin-out); gait dropdown hardcoded 5 of 8 gaits; default
  draft slots drifted from RECIPES (default oval launched the
  proven-broken config); poller stuck at "running" (`[FALL]` after
  MISSION COMPLETE; done keyed on the judge line); dog-0 undeletable +
  "delete all"; recipe notes updated with close-leg costs; octagon
  labelled honestly + smooth circle selectable.
- **C-33 · `SystemExit` swallowed in the launch thread** — unknown mission
  specs wedged launches silently; caught + surfaced as phase=error.
- **C-34 · GZ multicast off-host** — `GZ_IP=127.0.0.1`; the
  frozen-at-spawn "0/N dogs came up" class.
- **C-35 · TRAIL_MAX truncation; timeout-240 SIGKILL mid-lissajous;
  archive_log before truncation; reports (planned-vs-flown) per run.**
- **C-36 · Contention refuted at N≤3** — equal-load design (identical
  dash:100 ×N), 876 samples, zero ticks >4 ms at any N; the real 13-18 ms
  stalls were Time Machine. Load-budget model: per-tick cost is FLAT
  across mission kinds — DURATION is the load variable (dash@0.6 = 392
  dog-seconds vs star's 129).
- **C-37 · Chase cam** — live free-floating design existed (stale backlog
  entry); measured A/B: zero control-loop cost; lag characterized.

### Model / port foundations (the early wall of fixes)

- **C-38 · Eigen NEON alignment traps; JCQP AVX2→scalar/NEON; gcc-15
  Goldfarb gate; null LCM shim; qpOASES CMake flag clobber.**
- **C-39 · PeriodicTask free-ran on macOS** (500 Hz loop at 1.9 MHz) —
  absolute-deadline sleep.
- **C-40 · locomotionSafe 0.18 m lateral limit** (mini-cheetah's abad) —
  the original "MPC tumbles at gait start", plus the fabs(bool) typo.
- **C-41 · Gait numbers ≥10 collide with omni rewrite** — walking/
  walking2/galloping unreachable; moved to 20/21/22.
- **C-42 · Lateral capture point ~22× too weak** (stray ·dtMPC on y).
- **C-43 · Inline MPC solve 60-105 ms on the A7** — async worker →
  setup_problem data race (both solvers "correctly" returning zero force)
  → solver tuning (ρ=0.6/60, single precision, contact reduction 349→32
  ms) → **JCQP non-convergence under moving gaits** (¼ of required force;
  the fix that retired "why is it satisfied at z=0.204") → qpOASES on the
  Mac; WBC decimation caching; heading hold (upstream had NONE); walking's
  yaw-rate feedback (9.3→93.6 m); zeroVelHold; getFlightState port;
  entry-height ramp.
- **C-44 · Go1 model corrections vs Unitree's own binary** — knee gear
  9.4995 (not a second tau max), maxLegLength 0.430, real MPC inertia,
  rotor mass/inertia/locations (copy-paste from mini-cheetah), force cap
  175 (bodyweight ratio), mechanical joint limits (three limit sets,
  we'd used the wrong layer), WBIC Kd damping (trot@1.0 9-11×), atom
  Kp_ori roll 40→70.
- **C-45 · GamepadCommand uninitialised; block-buffered stdout losing
  logs; fall detector (z-threshold killed valid runs; process-exit
  semantics documented as hardware-wrong); stall "mitigation" worse than
  the stall (removed — detect and log only); GPS_HZ 10 (uncited) → 20
  (ZED-F9P) → 50 (NEO-M9V, datasheet-verified), selectable.**

### Real dog (separate thread from the sim work)

- **C-46 · "Wrong Model" red herring + EM_DAMP decode** — `sn[1]=5` print
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
  the shape (the suite missed C-1 this way).
- A case must exercise what its why CLAIMS it exercises — the star case
  described the dash interlude for weeks while running dash-less, and two
  real bugs (C-47) lived in the gap until an operator UI click found them.
- One fact, one place: the decel ramps, draft slots, gait dropdown, force
  cap, and recipe notes all drifted as duplicated sources of truth.
- `pgrep -f` pollers match their own command line (86 min lost); prefer
  the job's own state.
- After raw-API testing, verify `/api/state` against recipes — don't wait
  for the panel warning (now moot: automation no longer touches the draft).
- The first full suite after a DEFAULT change is the test that matters
  most — and the suite must never count "no verdict" as "pass".
