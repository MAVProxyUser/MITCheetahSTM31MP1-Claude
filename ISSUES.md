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

Last validation: **19/19 fast-suite PASS** (2026-08-28 ~23:20 + two
re-runs ~23:55, on the build carrying the OPEN-6 boot fix, the terrain
planner caps and the GPS-arbitrated instruments). NOT yet re-run on the
2026-08-29 conductor changes (launch-abort, orphan watchdog, single-server
guard) - those are server-side and touch no controller code, but the suite
is the thing that says so and it is queued. Read honestly: the run
itself scored 17/19 with `sector_recipe` and `lissajous_5_7` FAILing, and
both were the OPEN-21 pose feed, not the robot - sector's bridge GPS
spanned the correct 16.7 x 18.6 m box with 17/17 waypoints and
`RESULT: PASS` while the trail showed 43.1 m of 178.4 m. Both re-ran clean
on a fresh server (178.9 s and 394.5 s). Every other FAIL in that run
(`star`, `octagon_recipe`, `parallel_recipe`, `bounding_octagon_45deg`)
passed on its own in-suite retry.

---

## OPEN

### In progress

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

- **OPEN-23 · Chase-cam smoothness above 10 Hz** — `PARKED, MEASURED`. A
  knob with a price, not a defect. With the transport fixed (see CLOSED, was
  OPEN-19) a viewer now receives every frame the sensor renders, so the
  only remaining ceilings are the sensor's own `update_rate` and how often
  the camera model is teleported — both 10 Hz, both costing GPU inside the
  render loop. Both are now env knobs with the measured defaults unchanged
  (`CAM_UPDATE_RATE`, `CHASE_FOLLOW_DT`).
  **First A/B ran and is CONFOUNDED — do not quote it.** Campaign `tier2`
  (20 runs, alternating blocks) measured viewer fps with a probe that opens
  its own MJPEG connection per rep. That probe leaves a server-side handler
  alive for up to two minutes (`_mjpeg`'s idle timeout), so every rep after
  the first in a block measured a server with 2-3 stale viewers attached.
  It shows: 10.1 fps on the first rep of a block, then ~3.1 for the rest;
  15.5 then ~4.5 for the 30 Hz arm. The pattern is the probe, not the
  camera.
  The only clean comparison is the first rep after each server restart:

  | arm | viewer fps | GPU |
  |---|---|---|
  | 10 Hz / 0.1 s (shipped) | 10.1 | 6% |
  | 30 Hz / 0.033 s | 15.5 | 12% |

  So 30 Hz roughly **doubles GPU for +50% fps** — and does NOT give 30 fps.
  **tier2b (30 runs, 3 alternating blocks) re-measured with a probe that
  closes and a machine-checked drain — and the probe was NOT the confound.**
  `viewers@start` was 0 on every rep, yet the pattern is identical: the
  first run after each server restart gets the full rate (10.2 / 15.7), and
  every later run in the block gets a flat **~44%** of it (4.4 / 6.6) — in
  both arms. A constant fraction, not a decay, which smells like one extra
  renderer sharing the GPU from run 2 on (`status.sh` has already found a
  33-hour orphaned `gz sim` once). Also noted, not concluded: pass rate with
  cameras ON was 10/15 and 8/15 on `rough@2.0`, against 90% camera-dark in
  c6 — N=15, and confounded by whatever the fps thing is.
  **c10 ran (2026-09-03): tier2b's collapse does NOT reproduce.** Four
  runs after a fresh restart on the shipped default, recording per run the
  gz process count and ages, GPU %, the viewer's delivered fps, and
  `cam_feed`'s OWN receive rate (new 5 s stderr line):

  | run | gz procs | GPU | viewer fps | cam_feed rx fps |
  |---|---|---|---|---|
  | 1 | 1 | 10% | 10.0 | 10.0 |
  | 2 | 1 | 10% | 10.2 | 10.0 |
  | 3 | 1 | 9% | 10.2 | 10.0 |
  | 4 | 1 | 10% | 10.1 | 10.0 |

  Every run delivers the sensor's full rate; both remaining hypotheses for
  the 44% (an extra renderer sharing the GPU; something about run number
  after a restart) are dead by this data. The only procedural difference
  from tier2b is that c10 probes ~6 s after the first frame while tier2b
  probed immediately after a `drain()` poll loop — so tier2b's pattern is
  most plausibly an artefact of its own sequencing, but that is a guess
  and is recorded as one. **What is established**: in normal operation
  the panel receives every frame the sensor renders (c10 here, and the
  10.1 fps acceptance test in CLOSED/OPEN-19).
  **Disposition: PARKED, default unchanged.** The clean 30 Hz numbers
  (first rep after restart, N=3: 15.5 / 14.9 / 15.7 fps at 11–13% GPU vs
  10.1 at 5–6%) say the sensor at 30 Hz delivers ~15 fps for double the
  GPU — worth it only if someone wants smoother chase footage more than
  they want GPU headroom, and this rig has already had host load silently
  explain a sim failure. Not a defect; a knob with a measured price.
  **Reopen test**, so the next report is a diagnosis and not a hunt: read
  `/tmp/cheetah_conductor/cam_feed.log`. If `rx fps` says 10 while the
  panel shows less, the loss is server→browser; if `rx fps` is below 10,
  gz is rendering slower and the answer is in host load, not the panel. Deliberately NOT changing a default blind — this
  rig has already had machine load silently explain a sim failure.

---

## CLOSED (symptom → cause → fix → evidence)

### CLOSED (was OPEN-25) · The "corner collapse" was three measurement conditions compared as one

**Symptom.** 08:03: `corner:25:*` at 2.5, all angles ×2, read walking
3/20 and trotting 7/20 against "12/18 and 10/10 before" and yesterday's
c9 passes. Environment, world, timing, binary and the OPEN-17 deletion
were all exonerated by inspection within the hour; the 02:59 rebuild was
left as prime suspect.

**Cause — the record, not the robot.** The conductor log names the
terrain of every launch, and the controller logs carry `[plan] DEM
profile` only when a heightmap was sampled:
- **c9 (yesterday's OPEN-8 redo) ran every cell on `rough`**, with
  walking capped to 2.0 by the terrain table (`terrain caps cruise 2.50
  -> 2.00` in its logs). `corner_sweep.py` never named a terrain and
  `mission_runner` without `--terrain` inherited the panel's DRAFT —
  which was `rough` after campaign c8. Sixty runs recorded as flat @2.5
  were rough @2.0.
- **Today's flagship sweep started on `rough` for the same reason** (the
  draft was rough after c14) and flipped to flat mid-sweep when an
  explicit-`--terrain flat` proof dash reset the draft.
- **The Aug 28/29 rows were flat — on the pre-anchoring course**: since
  the 2026-08-30 plan anchoring, the 25 m approach is inside the plan at
  full planner cruise; before, nav ramped it. A different, easier course.

**What is actually true, from the conductor log (40 launches):**

| gait | terrain | `corner:25 @2.5`, anchored course |
|---|---|---|
| walking | flat | **3/20** (17 collapses) |
| trotting | flat | **6/15** (8 collapses) |
| trotting | rough | 1/5 |

2.5 into a corner on the anchored course is beyond both gaits — and
nothing ships wrong: the `corner` recipe validates walking there at 1.5,
and 2.5 is a probe speed. Walking at 2.5 on a flat *dash* remains
90–100%.

**Fix.** `mission_runner --terrain` defaults to **flat** and prints the
terrain every run (and whether it was defaulted); `corner_sweep.py`
passes `--terrain flat` explicitly (`6b9f8ef`). A harness must name its
ground; the panel's draft is for humans. SKILL.md rule 5f.

**Consequences for the record.** OPEN-8's CLOSED entry is corrected
below: its c9 redo was a rough-terrain measurement, so the "22 confirmed
/ 5 marginal / 3 walking artefacts" say nothing about flat; the flat
envelope in `corner_envelope.csv` describes the pre-anchoring course for
rows dated before 2026-08-30 and the anchored course after. OPEN-24 gets
a third reason its Aug 28 "10/10" never compared: different course.
A confirming A/B (HEAD vs the Aug 30 controller source, flat corner,
interleaved) is queued behind the suite to close the "rebuild" suspicion
with a number rather than an argument.

The issue's own log:

- *(was)* **OPEN-25 · Corner cells at 2.5 collapsed this morning, both gaits** —
  `SOFTWARE, LIVE, UNEXPLAINED`. The flagship row re-measure (08:03,
  `corner:25:*` at 2.5, all ten angles ×2, clean server): **walking
  3/20 (15%)**, **trotting 7/20 (35%)**. Yesterday 01:00 (c9, same
  course, same Aug 30 binary) walking 2.5@120/135 went PASS/PASS and
  before that walking was 12/18, trotting 10/10 (Aug 28). Walking at 2.5
  on *dashes* is fine today (c16: 6/6 after the 02:59 rebuild). Corners
  specifically.
  **Exonerated by inspection**: the course and plan (identical `3
  segments over 49.4 m`, default anchoring — the conductor's env carries
  no override); the world (flat plane, no heightmap); `host-run/` (only
  the binary changed); control-loop timing (maxPeriod ~3 ms both days);
  the OPEN-17 deletion (every removed line sat inside the two flag
  conditionals, and trotting was already 1/4 *before* that deploy).
  **Prime suspect: the 02:59 rebuild** (assert-read rewrite + the
  documented empty build type). The evidence that the collapse predates
  it is weak (c11's 38% baseline at N=13; one N=1 corner at 02:30); the
  evidence it follows it is c9-vs-today. Running now: the Aug 30
  controller *source* (`48d26dc`) built with the documented configure,
  interleaved against HEAD on `walking corner:25:120 @2.5`, N=6 each.
  **Operator action that would settle it outright**: the actual Aug 30
  binary and its `CMakeCache.txt` are in the Time Machine local snapshots
  `2026-09-01-124058` and `2026-09-02-125500` — mounting needs sudo
  (`mount_apfs -s com.apple.TimeMachine.2026-09-02-125500.local / /tmp/tmsnap`,
  then `host-run/mit_ctrl_sim` and `host-build/CMakeCache.txt` under the
  repo path). With that binary saved as `/tmp/bin_aug30`,
  `gazebo/tools/ab_interleaved.sh` answers the question in fifteen
  minutes.
  **Consequences while open**: today's flagship rows, c19, and the fast
  suite passes are measurements of whatever this is, not of the robot;
  nothing from after 02:59 today should be cited as a capability number
  until this closes.



### CLOSED · The `gazebo/` move missed two kinds of path: eleven `..`-counters and one pathlib

The 2026-09-02 move rewrote all 97 literal `stm32mp1/gazebo` references
and passed the build. It could not see paths built from pieces: eleven
scripts and `server.py`'s `REPO_ROOT` counted `..` one level too deep
(every launch failed with `FileNotFoundError` until fixed, `1b1e4a5`),
and `test_validated_missions.py` assembled the runner path with pathlib
(`REPO_ROOT / "stm32mp1" / "gazebo"`) — every pass of the fast suite
failed on entry with "can't open file" until `eb674a1`. Lesson: after a
move, grep for the *pieces* (`"stm32mp1"`, `"../.."`) and run the entry
points, not just the build.


### CLOSED (was OPEN-17) · The two parked estimator flags, measured at N=30 and removed

**Question.** Two opt-in estimator levers had never had an interleaved
A/B: `SIM_FORCE_GATE` (derate KF contact trust when foot force is not
physically consistent with load-bearing, IMM-KF style) and
`SIM_KF_VFLOOR` (floor the velocity-block covariance so the filter keeps
some authority after P collapses to ~1e-3 within a second of any run).

**Measurement.** Campaign c14, `rough/walking @2.25` uncapped — a
marginal cell with headroom in both directions — four arms interleaved
every rep, N=30 per arm, on the leak-free, correctly built binary:

| arm | pass | vs baseline | Fisher p |
|---|---|---|---|
| baseline | 18/29 (62%) | — | — |
| `SIM_FORCE_GATE=1` | 13/30 (43%) | −19 pts | 0.20 |
| `SIM_KF_VFLOOR=0.005` | 9/29 (31%) | −31 pts | **0.034** |
| `SIM_KF_VFLOOR=0.02` | 9/28 (32%) | −30 pts | **0.034** |

Time-ordered, the pooled pass rate is flat across the two hours (40–55%
per 20-rep window), so this is not a rig degrading late; 4 no-verdicts,
all harness timeouts; thread drift 1–2 throughout.

**Reading.** The gate's +26 points at N≈14 the day before (c11) was a
block-sized swing — at N=30 it is −19 and not significant either way,
which is "no benefit" for a lever that costs code. The floor is
**significantly harmful at two independent sane values** (the c11 arm
that passed `=1` to a (m/s)² knob is void and not counted). The P
collapse it addressed is real and documented in the `KFHEALTH`
diagnostic; stock MIT's tuning survives the test anyway. The earlier
"delayed a failure once" was N=1.

**Fix.** Both blocks removed from `PositionVelocityEstimator.cpp`, each
replaced by a comment carrying the table above — the OPEN-13 rule: a
lever leaves with the number that killed it, where the code was. Rebuilt
with the documented configure, deployed through the startup proof, one
walking flat dash through the conductor as evidence. No `SIM_` estimator
flags remain.

The issue's own log follows as it was written:

- *(was)* **OPEN-17 · Parked experimental flags** — `PARKED`, down from four to
  **two**: `SIM_CONTACT_DETECT` and `SIM_ABS_AIDING` were deleted with
  their code under OPEN-13 (both measured harmful; each removal carries the
  measurement that killed it). What remains is genuinely unproven rather
  than disproven, which is a different thing and deserves a measurement
  rather than a deletion:
  * `SIM_FORCE_GATE` — derates the KF's contact trust when the hypothetical
    foot force is not physically consistent with load-bearing. Never got its
    interleaved A/B: the only comparison run was ONE galloping dash each
    way, on this project's least reliable gait.
  * `SIM_KF_VFLOOR` — floors the velocity-block covariance, which collapses
    from ~0.025 to ~0.0007 within a second of any run (taking the Kalman
    gain on new leg-odometry evidence from ~0.20 to ~0.007). It measurably
    delayed a failure once (upright past t=313 s vs falling at t=112 s) —
    but that was measured BEFORE the `x_comp_integral` windup fix, so like
    every pre-windup number it describes a robot being commanded backward
    and cannot be cited about the current build.
  Both are queued for a proper A/B. The close condition is a measurement,
  not a decision: measure, then default-on or delete.
  **First real A/B (campaign c11, 2026-09-03):** `rough/walking @2.25`
  uncapped — a marginal cell, so a flag that helps has headroom to show it —
  three arms interleaved, N=13–14 each after no-verdicts:

  | arm | pass | rate | vs baseline | Fisher p |
  |---|---|---|---|---|
  | baseline | 5/13 | 38% | — | — |
  | `SIM_FORCE_GATE=1` | 9/14 | 64% | **+26 pts** | 0.26 |
  | `SIM_KF_VFLOOR=1` | 2/14 | 14% | −24 pts | 0.21 |

  `SIM_FORCE_GATE`: the only flag in this project's history to show a
  positive direction at N>1, and the first one whose A/B was actually
  interleaved. Not established — p=0.26 at this N — but it earns the N that
  can decide it (~30/arm).
  **`SIM_KF_VFLOOR`'s row is VOID, and the error is mine.** The flag takes
  a *value* in (m/s)², not a boolean (`atof(getenv(...))`; 0 = stock). c11
  passed `=1`: a floor 500–1000× above the collapse value the code
  documents (0.0007–0.002) and 5–50× above a fresh start (0.02–0.2). That
  measured a filter forced to distrust its own velocity almost completely,
  not the mechanism the flag exists to test. The −24 points say nothing
  about covariance flooring at a sane value.
  **Campaign c14 (queued)** runs the version that can decide both: same
  cell, four arms × N=30 interleaved — baseline, `SIM_FORCE_GATE=1`,
  `SIM_KF_VFLOOR=0.005` (a few times the collapse), `SIM_KF_VFLOOR=0.02`
  (the documented fresh-start value) — and records the runner's exit code
  per rep, because c11 produced 4 no-verdicts (9%, all at thread drift 2)
  with no way to say why.



### CLOSED (was OPEN-7) · Terrain-aware planning — every (terrain, gait) row measured; one cap, walking on rough, at 2.0

**Question.** Does non-flat ground need the planner to slow down, per
gait, and by how much?

**Mechanism (shipped, verified firing).** The conductor samples the
heightmap along each dog's own planned path at launch
(`terrain_profile.py` → `WP_TERRAIN_PROFILE` →
`BodyPathPlanner::loadTerrainProfile`, `301 samples over 30.0 m`); a
per-(terrain, gait) cap in `terrain.py`'s `GAIT_VMAX` clamps cruise
(`terrain cap: walking on rough is measured to 2.00 m/s`); `relief_k`
modulates speed locally by stride-scale height mismatch.

**Every row, `dash:30`, interleaved N=10 per rung on the leak-free,
correctly built binary, flat control in the same block:**

| gait | terrain | 1.5 | 1.75 | 2.0 | 2.25 | 2.5 | flat control | verdict |
|---|---|---|---|---|---|---|---|---|
| walking | rough | — | 100% | 90% | 70% | 5% (uncapped) | 90–100% @2.5 | **cap 2.0** — validated end to end (capped 2.5 → 89%, c7) |
| walking | rolling | — | — | 90% | — | 95% | 100% @2.5 | no cap |
| trotting | rough | 80% | 78% | 90% | — | (~55% on flat too) | 90% @2.0 | no cap — terrain never binds before the gait's own ceiling |
| trotting | rolling | 100% | — | 90% | — | 30% | 100% @2.0 | no cap — same; 2.5 is above trotting's straight ceiling |

`relief_k` (c8): parity with the 2.0 cap at every k, at lower throughput
— relief can only slow, and on uniform bumps that is a worse global cap;
stays 0. Its case is mixed terrain, which this harness does not have.

**What the rows say together.** The terrain hazard is `rough`'s
short-wavelength geometry, and it bites **walking specifically**, because
walking is the only gait that reaches the speeds where it matters (2.5)
without falling on flat first. Trotting's own straight-line ceiling —
2.0 validated, ~55% at 2.5 on any ground — binds before either terrain
does, so a trotting cap would encode nothing but the gait's cruise.
`rolling` (0.35 m, long wavelength) costs nothing measurable to either
gait through its validated cruise.

**Retractions along the way, kept so the shape is visible.** "0/3 → 9/9"
(small blocks); "55% at the cap rung" (measured through the thread
leak); "rough trotting 2.5 PASS ×3" (leaky server); and OPEN-24, four
hours chasing a trotting regression that was one lucky block and a
corner-for-straight misread. Every number in the table above is
post-leak, post-configure-fix, and interleaved.

The issue's own log follows as it was written:

- *(was)* **OPEN-7 · Terrain-aware planning: the GEOMETRY axis** — `CAP SETTLED AT
  2.0; relief_k STILL UNMEASURED`.
  **Working and confirmed**: the DEM is sampled along each dog's own
  planned path at launch (`conductor/terrain_profile.py` →
  `WP_TERRAIN_PROFILE` → `BodyPathPlanner::loadTerrainProfile`), reporting
  `[plan] DEM profile: 301 samples over 30.0 m, stride mismatch mean
  0.016 m max 0.069 m` — matching an independent hand measurement of
  `rough` exactly. The (terrain, gait) cap fires and is logged
  (`terrain cap: walking on rough is measured to 2.00 m/s`).
  **The value is now measured (campaign c6, 2026-08-31).** N=20 per rung,
  `dash:30`, one uninterrupted block, on a conductor with no thread leak —
  **0/80 no-verdicts**:

  | arm | pass | rate |
  |---|---|---|
  | rough @1.75 uncapped | 20/20 | **100%** |
  | rough @2.0 uncapped | 18/20 | **90%** |
  | flat @2.5 (control) | 9/10 | **90%** |
  | rough @2.5 capped (→2.25) | 15/20 | 75% |
  | rough @2.25 uncapped | 7/10 | 70% |

  Monotonic, and **2.0 lands exactly on the flat control**: at 2.0 the
  terrain costs nothing measurable, at 2.25 it costs ~20 points. So 2.25
  was a less-bad rung, not a ceiling. `terrain.py` now encodes **2.0**.
  **Every pre-2026-08-31 number in this file taken on a long-lived server
  is pessimistic and should be re-measured before being cited.** The
  conductor leaked ~1 thread per run (see CLOSED, the four-attempt leak
  hunt); that both dropped ~30% of c3's launches outright as "(no verdict)"
  AND depressed the pass rate of the runs that did complete. The pooled
  c3+c4 table this entry used to carry (`@2.5` 5%, `@2.25` 55%, flat 100%)
  was measured through that, which is why its 2.25 rung read 55% where c6's
  clean-server equivalent reads 70-75%. It is kept in git history, not here,
  to stop it being quoted as current.
  **The cap is now VALIDATED end to end, and `rolling` is answered
  (campaign c7, 70 reps, 1/70 no-verdict).** Same block, same session:

  | arm | pass | rate |
  |---|---|---|
  | flat @2.5 (control) | 10/10 | **100%** |
  | rolling @2.5 uncapped | 19/20 | **95%** |
  | rough @2.5 **capped → 2.0** | 17/19 | **89%** |
  | rolling @2.0 uncapped | 18/20 | 90% |

  Two conclusions. First, the capped path works: a request for 2.5 on
  `rough` is clamped to 2.0 and delivers 89%, matching c6's *uncapped*
  `rough@2.0` (90%) — so the cap machinery costs nothing beyond the speed
  it enforces. Second, **`rolling` does not need a cap at all**: 95% at 2.5
  uncapped, within noise of the flat control and slightly *better* than
  `rolling@2.0`. Adding a rolling entry to `GAIT_VMAX` would cost
  throughput for nothing. The terrain hazard is specific to `rough`'s
  short-wavelength geometry, not to non-flat ground generally.

  **`relief_k` is now measured, and stays 0 (campaign c8, 2026-09-03).**
  Interleaved, N=10 per arm, `rough/walking`, 2.5 requested, every relief
  arm UNCAPPED so the global cap could not confound it; duration from the
  controller's own `[nav] t=` lines, mean speed over PASS rows only (a fall
  ends a run early and would otherwise read as fast):

  | arm | pass | mean m/s |
  |---|---|---|
  | global cap 2.0 (control) | 7/10 | **1.61** |
  | uncapped, k=0 | 0/10 | — |
  | uncapped, k=0.25 | 9/10 | 1.48 |
  | uncapped, k=0.5 | 9/10 | 1.24 |
  | uncapped, k=1.0 | 10/10 | 0.95 |

  Reading: the cap's 7/10 is the low tail of a pooled clean-server **42/49
  (86%)** for that rung (c6 18/20, c7 17/19, c8 7/10), so k=0.25 is
  *parity* on pass rate, not a win — at 8% lower throughput. k=1.0's 100%
  is bought at 0.95 m/s, a crawl. Relief can only slow, and on uniformly
  rough ground "slow everywhere by the local mismatch" is just a worse
  global cap. It would earn its keep on MIXED terrain (fast on the smooth
  stretch, slow on the bumps), which this harness does not have — `rough`
  is bumps everywhere. Durations were deterministic to 0.2-0.3 s per arm;
  every bit of variance is in pass/fail.
  **Walking side of OPEN-7 is closed.** Remaining: the cap table has only
  a walking entry.
  **Trotting on rough (c12, 2026-09-03, stopped after 9 reps): the ceiling
  is BELOW 2.5, and the falls are real.** Rungs 2.5/2.75/3.0 gave 1/2,
  0/2, 0/2; the flat@3.0 control gave 1/3 (trotting at 3.0 on a straight
  is not a validated cell — the dash recipe is trotRunning). Every rough
  fall reads `[FALL] collapsed: roll≈0 pitch≈0 z=0.04–0.08 m` at t≈7–14 s,
  which looked like the week-old kinematic collapse test misfiring on a
  trot — so it was checked against ground truth rather than believed:
  the bridge's baro altitude drops 0.47 → 0.34 → 0.24 m over the last two
  seconds, IMU a_z spikes to 23, torques saturate at −29, and the nav
  trace shows the dog ramping 0.55 → 2.87 m/s by t=7 s. It folds at ~2.9
  m/s on 0.15 m bumps. Not the detector, and not gait engagement — the
  acceleration ramp reaching the terrain's ceiling.
  The pre-2026-08-31 bracket ("rough trotting 2.5 PASS ×3") is another
  leaky-server number that does not hold. Campaign **c12b** (running)
  moves the rungs to where they can resolve it — rough 1.5 / 2.0 / 2.25
  with a flat@2.5 control, interleaved, N=10 — and c13 (queued) does the
  same on `rolling`.
  **Control corrected (04:20)**: c12b's `flat@2.5` control is itself a
  ~55% cell on HEAD (c15, interleaved), so it cannot anchor anything. The
  record's validated trotting *straight* cell is **2.0** (5/5). c12b is
  replaced by **c12c** — `flat@2.0` control, rough 1.5 / 1.75 / 2.0,
  interleaved N=10 — and c13 becomes `flat@2.0` control with rolling
  1.5 / 2.0 / 2.5. Both are HEAD-binary measurements and stand regardless
  of OPEN-24's disposition.
  **OPEN-24 closed as a non-issue (04:30)**: HEAD interleaved against the
  pre-window binary is 5/8 vs 4/8 — trotting@2.5 on a sustained straight
  is ~55% on every binary, and the Aug 28 10/10 were corner probes. So
  trotting's straight ceiling sits between 2.0 (5/5 in the record) and
  2.5 (~55%); c12c and c13 measure its terrain rows with the 2.0 control.
  **c12c (rough/trotting, flat@2.0 control, interleaved N=10, 05:06):**

  | arm | pass | |
  |---|---|---|
  | flat @2.0 (control) | 9/10 | 90% |
  | rough @2.0 | 9/10 | 90% |
  | rough @1.5 | 8/10 | 80% |
  | rough @1.75 | 7/9 | 78% |

  Every rung sits at the control and every miss is an early fall exactly
  like the control's own miss — **rough does not bite trotting through
  2.0**. Trotting's own straight ceiling (2.0 validated, 2.5 ≈ 55% on
  flat and rough alike) binds before the terrain does, so a
  `rough: trotting` entry in `GAIT_VMAX` would encode nothing but the
  gait's cruise. **No entry.** The rough terrain effect is
  walking-specific at the speeds walking can reach (2.5 → 5% on rough vs
  90% flat); trotting simply cannot reach the speeds where rough would
  matter. c13 (rolling) is the last row.



### CLOSED (was OPEN-24) · There was no trotting regression — one lucky block and a corner-for-straight misread

**How it opened.** Giving trotting its terrain row, every rough rung
collapsed and then the flat controls did; `trotting@2.5` on flat read
~55% on the Aug 30 binary while the Aug 28 record showed the "same" cell
at 10/10. A regression in the Aug 28→30 window was the obvious call.

**What the bisect actually found, in order.**
- A real bug, unrelated: my Sep 2 Release configure had compiled MIT's
  assert-wrapped yaml reads out of the controller (own CLOSED entry).
  The first bisect point was invalid because of it, not "bad".
- `00e34bb` (pre-window) **4/4** in one block; `cc97788` 5/8; `d9cde6e`
  3/6; HEAD 4/7 — which named `c90e0ca`'s IMU zero-init.
- The `noinit` variant at HEAD read **6/6** in one block and the story
  "the zero-init removed an accidental estimator re-init" was written up —
  then the noinit runs' own logs showed zero NaNs and zero re-inits, a
  deliberate re-init (`reinit`) scored 4/6, and the KF-health traces of
  both binaries matched to three digits.
- **Interleaved** (arms alternated every run, binaries swapped through
  `deploy_host.sh DEPLOY_SRC`): c15 HEAD 4/8 vs noinit 5/7 — the same;
  c16 on *walking* HEAD 6/6 vs noinit 3/6 — the uninitialised struct is
  actively harmful, OPEN-6's zero-init is correct and protective;
  **c18 HEAD 5/8 vs `00e34bb` 4/8, p = 1.0** — the "good" end of the
  bisect is no better than HEAD.

**Conclusion.** *(Footnote, 09-03 08:40: the Aug 28 corner 10/10 was also on the pre-anchoring course — a third reason it never compared; see CLOSED/OPEN-25.)* `trotting@2.5` on a sustained 30 m straight is **~55% on
every binary from Aug 28 to HEAD**. The Aug 28 "10/10" were `corner:25`
probes, which brake for the turn and never sustain 2.5; the record's own
validated trotting *straight* cell is 2.0. `00e34bb`'s 4/4 was one block
at a true ~55% (P ≈ 10%), and the 6/6 that pointed at the zero-init was
another. Both bisect ends were block artefacts, and the issue chased them
for four hours of rig time and ~90 runs.

**What is kept.** The Release/assert fix and hardened deploy check; the
bisect harness (`/tmp/bisect_trot.sh`, `DEPLOY_SRC`, INVALID detection,
`VARIANT=`); the interleaved-A/B scripts as the template for any future
"is X better than Y" question; the corner-vs-straight distinction in how
trotting's ceiling is cited; and SKILL.md rule 5e. The lesson is not
about trotting.

The issue's own log follows as it was written through the night — every
point, every hypothesis and the order they died in — because the shape of
the mistake is the record:

- *(was)* **OPEN-24 · Trotting regression: `trotting@2.5` went from 10/10 to
  marginal between Aug 28 and the Aug 30 binary** — `SOFTWARE, BISECTING`.
  Found 2026-09-03 while giving trotting its terrain row (c12/c12b): every
  rough rung collapsed, then the flat *controls* did too. Today, on the
  deployed Aug 30 12:14 binary, `trotting@2.5` on flat is **dash 1/2,
  corner 0/1, corner-with-clamp-off 0/1**, and `flat@3.0` is 1/3. On
  2026-08-28 15:45–15:54 the same `corner:25:*` cell passed **10/10
  angles** — a ~0.1% event at today's rate. Falls are real, not the
  detector: bridge baro altitude 0.31 → 0.07 m, IMU a_z spikes, torques
  saturate, after the dog has ramped to cruise (2.5–2.9 m/s).
  **Not gait-wide**: `trotRunning@3.0` passes; walking is 90%+ across ~400
  runs this week. **Dead hypotheses, so nobody re-runs them**: the
  kinematic collapse test (ground truth agrees the body is on the deck);
  `CTRL_XDRAG_CLAMP` (defaulted to 1.0 before the yaml too, and clamp-off
  also collapses); the tuning-yaml commit `fabd240` (every
  `getenv→ctrl_tuning` conversion kept its literal default; the yaml sets
  only the clamp, at the code default); the OPEN-13 flag removal
  `48771f9` (the estimator's default P-reset was kept unconditionally).
  **Window**: seven controller-tree commits, `c90e0ca`/`cc97788` (OPEN-6:
  IMU zero-init + `valid` gate + datum capture), `48771f9`, `fabd240`,
  `e626865` (DEM sampling + xtrack), `0e7559e` (latch-limp), `48d26dc`.
  Nothing in it is trotting-specific by inspection, which is why this is
  being **bisected by binary** rather than reasoned: a persistent worktree
  builds each point, `deploy_host.sh` now takes `DEPLOY_SRC` so a
  worktree's binary ships through the same re-sign and load-check as any
  other, and each point runs N=4 `trotting@2.5` flat dashes (at ~95% vs
  ~40%, 4/4 separates the two). Known-good point: `00e34bb` (repo HEAD at
  the Aug 28 passes).
  **Bisect status (2026-09-03 03:06): `00e34bb` is 4/4 PASS — under
  today's conductor.** Rebuilt with the documented configure and deployed
  through the hardened `deploy_host.sh`, the Aug 28 binary passes
  `trotting@2.5` flat dash four times out of four (14 nav lines each)
  with the *current* spawn anchoring, yamls and bridge. That settles two
  things at once: the regression is **in the binary**, and the
  conductor-side hypothesis (`SPAWN_BEHIND_WP0`) is **dead** — the old
  binary is fine under the new plan.
  **Points so far** (`trotting@2.5` flat dash, same protocol, falls are
  all mid-ramp at nav line 5–6):

  | point | where in the window | result |
  |---|---|---|
  | `00e34bb` | before everything (Aug 28 14:42) | **4/4** |
  | `cc97788` | after OPEN-6, before OPEN-13 | 3/4 → being raised to N=8 |
  | HEAD `537659a` | everything, repaired build | **2/4** (+1 FELL, 2 PASS from a run set one rogue harness contaminated) |

  **`cc97788` at N=8: 5/8 (62%)** — matches HEAD (57%), not `00e34bb`
  (100%). The regression is inside OPEN-6's three commits. `d9cde6e` is a
  printf-only change, so the point being run at it (N=6) is a test of
  `c90e0ca`; the outcome is binary — bad → `c90e0ca`, good → `cc97788`.
  Read ahead of the result: the heading-datum hypothesis for `cc97788` is
  dead on inspection (the capture zeroes roll and pitch and always did; it
  only moved *when* the yaw datum is taken, ~0° either way on a north-
  facing dash). `c90e0ca`'s changes all look inert on flat — the friction
  derate needs `mu_terrain > 0` and defaults to −1, its `RobotRunner`
  lines are a diagnostic print, the IMU zero-init is benign — so whichever
  the point names, the candidate is a one-line change: the IMU struct
  zero-init (`c90e0ca`) or the `&& valid` gate on the datum (`cc97788`),
  each testable by a single-line revert at HEAD, rebuilt and run N=6.
  **`d9cde6e` (≡ `c90e0ca`) at N=6: 3/6, two collapses mid-ramp plus one
  completed-but-failed dash.** The regression is in **`c90e0ca`**, and
  its only behavioural binary change is the IMU struct zero-init. The
  mechanism that fits: before it, the uninitialised quaternion produced a
  NaN on tick 0, the NaN guard called `initializeStateEstimator()`, and
  the estimators were **recreated fresh one tick after the first real IMU
  packet** — by accident. Both OPEN-6 fixes removed that detour (the
  zero-init removes the NaN; the `valid` gate keeps the datum sane without
  it), and trotting at speed apparently needed the re-init. Two variants
  at HEAD decide it, each N=6 in a paused-c12b gap: `noinit` (put the
  detour back — running) and `reinit` (a deliberate one-shot
  `initializeStateEstimator()` on the first tick `valid` is true, no NaN —
  the principled fix if `noinit` confirms).
  **`noinit` at HEAD: 6/6.** And the mechanism story is **wrong**: the
  archived controller logs of all six runs show zero NON-FINITE events and
  zero re-inits — the uninitialised struct passes *without* any NaN
  detour. A zero quaternion also yields an identity rotation matrix (every
  product term vanishes), so orientation is the same either way before
  the first packet, and the `[nav]` traces of a noinit pass and a HEAD
  collapse are identical to two decimals through t = 9.4 s. There is no
  mechanism in hand. The claim now rests on 6/6 vs 4/7 measured in
  *separate blocks* — the non-interleaved comparison this project has
  already been burned by — so before any fix is written it gets an
  **interleaved A/B (c15)**: HEAD and HEAD+noinit binaries saved as files
  and swapped per run through `deploy_host.sh DEPLOY_SRC`, N=8 each,
  alternating. The `reinit` variant (running) is now expected to fail;
  if it passes, that is its own finding — and it is 2/2 as of this note.
  Also established: `Stm32mp1HardwareBridge` is a **stack object in
  `main()`**, so the pre-OPEN-6 IMU fields were uninitialised stack
  memory — garbage, but the same garbage every launch of the same binary,
  which is why `noinit` reproduces. Whatever the first ticks see in that
  garbage, a perfectly clean zero/identity start does worse; two
  different disturbances of the estimator's first ticks (garbage, and a
  deliberate rebuild after the first packet) both look better than none.
  After c15, one run per binary with `SIM_KF_HEALTH=1` (P diagonal and
  approximate gain every second) is saved for a side-by-side.
  **`reinit` at N=6: 4/6** — indistinguishable from HEAD (4/8). A
  deliberate estimator rebuild after the first packet does **nothing**;
  the "accidental re-init" mechanism is dead twice over (no re-init in the
  noinit logs, and a real one doesn't help). Whatever `noinit` does, it
  does through the garbage *values* themselves. One principled reading of
  "garbage beats zero": a zero accelerometer before the first packet is
  *free fall* to the KF (`a_world = R·a + g`, g = −9.81), whereas the
  physically correct resting reading is +9.81 — variant `accelg`
  (`accelerometer = (0,0,9.81)` at init) is prepared and queued behind c16
  as the next single-line candidate, interleaved against HEAD, N=8. It is
  a guess with a mechanism, not a claim.
  **c15, interleaved, HEAD vs noinit, N=8 each: HEAD 4/8, NOINIT 5/7.**
  They are the same. `noinit`'s 6/6 was a **block artefact**, the
  zero-init is not causal, and the `accelg` candidate is cancelled
  unrun. The `SIM_KF_HEALTH` side-by-side agrees: both binaries' P
  collapses to ~2e-4 within one second and the gains track to three
  digits. That forces the honest question about the bisect's *other*
  end: `00e34bb`'s 4/4 was also a single block (P ≈ 13% at a true ~57%),
  the Aug 28 "10/10" were `corner:25` probes that brake for the turn and
  never *sustain* 2.5, and every point after `00e34bb` pools to
  **31/54 ≈ 57%** regardless of commit. The record's own validated
  trotting *straight* cell is 2.0. It is now more likely than not that
  **there is no regression** — trotting@2.5 on a sustained straight has
  been marginal on every binary, and this issue chased one lucky block.
  **c18 decides it**: `00e34bb` interleaved against HEAD, N=8 each, same
  protocol as c15 — chained behind c16 (the walking A/B, running). If
  they match, OPEN-24 closes as a non-issue with this whole trail kept;
  if `00e34bb` really is ~100% under interleaving, the regression is real
  and its cause is still unknown.
  **c16 (walking flat@2.5, interleaved, N=6 each): HEAD 6/6, NOINIT
  3/6 — three collapses.** The uninitialised-struct binary is actively
  *worse* on the gait that passes 90%+ across ~400 runs this week. So
  OPEN-6's zero-init is not merely non-causal for trotting, it is
  **correct and protective**, and every "noinit" line of inquiry is
  closed for good. It also shows the interleaved protocol can see a real
  difference when there is one.
  Note for c12b/c13: every trotting rep they collect before this is
  settled is on the regressed binary and will be discarded.
  *(Earlier that night the same point read 0/4 and was recorded as
  INVALID, not bad — its binary died at startup for the reason in the
  CLOSED entry above (my Release configure, inherited by the worktree
  build), so it never ran a mission; the harness now flags a controller
  that dies at startup instead of counting 0/N. The conductor-side A/B
  (`anchor_ab.sh`, `WP_SPAWN_ANCHOR=0` vs 1) was likewise started against
  the broken deploy and is void; both re-run once the repaired binary is
  proven. Every trotting collapse cited above stands: all of it ran on the
  Aug 30 binary before 02:50.
  **Consequences while open**: c12b (rough/trotting) and c13
  (rolling/trotting) are paused/deferred — their numbers would describe
  the regression, not the terrain. Walking campaigns (c14, OPEN-17) are
  unaffected.


### CLOSED · A Release configure compiled the parameter reads out of the controller

**Symptom.** 2026-09-03 02:50: `deploy_host.sh` restored "HEAD's binary"
after a bisect point and the next campaign wrote `NONE` for every rep. The
controller printed two lines and died: `terminating due to uncaught
exception of type std::runtime_error: can't read type 3 from yaml file`.
The same binary passed the deploy script's load-check.

**Cause.** Upstream MIT code: every yaml read in
`ControlParameters::initializeFromYamlFile` and
`defineAndInitializeFromYamlFile` was written as
`assert(paramHandler.getValue(key, v))` — the read *is* the assert's
argument. This project sets `-O3` itself for every configuration, so the
documented `cmake ..` (no build type) is already optimised with asserts
on; the only thing `-DCMAKE_BUILD_TYPE=Release` adds is `-DNDEBUG`, which
deletes the asserts and with them every read. Ten sites. I introduced the
Release configure on 2026-09-02 while verifying the `gazebo/` move, built
but did not deploy; the bisect harness's restore step was the first deploy
of that build. Every measurement before 02:50 today ran the Aug 30 12:14
binary, which was built the documented way and is unaffected.

**Fix.** The ten reads are unconditional and throw a named error on
failure (`parameter "X" missing or unreadable in yaml`), correct in any
build type. `host-build` is reconfigured to the documented empty build
type so the rebuilt binary matches the week's data. `deploy_host.sh` now
refuses a binary that does not reach `PeriodicTask` or that dies with an
uncaught exception — "printed something" is what let this through.

**Evidence.** Standalone: the broken binary prints `[ctrl_tuning]`, `UDP
up`, and the exception; the fixed one prints 74 lines and continues into
`PeriodicTask` and the balance controller. Then one walking flat dash
through the conductor: **PASS**, flown 30.1 / plan 30.0, xtrack 0.18, 13
nav lines (run 2280, 03:01).


### CLOSED (was OPEN-8) · The per-gait cornering envelope, measured to the resolution anything reads it at

> **Correction (2026-09-03 08:40, see CLOSED/OPEN-25).** The c9 "second
> tranche" below ran every cell on **rough** terrain (with walking capped
> to 2.0), not flat: `corner_sweep.py` inherited the panel's draft
> terrain. Its 22/5/3 tally is a rough-terrain result and its three
> "walking artefacts" are void as flat evidence. The envelope's rows dated
> before 2026-08-30 are the pre-anchoring course; the flagship re-measure
> of 2026-09-03 gives the anchored course at 2.5: walking 3/20, trotting
> 6/15 on flat. The brackets for bounding, galloping, pronking and
> trotRunning were measured before both changes and describe the old
> course; re-measure before citing one at its ceiling.

**Question.** Per gait, at what speed does a solo `corner:25:<angle>` probe
stop passing, and does the answer depend on the angle?

**Measurement.** 210 cells in `unittests/corner_envelope.csv` (six gaits,
30–165° in 15° steps, rungs bracketed low-to-high), then campaign c9
re-ran every N=1 non-PASS cell twice more on the clean conductor (30
cells, 60 runs). Brackets:

| gait | ceiling | angle-dependent? |
|---|---|---|
| trotting | 2.5 passes everywhere; 2.6–2.8 marginal at scattered angles; 3.0 FELL ×3 at 45/90/135 (confirmed) | no — a speed limit |
| trotRunning | 4.0 passes everywhere; 4.5 FELL at 60/150 (confirmed) | mostly no |
| walking | 2.25 passes 45/90/135; 2.5 passes 8 of 10 angles, marginal at 75/105 | **no** — corrected by c9 |
| bounding | 1.5 passes 9 of 10; 2.0 FELL at 5 angles (all confirmed) | ceiling 1.5–2.0 |
| galloping | 1.3 passes everywhere; 1.4 FELL 45/90 (confirmed) | ceiling 1.3–1.4 |
| pronking | 0.9 passes everywhere; 1.0 marginal at 45/90 | ceiling ~1.0 |

**What c9 changed.** The pre-2026-08-31 verdicts were taken on a conductor
leaking ~1 thread/run, biased toward FELL. That bias turned out to be
**gait-specific**: 22 of 30 cells reproduced, 5 were marginal, and the 3
outright artefacts were all *walking* (2.25@135, 2.5@120, 2.5@135) — the
slowest gait, longest runs, most exposure to a stalling server. Every
other gait's ceiling stands. Walking's row was rewritten; the earlier
"angle-dependent" claim for it was the leak, not the robot.

**Why it closes rather than refines.** Nothing in the planner reads a
per-gait angle table: `a_lat_max` is one physics constant (2.5, derated
by measured friction to `0.9·μ·g`), and corner speed is continuous
geometry (`v ≤ sqrt(a_lat_max/κ)`, angle-graded fillets). The envelope is
descriptive data about where each gait's own dynamics give out, and it is
now measured to finer resolution than anything consumes it at. The five
marginal cells (N=3, split) are exactly what a ceiling looks like at N=3;
raising them to N=5+ would sharpen a number nobody reads.

**Also fixed on the way.** `corner_sweep.py` gained `--wait-for-gate`
(a closed launch gate is waited out, never recorded), `--redo-nonpass
--reps N` (the second tranche as a repeatable command), `--list` (dry-run
so a chained sweep can be validated before it fires), and its detail
column no longer quotes the runner's own timeout advice as if it were a
verdict.

The original OPEN-8 text follows, for the record of how the brackets were
built:

- *(was)* **OPEN-8 · The per-gait cornering envelope: the SPEED axis** — `FIRST
  TRANCHE MEASURED`, brackets still being tightened. 34 valid cells at
  45/90/135° on solo `corner:25:<angle>` probes, ladders run low to high
  with every rung measured:

  | gait | measured | what it means |
  |---|---|---|
  | trotting | FELL at 3.0 AND 3.5 at every angle; 3.0/45° reproduced **3/3** | ceiling between 2.5 and 3.0 |
  | trotRunning | PASS at 4.0 **and 4.5**, all angles | no ceiling found yet |
  | walking | PASS 2.0 all; 2.5 passes 45/90 but FELL at 135° | angle-dependent |
  | bounding | PASS at 1.5 and 2.0, all angles | no ceiling found yet |
  | galloping | PASS 1.1; FELL at 1.4 (45/90) | ceiling 1.1–1.4 |
  | pronking | PASS 0.8; 1.0 marginal | ceiling ~0.8–1.0 |

  **This retires a citation that could not be used.** The old "trotting 2.5
  PASS / 3.0 FAIL at ≥120°" bracket predated the `x_comp_integral` windup
  fix, so it was not evidence about the current build. Re-measured, it
  holds — and at EVERY angle, not just the tight ones, which makes it a
  speed limit rather than a cornering one.
  **Second tranche DONE (campaign c9, 2026-09-03): the 30 N=1 non-PASS
  cells re-run ×2 each on the clean server, 60 runs.** Result:

  | outcome | cells | which |
  |---|---|---|
  | **confirmed** (repeats agree with the original) | 22 | every bounding, galloping, trotRunning and trotting ceiling cell, walking 2.0@150 |
  | **marginal** (repeats split) | 5 | pronking 1.0@45/90, trotting 2.6@135, walking 2.5@75/105 |
  | **original was an artefact** (repeats all PASS) | 3 | **walking** 2.25@135, walking 2.5@120, walking 2.5@135 |
  | indeterminate | 1 run | trotting 3.0@45's second repeat TIMEOUT'd at 239 s (unknown verdict; the cell is FELL/FELL + ?) |

  So the leaky-server bias was real but **gait-specific: it hit walking
  and nothing else.** Walking is the slowest gait — longest runs, most
  exposure to a stalling conductor. Every other gait's bracket stands
  exactly as measured. The bracket table above is therefore right for
  trotting / trotRunning / bounding / galloping / pronking, and WRONG for
  walking, whose row should read: **2.25 passes 45/90/135°; 2.5 passes
  30/45/60/90/120/135/150/165° and is marginal at 75/105°** — not "angle-
  dependent", just a ceiling near 2.5 with noise at two angles.
  Fixed on the way: `corner_sweep.py`'s detail column was quoting the
  runner's own advisory text ("...check whether the mission had already
  reached MISSION COMPLETE / RESULT: PASS"), which made that TIMEOUT read
  like a hidden PASS. Advisory lines are now excluded.
  **What is left**: the five marginal cells at N=5+ if anyone needs those
  exact rungs; otherwise the envelope is measured to the resolution the
  planner's per-gait `a_lat_max` table uses, and this issue is ready to
  close once that table is re-read against the corrected walking row.



### CLOSED · The conductor thread leak: four attempts, and what finally found it

**Symptom.** A 12-hour-old conductor sat at 54 threads answering
`/api/state` in 4.66-4.90 s, against 4 threads and 0.0005-0.0019 s fresh.
It presented as "the chase cam has always been choppy" and as campaign c3
losing 18 of 60 launches to "(no verdict)" — the harness's own polling
timing out against a GIL-saturated server. A leak in the video path was
shrinking mission sample sizes.

**Four causes, three of them real and none of them the last one.** Each was
found, fixed, and declared the fix — on the strength of the DRIFT NUMBER
alone. The number kept moving and I kept attributing it to whatever I had
most recently touched:

| attempt | cause found | genuinely a bug? | drift after |
|---|---|---|---|
| 1 | `_subscribe_cameras` made one in-process `gz.transport13.Node()` per camera per launch, "released" with `self._gz_cam_nodes = []` | yes | +0.56/run |
| 2 | `_follow_chase_cams` made its OWN gz Node per run for `set_pose`, 70 lines below a docstring claiming the server "no longer touches gz-transport at all" | yes | +1.00/run |
| 3 | the `gazebo/` move broke `REPO_ROOT`, every launch failed, and failed launches left `pose_feed`/`cam_feed` children unreaped with their reader threads blocked | yes | — |
| 4 | **`_chase_stop.set()` existed ONLY in `stop()`.** A mission that simply COMPLETED left `_follow_chase_cams` looping forever on an Event nobody would ever set | **this was it** | +0.00/run |

**What actually found it.** Naming every thread the server starts
(`host_load`, `run`, `chase_follow`, `pose_reader`, `cam_reader`,
`cam_mutes`, `log_poller`) and reporting a `by_name` histogram in
`health()`. One clean teardown then said it outright:

    {MainThread:1, host_load:1, chase_follow:1, Thread:1}   children=0

with `pose_reader` and `cam_reader` correctly absent. One name left
standing, no inference required.

**Evidence the fix holds** — six back-to-back missions, settled histogram
after each:

    run 1..6   PASS   drift=1   children=0   {MainThread:1, host_load:1, Thread:1}

Identical every run, against +1.2 threads/launch at the start of the day.
Campaign c7 continues to report `drift=1` per rep under sustained load.

**The lesson, and it is the transferable part**: a canary that reports a
leak's SIZE lets you keep guessing at its cause — three times, here. One
that reports its NAME does not. And "fixed" was said three times on
reasoning plus a single measurement; what made the fourth claim different
was repeating the same measurement six times and watching it not move.
That check should have existed before the first claim.


### CLOSED (was OPEN-19) · Chase cam was screenshots in a JSON blob — and the leak behind it was corrupting campaign data

**Symptom.** Operator, 2026-08-31: "the cam has always been choppy, like
you are taking screen shots and stitching them together." Correct, and
more literally than intended.

**Cause — four stacked, only one of which anybody had named.**
1. *Cadence.* `app.js` polled `/api/state` on `setTimeout(poll, 400)`
   AFTER each response resolved, so the display ceiling was 2.5 fps
   against a 10 Hz sensor.
2. *Coupling.* Each frame was PIL-encoded to JPEG, base64'd (+33%), and
   shipped inside the shared whole-state JSON under the fleet's big
   `self.lock` — so video inherited that endpoint's latency and contention.
3. *DOM churn.* `renderFleet()` did an unconditional
   `cards.innerHTML = rows.map(...)` every tick, destroying and recreating
   every `<img>`. Each frame was a NEW element decoding a fresh `data:`
   URL with no continuity from the last — the stitched-screenshots effect
   exactly, and it made a streaming `<img>` impossible.
4. *A thread leak.* `_subscribe_cameras` created one
   `gz.transport13.Node()` per camera per launch and "released" them with
   `self._gz_cam_nodes = []`. Dropping a Python reference is a hope, not a
   teardown: the C++ discovery threads never unwound and nothing ever
   called unsubscribe.

**What the leak actually cost.** Measured on a 12-hour-old conductor:
54 threads and `/api/state` answering in **4.66–4.90 s**, against 4 threads
and **0.0005–0.0019 s** on a fresh one — ~2500x, growing at ~+1.2 threads
per launch (c4 telemetry: launch 1 = 4 threads/0.003 s, launch 29 = 38
threads/2.52 s). So the panel degraded over a session, which is why it read
as "always" choppy. It was not only cosmetic: **campaign c3 lost 18 of 60
launches to "(no verdict)"**, clustered at the end of each block, because
the harness's own polling of a GIL-saturated `/api/state` timed out. A leak
in the video path was silently shrinking mission sample sizes.

**Fix.**
- `cam_feed.py` (NEW) — every camera subscription for a run in ONE per-run
  subprocess, same shape as `pose_feed.py`'s OPEN-21 fix. A subscription
  cannot outlive a process that exits. It also takes the JPEG encode out of
  the server (3 dogs x 3 cams x 10 Hz was 90 PIL encodes/second holding the
  GIL against every HTTP request). Binary framing on stdout (JSON header
  line + exactly n bytes); mute state pushed back on stdin so unchecking a
  camera still skips the ENCODE, not just the display.
- `CamHub` + `/api/cam/<i>/<cam>.mjpg` — `multipart/x-mixed-replace`, on
  its own small lock, never the fleet lock. Slow viewers miss frames rather
  than backing up the producer. `.jpg` gives a single frame for assertions.
- `/api/state` now carries a MANIFEST (`{index: {cam: seq}}`), not pixels.
- `app.js` rebuilds fleet cards only when their SHAPE changes and writes
  volatile text into existing nodes, so the `<img>` survives the whole run.

**Evidence (live run, 2026-08-31).**

| | before | after |
|---|---|---|
| chase-cam display | ≤2.5 fps by design, ~0.2 fps measured | **10.1 fps** (the sensor's own rate) |
| `/api/state` | 23 KB, 4.7 s TTFB | 11 KB, **0.0006 s** |
| per-frame transport | base64 inside whole-state JSON | raw JPEG, own connection, 55 KB/s |
| server threads | 54 after 12 h | baseline 2, drift 5 with 6 live children |

Framing verified independently of the simulator (stubbed gz transport, 3
cameras x 20 ticks at 10 Hz): 60/60 frames, zero drops, every payload
decodes as a 480x270 JPEG.

**Bitrate was never the bottleneck** — 41–55 KB/s for one camera. The
operator's proposed lever (resolution/quality/codec) optimises the one
dimension that was not binding; the binding terms were transport and
cadence. H.264 stays deferred (see OPEN-23) and the post-run mp4 capture
it would build on already exists as `record_video.py`.

### CLOSED · Leak canary — a supervisor can only reap what it spawned

**Symptom.** Operator, 2026-08-31, on the above: "how can you get better at
not leaving your own processes running and corrupting data? seems like the
job of whatever is launching processes. I thought we had python doing that."

**Cause.** We do, and it worked perfectly. `self.procs` + `_watch_child` +
`_reap_and_confirm` own every child faithfully — and could never have
caught this, because **the leak was not a child**. It was an in-process
`gz.transport13.Node()` holding C++ threads: a resource the supervisor
never spawned and therefore could not reap. The generalisable rule is not
"remember to clean up", it is:

> every long-lived resource must be a CHILD PROCESS the supervisor owns,
> and teardown must VERIFY rather than hope.

**Fix.** OPEN-21 applied that to the pose feed and OPEN-19 now applies it
to the cameras — which is why both are subprocesses rather than tidier
in-process objects. Plus the part that does not depend on anyone's
discipline: `Fleet.health()` / `audit_threads()` compare live thread count
against the baseline captured at boot and shout in the panel and in every
campaign log when it drifts (`THREAD_DRIFT_ALARM`, default 8). Long-lived
MJPEG viewer threads are subtracted so an open panel is not mistaken for a
leak. The leak this was written for ran at +1.2 threads/launch and was
found only because a human noticed choppy video.

### CLOSED · A refused launch was silently eating campaign sample size

**Symptom.** 2026-08-31 12:06, mid-campaign: c5 recorded 15 consecutive
"(no verdict)" reps in 51 seconds and would have finished an "N=20" stage
having launched five missions.

**Cause.** A Time Machine backup began; the OPEN-16 launch gate correctly
refused every launch; `mission_runner.py` exited 1 — the same code as a
genuine failure. The harness could not tell "never launched" from "ran and
produced nothing", so each refusal consumed a rep. Nothing in the log said
the N had changed.

**Fix.** `mission_runner.py` gains `LAUNCH_REFUSED_EXIT = 5` and
`--wait-for-gate SECONDS`: on a refusal it waits out the gate (retrying
every 15 s) and launches when it clears. Campaign harnesses check for exit
5 and **retry the same rep without consuming it or writing a telemetry
row**. Verified live against the running backup: c6 held on rep 3 writing
no rows, instead of burning the stage.


### Closed from the OPEN list

- **CLOSED (was OPEN-16) · Time Machine I/O storms** — closed 2026-08-30 as
  MITIGATED-AND-ACCEPTED. **Symptom**: hourly backups start around :38 on
  this Mac and wedge the control loops for 16-18 ms against a 2 ms budget —
  a ~9x force impulse that drops every sprinting dog in the same instant,
  with clean logs either side. Two same-wall-second multi-dog kills sit
  inside backup windows (14:38:09 exactly at a :38 start; 16:44:09 six
  minutes into the 16:38 backup). **Mitigation, shipped**: the conductor
  REFUSES to launch while `tmutil status` reports `Running=1` (ddeedc7),
  with the operator commands in the refusal message. **The residual is
  accepted, not solved**: the gate cannot protect a mission from a backup
  that BEGINS mid-run, and it never could — that is an OS-level scheduling
  fact, not a defect in this code. `sudo tmutil disable` before a long
  session is the real fix and is an operator action. Nothing further to
  build; reopen only if a stall is ever traced to a backup window with the
  gate active AND the backup having started before the launch.

- **CLOSED (was OPEN-18) · Spiro dense-weave variant** — closed 2026-08-30
  as WON'T-DO, with the arithmetic that settles it. The reference image's
  denser weave needs `k = (R-r)/r` just above 8 (e.g. 57/8), which does
  produce the look while keeping exact 8-fold symmetry — but it closes only
  after 8 full revolutions instead of one, and arc length scales with
  revolution count. Scaled to the same 9 m outer radius that curve is
  **1660 m long**: even at a 1.5 m waypoint spacing (already too coarse to
  resolve the woven centre) it is **~1107 waypoints against the shared
  `MAXWP=768`**, and roughly an **18-minute** run against a catalog whose
  current longest is 562 s. Raising `MAXWP` is a shared-constant change
  with unknown reach into every other mission, and an 18-minute run is a
  scope jump rather than a tuning tweak — all to improve on a result that
  already ships, is verified (`spiro:9.0:8` PASS 119.2 s ×3), and is
  honestly caveated as a single-layer rendition rather than an exact match.
  Recorded so nobody re-derives the parameter search from scratch.

- **CLOSED (was OPEN-13) · Pre-hardware env-var consolidation** — closed
  2026-08-30, all three parts done.
  1. **Dead flags deleted with their code.** All eight the `SIM_` audit
     called dead: `SIM_FLIGHT_COST_GATE` (harmful — force 39–42 N/foot →
     6.1), `SIM_CONTACT_DETECT`/`_BAND`/`SIM_FREEFALL_G` (regression —
     5.64/5.67/5.71 m against a 20.68–25.24 m baseline),
     `SIM_BALLISTIC_Z` (null), `SIM_KF_UNCAP` (Unitree ship the identical
     covariance cap), `SIM_ABS_AIDING`/`SIM_AID_TAU` (position aiding, net
     harmful). Each removal carries the measurement that killed it.
     VELOCITY aiding is untouched and still defaults ON.
  2. **Tuning folded into yaml.** `host-run/ctrl_tuning.yaml` (mirrored to
     `stm32mp1/deploy_pkg/`), resolution **env > yaml > code default** via
     `common/include/Utilities/CtrlTuning.h`, 25 converted call sites, and
     loaded EAGERLY so a run says which config it found before the robot
     moves (`[ctrl_tuning] loaded N values from <path>`). The env override
     is kept on purpose — every sweep harness drives configuration that way
     — what changed is that the DEFAULT is written down instead of being a
     literal beside a `getenv` in one of eight files. The file also records
     the levers measured and REJECTED (`CTRL_BANK`, `CTRL_CORNER_CROUCH`)
     so they are not re-derived.
  3. **The fall detector reworked for hardware.** It used to zero the legs
     and `_exit()` the process, which also stops whatever feeds the motor
     watchdog — "stop talking to the motors and disappear" at exactly the
     moment a human needs the machine holding still and answering. Default
     is now LATCH-LIMP-AND-HOLD: loop keeps running, all four legs
     commanded to zero every tick, the latch checked BEFORE the estimator
     and controller so nothing downstream can re-command them, and it does
     not clear itself. `SIM_FALL_EXIT=1` restores the exit and the
     CONDUCTOR sets it explicitly — a sweep asks for what it needs rather
     than every machine inheriting a harness's convenience. And the
     collapse test no longer reads the ESTIMATE: body height above its own
     FEET comes from per-leg FK rotated by the orientation estimate, which
     cannot drift. That branch had misfired twice here — 0.15 killed a day
     of valid runs while Gazebo truth said the robot was walking, and 0.10
     fired during commanded lie-downs — both times estimator error rather
     than robot state.

- **CLOSED-55 · Fleets of 4 or more: THREE IS THE CAP, by decision** —
  closed 2026-08-29 by operator instruction: *"lets just say 3 is it,
  period"*. The N≥4 failure is real and was never root-caused (every dog
  hits `STATE ESTIMATE WENT NON-FINITE` before standing; RTF, loop
  starvation, sensor wiring, a startup race and settling time were all
  ruled out), and it is now an accepted product limit rather than an open
  question. `mission_runner.py`'s `max 3 slots` and the panel's `SLOTS
  (MAX 3)` are the DESIGN, not a stopgap.
  Recorded honestly: this closes without the retest that was queued. The
  N≥4 symptom is the same message the OPEN-6 fix eliminated for a single
  dog (uninitialised `VectorNavData` read as stack garbage at control
  iterations 0-1), so it is genuinely plausible that N≥4 now works — and
  that is no longer a question this project is asking. Anyone who wants it
  back needs to lift the 3-slot cap in `mission_runner.py` AND the panel
  before it can even be measured; the cap silently rejected two attempts
  (`mission_runner.py: error: max 3 slots`) which is how this got noticed.

- **CLOSED-54 · The pose-feed decay is gone: measured over 43 launches on
  one server** (was the last of OPEN-21, closed 2026-08-29) — the close
  condition was a long campaign on a single never-restarted server, because
  the failure was always rate-based and a working first run proves nothing.
  Measured with `unittests/feed_health.py`, which splits the conductor's own
  log per SERVER LIFETIME (a count spanning restarts hides exactly this
  effect):

  | feed | launches in ONE server | feed-trouble events / launch |
  |---|---|---|
  | in-process Node | 7 | 0.143 |
  | in-process Node | 15 | 0.467 |
  | in-process Node | 37 | **0.838** |
  | per-run subprocess + `GZ_RELAY` | 25 | 0.080 |
  | per-run subprocess + `GZ_RELAY` | **43** | **0.023** |

  **The accumulation signature is gone**, and that is the actual claim: on
  the old path the rate CLIMBED with launches (0.14 → 0.47 → 0.84); on the
  new path it does not (0.080 at 25, 0.023 at 43). At 43 launches — MORE
  than the 37 that produced 0.838 — the rate is 36× lower.
  **One residual event in 43 remains**, and it is not the decay: it is the
  intermittent discovery failure of CLOSED-53, which is now DETECTED and
  retried at every layer instead of being silent (`pose_feed.py` reports
  itself useless, the server restarts it, the launch retries discovery on a
  fresh gz, and the bridge-GPS arbiter gates the run NOFEED rather than
  letting a bad verdict through). Reopen only if `feed_health.py` shows the
  rate climbing with launch count again — that, not the presence of any
  single event, is what this issue was ever about.

- **CLOSED-53 · gz-transport discovery silently failed because it never
  used loopback** (was OPEN-22, and the shared root of OPEN-21, closed
  2026-08-29) — **symptom**, two faces of one bug: launches where no sensor
  topic ever advertised (`0/N dogs came up`, world built fine, `gz.log`
  EMPTY), and runs where a brand-new subscriber logged `subscribe -> ok`
  and then received nothing at all (trail 0.0 m of a 71.2 m plan while
  bridge GPS showed the whole course flown). **Root cause**: `GZ_IP=127.0.0.1`
  — already set, and already credited with fixing an earlier multicast
  failure — only sets the address a participant ADVERTISES. Discovery
  itself still multicasts to 239.255.0.7, and this host routes `224.0.0/4`
  out over **en0/en1**, the physical interfaces, never loopback:

  ```
  $ netstat -rn -f inet | grep 224
  224.0.0/4   link#20  UmCS   en0 !
  224.0.0/4   link#17  UmCSI  en1 !
  ```

  So every discovery packet in a single-host simulation was leaving the
  machine's real network interface, and any moment that path was unhealthy
  — a flapping Wi-Fi link, a VPN toggle, a DHCP renewal, an interface
  asleep — discovery failed. SILENTLY, because a send that merely goes
  nowhere logs nothing (unlike the documented "No route to host" case,
  which does). That is why it was never root-caused: there was no evidence
  to read. **Found by instrumenting it**: `gz -v 3` + `GZ_VERBOSE=1` into
  the per-run archived `gz.log` printed `Bind at: [udp://239.255.0.7:10317]
  for msg discovery`, which is the whole answer. Cost measured before
  shipping, per the operator's constraint that instrumentation must not bog
  the process down: **22 → 32 log lines on a 12 s run, all at startup**,
  no per-message cost (verbosity gates console output, not publishing).
  **Fix**: `GZ_RELAY=127.0.0.1` — gz-transport's unicast discovery relay.
  Discovery now also unicasts to the listed peer, so on a single-host rig a
  participant is found without any multicast packet needing to succeed. One
  extra datagram per discovery beat, no privileges required (the
  alternative is a root-only `route add -net 239.0.0.0/8 -interface lo0`),
  and nothing changes about how data flows once peers connect.
  **Also shipped, and kept**: the launch retries discovery up to 4 times on
  fresh gz processes before giving up, announcing each attempt through the
  orchestration log, and counts every occurrence in
  `RUN_DIR/discovery_stats.json` (persisted, because an in-memory counter
  would reset exactly when this failure takes the server with it) with the
  running rate on every line. `CONDUCTOR_DISCOVERY_WAIT_S` /
  `_ATTEMPTS` / `CONDUCTOR_GZ_VERBOSITY` are ordinary config.

- **CLOSED-52 · The pose feed's in-process subscription decayed with
  accumulated launches** (was the substance of OPEN-21, closed 2026-08-29)
  — **symptom**: the conductor's `gz.transport13.Node`, held for the life
  of the server, measurably lost the feed as launches accumulated — partial
  trails first, then a dead feed, with the in-process self-heal only ever
  transient. Measured from the server's own logs, per server lifetime:

  | feed | launches in one server | feed-trouble events / launch |
  |---|---|---|
  | in-process | 7 | 0.143 |
  | in-process | 15 | 0.467 |
  | in-process | 37 | **0.838** |

  The rate CLIMBS with launches inside one process, which is what
  accumulation looks like from outside. **Fix**: a subscription cannot
  outlive a process that has exited, so it now lives in one that does —
  `gazebo/conductor/pose_feed.py`, started per RUN, killed with
  the run, taking its discovery state with it; the server's long-lived
  process no longer touches gz-transport at all. It streams JSON lines at
  ≤20 Hz and the server applies them through the SAME `_apply_pose()` the
  old callback now also calls — identical trail decimation, speed EMA and
  freshness heartbeat — so the SOURCE changed and no validated behaviour
  was re-derived. `CONDUCTOR_POSE_INPROC=1` restores the old path for A/B.
  **And the soak immediately showed this was necessary but NOT sufficient**,
  which is what led to CLOSED-53: at launch 8 a brand-new feed process
  logged `subscribe -> ok` and received nothing. So `pose_feed.py` now
  fails loudly on its own behalf — no first message within 8 s of a
  successful subscribe, or a 10 s gap mid-run, and it exits non-zero saying
  which. The server restarts it (3 attempts per run, then deliberately
  leaves it down so the bridge-GPS arbiter gates that run NOFEED rather
  than fabricating a verdict), and `pose_feed.log` is archived per run.
  A silent failure is now a reported and retried one at every layer.

- **CLOSED-51 · The dead pose feed was corrupting the INSTRUMENTS, not
  just the trail** (split out of OPEN-21, closed 2026-08-28/29) —
  **symptom**, operator-reported: "I keep randomly seeing [DESYNC] ...
  sometimes I look up and the dog is moving and that message pops, other
  times it's just stopped dead." Both were happening and nothing on the
  panel could tell them apart. **Root cause**: EVERY world-motion
  instrument — the drawn trail, the live DESYNC monitor, the post-run
  INVALID gate — read the SAME gz pose feed, so when it went quiet a
  healthy dog's displacement read zero and the instruments accused the
  robot. The DESYNC message even *named GPS it never read*; the monitor
  only ever differenced two pose samples. **Evidence**, all on runs that
  had already PASSED:
  - **run869** (`dash:100`): DESYNC ×2, "world/GPS moving 0.00 m/s", while
    bridge GPS moved 37.4275 → 37.4284 lat (~100 m), 1/1 waypoints. The
    `DESYNC → CLEARED → DESYNC → CLEARED` alternation was the tell — a
    genuinely blocked dog does not recover and re-block every five seconds.
  - **run876** (`sector:15:3`): gated INVALID at "flew 43.1 m of a 178.4 m
    plan", while bridge GPS spanned 16.7 × 18.6 m — the correct box for a
    15 m flower — with 17/17 waypoints and `RESULT: PASS`.
  - **run870 / run877**: same, at 0.0 m of trail.
  **Fix**: bridge GPS is the independent ARBITER for both instruments — it
  comes off the sim's NavSat over UDP and is untouched by gz-transport.
  DESYNC will not fire while the pose feed is stale (`_pose_last_t` older
  than the tick — two stale reads difference to zero, indistinguishable
  from a stopped dog), and a bad-looking window checks GPS first: GPS
  moving ⇒ log `pose feed is LYING, not the dog` and suppress. The gate
  splits a short trail into NOFEED (GPS span ≳ the planned course's span ⇒
  infrastructure, re-run) versus INVALID (GPS agrees the body did not move
  ⇒ a robot result). **Validated offline against the archived bridge logs
  before shipping** — run876 25.0 m GPS span vs 24.8 m plan span → NOFEED,
  run870 25.1 vs 25.5 → NOFEED, a stationary dog still INVALID — and then
  seen firing live: `pose feed is LYING, not the dog: feed shows 0.00 m/s
  but bridge GPS moved 29.2 m over the same window`.
  **This also explains a suite cascade**: INVALID does not trigger the
  harness's conductor recycle and NOFEED does, so misclassifying a feed
  failure as INVALID sent it down the path that never recovers, and every
  case after the feed died inherited it.

- **CLOSED-50 · Terrain, GEOMETRY axis: the envelope, the angle cells, and
  the generator bug underneath both** (split out of OPEN-7, closed
  2026-08-29) — **symptom**: `rough`/`rolling` passed 18 of 18 gait × speed
  cells, which was too clean. **Root cause of the false result**: the
  terrain generator could not produce ground rough enough to challenge
  anything. `GRID = 129` over a 400 m map is **3.12 m per pixel**, so the
  finest representable feature was ~6 m — about ten body lengths, meaning
  all four feet were always on one plane — and the frequency bands were
  written as cycles-per-map rather than metres, so `rough`'s band of 6–14
  meant wavelengths of **28–67 m**. Both code comments described the intent
  correctly ("short bumps... exercise foot placement") and neither matched
  the output. Measured along the 30 m dash corridor, the ground the dog
  actually crossed was 0.10–0.14 m of relief at a **≤1.9% grade**: flat.
  **Fix**: `GRID = 1025` (0.39 m/px, shortest honest wavelength ~1.6 m —
  stride scale) and wavelengths declared in metres (rough 1.5–6 m, rolling
  25–80 m), verified on the regenerated maps BEFORE re-running anything —
  rough now gives a per-stride (0.35 m) height mismatch of 21 mm mean /
  69 mm max where it was ~0, and rolling 0.365 m of relief where it was
  0.102. The 4× finer collision mesh was measured free, not assumed:
  `maxPeriod` 2.99–3.14 ms, zero over-4 ms ticks.
  **Result on real ground**: 27 speed cells (flat as the control) and 9
  angle cells. flat and rolling pass every rung tried, up to walking 2.5 /
  trotting 3.5 / trotRunning 4.5. Angles: **9/9 PASS at 45/90/135° on all
  three terrains**, wall times matching across terrains to under 0.3 s.
  The useful signal is DEVIATION, not pass/fail — walking's worst
  cross-track is 0.06–0.12 m on flat, 0.09–0.10 m on rolling and
  **0.25–0.29 m on rough**, which is the shape the geometry predicts:
  stride-scale relief perturbs foot placement, long-wavelength hills do
  not. The coarse-grid rows are kept as
  `unittests/terrain_envelope_speed_coarsegrid.csv` — evidence for this
  entry, not a result. **Caveat recorded so nobody quotes it**: the
  `xtrack` column on `corner:` cells reads 6.9–10.3 m and is identical
  across terrains, an artefact of measuring cross-track against a 2-point
  plan the dog starts 25 m behind.

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

---

## 2026-08-29: THE ENVELOPE TABLES ARE MOSTLY N=1, AND N=1 IS NOISE HERE

The experiment that had to be run before any more cells were added, and it
invalidates the per-angle reading of the whole cornering matrix.

The single-rep grid showed failures scattered across angles at the rung
above each gait's ceiling - `bounding@2.0` failing 30/75/105/120/150 while
passing 45/60/90/135/165. That reads like an angle effect. Five reps at one
"FELL" angle and one "PASS" angle, per gait, says it is not:

| cell | grid said (N=1) | 5 reps say |
|---|---|---|
| bounding@2.0 30° | FELL | **PASS ×4, FAIL ×1** |
| bounding@2.0 45° | PASS | **FAIL ×1, PASS ×4** |
| trotRunning@4.5 60° | FELL | **PASS ×5** |
| trotRunning@4.5 75° | PASS | **PASS ×5** |
| walking@2.0 150° | FELL | **PASS ×5** |
| walking@2.0 135° | PASS | **PASS ×5** |

A cell recorded as FELL reproduces 5/5 PASS. A cell recorded as PASS
reproduces 4/5. **The scatter was marginality, not geometry** - a gait
sitting on its own ceiling fails a fraction of the time regardless of the
corner, and a one-rep-per-cell grid samples that fraction and draws a
picture of it.

The same holds on the terrain side. flat `trotting@3.0` is 2/3, flat
`trotting@3.25` is 2/3, flat `walking@2.25` is 2/3 - all rungs previously
scored from one run.

**Consequences, stated plainly:**
1. **No per-angle ceiling in OPEN-8 is trustworthy at N=1.** The full
   10-angle grids are still useful as a coarse map of where a gait is
   comfortable, but "FELL at 150°" is not a finding.
2. **The rough-vs-flat walking result survives but is weaker than it
   looked**: rough@2.5 FELL 2/2 against flat@2.5 PASS 2/2 is a real
   difference in the right direction, but both are N=2 on a stack now
   demonstrated to be marginal at these rungs. It needs N≥5 per cell
   before `v_terrain_max` gets a number.
3. **The measurement standard changes**: a cell near a ceiling needs ≥5
   reps, and a ceiling is where the PASS RATE crosses (say) 80%, not where
   the first failure appears. This is the same lesson this file already
   carries as "repeat every marginal cell before believing it" and as the
   stop-at-first-failure ladder trap - applied to a whole matrix rather
   than a single cell.
