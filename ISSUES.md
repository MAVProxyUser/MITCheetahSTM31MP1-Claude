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

- **OPEN-7 · Terrain-aware planning: the GEOMETRY axis** — `CAP SETTLED AT
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

  **Still open**: `relief_k` — defaults 0, inert, never measured. Its own
  sweep was confounded by the speed cap; re-run with `WP_TERRAIN_VMAX=-1`.
  That is now the ONLY unmeasured piece of this issue.

- **OPEN-8 · The per-gait cornering envelope: the SPEED axis** — `FIRST
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
  **Correction (2026-09-02): the list above was already done.** The CSV
  holds **210 cells**: trotting 2.5/2.6/2.7/2.8/3.0/3.5, galloping
  0.8–1.4, pronking 0.6–1.0, bounding 1.0–2.0, trotRunning 3.5–4.5, walking
  2.0–2.5, most at the full 30–165° grid. This entry had not been updated
  since the first tranche and was still asking for rungs the CSV already
  contains.
  **What is actually left — the second tranche, running as campaign c9**:
  every FELL/FAIL in that CSV is **N=1**, and every one was measured on the
  conductor that was leaking ~1 thread per run (fixed 2026-08-31), which
  both dropped launches and depressed pass rates — i.e. biased toward
  exactly those verdicts. PASS cells are robust to that bias; the 30
  non-PASS cells are not. `corner_sweep.py --redo-nonpass --reps 2` re-runs
  each of them twice more on the clean server (60 runs) and appends rows;
  the tally then aggregates per cell. A bracket only moves if a cell's
  repeats disagree with its original.

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

- **OPEN-17 · Parked experimental flags** — `PARKED`, down from four to
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
- **OPEN-23 · Chase-cam smoothness above 10 Hz** — new, and a measured
  trade rather than a defect. With the transport fixed (see CLOSED, was
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
  **Next: campaign c10** (chained last, per priority) — 4 runs after a
  restart, recording per run: `gz` process count and ages, GPU %, the
  viewer's delivered fps, and `cam_feed`'s OWN receive rate (new 5 s log
  line) so sensor-side rate and delivered rate can be told apart. Deliberately NOT changing a default blind — this
  rig has already had machine load silently explain a sim failure.

---

## CLOSED (symptom → cause → fix → evidence)

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
