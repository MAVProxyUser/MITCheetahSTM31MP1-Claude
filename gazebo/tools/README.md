# gazebo/tools

Analysis and campaign scripts. Two rules everything here follows, both learned
the hard way:

- **Nothing writes to `/tmp`.** Paths come from `paths.sh` / `gazebo/paths.py`,
  rooted at `$CHEETAH_DATA` (default `<repo>/../rundata`).
- **A campaign that cannot say which run its data came from is not evidence.**
  See "the identity guards" below.

## Running campaigns

| script | what it does |
|---|---|
| `campaign_lib.sh` | shared helpers: markers, deadline waiters, and the identity/health guards below |
| `paths.sh` | the one definition of `$CHEETAH_DATA`, `RUN_DIR`, `ARCHIVE_DIR`, `CAMPAIGN_DIR`, `LOG_DIR` |
| `campaign_truth.sh` | the general interleaved campaign: pose truth + contact truth + a snapshot per run. `CAMPAIGN_SLOT` overrides the mission |
| `campaign_c22_truth.sh` | the c22 estimator-vs-truth campaign |
| `run_queue.sh` | queues jobs under `timeout`, health-checks the conductor between them, and does not stop on a failure |
| `rig_watchdog.sh` | restarts a dead conductor |
| `ab_interleaved.sh` | the generic two-arm interleaved A/B |
| `bisect_point.sh` | bisects a single scalar until a run flips |

## The identity guards — read before writing a new campaign

`shm_reaper.dump_snapshot()` reads shared memory that **outlives the process
that wrote it**. A run that aborts before the controller starts therefore
leaves the *previous* run's data in the ring, and both the ring and
`$RUN_DIR/ctrl_0.log` will hand it over without complaint. Measured: one
campaign recorded the same run six times as six different data points.

- `campaign_launched_run_id <runner-log>` — the only run id that cannot be
  stale. It reads `[runner] launched run N` from the runner's **own stdout**,
  which is truncated per invocation and simply lacks the line when a launch was
  refused.
- Pass that to `dump_snapshot(..., expect_run_id=...)`. It compares against the
  id carried *inside* the ring and returns `None` on a mismatch rather than
  archiving fiction.
- `campaign_run_id` — weaker (reads `ctrl_0.log`, which goes stale in the same
  way). Fine for recording in a CSV, not for the check.
- `campaign_health_gate <verdict>` — consecutive `NONE` verdicts are a host
  problem, not a result: clears a wedged fleet at two, stops the campaign at
  four rather than keep writing rows.

## OPEN-26 — the fall mechanism (closed)

`open26_estop.py` is the one that settled it: falls are a safety E-stop
triggered by attitude, read at the *triggering event* rather than the final
pose. The rest are the hypotheses that were tested and killed on the way —
`_precursor`, `_mechanism`, `_divergence`, `_contactloss`, `_ground`,
`_excursion`, `_attitude`, `_firecheck`, `_firetime`, `_three_way`, `_pool`,
`_contact_score`. Kept because a killed hypothesis is a result.

## OPEN-27 — the finish-line fall (closed)

| script | what it does |
|---|---|
| `wkc_settle_ab.sh` | the N-arm interleaved A/B that closed it. Arms are `NAME:ENV` pairs |
| `stopfix_score.py` | scores peak attitude **after the stop**, windowed from the trace's own velocity — two clocks live in these traces and a log-keyed window scores the wrong seconds |
| `stopfix_ab2.sh` | the earlier dash-based A/B; its header records why the first one measured nothing |

## OPEN-28 — what limits the course (open)

| script | what it does |
|---|---|
| `open28_corner.sh` | isolated `corner:<leg>:<angle>` sweeps. 0/66 falls, 30°–180° |
| `open28_subcourse.sh` | runs `course:<name>` arms interleaved. Used for the shape and recovery-distance sweeps |
| `open28_duration.sh` | duration vs features, with `wkc_finals` as the positive control |
| `design_course.py` | generates a `.course` file plus the judge's map from a `turn,leg` list. Every sub-course here was cut from `wkc_finals`' own list |
| `wkc_sweep.sh` | the course speed ladder |
| `open28_clip.sh` / `open28_sched.sh` | `open28_subcourse.sh` with the bridge dump on (`BRIDGE_DUMP=$DIR/bridge_{RUN}.csv`): the torque-clip A/B and the `CTRL_MPC_SCHED_LEAD` dose-response |
| `open28_handoff.py` | **the per-exchange scorer, v2.** Every scheduled flip (`c` rising) against FK foot height, body vz/z and the bridge's per-joint `tau_ff`: early touchdown, when the old pair's MPC force is cut, unsupported time, body drop, crossings. v1 read `foot_fz` as a force — it is foot SPEED — and is withdrawn |
| `open28_mpcin.py` | reads `[MPCIN2]` lines (`STM32MP1_MPC_IN=2`, every inline solve) and reports the contact table's lead over the gait's own segment at the solver's input |
| `open28_torque.py` | per-joint commanded torque against the SDF limits from the bridge dump; the torque-clip anatomy |
| `open28_exitroll.py` | peak roll in the 2.5 s after a reversal apex (the `WP_VSLEW` endpoint) |
| `open28_tail.py` | the Gumbel tail test that showed a second mode in the `hp_gap` family |
| `open28_yawtruth.py` / `open28_footcontact.py` / `open28_mechanism_check.py` | contact-truth and yaw-truth scorers for the campaigns that carried the contact feed |
| `open28_support.py` / `open28_entry.py` | **withdrawn** — both read `foot_fz` as a force; their `fz` columns are sums of foot SPEEDS. Left in place until the campaign that might still call them ends, then fixed |

## OPEN-31, item 5, item 6 — the 2026-09-10 afternoon chain

| script | what it does |
|---|---|
| `campaign_chain_20260910.sh` | the chain that ran after the speed ladder and the fleet re-test: waits for the ladder harness to EXIT before patching the harness (a running bash script is never edited), waits for the fleet marker and an idle conductor, deploys the OPEN-31 binary through `deploy_host.sh` (backup, restore on failure), then runs the interleaved A/Bs back to back. A file rather than an inline `bash -c` so the sequence behind the numbers is in git |
| `open28_subcourse.sh` (extended) | per-arm `TERRAIN=<kind>` token (consumed like `SPEED=`, sets that run's `--terrain`), and a `COURSES` entry with a colon is passed as a raw slot spec (`dash:100`, `star:10.514:5`) instead of `course:<name>` |
| `open31_score.py` | joint-limit A/B: verdicts, the heartbeat's own `clamps=/stops=` counters summed per run from the archived ctrl log, and from the bridge dump the calf's time outside Unitree's operational range (−151..−53°) and on the mechanical stops, by phase (stand-up / locomotion / finish) |
| `fleet_retest_score.py` | the 3-dog dash re-test: per rep the run id, verdict counts, the sim's real-time factor sampled during that run (`fleet_dash_retest.log.rtf`), and per dog the ESTOP time, speed and attitude at the trip from the shm trace |
| `item5_score.py` | contact-gate A/B: verdicts, `[ESTERR]` forward/lateral velocity error over cruise (mean, p90) per arm, and the gate's own `vetoed X of Y` line so an arm that claims to gate is shown to have fired |

## Data handling

| script | what it does |
|---|---|
| `distill.py` | raw rings → a 50 Hz descent window plus a judged summary. `--prune` deletes a raw snapshot only after re-reading its distilled record. Took 668 snapshots / 20 GB down to 31.6 MB |
| `fall_index.py` | indexes the archive by fall kind |
| `add_foot_contacts.py` | adds ground-truth contact sensors to a world |
| `c22_truth_vs_estimator.py` | scores the state estimator against pose truth |

## Studies

| script | what it does |
|---|---|
| `mppi_replay.py` | replays captured QP problems against a batched sampler on MPS. Answered OPEN-29 stage 1: cold-start sampling is structurally impossible here (0/4096 feasible at every sigma) and warm-started it adds nothing over reusing the previous answer |

Capture the QP problems it reads with `MPC_DUMP=<path>` (and
`CTRL_USE_JCQP=1` on this Mac, which ships `use_jcqp: 0`).
