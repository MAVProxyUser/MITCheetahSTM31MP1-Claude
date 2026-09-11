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

Last validation: **20/20 FULL-suite PASS** (2026-09-11 06:13) and
**13/13 fast-tier PASS** (12:43, on the fall judge's four criteria and the
bridge on the real-time band; 05:12 before them), both on the shipping binary: operational
joint limits ON, speed-scheduled contact-table lead, contact gate OFF;
includes the new `dash_trotting_30` sprint case (70.5 s) and, the same
morning, the 3-dog `dash:100` at 3.0 at 18/18. Previous: 19/19 fast-suite PASS (2026-08-28 ~23:20 + two
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

- **OPEN-36 · A fall that comes to rest propped at 40.5° and 0.11 m is
  neither "tipped" nor "collapsed" to the judge: the FSM ping-pongs for the
  rest of the run and the harness books a NONE** — `SIM HARNESS`. Opened
  2026-09-11 11:40 (runs 5053 and 5090, both `wkc_box` at 2.6, both the
  0.4-budget arm, both on the wp4 leg where that arm's other falls are).
  Symptom: `alon04 rep7 NONE wp=4 nofall peak pitch=0.0 roll=40.5 wz=0.00`
  after a 300 s timeout. What happened: the dog fell at t ≈ 22 s, the body
  came to rest on its folded legs at 40.5–40.7° of roll and kin_z 0.11 m,
  and for 250 s the controller printed `Unsafe locomotion: roll is 40.500
  degrees (max 40.000)` / `[Recovery Balance] ... Folding legs` / `[FSM
  LOCOMOTION] On Enter` in a loop (61 615 times) while the mission kept
  commanding wp4 at 2.60 m/s. No E-stop — RecoveryStand exempts the
  orientation check — and no `[FALL]`: the judge's "tipped" bar is
  `SIM_FALL_DEG` 50° and its "collapsed" bar is kin_z < `SIM_FALL_Z` 0.10 m;
  this rest sits between them. Cost: five minutes of rig per event and a
  fall filed as no-run (and, in chain Q, the 0.4 arm's rep 1). **Fix
  (built 11:45, deployed when the rig is between runs)**: a third criterion
  in `RobotRunner.cpp` — low AND tilted, `kin_z < SIM_FALL_DOWN_Z (0.15)`
  with roll or pitch past `SIM_FALL_DOWN_DEG (30°)`, held the same
  `SIM_FALL_HOLD_S` 0.5 s — logs `[FALL] down at an angle:` and ends the run
  like the other two. A lie-down is level, a walking dip is level, and the
  mission's own recovery waits for a settled orientation, so neither is
  touched. **12:15 — not enough (run 5105, box at 2.6, threshold-2.5 arm):
  pitch went 15° → 40.1° inside one stride, the FSM's own "Unsafe
  locomotion" bar (40°, instantaneous) beat the 28.65°/60 ms safety check
  to it, and the body sat at 40–43° of pitch on its folded rear legs at
  kin_z 0.17 — above the 0.15 m bar — ping-ponging 346 times for ~20 s
  before it tipped past the E-stop. Fourth criterion: tilted past 30° for
  `SIM_FALL_TILT_HOLD_S` (2.0 s) at ANY height (`[FALL] tilted for
  seconds:`); locomotion never holds 30° for two seconds, a lie-down is
  level, the mission's recovery waits for a settled orientation. Built
  12:17; deployed in the gap after chain T (the controller unlinks its
  socket path on bind, so the deploy's 3 s proof run must never overlap a
  live run; a waiter deployed it at 12:22:32, one second after the campaign's
  marker; previous binary kept as `mit_ctrl_sim.pre_tiltjudge`). The 11:32
  criterion already ended the shipped arm's rep-5 fall as `down` instead of
  a 5-minute NONE.**
  **Also found here**: a Time Machine backup (11:23, to the newly
  mounted `/Volumes/Backups2026`) makes the conductor refuse launches; the
  runner waited inside its own 300 s deadline, so the harness booked NONE
  rows for runs that never existed and hit its 2-NONE fleet clear. The
  harness now waits out the backup BEFORE starting the deadline
  (`open28_subcourse.sh`, installed by rename so the running campaign kept
  its inode). **Done means**: a fall at rest at any angle above 30° ends the
  run inside a second with a `[FALL]` line; no NONE row is ever a fall or a
  backup.
- **OPEN-35 · The bridge process stalls for 100–250 ms under host load and the
  controller runs on frozen state — two falls today were the harness, not
  the robot** — `SIM HARNESS`. Opened 2026-09-11 11:20. Symptom: a fall at
  cruise with no precursor in any robot channel. Evidence: run 5070
  (hp_gap20 at 2.1, 10:48) — the state the controller consumed (orientation,
  foot kinematics, kin_z) bit-identical for 63 ticks (124 ms) while the
  control loop kept its 2 ms period; the bridge's own line for that second
  reads `stalls>5ms=1/worst=128.7ms rx_backlog_max=64` (64 commands queued
  = 128 ms of the controller sending into a bridge that was not running);
  on return the state jumped −8.9° roll / −6.7° yaw and the safety E-stop
  fired 60 ms later. Run 5058 (wkc_box at 2.6, 10:30): 247.5 ms, backlog
  124, the "level collapse" of chain Q — and what a level collapse IS: on
  the return the locomotion safety check tripped and the FSM ping-ponged
  LOCOMOTION ↔ RecoveryStand (`[FSM LOCOMOTION] On Enter` / `[Recovery
  Balance] ... Folding legs`, alternating every few ms) until the body was
  on the ground at 0° roll and pitch. Today's 305 per-run bridge logs
  carry 24 mid-run stalls over 20 ms and five over 100 ms (01:22, 02:14,
  10:30, 10:31, 10:49); a stall over 100 ms at cruise is a coin-flip fall.
  Two classes by the backlog: backlog ≈ stall × 500 Hz means the bridge
  alone stopped (5058, 5059, 5070, 01:22); backlog ≈ 2 with a 140 ms stall
  (02:14) means the controller stopped sending too — a machine-wide stall.
  The bridge is a 36 MB Python process (no GC pause explains 250 ms); it
  sleeps 2 ms per cycle at default QoS on a desktop that was running
  Spotlight at two cores (`mds_stores` 136 %, `corespotlightd` 60 %, `mds`
  95 % — indexing is off for `/` but ON for `/Volumes/Backups2026` and
  `/Volumes/ExternalLife`, `mdutil -a -s`), a window server at 30 %, Photos
  analysis and the sim itself, load average 15–17 on 14 cores.
  **Instruments (shipped)**: `gazebo/tools/state_freeze_scan.py` finds the
  class in any snapshot — identical state under motion with the loop period
  intact, the jump on return — and `campaign_freeze_report.py NAME` splits a
  campaign's falls into genuine and harness per arm. **Mitigation (shipped
  11:25, `BRIDGE_RT`, default on)**: the bridge's main loop and its IMU /
  joint-state callback threads ask macOS for the mach time-constraint band
  (period 2 ms, computation 0.5 ms, constraint 2 ms; any process may, no
  root); a 3 s probe on the loaded host took the worst `sleep(2 ms)`
  overshoot from 1.05 ms to 0.02 ms. The bridge names it in its log
  (`[bridge] scheduling: ... kr=0`) and the stats line carries
  `rt_threads=elevated/refused`. **Verified 11:50 on the first run after
  the backup**: `kr=0`, the bridge's main thread at priority 97 against the
  controller's 31 (`ps -o pri`); the first cut re-elevated the callback
  threads on every call (gz-transport recreates the Python thread state per
  callback, so `threading.local` was fresh each time — 8574 calls in 6 s),
  fixed by keying on the native thread id. Host actions taken: `corespotlightd`
  (user-owned) reniced to 20 with the background task policy (authorized:
  "you can stop spotlight"); `mds`/`mds_stores` are root's — the durable
  fix is the user's `sudo mdutil -a -i off`. Also found on the way: the
  disk at 96 % with 18 GB free and a 64 GB snapshot archive growing 21
  GB/day with no retention → `archive_compact.sh` packs snapshots older
  than 24 h to `.json.zst` (8.7×, reversible) at background QoS and every
  reader resolves either name through `snapio.py` (note: the backup's own
  APFS local snapshot pins the deleted originals, so the disk reads worse
  until it is thinned; `bridge_stall_report.sh` is the yardstick below).
  **Done means**: over the
  next 300 runs on the RT band, zero mid-run bridge stalls over 100 ms and
  `campaign_freeze_report.py` showing zero harness falls; today's baseline
  is 5 and 2. Readings: 12:04, 10 runs on the band — 0 stalls ≥ 20 ms;
  12:45, 37 runs — 0 bridge-alone stalls, ONE machine-wide 78 ms stall
  (run 5122, backlog 2: the controller stopped sending too; survived). That
  class is the controller's own scheduling, so its periodic tasks went on
  the same band (`PeriodicTask.cpp`, `CTRL_RT=0` disables; deployed 14:26:45
  in the gap after chain V's first campaign, `kr=0` for `robot-control` and
  `unitree-rs485`, a thread at priority 97 beside the process's 31).
  Afternoon reading (14:45, 70 runs on the RT bridge, 11 of them with the
  controller on the band too): 2 stalls ≥ 20 ms, 0 ≥ 100 ms — the 78 ms
  machine-wide one above and a 56 ms bridge-side one (run 5157, backlog 28,
  at second 180 of a pass, i.e. during the lie-down). A bridge-side stall on
  the real-time band is not scheduling; the remaining suspect is the loop's
  own 1 Hz stats `print(..., flush=True)` blocking on the log file under
  disk load. Left as the next lever if the class persists.
- **OPEN-31 · Joint-limit hygiene before hardware: the calf is driven into
  its mechanical stop by the boot fold and the lie-down, and to full
  extension in locomotion; nothing enforces Unitree's operational range** —
  `IN PROGRESS, SOFTWARE`. Measured 2026-09-10 from the bridge dumps (sim joint
  angles at 100 Hz, URDF convention) of five clean-harness runs:

  | phase | calf range | below −151° (operational) | above −53° | abad at the ±49.5° stop |
  |---|---|---|---|---|
  | stand-up (first 14 s) | −161.5 .. −90.7 | **20.6 %** of leg-samples | 0 | 6.8 % |
  | locomotion (wkc 1.9) | −144 .. −50.9 | 0.00 % | 0.06–0.09 % | 0.00 % |
  | locomotion (hp_gap20 1.9) | −157 .. −50.9 | 0.01 % | 0.22–0.36 % | 0.01 % |
  | finish (last 12 s) | −161.6 .. −51 | **3.7–30.5 %** | 0–0.3 % | 0.6–14 % |

  −161.5° IS the world's mechanical calf limit and −50.9° its other end:
  the boot fold and the lie-down park the calves against the stop for
  seconds at a time (kp = 8 driving a target the joint cannot reach — on
  hardware that is a stalled motor), and in locomotion the leg reaches
  full extension a few times per run (the swing/stance kinematics are
  allowed `_maxLegLength = 0.430`, Unitree's number, while the knee limit
  makes 0.385 the real reach — so the planner can ask for footholds only a
  straight leg reaches). Unitree's operational clamp (abad ±55°, thigh
  −33..165°, calf −151..−53°) is enforced nowhere in this port (known,
  see the joint-limit table in CLAUDE.md); abad and thigh stay inside it,
  the calf does not. Three fixes, each needing its own A/B because each
  touches a validated behaviour: (1) a controller-side soft clamp on the
  commanded joint targets to the operational set, with a counter so any
  binding is visible; (2) lie-down and boot-fold targets that keep the calf
  above −151°; (3) `_maxLegLength` back at the kinematic reach.
  **Status 12:40** — fix (1) is built (`fd45e08`): `LegController::
  updateCommand` clamps every joint PD target into the operational set
  less a 2° margin and adds a capped spring-damper soft stop (100 N·m/rad,
  12 N·m cap) when the JOINT itself is past the range, both counted and
  printed by the heartbeat (`[stm32mp1] joint limits: clamps=N stops=M`);
  knobs `CTRL_JOINT_LIMITS{,_MARGIN_DEG,_K,_D,_TAU}` documented in
  `ctrl_tuning.yaml` (which, it turned out, had never been tracked —
  `host-run/` is ignored; force-added). Note from the dumps: during the boot
  fold the COMMAND never goes below −151° (0.00 %) while the joint sits on
  the −161.5° stop 17 % of the time — the leg folds past its target under
  gravity at kp 8, so it is the soft stop, not the clamp, that acts there.
  **Deployed 13:02 and A/B'd** (`open31_jointlimits`, hp_gap20 at 1.9,
  8 reps, arms interleaved, dumps on, `open31_score.py`):

  | arm | PASS | calf on the −161.5° stop, stand-up / finish | calf < −151° (soft-stop band) | active commands outside −151..−53°, locomotion | peak pitch, median |
  |---|---|---|---|---|---|
  | limits OFF | 8/8 | **17.5 % / 6.0 %** of leg-samples | 20.6 % / 29.7 % | **2.3 %** (all on the extension side, > −53°) | 16.6° |
  | limits ON | 8/8 | **0.00 % / 0.08 %** | 19.8 % / 28.8 % | **0.00 %** | **12.7°** |

  Counters: 0 / 0 with the flag off, a median 22,962 clamps and 12,074
  soft-stop events per run with it on — the feature demonstrably fires.
  The joint no longer touches the mechanical stop anywhere in the run; it
  still sits up to 1.5° past the operational edge during the boot fold
  and the lie-down (the soft stop is a spring, and gravity on a belly-down
  fold compresses it — that band is the design, not a leak). No verdict
  regression, and one effect that was not asked for: the peak pitch of
  the run, which sits at the exit of the 180° reversal (t ≈ 70 s) in
  every trace, is **lower with the limits on in 8 of 8 pairs** (2.3–5.7°,
  mean 4.0°; sign test p = 0.004). Candidate mechanism, not established:
  the 2.3 % of locomotion commands the clamp removes ask for a calf
  straighter than −53°, i.e. a foothold only a straight leg reaches, and
  the reversal exit is where the swing legs reach farthest. Side effect
  recorded: 0–5 yaw-saturation events in 4 of 8 limits-on runs, none with
  them off. Host column: `loop_max` 7.5–39 ms on the first nine runs of
  BOTH arms (13:04–13:21, while Spotlight's indexer sat at 86 % CPU at
  campaign start) and ~3 ms on the last seven — time-clustered host noise,
  not an arm effect; no verdict depended on it. **Fix (1) ships ON**
  (`CTRL_JOINT_LIMITS: 1` live in `ctrl_tuning.yaml`). The wkc_finals
  repeat (`open31_jointlimits_wkc`, 23:23, N = 6 interleaved) says the same
  on the second course: 6/6 vs 6/6, calf on the stop 17.4 % → 0.00 % of
  the boot fold and 8.4 % → 0.00 % of the finish, active locomotion
  commands outside the range 1.87 % → 0.00 %, and peak pitch **lower with
  the limits on in 6 of 6 pairs** (2.8–6.2°, mean 4.8°). Fourteen of
  fourteen pairs across two courses now; the effect is real and it is a
  side benefit, not the purpose. Fixes (2) (boot-fold / lie-down targets that keep the calf
  above −151°) and (3) (`_maxLegLength` at the kinematic reach) stay filed:
  (2) is now cosmetic on the sim (the soft stop holds the joint off the
  stop) but on hardware a fold target the joint cannot reach is still a
  stalled motor, and (3) is the mechanism candidate above.

- **OPEN-29 · MPPI on the Mac GPU — Stage 1 answered, and it is not
  encouraging** — `CLOSED 2026-09-10, NOT PURSUED`. Stage 1 was the
  decisive cheap test and it said no (0/4096 feasible cold-start samples;
  warm start collapses to reusing the previous QP solution; the GPU buys
  nothing a convex QP does not already have). Stage 2 is not scheduled. Operator asked whether an M4's GPU
  could prove an MPPI controller works with this codebase, up through the
  Westminster course. Stage 1 was the cheap decisive test: can *sampling*
  solve the problem `SolverMPC.cpp` already assembles, on real captured
  states? `SolverMPC.cpp` now captures its contact-reduced QP and its own
  answer (`MPC_DUMP`, off by default), and `gazebo/tools/mppi_replay.py`
  replays them against a batched MPPI-style sampler on MPS.

  400 solves captured from a real trot at 1.9 (nv = 60 trot / 120 stand,
  nc = 100/200, horizon 10). Three results:

  1. **Cold start is structurally impossible, not a tuning problem.** The
     feasible fraction of K = 4096 isotropic samples is **0.0000 at every
     sigma from 0.05 to 2.0**. The origin sits on the lower boundary of all
     100–200 friction-cone rows (`l = 0`), so feasibility needs that many
     one-sided conditions to hold at once. The sampler simply keeps choosing
     its own incumbent — the giveaway was identical cost at every K.
  2. **Warm-started it works, and adds almost nothing.** From the previous
     solve the feasible fraction is 0.99 at sigma 0.05. But median cost
     excess over the QP is +0.0001…+0.0002 — *the same as the control that
     reuses the previous answer unchanged*. It only earns its keep in the
     tail: p90 excess falls 0.81 → 0.06 between K = 1 k and 16 k.
  3. **The GPU is real but the workload is small.** MPS vs CPU per solve:
     3.4/9.2 ms at K = 1 k, 3.9/23.6 at 4 k, 8.6/86.1 at 16 k, 33.0/340.5 at
     64 k — ~10× at the top, and sub-linear below 4 k because it is
     launch-latency-bound, not compute-bound. K = 16 k costs 8.6 ms: inside
     the 26 ms inline MPC budget, well under the board's 82 ms JCQP, and
     5–14× *slower* than qpOASES's 0.6–1.7 ms on this same Mac.

  **Verdict.** Sampling is affordable on this GPU and useless on this
  problem — it is convex, which is sampling's worst case, and ADMM already
  owns it. Any real MPPI has to change the formulation to something
  non-convex (contact timing as a decision variable, non-quadratic cost,
  fuller dynamics), which is a new controller and a research project, not a
  port. It would also always need a feasible interior point handed to it,
  since it cannot bootstrap. And none of it reaches the target: the STM32MP1
  has no compute GPU, so this is an argument about the algorithm and for a
  different board, not a path to shipping.

  Stage 2 (Python MPPI in the loop over the existing UDP bridge, flat dash)
  and Stage 3 (the course suite) are not started, and Stage 1 does not
  recommend them as written.

  Byproducts, both default-off: `MPC_DUMP` / `MPC_DUMP_MAX` capture, and
  `CTRL_USE_JCQP` to override the yaml so the board's solver can be studied
  on this host (the Mac ships `use_jcqp: 0`, the board ships `1`).

- **OPEN-30 · The lie-down tips onto its side at the finish, and it is not
  OPEN-28** — `CLOSED 2026-09-10` (the arrival, not the lie-down: the
  stick was at zero under a body still at cruise; the ramp now seeds from
  the body's speed and the settle bails on a worsening trend — 15/15 clean
  finishes against 11/15, details at the end of this entry). On `hp_gap20`, of 51 runs that reached the end without
  an E-stop, **12 roll past 28.65° during the stop/lie-down and 10 fail the
  judge** (`laydown: z=0.078 roll=40.5 → BAD`). Present in every `WP_VSLEW`
  arm (4/2/4/2), so not the slew. The roll comes to rest at **40.5° in four
  separate runs to the decimal** — a deterministic stop against geometry, not
  a dynamic fall — at z ≈ 0.06–0.08, below the `setStandUpHeight(0.15)`
  target. This is the OPEN-27 lie-down chain (PASSIVE hop → STAND_UP at 0.15 →
  edamp) going over sideways on this course; `wkc_finals` did not show it
  (FIX 9/14 PASS). Excluded from every OPEN-28 count as "lie-down tips".

  **Anatomy (2026-09-10, the 38 finishes of `open28_fix2x2`, 9 tipped).**
  Every finish is the same sequence. The waypoint fires at 1.5 m with the
  body at 0.2–0.5 m/s; `[stop] shedding 0.00 m/s` — the planner's end brake
  had the STICK at zero already, so the ramp is 0.6 s of zero-velocity
  locomotion under a moving body (the OPEN-27 dwell, on this course); the
  body pitches **8–12° nose-down** in it; the stop wait's 8° attitude bail
  fires at 0.1–0.4 s with 0.2–0.5 m/s still on the body (36/38); the settle
  watch enters at 9–13° and its 8° bail fires on the first sample
  (0.03 s, 38/38 — "BALANCE_STAND is diverging" was judging the ENTRY angle);
  the lie-down begins from that posture and 9/38 come to rest at 24–45°
  roll (40.5° to the decimal five times: the body propped on the folded
  legs). Nothing in the chain had a chance to level the body.

  Fix built (deployed after the running campaign), both halves A/B-able:
  (1) `decelerateAndConfirmStopped` floors its seed at the body's measured
  speed (`WP_STOP_SEED_MEASURED`, default on) so the stick follows the body
  down instead of standing at zero under it; (2) `settleOnFeet` bails on a
  worsening trend — entry + 3°, floored at the old 8°, capped at 20° —
  instead of the entry angle (`WP_SETTLE_TREND`, default on), so a body that
  comes in at 10° gets its 1.5 s of BALANCE_STAND.

  **First 12 runs of the A/B (`open30_finish2_partial12.csv`, 07:04–07:23,
  stopped early — see below):** of the runs that reached the finish,
  fin_new **3/3 PASS** — `[stop] seed raised from the stick's 0.00 to the
  body's 1.9–2.0 m/s`, a 40-step ramp, the wait COMPLETES (speed < 0.15,
  no attitude bail), settle in 0.23 s at worst 3.2°, laydown roll 0.9–7.2°
  — against fin_old **1/3**, the two failures tipping to 40.5° from a
  0.03 s settle bail exactly as anatomised above. The body arrives at the
  final waypoint at CRUISE speed (2.0 m/s at 1.5 m out — the planner's
  end brake never slowed it, only the stick), so the measured seed is
  doing all of the deceleration. n = 3 vs 3; the campaign was stopped
  because both arms were collapsing mid-course at 3/6 (below) and the
  finish cannot be measured on a course that is not reached.

  **The full A/B (`open30_finish3`, 12 + 12 interleaved on `hp_gap20`,
  08:05–08:55, host stragglers swept, every run reached the finish):**

  | arm | finishes | wait completed (speed < 0.15) | settle attitude | laydown attitude | tips (roll ≥ 24°) | PASS |
  |---|---|---|---|---|---|---|
  | fin_old (stick seed, entry-angle bail) | 12 | 1/12 (11 attitude bails at 0.16–0.38 s with 0.25–0.46 m/s still on the body) | roll 1–9°, pitch 3–13° | pitch 13–14° in 6 of the passes | **2** (40.5° both) | 10/12 |
  | fin_new (body seed, trend bail) | 12 | **12/12** (seed 1.91–1.99 m/s, a ~40-step ramp) | roll 0.0–0.7°, pitch 1.4–1.8° | roll 0.0–4.1°, pitch 0.0–0.3° | **0** | **12/12** |

  Pooled with the partial: **15/15 vs 11/15** on the verdict, and every
  continuous endpoint moved by an order of magnitude in the direction the
  anatomy predicted. Both halves ship on by default
  (`WP_STOP_SEED_MEASURED`, `WP_SETTLE_TREND`); the planner's end brake
  leaving the body at cruise 1.5 m from the point is left as it is — the
  ramp now handles it, and shortening the acceptance radius would change
  every course's geometry. This is the finish-line fall class the operator
  ruled out ("we should NEVER do that EVER"), on the course where OPEN-27's
  fix had not reached.

- **OPEN-28 · What now limits `wkc_finals` is sustained cruise, and it is
  OPEN-26's mechanism** — `CLOSED 2026-09-10`. **What limited the course
  was the simulation harness, in two layers, not the robot:** (1) macOS
  holds loopback UDP datagrams for 20–45 ms about once a second, and a
  freeze landing on a stance exchange held the swing command through the
  flip — the "no precursor in any channel" collapse; fixed by moving the
  controller↔bridge link to Unix-domain datagrams (9 → 0 mid-course
  collapses on hp_gap20, same binary); (2) sustained host contention from
  non-mission stragglers (a `gz topic -e` spinning for 12 days at 52 %
  CPU, two orphaned feeds) turned pass rates 100 % → 35 % while every
  process check said the rig was clear; swept, and the gate now sweeps.
  With both gone: `wkc_finals` at 1.9 **24/24 PASS, 16/16 waypoints, 0
  crossings, 0 sinks** (`open28_wkc_vcap`, both arms, interleaved; ended
  at 24 only because a Time Machine backup began and the launch gate
  refused, correctly). The robot-side finding that survives is the MPC
  contact table's physical lead of 3 segments — measured, documented, and
  left as it is because every shorter lead collapsed more. The speed
  ladder above 1.9 is the next campaign; the sixteen dead hypotheses below
  were all tested against a harness that was the cause. With the finish-line fall
  closed (was OPEN-27), the remaining failures are mid-course: 32 of 162 runs
  across four rounds, 18 of them while navigating to wp7. That concentration
  is **exposure, not a location** — "wp7" is the longest sustained-cruise leg
  on the course (the falls happen 30-57 m from the target, mid-leg, heading
  constant, `w ≈ 0`, `v = 1.90` steady), so it simply offers the most seconds
  at cruise. It is arm-independent, appearing in every arm of all four
  rounds including untreated ones.

  Traced to the event, it is **OPEN-26 exactly**: steady trot for seconds,
  then pitch runs 9.7° → 28.2° → 33.5° in about 0.6 s, crosses
  SafetyChecker's 28.65°, `op_mode` goes to 2, `track_err` drops to the ~3.2
  floor that means the leg commands have been zeroed, and the body drops.
  Two of two traces examined, both flavours (level fold and rolled) share it.

  So this is not a new defect — it is the known capability ceiling at
  1.9 m/s, and OPEN-26 already established that the 28.65° limit is not what
  binds (a four-point dose-response showed no ordering, p = 1.000, and only
  9/67 excursions ever recover). The open question is a control one: what
  makes the pitch run away.

  **Three hypotheses killed, each by its own control:**

  | hypothesis | the number | the control | verdict |
  |---|---|---|---|
  | overspeed | runaway starts at 2.08 m/s median against a 1.90 command | non-falling runs reach p99 2.12, max 2.26; **12/12 exceed the lowest fall onset** | dead — it is the gait-cycle peak |
  | host stall | — | control period holds 1.1–2.7 ms right through the window | dead |
  | turning | 16/23 falls have \|wz\| > 0.25 in the preceding 2 s | **matched** 2 s windows of ordinary cruise: 62.2 % contain one too. p = 0.53 | dead |

  The turning one is worth keeping as a warning: scored per-sample, ordinary
  cruise is only 5.4 % turning and the same 16/23 comes out at p ≈ 0. The
  signal was entirely an artifact of comparing a *window* statistic against a
  *sample* rate. Match the statistic to the claim.

  **Kept:** these falls are **deterministic** — `BASE`, `FIX` and `SKIP` on
  the same rep give identical numbers to two decimals (1.99 / 1.86 / 2.09 /
  2.30). Same seed, same fall. That makes the event bisectable rather than a
  dice roll, which is a far better position than the other two OPEN-26 threads
  ever had. Also: `dash:60` at 1.9 is **3/3 PASS**, so a straight does not do
  it, and the speed sweep with the OPEN-27 fix in now reads 1.5/1.7/1.9 all
  3/3 with 2.1 at 1/3 — the reliable ceiling moved 1.7 → 1.9.

  **No isolated feature reproduces it.** `corner:<leg>:<angle>` puts one
  corner with a real approach and a real exit under the same speed:

  | angle | fell | median peak pitch | max |
  |---|---|---|---|
  | 30–135° (leg 12 m) | **0/36** | 5.2–6.7° | — |
  | 150° | 0/6 | 6.4° | 6.9° |
  | 160° | 0/6 | 6.2° | 6.6° |
  | 170° | 0/6 | 5.5° | 6.2° |
  | 175° | 0/6 | 5.3° | 5.5° |
  | **180°** (the course's own reversal) | 0/6 | **13.5°** | 14.9° |

  0/66, and the worst peak anywhere is 14.9° against a 28.65° limit. The 180°
  reversal is measurably the hardest — it doubles the peak — but it still has
  half the margin it needs. `dash:60` at 1.9 is 3/3 PASS. So neither the
  straights nor any single turn is what fails.

  **A stale-snapshot trap, found and guarded.** Six rows came back identical to
  two decimals — one run recorded six times. shm segments outlive the process
  that created them, so a run that aborts before the controller starts leaves
  *both* `ctrl_0.log` and the ring holding the previous run's contents, and
  `dump_snapshot` returns them. By each trace's own `[RUNID]`: rounds 1–3 were
  0 stale out of 35/42/41; round 4 was **5 stale of 42**, all one run, all at
  the campaign's start. Not constant, so it must be checked per run.
  Impact, checked not assumed: all five report wp=7 and sit in the excluded
  bucket, so OPEN-27's headline is unchanged (BASE 4/8 vs FIX 0/10,
  p = 0.0229); OPEN-28's round-4 count moves 15 → 10, weakening the wp7
  clustering. `campaign_run_id()` now goes into every campaign row.

  **Chaining is dead too, and the dose-response is what killed it.** Three
  sub-courses cut from the course's own turn list, 10 reps each:

  | sub-course | fell | median peak pitch | max |
  |---|---|---|---|
  | `wkc_box` (90/90/90) | 0/10 | 7.5° | 19.1° |
  | `wkc_weave` (75/−75/75/−75) | 0/10 | 8.3° | 12.5° |
  | `wkc_hairpin` (90 then −180) | **0/10** | **16.1°** | 20.9° |

  The hairpin roughly doubles the excursion of the other two, so sequencing
  does cost something — but nothing fell. Then the dose-response on the
  variable the hypothesis actually names, recovery distance before the
  reversal, 8 reps each:

  | gap | fell | median peak pitch | max |
  |---|---|---|---|
  | 7 m | 0/8 | 18.5° | 22.0° |
  | 10 m | 1/8 | 17.8° | 21.7° |
  | 14 m | 1/8 | 18.9° | 34.2° |
  | 20 m | 2/8 | 16.5° | 38.5° |

  Slope −0.13°/m, R² 0.47 — no gradient, and the falls go the *wrong way*.
  More recovery is not better. Approach speed into the reversal was the
  obvious follow-up and it is also flat: fell 2.19 m/s, passed 2.20,
  p = 0.459.

  **What it did buy: a 50 s reproducer.** The `hp_gap` family falls 4/32
  (12.5 %) with peak pitch reaching 34–38°, against a 250 s course that falls
  20–30 %. Five times the falls per hour of rig time.

  **The live lead — the body sinks BEFORE the pitch runs away.** Read
  backward from the E-stop rather than forward from the last calm sample,
  both traces examined in detail show the same order:

  ```
  hp_gap14 rep6      z      pitch
    t-0.55s        0.271      5.1     sustained roll -10 deg, height held
    t-0.50s        0.254      3.8     <- HEIGHT GOES FIRST
    t-0.45s        0.247      9.6
    t-0.35s        0.232     17.2
    t-0.20s        0.195     25.7
    t-0.05s        0.179     29.9     -> E-stop
  ```

  `kin_z` tracks `z` the whole way down, so the legs really are shortening —
  this is loss of support, not an estimator artifact. Across the `hp_gap`
  family, falls differ from passes on exactly the quantities that picture
  predicts, though at n = 4 falls and with tiny effects: height lost across
  the run 0.009 vs 0.004 m (p = 0.035), median |roll| at cruise 0.90° vs
  0.70° (p = 0.010), median foot-force sum 9.39 vs 9.56 (p = 0.005, falls
  *lower*). Three tests at n = 4 is where small-sample luck bites, so these
  are a direction to test, not a result.

  **Duration is part of it, but not all of it.** `wkc_finals` (positive
  control) / `long_easy` (200 m of 40° turns) / `dash:200`, 6 reps at 1.9,
  all fell **1/6**. A 200 m straight falls; a 60 m one did not. But the peak
  excursions are nothing alike — 17.9° on the course against 5.6° and 7.7° —
  so the course is doing something the others are not, at the same fall rate.

  Normalised to falls per 100 m of cruise, everything measured so far:

  | mission | fell | m/run | falls per 100 m |
  |---|---|---|---|
  | `corner:12:{30…135}` | 0/36 | 36 | 0.000 |
  | `corner:8:{150…180}` | 0/30 | 24 | 0.000 |
  | `wkc_weave` / `box` / `hairpin` | 0/30 | 82 | 0.000 |
  | `long_easy` (40° × 10) | 1/6 | 388 | 0.043 |
  | `wkc_finals` | 1/6 | 203 | 0.082 |
  | `dash:200` straight | 1/6 | 200 | 0.083 |
  | `hp_gap07…20` | 4/32 | 84 | **0.148** |

  Distance alone would give every row the same rate; it does not. But every
  zero in that table is also a short mission, so the two are confounded at
  these sample sizes — which is the honest statement, not a mechanism.

  **Approach length and loss of support: both dead at full n.** `hp_gap07`
  2/46 vs `hp_gap20` 5/46, p = 0.434 — the n = 8 signal did not hold. And the
  support picture, scored at n = 5 falls against 85 passes with a Bonferroni
  threshold: height lost across the run p = 0.374 (it was 0.035 at n = 4),
  ride height *higher* in the falls, |roll| p = 0.046. Only foot-force sum
  survived, and that was small-sample luck too — re-scored on crossings below
  it is p = 0.102. The sequencing claim I made from two hand-picked traces
  ("height goes first") is **2 of 5** measured per fall. It does not hold.

  ### The settling test: it is NOT margin. There is a second mode.

  With ten hypotheses dead, the remaining possibility was that there is
  nothing to find — that every run draws a peak excursion from one smooth
  distribution and falls when the draw crosses 28.65°. That is testable and it
  failed. Fit only the *body* (peaks below a cut well under the limit, with
  that cut's truncation in the likelihood), then extrapolate:

  | threshold | observed P(peak ≥ x) | predicted | |
  |---|---|---|---|
  | 25.0° | 0.154 | 0.058 | 2.7× |
  | **28.65°** | **0.131** | **0.027** | **4.9×** |
  | 32.0° | 0.115 | 0.014 | 8.2× |
  | 36.0° | 0.088 | 0.006 | 15× |
  | 40.0° | 0.081 | 0.003 | 27× |

  The real tail is far heavier than the body predicts and the gap *widens*
  with distance — from 25° to 40° the observed probability barely halves,
  where a Gumbel drops twenty-fold. Insensitive to the fit cut (22° → 0.018,
  25° → 0.027 against 0.131 observed).

  Pooling missions of different difficulty would manufacture a heavy tail, so
  it was refit **within** each family: `hp_gap` predicts 0.003 and observes
  0.215 (**69×**), `corner` 12×, `wkc_sub` 4.8×. Not a mixture artefact.

  **So OPEN-28 is a real distinct mechanism, and roughly 13 % of runs enter
  it.** Once entered the excursion runs a long way — that flat tail from 25°
  to 40° is the signature.

  ### And the event to study was the wrong one all along

  260 runs: **34 cross the limit, 14 fall.** Every fall crosses; nothing falls
  without crossing; **59 % of crossings recover** via the ESTOP ladder.
  Recovery is downstream noise. Studying falls threw away 20 of 34 events,
  which is why every discriminant came back null at n = 4–14.

  Re-scored on **crossings** (`--outcome crossing`, n = 22 vs 76 in `hp_gap`):
  still nothing separates at Bonferroni — ride height p = 0.640, sag 0.059,
  roll 0.044, foot force 0.102. So the mode is not predicted by any
  *run-level* property. It is entered by something momentary.

  ### Found: the mode is entered by a yaw-rate overshoot

  Event-aligned, each entry against **its own run's earlier cruise** (31 entry
  windows vs 812 control windows, so every run-level confound is identical by
  construction):

  | channel | entry | own cruise | p |
  |---|---|---|---|
  | `wz` yaw rate | 1.671 | 0.220 | <0.0001 |
  | `wx` roll rate | 2.339 | 0.901 | <0.0001 |
  | `wy` pitch rate | 1.880 | 1.096 | <0.0001 |
  | max joint track error | 29.2 | 22.9 | <0.0001 |
  | peak foot-force sum | 14.0 | 11.1 | <0.0001 |
  | **all four feet off ground** | **0.000** | **0.000** | — |
  | **control period** | **3.71** | **3.57** | **0.958** |

  `wx` and `wy` are near-tautological (they are the derivatives of the
  attitude being thresholded). **`wz` is not** — yaw rate is independent of
  the pitch/roll definition, and it is up **7.6×**. Timing is definitively
  dead (p = 0.958) and there is no airborne phase.

  And it is a genuine **precursor**, not part of the event:

  - `|wz|` passes 1.0 rad/s before the attitude passes 12° in **29 of 29**
    entries, by a median of **2.0 s** (censored at the 2 s search window, so
    the true lead is longer).
  - In a window ending 0.6 s *before* any attitude rise: entering runs
    **1.53 rad/s (88°/s)**, never-entering runs **0.21 rad/s (12°/s)**,
    p < 0.0001, z = +8.63.

  **And the body is not doing what it was told.** The follower's yaw command
  is capped at `WP_MAX_YAWRATE`, default **1.20 rad/s**, and the run maximum
  is exactly 1.20 in every trace. The body reaches a median **1.78** and up to
  **1.99** — a ratio of **1.48**, with **24 of 28** entries exceeding anything
  commanded by more than 25 %. This is a yaw-tracking **overshoot**, not the
  planner asking too much.

  That also explains the earlier turning null: that test used `|wz| > 0.25`
  rad/s (14°/s) as "turning", which is ordinary cruise. Entry involves 88°/s.
  Right quantity, wrong value — the same trap as
  `feedback-effect-size-does-not-transfer`.

  **The commanded yaw is not the lever.** `WP_MAX_YAWRATE` 0.8 / 1.0 / 1.2 on
  `hp_gap20`, 30 reps each, interleaved:

  | yaw cap | crossed 28.65° | fell | median peak attitude |
  |---|---|---|---|
  | 0.8 | 7/29 | 3/29 | 18.8° |
  | 1.0 | 5/29 | 2/29 | 18.5° |
  | 1.2 | 8/29 | 3/29 | 18.4° |

  Flat. Cutting the commanded ceiling by a third changes nothing — and reading
  the command **at the moment of the excursion** (the `[nav]` ring timestamps
  do align with the records; it was the `[stm32mp1]` heartbeat on the other
  clock) says why:

  - body **1.70 rad/s** (97°/s) against a command of **0.30 rad/s** (17°/s)
  - **ratio 6.4×**, and in **20 of 41** entries the follower is asking for
    under 0.3 rad/s — the dog is being told to go nearly straight

  **So the yaw is a disturbance, not a response to a demand.** Capping a
  command that is not the source cannot help, which is exactly what the sweep
  shows.

  ### The candidate cause, and it is in MIT's cost function

  ```
  float Q[12] = {0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1};
                  roll pitch yaw  x  y  z  wx wy  wz  vx  vy  vz
  ```

  `Q[6]` and `Q[7]` — **roll rate and pitch rate — are exactly zero**, and the
  roll and pitch *angle* weights are 0.25, the smallest in the vector. With no
  cost on the attitude rates there is nothing in the objective that resists
  the angular velocity a yaw disturbance produces, and the excursion runs to
  the limit.

  Unitree's own second vector, recovered from `Legged_sport` .rodata and
  already wired as `CTRL_MPC_Q=1`, is `{0.5,0.5,10, 20,20,15, 0.1,0.1,1,
  0.5,0.5,0.5}` — **non-zero rate weights**. It moves four things at once
  though, so `CTRL_MPC_Q=2` was added: MIT's vector with *only* the rate
  weights changed (`CTRL_MPC_QWX/QWY/QWZ`), so the effect is attributable.

  **The rate-damping hypothesis is refuted.** 30 reps each on `hp_gap20`,
  interleaved:

  | arm | crossed 28.65° | fell | median peak attitude |
  |---|---|---|---|
  | `q0` MIT default | 11/30 | 3/30 | 19.8° |
  | `q2` rate damping alone | 10/30 | 3/30 | 18.2° |
  | `q1` Unitree's full vector | **4/30** | 1/30 | 18.1° |

  `q2` vs `q0`: **p = 1.000**. Giving the attitude rates non-zero weight —
  the specific thing I predicted would matter — changes nothing. That arm also
  raised the yaw-*rate* weight 0.3 → 1.0, and that did nothing either. So
  `Q[6] = Q[7] = 0` is not why the disturbance goes unrejected, however
  suggestive it looked.

  **But Unitree's full vector does something**: 4/30 vs 11/30 crossings,
  Fisher **p = 0.072**. Suggestive, not significant — which is where this
  project adds an arm rather than N.

  `q1` differs from `q0` in four independent groups, and the rates are already
  excluded, so the effect is in one of the other three:

  | group | MIT | Unitree | arm |
  |---|---|---|---|
  | roll/pitch angle, height | 0.25, 0.25, 50 | 0.5, 0.5, 15 | `q5` |
  | **x/y position** | **2, 2** | **20, 20** | `q3` |
  | linear velocity | 0.2, 0.2, 0.1 | 0.5, 0.5, 0.5 | `q4` |
  | attitude rates | 0, 0, 0.3 | 0.1, 0.1, 1.0 | `q2` — **dead** |

  Position is a **10×** change and by far the largest.

  **Did not replicate.** `q1` 4/30 vs 11/30 became 7/22 vs 4/23 — the opposite
  direction — pooled p = 0.499. The control arm's own rate swung 0.37 → 0.17
  between campaigns; the base rate moved more than the effect. Every isolated
  group flat. The cost vector is not the lever.

  **The heading-feedback gate: dead.** `CTRL_YAW_RATE_ALWAYS` 0 vs 1, 30 reps
  each, interleaved: crossings 6/30 vs 5/30, falls 2/30 vs 2/30, median peak
  roll 14.9° vs 15.6° — p = 1.00 on all three. The gate is real and it is off
  in the failing regime, but turning it on changes nothing. The first four
  rows had read 46.7/45.6 vs 18.1/15.3 on roll; that was two unlucky control
  draws, and I flagged it as n = 2 at the time. Thirteen hypotheses dead.

  ### Where the crossings actually are, and what the trace cannot see

  The nav lines sit ~18 s **behind** the records (I had this right in
  OPEN-27, then wrongly "corrected" it). Placed on the nav clock, the moving
  crossings are **mid-straight at full cruise** — 9 m into the opening leg,
  3 m into the westbound leg, 12 m after the reversal — not at the reversal
  and not at a corner. The reversal work answered a real but different
  question: `WP_VSLEW` cuts the exit roll 14.5° → 4.9° with a clean
  dose-response and takes exit-window crossings to zero, but the crossings
  were never mostly there.

  The moving crossings look like this, every one: **≥6 s of nominal trot at
  1.9–2.0 m/s, z = 0.27, pitch 2–3°, wz ≈ 0 — then z drops ~3 cm in ~0.3 s
  and the attitude jumps 25° in 0.2 s.** Tested at that instant with matched
  controls: force envelope 0.84 vs 0.87 (p = 0.93 — **withdrawn**, it summed
  foot SPEEDS, see the `foot_fz` correction below); loop period over the last
  second 4.33 vs 4.26 ms (the 0.3 s spike is during the collapse); mean
  feed-forward torque identical until −0.4 s, exceeds +3σ a median **0.13 s**
  before the crossing, and z/attitude precede it in 11–12 of 16 — the
  controller is *fighting* the collapse, not causing it. Fourteen hypotheses.

  Also found: `track_err[0..3]` are not per-leg errors — they are `cmdKinZ`,
  mean joint error, worst joint error, mean |τ_ff| (`RobotRunner.cpp`, the
  `_te4` block). Slot 3 read as "leg 3 in 17/17 falls" for an hour.

  Something under the body **gives, instantaneously, with no precursor the
  trace can carry**: a foot slipping, a contact-solver pop, or a
  self-collision. The trace has no foot-level horizontal data, so it cannot
  tell them apart. Gazebo contact truth and pose truth can. Next campaign:
  `hp_gap20` under `campaign_truth.sh` with the qpOASES QP capture on, dumps
  kept only for runs that cross.

  The **lie-down tips** (roll ≈ 29° at z ≈ 0.10, no E-stop, 8 of 9 at the
  finish) are a separate judge-level issue and are excluded from "crossings".

  ### The candidate that fits everything: Gazebo's silent torque clip

  The controller clamps joint torque at `setMaxTorqueCheetah3(208.5)` — a
  **Cheetah 3** number, on a Go1. The bridge applies the impedance law with no
  clamp. The only limit the Go1 ever meets is the SDF's **23.7 N·m** (abad,
  hip) and **35.55 N·m** (knee), enforced by Gazebo and reported to nobody.
  The QP is allowed `f_max = 175 N` vertical and ±70 N in the friction cone
  per foot; at the Go1's levers that is ~24 N·m vertical plus ~14 N·m
  horizontal on a knee limited to 35.55, and the hip/thigh at 23.7 is closer
  still. **The controller's own bounds permit forces the motors cannot
  deliver, and nothing tells it.**

  Every measured property fits: instantaneous; no precursor in any body-state
  or control channel; joints and height depart *together* (the torque
  available fell, not the command); commanded torque ramps 0.18 s later into a
  cap already binding; deterministic per seed; worse with speed.

  Instruments, all committed: `server.py` now forwards `BRIDGE_*` from a slot's
  extra to the bridge (`BRIDGE_DUMP` is the only record carrying both `q` and
  `tau_ff` per joint, and per-run extra never reached the bridge before);
  `fleet_world.py::apply_effort_scale` (`GO1_EFFORT_SCALE`, diagnostic only —
  the real motors do not scale); `open28_torque.py` reads the dump back;
  `open28_clip.sh` runs base vs ×2 interleaved with the dump on both.

  **The A/B closed: 11/30 vs 6/30 moving crossings, p = 0.25.** Not
  separable, and consistent with what the per-exchange analysis then found —
  doubling torque rescues intermediate cases, because it only acts on the
  *recovery*; it does not touch ~~the support hole that initiates the fall~~
  (the "support hole" was a misread field — see the `foot_fz` correction).

  ### Through the instrument at n=33 runs, 86 sink events — the picture inverts once more

  The probe's "knees at 100 % in cruise" was a **one-run artefact.** Across 33
  dumps the cruise p99 is 0.66–0.87 of the limit and **0.0 % of cruise samples
  exceed it**, in passed and crossed runs alike. The clip does not bind in
  ordinary cruise, and it does not initiate anything.

  What the dumps *do* show, once the event is defined properly. The 2 cm sink
  I had been treating as the collapse's first symptom is **routine**: 86
  events in 33 runs, one every ~17 s of cruise, split exactly 43/43 between
  the two diagonal stance pairs, from a body that is *calmer* than average
  (|wy| 0.99 vs 1.48 rad/s before onset, p = 0.003). It happens equally in
  both arms (39 vs 47 events), so the torque limit has nothing to do with
  causing it. Almost all recover.

  **The ones that don't are the ones the controller cannot afford:**

  | | escalated (8) | recovered (78) | p |
  |---|---|---|---|
  | demand 0.3 s *before* onset | 0.82× | 0.92× | 0.018 |
  | **demand in the first 0.15 s after onset** | **1.73×** | **0.92×** | **< 0.0001** |
  | escalation per sink, base | 6/39 | | |
  | escalation per sink, ×2 limits | 2/47 | | 0.13 |

  Within 150 ms of a sink starting — before the attitude has moved — the
  controller asks for **1.73× the motor's torque** on the sinks that will go
  over, and **0.92×** on the ones that will recover. Recovery from an ordinary
  sink already uses **92 % of the motor.** There is no reserve. A somewhat
  larger sink needs ~1.7× and gets 1.0×, clipped silently by Gazebo, and the
  leg cannot hold. With the limits doubled the same demand is met and
  escalation drops 15 % → 4 % (not yet separable at 86 events; the 60-run
  campaign is still collecting).

  **Correction, one hour later — they are not the same event.** Measured in
  the *first moments* rather than over 0.3 s:

  | | escalated (8) | recovered (80) | p |
  |---|---|---|---|
  | descent rate, first 50 ms | **0.45 m/s** | −0.04 m/s | < 0.0001 |
  | depth at 50 ms | 4.2 cm | 2.5 cm | < 0.0001 |
  | attitude by 150 ms | **23.3°** | 8.6° | < 0.0001 |

  A recovered sink is a 2 cm dip that has already stopped descending. An
  escalating one is **falling at 0.45 m/s within 50 ms** and is at 23° — a
  fifth of a degree from the E-stop — by 150 ms. The 1.73× demand is the
  controller's proportionate *response* to a body already going down, not a
  cause; the ×2 arm's own two escalations had their 1.71–1.74× demand **met**
  and fell anyway. So the torque clip does not initiate, and doubling torque
  rescues only the intermediate cases (base's 1.19× / 5.9 cm one, say). The
  recovery-margin story is real but secondary.

  What is primary: **roughly 1 sink in 11 is a sudden loss of support** on
  flat ground at steady cruise — support removed in under 50 ms, no precursor
  in any channel, both diagonal pairs equally, deterministic per seed. Nothing
  with any torque budget catches something that fast. The question is now
  exactly what removes the support. The trace cannot see a foot slide; Gazebo's
  contact truth can, and that campaign is queued behind the A/B.

  ### The anatomy of an escalation, with the controls it needs

  Measured at the onset of every sink — the 9 that escalated, the 95 that
  recovered, and 234 random cruise instants — from the bridge dump's joint
  angles and the trace's foot heights:

  | at onset | escalated | recovered | random |
  |---|---|---|---|
  | first joint to leave its command (>0.15 rad) is a **swing hip** | **100 % (9/9)** | 37 % | 39 % |
  | first leg geometry to move (>2 cm) is a **stance** leg | **100 % (9/9)** | 63 % | 24 % |
  | a stance joint over its torque limit, 60 ms before | 100 % | 88 % | 49 % |
  | that swing hip over its torque limit, 60 ms before | **22 %** | 75 % | 57 % |

  Two rows discriminate and two do not. The stance-joint clip is a feature of
  *sinks* (100 vs 88 %), not of escalation — the torque story is finally dead
  as a cause. What every escalation has, and most sinks do not, is a **swing
  leg's hip pushed ≥0.15 rad off its trajectory while its motor has headroom**
  (22 % clipped vs 75 % on ordinary swings), followed within ~30 ms by a
  stance leg giving way. Rear hips in 7 of 8. A soft swing-leg gain
  (kp = 8 N·m/rad) deflects 0.15 rad under ~1 N·m — a few newtons at the
  foot. That is a swing foot **hitting something**.

  On flat ground the something is another leg. Hindlimb–forelimb interference
  at speed: the front stance foot drifts back through stance at 2 m/s while
  the rear swing foot reaches forward, and on a 0.38 m body the nominal
  clearance at end-of-stance is a few centimetres. All twelve leg links carry
  `self_collide`, so it is modelled. Testable from the dump alone: joint
  angles → forward kinematics → foot-to-calf distance at onset.

  **Tested, and dead (16th).** MIT's `computeLegJacobianAndPosition` ported
  verbatim (self-check: stance feet at −0.277 m, |y| 0.128 m — convention
  correct). Minimum swing-foot-to-stance-calf distance over [−100, +50] ms
  around the sink: escalated **18.3–20.5 cm** (median 19.3), recovered 18.3,
  random 18.9; **0 events under 3 cm in any group.** No leg touches another
  leg. Whatever pushes the swing hip off its command is not a leg.

  ### The handoff, first version — WITHDRAWN: `foot_fz` is foot SPEED

  Named error, per the rule at the top of CLAUDE.md. On 2026-09-04 the trace
  field `foot_fz` was changed from a force to the per-leg **world-frame foot
  speed in m/s** (`ShmTrace.h` says so in the header — "the name is now
  historical"; `RobotRunner.cpp` computes `|rBody^T (vBody + ω×r + v_leg)|`).
  The OPEN-26 scripts written that morning use it as a speed. From 09-05 the
  OPEN-28 analyses read it as a force again: the "force envelope" control,
  `open28_support.py`'s `fz`, `open28_entry.py`'s `fzmin/fzmax`, the "loaded
  leg switched to swing" and "what load did the soft leg take" checks, and the
  first `open28_handoff.py` — "new pair loaded (Σfz ≥ 4)", "old pair off
  (fz < 1)". The memory rule *read the writer, not the field name* was written
  on this very issue after `track_err[3]`, and a field named `fz` whose header
  comment says SPEED was read as force anyway. The check has to be a grep of
  the header for every field a script consumes, at the time the script is
  written.

  Withdrawn: the "force envelope" dead hypothesis (never measured); the table
  that read "new pair loaded +25 ms after old pair off, 6/11 escalations vs
  4/313" — what it measured is that at escalations the NEW feet were still
  moving ≥ 4 m/s summed 25 ms after touchdown, i.e. a foot that lands and
  keeps moving, with the old pair planted; the conclusion "the contact table
  is behind the feet" drawn from it; and the lead-1 "hole rate" baseline.
  What survives: the escalation/recovered split by early touchdown (foot
  height against the schedule), the old-pair torque 11.4 vs 6.4 (bridge), and
  the campaign's raw data, which is re-scored below.

  ### On honest signals: every exchange is a ballistic hop, and the contact table runs (knob + 2) segments ahead of the feet

  `open28_handoff.py` v2 uses only checked writers: `c0..c3` (the gait
  SCHEDULE — `contactEstimate` copies it), `z + foot_z` (FK foot height),
  `z`/`vz`, and the bridge's `tau_ff` on its wall clock, aligned by first
  dumped row = first trace record (validated against the E-stop edge on five
  FAIL runs: ±6 ms; a cross-correlation "refinement" was tried and moved the
  offset by +50..+106 ms because Σ|τ| is periodic at the half-cycle — do not
  refine). An exchange is a scheduled flip (a pair's `c` rising); everything
  below is relative to that instant.

  **The 60-run baseline (knob 1, the shipped default), 38,700 exchanges** —
  old-pair max knee |τ_ff| at −80/−60/−44/−30/−22/0/+8 ms:
  **20.4 / 16.5 / 1.0 / 0.9 / 0.9 / 17.3 / 13.4** N·m. The stance pair's MPC
  force is CUT at the −66 ms boundary — three MPC segments before its
  scheduled swing — and it stays on the ground unloaded for 66 ms of its
  110 ms stance. The trace agrees without the bridge: body vz at
  −44/−22/0/+8/+30 ms = **+0.30 / +0.05 / −0.16 / −0.20 / 0.00** m/s, i.e.
  a_z ≈ −10 m/s² over [−44, +8] — **free fall for ~50 ms of every 110 ms
  half-cycle** — then +9 m/s² as the NEW pair, which lands 16 ms EARLY on
  swing gains (kp 700 Cartesian in the WBC's swing task), arrests it. The
  trot is a sequence of hops with a 0.8 cm bob; the early touchdown is not a
  defect, it is what has been catching the body. The 17.3 N·m burst on the
  old pair at 0..+8 ms is swing initiation.

  **Manipulation check** (the SCHED_LEAD campaign, 15 runs so far): knob
  0 / 1 / 2 → cut at the **−44 / −66 / −88 ms** boundary (±10 ms sampling),
  unsupported time (neither pair both on the ground and commanded) 18 / 44 /
  86 ms per exchange. The knob works, 22 ms per step, and the effective lead
  is **knob + 2**. One of the two extra steps is in the code: `run()` takes
  `mpcTable = gait->getMpcTable()` — a pointer INTO the gait's `_mpc_table` —
  then the async prefetch block, which is not gated on `_mpcAsync`,
  recomputes `_mpc_table` one segment ahead and "restores" only
  `setIterations`, so the inline solve reads the NEXT segment's table through
  the aliased pointer. MIT's own +1 on top makes +2. The third step is
  measured, not yet located; a per-solve print of `_iteration` against
  `table[0..3]` settles it and is queued behind the campaign.

  **The third step, explained (2026-09-10, from the data already taken):**
  the per-solve print shows the QP receives `table[seg + knob]` exactly, so
  the extra segment is not in the code path — it is the OPTIMISER. At
  knob 0 the departing pair's knee torque does not cut, it tapers
  (17 → 15 → 12 → 10 → 8 → 6.6 N·m over the last 60 ms, then the swing),
  where every table-driven cut is a cliff. The QP sees the pair leave at
  step 1 and unloads it during step 0 because MIT's weights make that
  nearly free: the vertical-velocity weight is 0.1 against 50 on height,
  and one step of free fall costs 2.4 mm of height (50 × 0.0024² ≈ 3e-4)
  and 0.22 m/s of vz (0.1 × 0.22² ≈ 5e-3) against a force cost of
  4e-5 × 60² ≈ 0.14 saved. So "physical lead = knob + 1" is the solver
  anticipating the schedule by one step, and the shipped behaviour is
  table lead 2 plus that anticipation. Not a defect to fix; recorded so
  the knob's semantics stop being a mystery.

  This retires "the contact table is behind the feet": it is AHEAD of them,
  by 66 ms, and the body is unsupported for roughly the second half of every
  stance. OPEN-28's crossings are the cases where the early-touchdown catch
  is defeated — which is why doubled torque rescued the intermediate cases
  (a stronger catch) and no planner-side lever moved the initiation.

  ### THE INITIATOR, FOUND: the bridge freezes the command for 40 ms once a second, and a freeze at the exchange is the collapse

  Re-doing the crossing anatomy on honest signals (the last scheduled
  exchange before each sink, against the same run's ordinary exchanges,
  86 runs): at **20 of 26** crossing exchanges the NEW pair's last touchdown
  came ≥ 30 ms AFTER the flip (ordinary: 0.7 %), and in ordinary cruise a
  late landing is followed by a > 2 cm sink **49 %** of the time against
  **0.06 %** after a normal one. The joint commands in the bridge dump then
  showed what a "late landing" is: the swing command (hip q_des −0.50, hip
  τ −19.6, knee +12.8 — the swing PD driving the foot forward) is HELD past
  the flip for 20–40 ms, the just-landed feet skate at 1.8–2.1 m/s instead
  of stopping (ordinary: 0.04–0.09 m/s), the body free-falls on, the
  stance switch then arrives with the body 0.7 cm down at −0.47 m/s and the
  WBC sweeps the new pair back violently (hip q_des −0.50 → −1.30 in 30 ms,
  knee τ −29 N·m), the feet fling 4 cm into the air, the old pair barely
  lifts, and z drops 3 cm in 30 ms.

  The hold is the **bridge's receive thread being starved**. The dump rows
  (every 5th command, 100 Hz) show gaps of 33–45 ms roughly once per
  1.01 s, followed by 3 rows at 0–2 ms spacing — a backlog drained in a
  burst. So the controller kept sending (its own period never left
  2.4–2.9 ms; the E-stop edge aligns the two clocks to ±6 ms), the packets
  sat in the socket buffer, and `udp_rx` did not run for ~40 ms while the
  bridge's MAIN loop — whose `stalls>5ms` counter reads **0** every second —
  kept applying the stale `cmd`. The instrument measured the wrong thread.

  Conditioned on every scheduled flip in the 86 runs (54,000 exchanges):

  | exchange class | n | sink > 2 cm | crossing ≤ 400 ms |
  |---|---|---|---|
  | no receive gap within [−80, +60] ms | 52,960 | 0.32 % | 0.05 % |
  | a ≥ 30 ms gap elsewhere in that window | 454 | 10.4 % | 0.00 % |
  | **a ≥ 30 ms gap STARTING in [−25, +5] ms of the flip** | **128** | **24.2 %** | **14.1 %** |

  A 280× enrichment. 74 of 86 runs carry at least one such gap (26 of them
  crossed); of the 12 that carry none, 3 crossed. The gap is the initiator;
  the table-lead free fall above is what makes an exchange fragile enough
  for a 40 ms freeze to kill it — under the shipped lead the body is
  already falling at −0.16 m/s when the frozen swing command holds the
  catching feet forward.

  This is a defect of the SIM HARNESS (`cheetah_gazebo_bridge.py`), not of
  the robot, and it has been in every result since the bridge existed. It
  also reframes the campaign history: the "second mode" in the tail test,
  the base rate swinging 0.37 → 0.17 between campaigns, and every
  planner-side null are what a ~1 Hz external hazard with a 30 ms lethal
  window looks like from the outside. On hardware the analogue is the
  RS485 thread — which this port already flags for a four-bus harness.

  **The drain did not fix it, and that located the freeze.** With the
  command socket drained from the bridge's main loop, the dump still showed
  gaps of 30–63 ms at 0.09–0.21/s and `rx_backlog_max` of 10–22 packets once
  a second — a burst of queued packets arriving in one 2 ms pass. The
  controller's motor task (the sender, a separate thread — now given its
  own line in the heartbeat, `[stm32mp1] motor task: maxPeriod`) ran at
  p50 3.0 / p90 3.7 ms with one 46 ms event in 70 s. So neither end
  stalled: **the macOS kernel holds loopback UDP datagrams**. Reproduced
  standalone with a Python sender/receiver pair on a quiet host: send-to-
  receive latency max 37 ms, 12 stalls ≥ 20 ms in 20 s, `sendto` never
  blocking, seq contiguous — and unaffected by 1 Hz `/api/state` polls or
  a 1 Hz 2 MB loopback TCP transfer. An **AF_UNIX datagram** pair measured
  beside it: latency max 0.8 ms, zero delivery stalls.

  Fix (2026-09-10 04:06): `rt_gazebo.cpp` and `cheetah_gazebo_bridge.py`
  carry commands and sensors over Unix-domain datagram sockets
  (`$GAZEBO_SOCK_DIR/cmd_<port>.sock`, `sensor_<port>.sock`) when the
  directory is set and the peer is 127.0.0.1; the UDP ports stay bound so
  the conductor's stale-port sweep still works. (Scorer note, same day:
  the bridge/trace alignment by first row is only good to a few tens of
  ms — the motor task starts before the RobotRunner's clock — and one run
  read 44 ms off, which shifted its torque timeline by two segments while
  its trace-only vz/z were identical to its neighbour's. `align()` now
  refines by cross-correlation within ±80 ms only, narrower than half a
  gait period so it cannot lock onto the wrong tooth; validated against
  the E-stop edge to ±8 ms.); `server.py` sets the
  directory for both processes, and a slot extra of `GAZEBO_SOCK_DIR=`
  (empty) puts a dog back on UDP for A/B. The board is unaffected (remote
  peer → UDP as before). Validation run 4427 on the unix transport: **0
  receive gaps ≥ 20 ms in 87 s (worst 14 ms; UDP: 0.08–0.21/s ≥ 30 ms,
  worst 63), backlog ≤ 2 packets per pass** — and the run still collapsed
  at wp2, with the SAME anatomy as the gap-initiated ones and no gap: the
  new pair reaches the ground moving 0.7–1.3 m/s and never takes the load
  (vz keeps falling to −0.39 m/s through +36 ms, ordinary exchanges bottom
  at −0.2 by +8), then the retraction, then z −2.5 cm in 30 ms. So the
  frozen command was one trigger of a failure the exchange can produce on
  its own when the catching feet skate instead of gripping; the campaign's
  no-gap crossing rate (0.05 % of exchanges, 3 of 12 gap-free runs) is what
  the transport fix converges to, and the table lead (k0: the body is not
  falling when the feet arrive) is the lever on the residual.

  ### `open28_fix2x2` — transport × lead, 60 runs interleaved on `hp_gap20` (2026-09-10 04:09–05:54)

  | arm | transport | effective lead (cut before the flip) | receive gaps ≥ 30 ms /s | mid-course collapses | reversal-exit collapses (low speed) | finish tips (5/5 wp, then over) | PASS |
  |---|---|---|---|---|---|---|---|
  | udp_old | UDP | +3 (−66 ms) — the shipped behaviour | **0.253** | **9** | 1 | 1 | 4 |
  | unix_old | unix | +3 (−66 ms) | 0.005 | **0** | 2 | 4 | 9 |
  | unix_k1 | unix | +1 (−22 ms) | 0.018 | 5 | 0 | 1 | 9 |
  | unix_k0 | unix | 0 (MIT's own alignment) | 0.020 | 4 | 1 | 3 | 7 |

  Per exchange: a receive gap starting within 25 ms of a flip crossed 5.3 %
  of the time on UDP (3/57); the unix arms had 16 such gaps between them
  and crossed on none. Trace crossings at speed: udp_old **10/15**,
  unix_old **1/15** (Fisher p ≈ 0.001), unix_k1 4/15, unix_k0 4/15.

  **Three things this settles.**

  1. **The mid-cruise collapse is the transport.** Same binary, same lead,
  same course: 9 mid-course collapses on loopback UDP, 0 on unix datagrams.
  OPEN-28's headline mode — "≥ 6 s of nominal trot, then z −3 cm in 0.3 s
  and the attitude jumps 25° in 0.2 s, no precursor in any channel" — was
  a 40 ms freeze of the command stream landing on a stance exchange. No
  precursor existed because the trigger was outside the robot.

  2. **A shorter table lead does not help, and probably hurts.** Both
  shortened leads collapsed more (5 and 4 mid-course against 0) at
  n = 15 (pooled 9/30 vs 0/15, p ≈ 0.05), in the same direction as the
  dose-response (cut −44 ms crossed 10/20 against −66's 7/20). The
  "free-fall window" is real and it is not the lever; MIT's own alignment
  is the worst of the three here.

  **Then an anomaly, not yet explained (07:04–07:23):** on the corrected
  build (knob 2 = physical 3, cliff verified at −66 ms, unsupported 30 ms,
  unix transport, control loops clean at 3–5 ms), `hp_gap20` collapsed
  mid-course **6/12** — where the 2×2's unix_old arm (alias path, knob 1,
  the same physical lead) collapsed 0/15 two hours earlier. Five of the six
  are ROLL falls (25–35°, pitch 4–9°) on the wp00→wp01 leg, a different
  signature from the pitch-led crossings. The two lead paths are equal at
  the solver input and at the knee-torque cliff, so either they differ in
  something not yet measured, or the host changed. Running now
  (`open28_leadpath`, 10 reps each, interleaved): the exact 2×2 control
  (alias 1, knob 1) beside the fixed path at knob 2 and knob 1. If the
  control reproduces its 0/15 the fixed path is not equivalent and the
  default reverts to the alias path until the difference is located; if the
  control also collapses, the rig changed.

  **Resolved: the rig changed.** The control collapsed 2/4 too (all three
  arms 2–3 of 4), and the controller's own heartbeat showed stalls of
  46–191 ms in half the runs. `ps -r` found the load: a `gz-transport-topic
  -e -t /world/go1_world/pose/info -n 1` that had been spinning for **12
  days at 52 % CPU (8,361 CPU-minutes)** — a one-shot echo that never got
  its message — plus `pose_feed.py` and `contact_feed.py` orphaned by the
  withdrawn contact-truth campaign 12 h earlier at ~28 % each, beside the
  operator's own firmware simulator pinning a core. None are mission
  processes, so every hygiene check reported the rig clear. Killed the
  three (not the operator's process); load 5.9 → 3.6. Control batch on the
  same binary and config, six runs: **6/6 PASS**, three of them through
  single stalls of 44, 47 and 469 ms — so the binary is exonerated, the
  lead-path arms are equivalent, and every mid-course rate measured
  07:00–07:46 is void. What sustained contention does that a single stall
  does not is not measured; the campaign gate now sweeps stragglers by CPU
  and age and records the loop's worst period per run in the CSV.

  **And a mistake of my own, caught six runs in.** I set the default
  `CTRL_MPC_SCHED_LEAD` to 3 on the reasoning "knob = physical lead with
  the alias fixed", which rested on the solver-input print (table0 =
  table[seg + knob], true) and on the scorer's "old tau drop" column
  (biased near the flip by the swing-initiation burst). The first six runs
  of the finish A/B on that build collapsed **6/6 mid-course**, and their
  knee-torque cliff sat at **−88 ms** with 76–80 ms unsupported — the
  dose-response's worst arm (physical lead 4). Re-reading every arm's
  torque timeline: the physical lead is **knob + 1** (fixed path: knob 0
  → cut −22, 1 → −44, 3 → −88; alias path: knob + 2). One segment between
  the solver's table index and the legs is still unlocated. The default is
  now **2** — physical 3, bit-for-bit what every result in this file was
  measured at — and the rule for the next default change is written into
  memory: one probe run, and read the manipulation-check columns, before
  a campaign. The six runs are set aside as
  `open30_finish_INVALID_lead4.csv`.

  3. **What is left on `hp_gap20` is not OPEN-28.** On the clean transport
  the failures are finish-line tips (OPEN-30: the dog reaches all five
  waypoints, stops, and goes over at 33–45° roll during the lie-down —
  4 of 13 finishes at the shipped lead, 9 of 38 across the unix arms) and
  two low-speed collapses at the reversal apex/exit (v 0.7 m/s). Neither is
  a cruise collapse. OPEN-30 is now the dominant failure on this course
  and is next.

  **Transfer to `wkc_finals` (`open28_wkc_transport`, 15 + 15
  interleaved, 05:56–06:56):** udp_old **1 PASS**, 12/15 crossings at
  speed; unix_old **3 PASS**, 7/15 crossings. Receive gaps 0.319/s → 0.006/s.
  On this course a lethal-window gap crossed 5.0 % of the time (4/80) and
  the no-gap exchanges 0.21 % — against hp_gap20's 0.05 % — so the transport
  was only ever the initiator of about a third of the wkc crossings; the
  rest are a no-freeze collapse of the same exchange. Checked whether
  shorter freezes (≥ 12 ms) carry the remainder: crossing rate 0.17 % with
  one at the flip and 0.17 % without — no. **6 of the 7 clean-transport
  crossings are on the wp06→wp07 leg at a body speed of 2.04–2.13 m/s**
  (commanded 1.9): the long sustained-cruise leg, the one the record
  already singled out as "exposure, not a location". The speed there is
  the next thing to test.

  **Interim, 09:35 (`open28_wkc_vcap`, 10 of 30):** with the host's
  stragglers swept (the 12-day `gz topic -e` at 52 % CPU and two orphaned
  feeds, see the hp_gap20 anomaly above) and Spotlight kept off the
  snapshot archive, `wkc_finals` at 1.9 is **10/10 PASS, 16/16 waypoints,
  in BOTH arms** — the plain unix configuration that went 3/15 at 05:56 and
  the speed-capped one alike — through single loop stalls of 45–197 ms.
  So the 05:56 residual (7/15 crossings at 2.0–2.1 m/s, "no freeze
  involved") was measured under sustained host contention as well, and
  the speed-dependence table above was taken in that state. Whether an
  exchange still fails more often above 2.0 m/s on a quiet host is what
  the final 30 answers from their dumps; the cap has bound only during
  accelerations so far (`[VCAP] body 0.98 over stick 0.93`).
  **Speed ladder result** (`open28_wkc_ladder`, 2026-09-10 11:06–12:51,
  32 runs, four rungs interleaved one rung per rep, transport-fixed build):

  | cruise | PASS | where the failures trip | peak pitch, median (deg) |
  |---|---|---|---|
  | 1.9 | **8/8** | — | 20.2 |
  | 2.0 | **8/8** | — (one run reached 28.2) | 22.7 |
  | 2.2 | 1/8 | 7/7 after wp08, in the −180° reversal | 30.9 |
  | 2.4 | 0/8 | 8/8 after wp08, in the −180° reversal | 33.0 |

  Every failure is the same event: the orientation ESTOP (pitch 29.7–36.0°
  against the 28.65° limit) between wp08 and wp09 — the 9 m leg that IS the
  hairpin reversal — never the mid-cruise collapse this issue was about.
  Peak pitch rises monotonically with speed and 2.0 touched 28.2° once, so
  2.0 is the edge of the envelope, not a safe cruise; the course's limit on
  this build is one feature's pitch excursion, and the next lever is the
  reversal's entry/exit shaping (`WP_VSLEW`, the settle/exit knobs) at 2.2,
  not cruise. Host column: `loop_max` 2.6–17 ms across passes and failures
  alike, one 233 ms run (4615) among the 2.2 failures; four of the eight
  2.4 failures ran under 4.4 ms, so the separation does not rest on it.
  Harness note, corrected: from 12:04 a mis-keyed launcher started the
  3-dog fleet re-test during the ladder. The archive timeline shows the
  conductor serialised the launches — 4626 → 4627 (fleet) → 4628 → 4629
  (fleet) → 4630 — so the ladder rows ALTERNATED with the fleet reps rather
  than overlapping them, ran on an idle rig, agree with the other reps, and
  are kept; what was void was the fleet reps' intent (and the cap-to-2 they
  triggered), not the ladder rows. The earlier "rows after 12:04 are void"
  was written before the timeline was read. What those two accidental
  fleet reps did show: 3 dogs on `dash:100` trotting 3.0 all tripped the
  orientation ESTOP (pitch 29.7–32.2°) at 3.05–3.12 m/s cruise at
  DIFFERENT times (28, 36, 48 s), loops ≤ 4.25 ms, unix transport clean
  (500/s, backlog ≤ 2), while the 2-dog rep passed 2/2 — the old
  "trotting dash fails in parallel" item survives the transport fix; the
  chained 6-rep re-test with an RTF sampler is what answers it.

  Host-state change mid-campaign, for the record: at 10:18 (after row
  ~22) the operator authorised killing the other simulators and the
  firmware simulator that had held a full core for 6.6 days was stopped;
  rows before and after it are not on the same host. Spotlight's
  `corespotlightd` was at 74 % at that moment — `.metadata_never_index`
  in `rundata/` is not honoured on a directory, only at a volume root, so
  excluding `rundata` from indexing has to be done in System Settings →
  Spotlight Privacy; noted for the operator rather than changed.

  **Pre-registered before the campaign reports** (per the impossible-ordering
  rule): the story "the free-fall window is what the escalations exploit"
  predicts crossings and sinks ordered lead 0 ≤ 1 ≤ 2. It FORBIDS lead 2
  being the best arm. If lead 2 wins, the window is not the lever and the
  story is wrong, whatever the p-value.

  **The dose-response reported (60 runs, 20 per arm, interleaved):**

  | knob | cut (ms before the flip) | unsupported / exchange | sink rate | crossed | receive gaps ≥ 30 ms per s | crossing given a lethal-window gap | crossing per no-gap exchange |
  |---|---|---|---|---|---|---|---|
  | 0 | −44 | 14 ms | 0.9 % | **10/20** | 0.098 | 12.9 % (4/31) | 0.16 % |
  | 1 (shipped) | −66 | 32 ms | 0.6 % | **7/20** | 0.106 | 15.6 % (5/32) | 0.05 % |
  | 2 | −88 | 80 ms | 0.7 % | **13/20** (+2 zombie E-stops) | 0.091 | 0.0 % (0/18) | 0.10 % |

  The pre-registered forbidden ordering did not occur — lead 2 is the worst
  arm, decisively. But the monotone half of the prediction fails: 44 ms of
  free fall crossed no less often than 66 (10 vs 7, p ≈ 0.5), and the
  lethality of a receive gap is the same at both (13 % vs 16 %). So the
  free-fall window is NOT what makes a frozen command lethal — a swing
  command held through the stance switch skates the feet whatever the body
  is doing — and the table lead only matters once it is long enough to
  break ordinary exchanges (88 ms: sinks no more frequent, but the
  crossings come from no-gap exchanges and the arm collapses 18/20). The
  bridge fix is the primary fix; the lead fix is a robot-side defect in its
  own right (66 ms of every 110 ms stance unsupported, on hardware too) but
  it is not the lever on OPEN-28's crossings. The gap rate is equal across
  arms, as it must be for a bridge defect.

  Next: (1) done — above; (2) a build that fixes the aliasing and prints
  the table per solve, probed at knob 0 and −1 until the cut sits at the
  flip; (3) that configuration A/B'd interleaved against the default on
  `hp_gap20` and `wkc_finals`.

  ### Two of my own claims corrected

  The **"29/29 yaw precursor" was the hairpin itself.** Peak |wz| ≥ 1.0 rad/s
  occurs in **100 % of `hp_gap20` runs**, crossers and non-crossers alike
  (medians 1.92 vs 1.89). My control was random cruise windows, which never
  contain the reversal — a matched-control failure. The yaw precedes the
  attitude because the reversal precedes everything after it, not because it
  predicts anything.

  An apex detector then silently kept crossers and dropped passes (a passing
  run's global speed minimum is its *final stop*, which never regains 1.5 m/s)
  and produced a 27-vs-1 split. Bounded strictly between first and last
  cruise: **89 crossed, 299 did not.**

  ### What the reversal exit actually shows

  Nothing at it discriminates. Apex speed, exit duration, exit acceleration,
  yaw carried out, roll during and after, gait phase at the apex and at peak
  yaw — all identical, p 0.07–0.93. **Peak roll in the second after exit:
  14.06° vs 14.08°.** Crossers leave 10° at apex + 1.14 s (p10 0.97, p90 2.06).

  So every run is pushed to ~14° by the exit; 23 % keep going to 28.65° and
  77 % recover, split by nothing measurable beforehand. Same course, same
  commands, same body state, deterministic per seed. **A marginally stable
  exit decided below the trace's resolution.** The answer to marginal
  stability is margin, not a trigger.

  The margin is in plain sight: **the exit accelerates at a mean 2.30 m/s²
  against `WP_ALON = 0.4`, in every run.** `follow()`'s turn-first branch
  scales `v` down by heading error and snaps it back to the next leg's cruise
  the instant the error closes; the planner's `a_lon` shapes the *profile*,
  and nothing between `follow()` and the stick rate-limits the live command.

  **`WP_VSLEW` works exactly as built, and it is not the OPEN-28 fix.** Dose-
  response on `hp_gap20`, ~24 reps per arm, interleaved:

  | `WP_VSLEW` | exit accel (m/s²) | median exit roll | p90 | moving crossings |
  |---|---|---|---|---|
  | off | 2.30 | 15.5° | 18.3° | 12/25 |
  | 1.5 | 1.13 | 11.1° | 13.2° | 10/25 |
  | 0.8 | 0.74 | 7.6° | 8.6° | 10/24 |
  | 0.4 | **0.38** | **4.8°** | 5.2° | 13/24 |

  Exit acceleration lands on target; exit roll is monotone and every arm is
  p < 0.0001 against off. **Crossings are flat.** So the reversal exit was a
  real, measurable, now-fixed defect — and not where the collapses come from.
  Kept as a knob (default off) pending a decision on the course-time cost.

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
  **Materially de-risked 2026-09-04**, from this project's own reversed
  binary plus two public sources the operator supplied
  (`aatb-ch/unitree_crc`, `imcnanie/gooddawg`):
  * **The framing is already RIGHT, and an earlier worry here was wrong.**
    `motor_msg_GO-M8010-6.h` being vendored-but-never-included looked like
    the driver might not speak to Go1 legs at all. It does. aatb-ch's
    capture of a real Go1 RS485 bus gives header **0xFEEE**, CRC32 over
    **7 words (28 bytes)** — and `rt_unitree`'s
    `crc32_core(&pkt, (sizeof(pkt)>>2)-1)` with `sizeof(pkt)==34` is
    exactly 7 words. Same header, same frame, same CRC coverage.
  * **Gear ratios were wrong and are now FIXED.** The driver used
    `const float g = 9.1f; // A1 gear ratio` for all twelve joints, while
    `Go1.h` had long carried the values recovered from Legged_sport's
    constant pool: abad/hip **6.333**, knee **9.4995**. The model was
    corrected and the driver never was. Joint<->motor conversion was off
    by 9.1/6.333 = **1.44x** on abad and hip, and since kp/kd go as
    1/g^2, commanded stiffness was out by **2.06x**. On a first bench
    spin-up that is a small commanded move driving the motor 44% past the
    target at twice the stiffness. Now per-joint from the binary's numbers.
  * **Baud was wrong and is now FIXED**: 4 Mbaud is the A1; the Go1 leg
    motors run **5 Mbaud** (gooddawg, which drives Go1 legs off the body).
    `$UNITREE_BAUD` overrides for an A1 bench.
  * **Known-good bench hardware**: gooddawg drives Go1 legs standalone
    with a **ROBOTIS U2D2** over RS485 at **23-25 V** ("low voltage may
    cause a brownout" — consistent with the binary's `_batteryV = 24.0`),
    and sets the encoder zero by **fully extending the leg by hand before
    powering**. That removes "waiting on the fast RS485 adapter" as the
    blocker: the U2D2 is the adapter.
  **BENCH ADAPTER, settled with a vendor spec.** The U2D2 is **FT232HL,
  max 12 Mbps** (ROBOTIS' own comparison table) against the Go1 motors'
  5 Mbaud - 2.4x margin. The same table shows why the obvious alternatives
  fail: the older USB2Dynamixel is FT232RL at **max 3 Mbps**, and so is
  every FT232R-based USB-RS485 cable including FTDI's own USB-RS485-WE.
  The U2D2 self-directs, so set `use_rs485_hw_de = false`; the driver
  already treats a failed `TIOCSRS485` as non-fatal for exactly this case.

  **BOARD-SIDE BUSES: the RED gives ONE, the BRK gives four.** Read off
  `OP Revo Redux/Octavo/OSD32MP15x_RED_7x_sch-V1_2-1.pdf` rather than
  assumed:

  | UART | on the RED | usable for RS485? |
  |---|---|---|
  | USART2 (PD3-PD6) | wired to the WiFi/BT module (`USART2_TX->BT_UART_RXD`, `USART2_RTS->BT_UART_CTS_N`) | no |
  | UART4 | the 6-pin 2.54 debug console (`board_setup.sh`: "ttySTM0 is the console") | no |
  | **USART3** | **JP20 pin 8 = TX (PB10), pin 10 = RX (PB12), pin 11 = RTS/DE (PG8)** | **yes - the only one** |

  So `rt_unitree`'s four-bus default (`ttySTM1..4`) cannot be satisfied on
  the RED. One bus carries a 4-motor bench fine (~1.1 ms of wire time at
  112 bytes/motor, 5 Mbaud) but not twelve (~3.4 ms, past a 2 ms tick).
  The **BRK breaks out 138 GPIO pins** - effectively the whole SiP - so
  four USARTs can be pinmuxed there. That is the board to build a
  four-bus leg harness against; the RED is a one-bus bench.

  **The bench no longer needs the board at all (2026-09-04).** `rt_unitree`
  was Linux-only - `termios2`/`BOTHER` for the custom baud, `TIOCSRS485`
  for direction - so `unitree_probe` could only run on the MP1. It now has
  a Darwin path: `IOSSIOSPEED` for the 5 Mbaud (which is not in the `Bxxx`
  table), and `set_rs485()` returns -1 so the caller takes its existing
  non-fatal "assume auto-direction transceiver" branch, which is exactly
  what a U2D2 is. `stm32mp1/tools/build_tools_host.sh` builds a native
  binary; the cross build is untouched and the board remains the
  deployment target.

      bash stm32mp1/tools/build_tools_host.sh
      ls /dev/tty.usb*
      stm32mp1/tools/bin_host/unitree_probe /dev/tty.usbserial-XXXX 5000000 0

  So one motor + a U2D2 + a Mac closes the remaining unknowns, with no
  board, no transceiver, no soldering.

  What remains genuinely unverified: the field scalings (T x256, W x128,
  Pos x16384/2pi, K_P x2048, K_W x1024), the FOC mode value, and per-joint
  sign/offset. Those need a motor.
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
  
  **c19 (interleaved, server env swapped every run, N=8 each) answered it
  — and found a bug in this project's own rate limiter:**

  | arm | `cam_feed` receives | viewer gets | GPU | CPU |
  |---|---|---|---|---|
  | 10 Hz / 0.1 s | 10.0 fps | 10.1 | 6% | 30% |
  | 30 Hz / 0.033 s | **30.4 fps** | **15.1** | 12% | 33% |

  The sensor at 30 Hz renders 30 fps and the per-run subprocess *receives*
  30.4 of them — then delivers 15.1. Half the frames died in
  `cam_feed`'s own rate gate: `--rate` defaults to the same 30, so
  `min_dt` sat exactly on the arrival interval and ordinary jitter put
  about half the frames a hair under it. The earlier reading, "30 Hz only
  buys ~15 fps for double the GPU", was measuring that bug, not the
  camera. Fixed by giving the gate a 10% tolerance
  (`min_dt = 0.9 / rate`); proven with the stubbed-gz harness, 60/60
  frames emitted at 30 Hz where it previously dropped half. **The
  receive-rate log added for exactly this purpose is what separated
  sensor from transport** — without it the loss was invisible.
  **c20 ran and this is the real number** (2026-09-03, same interleaved
  protocol, server env swapped every run, N=8 each, `dash:30` walking @2.0
  flat, raw rows in `gazebo/conductor/data/c20_open23_camrate_ab.csv`):

  | arm | `cam_feed` receives | viewer gets | GPU | CPU | PASS |
  |---|---|---|---|---|---|
  | 10 Hz / 0.1 s (shipped) | 10.0 fps | **10.1** (9.9-10.2) | 6.1% | 36% | 7/8 |
  | 30 Hz / 0.033 s | 30.4 fps | **27.9** (27.0-28.6) | 12.2% | 37% | 8/8 |

  **2.77x the frames for 2.0x the GPU**, and the viewer now gets 92% of
  what the sensor renders instead of 50%. Every previous number on this
  issue -- the 15.5 that got it parked, and the 15.1 in c19 -- was
  measuring the rate-gate bug above, not the camera. The trade the parked
  disposition described ("double the GPU for +50% fps") never existed.
  CPU is unchanged and the pass rate does not separate (7/8 vs 8/8, one
  fall in the 10 Hz arm).

  **Disposition: still not moving the default, and now for a stated
  reason rather than a shrug.** 12% of the GPU is cheap for one dog; this
  conductor runs fleets of three, the cost is in the render loop, and
  host load has already silently explained a sim failure on this rig
  once. What changes is that `CAM_UPDATE_RATE=30 CHASE_FOLLOW_DT=0.033`
  is now a *documented, measured* choice that delivers genuinely smooth
  chase footage -- the operator's original complaint was "choppy, like
  you are taking screen shots and stitching them together", and 28 fps is
  the answer to it. **Open sub-question, worth one short campaign**:
  whether 12% per dog is additive across a 3-dog fleet. If it is not, the
  default should move.
  
  **Reopen test**, so the next report is a diagnosis and not a hunt: read
  `$CHEETAH_DATA/conductor/cam_feed.log`. If `rx fps` says 10 while the
  panel shows less, the loss is server→browser; if `rx fps` is below 10,
  gz is rendering slower and the answer is in host load, not the panel. Deliberately NOT changing a default blind — this
  rig has already had machine load silently explain a sim failure.

---

## CLOSED (symptom → cause → fix → evidence)

- **CLOSED (was OPEN-34) · The 100 m dash at 3.0 regressed to a coin flip — filed as a 3-dog
  failure, found to be the contact-table lead default, fixed with a speed-scheduled lead** — closed 2026-09-11 04:40.
  Re-measured 2026-09-10 13:00 (`fleet_dash_retest.log`, six 3-dog reps of
  `dash:100` trotting 3.0 on the unix-socket transport, fleet cap held at
  3): **1 of 18 dog-runs passed**; the accidental 2-dog rep earlier that
  day passed 2/2 and the solo dash at this speed is table-grade (3/3 at
  3.1 commanded). Every failure is the same event: the orientation ESTOP,
  pitch 29–38.5° with roll under 22°, at 2.9–3.1 m/s, and on each dog's
  OWN clock it is the same event every time: the ramp starts at 19.4 /
  24.4 / 29.4 s (the conductor's 5 s per-slot mission delay), cruise
  (> 2.9 m/s) is reached 5.5 s later, and the ESTOP follows **2.7–3.2 s
  after reaching cruise, 17–19 m from the start, on all six dog-runs of
  reps 5–6** — deterministic, not a coin flip. Measured during the
  reps, all clean: sim real-time factor mean 1.000 / p5 0.997 / p50 1.000
  (`/stats`, 20 s windows); command path 500/s with backlog ≤ 2; IMU path
  499–501/s with a worst gap ≤ 4 ms in cruise (the new bridge `imu_rx` /
  `imu_gap_max` fields — the one ~100 ms gap per run is in the first second,
  before the controller connects); control loop ≤ 3.4 ms on reps 4–6. Reps
  1–3 carried 10–12 ms loop hiccups on every dog, which were **my own RTF
  sampler** (`gz topic -e -t /stats` prints at 1 kHz) — killed before rep 4,
  and the failure rate did not change (0/9 after), so those hiccups are
  exonerated as the cause and recorded as an instrument load a campaign
  must not carry. With RTF, transport, sensor freshness and loop timing all
  measured clean, what remains is something that scales with three dogs in
  ONE engine: shared physics (though DART solves a dog touching only the
  static ground as its own island), or a world-build difference in the
  3-slot `fleet.sdf`. Two discriminating experiments, both manual: the same
  three dogs in three SEPARATE engines (`sim_up_n.sh`), and one active dog
  beside two idle spawned dogs in one engine.
  **14:12 — the premise is in doubt.** The first four SOLO `dash:100` runs
  at 3.0 on the freshly deployed OPEN-31 binary (chain A's
  `item5_contactgate_dash`) failed 4/4 with the identical signature (pitch
  trip 30–34° about 5 s after reaching cruise, 17–19 m out) — and the
  heartbeat's clamp counters go from 0 to 600–900/s the moment cruise is
  reached. So either the operational joint clamp breaks the sprint (the
  swing legs at 3.3 m/s ask for a straighter calf than −53° and the clamp
  cuts the reach), or the solo sprint already failed on this lineage and
  the "one passes, three fail" reading was built on a table that was never
  re-run on the current build. Chain A was stopped at that campaign; chain
  B (`campaign_chain_20260910b.sh`) runs, in order: the sprint with the
  clamp off vs on (interleaved, N = 6), the PREVIOUS binary on the same
  sprint if both arms fail, the item 6 A/Bs with whichever clamp setting
  ships, and a bracket of the OPEN-28 cruise knobs (table lead, transport)
  if the clamp is not the cause.
  **14:35 — the premise is gone; this is a solo-sprint regression.**
  `dash30_jointlimits` (solo, interleaved, N = 6 each): clamp OFF 1/6,
  clamp ON 1/6 — same signature, the clamp is exonerated. The 07:04 binary
  (the one the 2-dog reps passed on) redeployed for `dash30_oldbin`: 2/6,
  its two passes peaking at 24–25° of pitch, a few degrees under the
  28.65° limit. The one clean pass of the day (run 4695) cruised at a
  genuine 3.0–3.3 m/s by GPS with a 6° peak, so the course is achievable;
  the sprint is simply marginal now, solo, on both binaries (4/18 solo
  dog-runs vs 1/18 in fleets — not distinguishable at these N). Against
  the late-August table (trotting 3.1 commanded crossed 3/3, the wall at
  3.2) that is a regression, and no solo trotting dash above 1.9 had been
  run on any build since — the "one passes, three fail" reading was built
  on a table nobody re-ran (CLAUDE.md rule 3, again). Audit of every
  default that changed since that table (commit a3f4aa2): the world
  gained only four foot-contact LABEL sensors; the bridge's PD/torque path
  is unchanged; the mission side gained finish/boot knobs only
  (`WP_ADEC`, `WP_SETTLE_*`, `WP_STOP_ATT_DEG`, `WP_VCAP_*` at 0); the
  controller/bridge ship three behaviour changes ON — the contact-table
  lead implementation (`CTRL_MPC_TABLE_ALIAS` off, knob 2), the unix-socket
  transport with the bridge draining commands from its main loop, and the
  joint clamp (exonerated). Chain B brackets lead 1 / lead 3 / UDP; chain C
  (`campaign_chain_20260910c.sh`) adds the exact shipped lead (knob 1 +
  alias), the bridge receive thread, and velocity aiding off. The table's
  own provenance widens the window: its rows are dated 2026-08-22
  (`git blame`), so the x_comp_integral clamp (default 1.0 since 08-27; the
  table ran stock's unbounded integrator) and velocity aiding (default ON
  since 08-28) are inside it too — chain C carries aiding-off, chain E
  (`campaign_chain_20260910e.sh`) carries the clamp at stock and a run
  with the four foot-contact label sensors stripped from the proto. 
  **22:35 — FOUND: it is the contact-table lead.** `dash30_bisect` (solo
  `dash:100` trotting 3.0, four arms interleaved, 5 reps each, clamp off
  in all):

  | arm | table lead | PASS | peak pitch of the passes |
  |---|---|---|---|
  | base (today's default) | knob 2 = physical 3 | 2/5 | 7.1°, 24.4° |
  | **lead1** | **knob 1 = physical 2** | **5/5** | **8.0–10.7°, roll 3–11°** |
  | lead3 | knob 3 = physical 4 | 0/5 | — |
  | udp (loopback UDP transport) | knob 2 | 1/5 | 19.9° |

  Pooling every knob-2 sprint of the day (base, both clamp arms, the 07:04
  binary): **6/23 at physical lead 3 against 5/5 at physical lead 2**
  (Fisher p ≈ 0.002), and the lead-2 passes are clean where the lead-3
  passes are marginal. The transport is not it (UDP sits at the base
  rate). So the regression is the lead default this morning's OPEN-28
  rework shipped (`98202f9`, 07:05: alias removed, knob 2 "= the shipped
  physical 3") — chosen on hp_gap20 at 1.9 where the record says the
  shorter physical leads collapsed more, never run at sprint speed. Two
  things still to settle before the default moves: chain C's `ship` arm
  (knob 1 + the old alias, bit-for-bit the 08-22 behaviour) says whether
  the "shipped = physical 3" accounting was ever right in this regime,
  and chain F (`campaign_chain_20260910f.sh`) runs knob 1 vs 2 on hp_gap20
  at 1.9 (the cost) and on wkc_finals at 2.2 (the course limit that died on
  the reversal's pitch trip 7/8 this morning). If knob 1 costs the
  hairpin, the lead becomes speed-scheduled like the MPC segment.
  **23:45 — chain C: a SECOND independent fix, and the timeline
  resolves.** `dash30_bisect2` (solo 3.0, 5 reps, interleaved, clamp off):

  | arm | PASS | peak pitch of the passes |
  |---|---|---|
  | base (default lead 2, aiding on) | 2/5 (one "NONE" was a 38.5° ROLL trip that sat ESTOPped under the 50° fall detector until the runner timed out) | 5.8°, 21.2° |
  | ship (knob 1 + the old alias) | 3/5 | 7.5°, 16.6°, 20.5° |
  | rxthread (bridge receive thread) | 1/5 | 8.6° |
  | **noaid (velocity aiding OFF)** | **4/5** | **4.5–5.0°** |

  So velocity aiding — default ON since 08-28, AFTER the 08-22 table — is
  the other lever: with it off the sprint passes 4/5 with the cleanest
  attitude of the day, at the default lead. The table's configuration was
  exactly "old lead, no aiding", and that arm reproduces it (4/5 against
  3/3). Read together with chain B: two independent knobs each rescue the
  sprint (lead 1 at 5/5 with aiding on; aiding off at 4/5 with lead 2), the
  bridge receive path and the old alias do not. Mechanism candidate, not
  established: at 3+ m/s the GPS Doppler correction (K ≈ 0.7 at 20 Hz) steps
  the velocity the MPC tracks, and a longer table lead anticipates into that
  noise. Chain F was replaced by F2 (`campaign_chain_20260910f2.sh`): four
  arms (default, lead 1, aiding off, both) on hp_gap20 at 1.9, wkc_finals
  at 2.2, and the 3.0 sprint — the cost of each knob where the defaults were
  chosen, and the confirmation. The shipping decision waits for it.
  **00:50 — chain E rules out the last two candidates.** `dash30_xdrag`
  (interleaved, 5 reps): the clamp default (1.0) 4/5 vs stock's unbounded
  integrator 2/5 — the clamp is not the regressor, if anything it helps.
  `dash30_nosensors` (proto's four foot-contact label sensors stripped,
  6 runs, proto restored from git afterwards): 4/6 with clean peaks, the
  same hour base ran 4/5 — the labels are innocent. Note the default's own
  rate wandered from 2/5 (chains B, C) to 4/5 (chain E) with the host no
  quieter (WindowServer 40 %, BambuStudio 17 %, one pass carrying a 25 ms
  loop hiccup): the sprint at the default is a coin flip with wide
  between-campaign scatter, which is exactly why every decision arm in F2
  runs interleaved against the default in the same hour.
  **01:41 — F2 on hp_gap20 at 1.9 (the lead default's home, 6 pairs ×
  4 arms): no verdict cost, a measurable roll cost.** All four arms 6/6.
  Peak roll at the reversal apex, median (max): default 14.6° (15.4),
  lead 1 **17.9° (18.4)**, aiding off **20.0° (23.5)**, both 20.8° (22.3);
  per pair against the default, lead 1 +3.4° (6/6), aiding off +5.9°
  (6/6), both +6.1° (6/6). Peak pitch moves the other way (12.0 vs 14.1).
  So the longer physical lead the hairpin was tuned on really does buy
  roll margin there, and each sprint fix spends some of it — remembering
  that on concrete the default already peaks at 24.4° (27.0) at this same
  apex, lead 1 on a grippy hairpin would sit within a couple of degrees of
  the 28.65° trip. Yaw-saturation counts are also higher with both knobs
  moved (up to 37 per run). The decision therefore turns on wkc_finals at
  2.2 (does either knob move the course's pitch-trip limit) and the 3.0
  confirmation, both running; a speed-scheduled lead (2 below ~2.5 m/s,
  1 above, switched only at a cycle wrap the way gait changes are) is the
  candidate if the sprint needs lead 1 and the hairpin keeps lead 2.
  **02:39 — wkc_finals at 2.2 (5 reps × 4 arms, interleaved) settles the
  shape of the fix, and overturns this morning's ladder:**

  | arm | PASS | where it fails | peak pitch median | peak roll median |
  |---|---|---|---|---|
  | default (lead 2, aiding on) | **5/5** | — | 23.9° | 10.1° |
  | lead 1 | 2/5 | wp9, the reversal, pitch 30–33° | 30.5° | 19.0° |
  | aiding off | 4/5 | wp9 once | 24.5° | 11.4° |
  | both | **0/5** | wp7 ×3 by ROLL (34–41°), wp9 ×2 | 24.9° | **34.5°** |

  Lead 1 costs the course its 2.2 (2/5 against 5/5) through exactly the
  reversal pitch trip the ladder documented, and the two knobs together
  are destructive (roll trips at the box corners). So the sprint fix
  cannot be a global lead of 1: **it has to be speed-scheduled** — the
  longer lead below ~2.5 m/s where every course lives, the shorter one at
  sprint speed — adopted only at a cycle wrap, trotting only (the other
  gaits were never measured at lead 1). And the default's 5/5 at 2.2
  contradicts the ladder's 1/8 at the same speed on the same course this
  morning: that ladder ran the OLD alias path (`CTRL_MPC_TABLE_ALIAS=1`
  with knob 1, "bit-for-bit the shipped physical 3") under an 86 %
  Spotlight load; tonight's default is knob 2 on the fixed path on a quiet
  host. Either the alias path is not the same lead after all (chain C's
  ship arm also sat between base and lead 1 at 3.0), or the morning's
  limit was host load. `wkc22_ship` (chain G) runs ship vs default at 2.2
  interleaved to say which; the ladder's "limit between 2.0 and 2.2" is
  suspended until it does.
  **02:58 — the sprint confirmation (`dash30_final`, 5 reps × 4 arms) and
  the night's pooled 3.0 tally.** Confirmation: default 1/5, lead 1 **5/5**
  (peaks 8.8–12.1°), aiding off **5/5** (peaks 3.5–5.5°), both **0/5**.
  Every solo 3.0 sprint since 14:00, by configuration:

  | configuration | PASS |
  |---|---|
  | lead 1, aiding on | **10/10** |
  | lead 2, aiding OFF | 9/10 |
  | lead 2, aiding on — today's default, every plain arm pooled | **10/28 (36 %)** |
  | lead 2, aiding on, 07:04 binary | 2/6 |
  | lead 1 + the old alias, aiding on | 3/5 |
  | lead 3, aiding on | 0/5 |
  | lead 1, aiding OFF (both knobs) | 0/5 |
  | lead 2, aiding on: UDP / rx thread / clamp on / x_drag stock / no label sensors / gate on | 1/5, 1/5, 1/6, 2/5, 4/6, 0/2 — all at the base rate |

  Two independent, non-additive fixes; the pair is destructive. With
  wkc_finals at 2.2 (lead 1 costs the course, aiding off does not, both
  fail by roll) and hp_gap20 at 1.9 (each costs roll margin at the apex),
  the shipping choice is the **speed-scheduled lead** — the longer lead
  everywhere the courses live, the shorter one only in the sprint band,
  adopted at a cycle wrap, trotting only, aiding left ON (it is the
  galloping scale fix and it is not what broke). Built and committed
  (`745c5b6`); chain H deploys it after chain G and verifies it against the
  pinned default on the sprint, wkc 2.2 and hp_gap20 1.9. Aiding-off is
  recorded as the equally effective alternative that costs more hairpin
  roll (+5.9° vs +3.4°) and forfeits the galloping fix.
  **03:31 — chain G, ship vs default at wkc 2.2 (5 reps interleaved):
  default 2/5, old alias path 4/5** (Fisher p = 0.52 — nothing), against
  the default's 5/5 an hour earlier and the alias path's 1/8 this morning.
  Pooled, 2.2 on wkc_finals is a coin flip for BOTH lead implementations
  (12/23 across three campaigns, every failure the reversal's pitch trip
  at 29–32° against passes peaking 22–27°), with hour-to-hour swings that
  the bridge and loop instruments do not explain (identical stall,
  backlog and IMU-gap columns in passing and failing runs; the conductor
  at 7 threads / 122 MB after 23 h). So the ladder's morning reading holds
  in its honest form — **2.0 is the last reliable rung, 2.2 is ~50 %** —
  and the alias-vs-fixed question does not decide anything at 2.2.
  **08:08 — and the margin knob buys the rung.** `wkc22_vslew` (5 reps ×
  3 arms interleaved, scheduled-lead binary): `WP_VSLEW` off → 4/5, pitch
  median 26.0° (max 32.3, the one fall at the reversal); **1.0 m/s² →
  5/5, pitch median 16.5° (max 17.4), roll 8.1°, zero yaw saturations**;
  2.0 m/s² → 4/5, 24.0° (indistinguishable from off). Per pair the 1.0
  slew takes 8.7–15.8° off the reversal-exit peak (mean 11.1°, 5/5), for
  about one second of mission time (180.5 vs 179.4 s). The `[VSLEW]`
  lines show it binding exactly where the story says: on the command's
  RISE out of the reversal (wanted 0.25 → allowed 0.02, 0.62 → 0.57). A
  slew of 2.0 never binds hard enough to matter. Chain N (08:38, wkc 2.4, 5 pairs): control **0/5** (pitch median
  31.7°, every fall the reversal), **slew 1.0 → 4/5** (pitch median 24.7°,
  one reversal trip at 29.9°), mission 171 s against 180 s at 2.2. So the slew moves the
  course's usable cruise a full rung, from a 2.2 coin flip to a
  4-in-5 2.4, and the 2.4 peaks (22–26°) are about where the unslewed
  2.2 sat — the margin it buys is worth roughly 0.2 m/s of cruise on this
  course. On the concrete hairpin at 2.1 (08:58, 5
  pairs): control 4/5 with roll peaks 23–29° (one 29.4° pass under the
  debounce, one first-corner fall), **slew 1.0 → 5/5 with roll 11.3–12.2°**
  — the slew takes 11.8–17.2° (mean 13.7°) off the reversal-apex roll in
  5/5 pairs on the grippy surface and puts pitch at 16–22°. That is the OPEN-33 friction
  finding answered: the roll that grip adds at the reversal is a
  rise-of-command effect, and the slew removes it. **`WP_VSLEW=1.0` ships
  in the `course:` recipe** (`458be53`). Chain O (09:45) restarted the
  conductor the safe way, confirmed the served recipe carries the slew,
  and verified it against an explicit `WP_VSLEW=0` with no other change:
  wkc 2.2 recipe 4/4 (pitch median 16.1°) vs off 4/4 (23.6°); hp_gap20
  1.9 recipe 4/4 (roll median 9.8°) vs off 4/4 (15.1°). The recipe arm's
  own ctrl log shows the slew binding on the reversal's rise.
  **The course's envelope on the shipped recipe (chain P, 10:18, 5 reps
  per rung interleaved): 2.4 → 4/5, 2.6 → 0/5, 2.8 → 0/5.** Both dead
  rungs fail at wp7 — the third 90° corner of the box — by pitch (30–38°),
  not at the reversal any more: the slew fixed the feature that bounded
  the course at 2.2 and the next feature takes over 0.2 m/s later. So
  wkc_finals on this build runs reliably to 2.0, at 4-in-5 at 2.4, and
  the box corners are the next lever if anyone wants 2.6. First probe of
  that lever (chain Q, 10:42, the `wkc_box` sub-course at 2.6, 5 reps ×
  `WP_ALON` 0.4 / 0.3 / 0.2): 2/5, 3/5, **4/5**, with the 0.2 arm's passes
  the only clean ones (pitch 10–21°; 0.4's passes sit at 23–25°, 0.3's at
  29–37°). A gentler longitudinal budget helps the box in the expected
  direction but the third corner is still a coin flip at 2.6. The one 0.2
  run that "collapsed LEVEL" (run 5058, 5° peak pitch, a height collapse at
  wp4) was NOT the box: the bridge stalled for 247 ms with 124 commands
  queued (`stalls>5ms=1/worst=247.5ms rx_backlog_max=124`), the state the
  controller consumed froze for 244 ms at 1.99 m/s, and the body came back
  folding — the harness class of OPEN-35, found only when the scanner was
  written an hour later. Read with that fall removed, chain Q's 0.2 arm is
  4/4. Not shipped; N = 5.
  **Box at 2.6, N = 8 (chain R, 11:03–11:41, interleaved, harness falls
  checked with `campaign_freeze_report.py`: none):** `WP_ALON` 0.2 →
  **6/6** (pitch 10.3–16.1°, roll 6.9–7.7°); 0.4 → **3/7** with the
  seventh a fall the judge missed (run 5090, OPEN-36) and one of the
  three passes a 43° pitch / 31° roll near-miss (run 5088); the last two
  reps were lost to a Time Machine backup (OPEN-36). With chain Q: 0.2 is
  **10/10**, 0.4 is **5/12** (Fisher p = 0.003). WHERE the 0.4 arm dies is
  the finding: every fall (5056, 5062, 5082, 5084, 5086, 5090) is on the
  21 m leg to wp4, 3–5 s after the corner, at cruise — the estimate reads
  2.65–2.85 m/s under the 2.60 command, pitch builds stride over stride
  from 10° to 30° over ~0.7 s with the height sagging 0.27 → 0.19 m, and
  the E-stop takes it. That is the lead-2 trot above ~2.7 m/s — the sprint
  regression's regime (OPEN-34) — reached below the lead schedule's 2.7
  COMMANDED threshold, and 0.2 "fixes" it by never letting the leg get
  there (the ramp from the corner takes 6.5 s of an 8 s leg). Not a
  corner lever, a cruise one. Chain T runs the box at 0.4 under three lead
  policies (shipped 2.7 threshold / threshold 2.5 / lead 1 pinned) to
  separate "lead 1 rescues 2.6" from "the switch itself hurts"; the
  recipe's `WP_ALON` stays 0.4 until that is in.
  **Truth check (chain T's control arm, run 5101, `SIM_ESTERR=1`, 12:06):
  on the wp4 leg the ground-truth forward speed reaches 2.83 m/s under
  the 2.60 command and the estimate reads 2.78 — within 0.05 of truth the
  whole leg (`box_lead_score.py`). The over-speed is the body's, not the
  estimator's: the MPC's velocity tracking overshoots the command by
  0.2 m/s at this speed and the trot is then run at 2.7–2.85 on lead 2.**
  **Chain T result (12:22, box at 2.6 on the 0.4 budget, 5 reps × 3 lead
  policies interleaved, truth logging on, harness falls: none):** shipped
  schedule (threshold 2.7, lead 2 on every leg) **2/5**, fall pitch 35–46°;
  threshold 2.5 (`CTRL_MPC_LEAD_V_HI=2.5`, lead 1 adopted at 2.50–2.57 on
  each leg's ramp, back to 2 at the corners) **3/5 course-complete** (2
  PASS + 1 clean course failed by the lie-down judge at 21° roll, OPEN-30's
  mode; the 2 falls on the wp4 leg at 2.7 truth, on lead 1, 40 s after
  the switch); lead 1 pinned everywhere (`CTRL_MPC_SCHED_LEAD=1`) **5/5**,
  peak pitch 19.5–25.6° (median 20.5°) against the shipped arm's 35°
  median, at the same 2.72–2.77 m/s truth on the leg. So the shorter lead
  carries the 2.6 box; adopting it mid-ramp at 2.5 carries it less well
  than having it from the corner. Lead 1 everywhere is not free — the
  09-10 F2 sweep found it costs `wkc_finals` at 2.2 — so the candidate is
  an EARLY switch: threshold 2.0 (hysteresis 1.8), which leaves hp_gap20
  1.9 on lead 2 untouched, puts wkc's 2.2 legs and the box's 2.6 legs on
  lead 1 from the bottom of the ramp, and keeps lead 2 at the corners and
  the reversal. Chain V measures it on the full `wkc_finals` at 2.2 and
  2.4 and on the box at 2.6 against the shipped schedule and pinned lead 1.
  **Chain V, `wkc_finals` full course (14:27, 5 reps × 6 arms interleaved,
  harness falls: none):**

  | arm | 2.2 | 2.4 |
  |---|---|---|
  | shipped schedule (lead 2 on every leg) | 5/5, pitch median 18.4° | **2/5**, three falls on the leg after wp8 (pitch 30–34°) |
  | early switch (2.0 / 1.8) | 5/5, 17.0° | 3/5 PASS, 4/5 course-complete (one wp8-leg fall, one clean course failed by the lie-down judge right after the switch back to lead 2 in the arrival) |
  | lead 1 pinned | 5/5, **14.7°** | **5/5**, 23.8° |

  Lead 1 pinned is now 10/10 on the full course and 5/5 on the 2.6 box
  against the shipped schedule's 7/10 and 2/5, with the lowest pitch at
  2.2. The early switch fixes most of the 2.4 leg but its switch-back
  during the arrival has failed the lie-down judge twice today (2 of 6
  course-completes across chains T and V, against 0 of 19 for the other
  policies). The 09-10 F2 cost of lead 1 on wkc 2.2 does not reproduce on
  the slew recipe — the reversal it was charged to is the feature the
  slew removed. Remaining cell before lead 1 becomes the default: the
  hairpin apex at 1.9 and 2.3 (chain W).
  **Chain V, the 2.6 box on the 0.4 budget (14:39, 5 pairs, first runs
  with the controller's periodic tasks on the real-time band too, harness
  falls: none):** shipped schedule **3/5** (two wp4-leg falls, one ended by
  the new `down` judge), early switch **5/5** (pitch 22–29°, no lie-down
  failure this time). Pooled for the shipped schedule on this cell today:
  10/22; lead 1 pinned 5/5; early switch 5/5; threshold 2.5 3/5.
  **Chain W, the hairpin (15:21, 5 reps × 4 arms interleaved, harness
  falls: none): hp_gap20 1.9 — shipped schedule 5/5 (pitch median 10.9°,
  roll 10.2°) vs lead 1 pinned 5/5 (8.3°, 10.7°); 2.3 — 5/5 (18.6°, 11.2°)
  vs 5/5 (18.8°, 13.2°).** The 09-10 charge against lead 1 on the hairpin
  apex (+3.4° roll) is 0.5–2° here, inside run-to-run spread, with less
  pitch at 1.9. **SHIPPED 15:22: `CTRL_MPC_SCHED_LEAD: 1` in
  `host-run/ctrl_tuning.yaml` — lead 1 (physical 2) everywhere, the speed
  schedule left in the code, off.** Today's interleaved ledger for lead 1
  against the schedule: box 2.6 5/5 vs 10/22, wkc_finals 2.4 5/5 vs 2/5,
  wkc 2.2 5/5 vs 5/5 (pitch 14.7° vs 18.4°), hp_gap20 1.9 and 2.3 5/5 vs
  5/5, the 3.0 sprint 10/10 (already lead 1 under the schedule). Chain X
  runs the fast suite on the new default; chain Y probes the envelope it
  opens: wkc_finals at 2.6 and 2.8, hp_gap20 at 2.5 and 2.7.
  **hp_gap20 on the shipped recipe (chain R, 11:03, 5 reps per rung
  interleaved): 2.1 → 4/5 (pitch median 18.1°, roll 10.5°), 2.3 → 5/5
  (pitch 22.4°, roll 13.4°).** The one 2.1 fall (run 5070) is not the
  course: the state the controller consumed — orientation, foot
  kinematics, kin_z — was bit-identical for 63 consecutive ticks (124 ms,
  t = 73.27–73.40 s) at 2.17 m/s mid-leg, 5 s after the reversal, while
  the control loop kept its 2 ms period (max 2.43 ms); the estimate
  decayed to 0.81 m/s on the frozen kinematics, and when the state came
  back it jumped −8.9° roll / −6.7° yaw in one tick, the safety E-stop
  fired 60 ms later. That is the harness-stall class (a sensor path stalled
  under a running controller, the thing the loop's own period counter
  cannot see). New instrument: `gazebo/tools/state_freeze_scan.py` finds
  these in any snapshot (identical state under motion, loop period
  intact, jump on return); the other nine runs of the ladder carry no
  freeze over 10 ms except one 34 ms freeze at 0.7 m/s on the final
  approach and one 10 ms freeze that came with a 14 ms loop period. So
  on μ 0.6 the hairpin course runs 5/5 at 2.3 with less roll (13.4°) than
  the unslewed recipe carried at 1.9 (15.1°): the slew moved this
  course's envelope from 2.0 to at least 2.3. **Top of that envelope
  (chain S, 12:03, 5 reps per rung interleaved, first runs on the RT-band
  bridge and the down-judge binary): 2.5 → 3/5 (pass pitch median 23.6°,
  roll 10.9°), 2.7 → 0/5.** All seven falls genuine by the freeze scan,
  all on the leg to wp1 — the 2.7 runs by the pitch E-stop (29–38°) as the
  dog accelerates out of the first corner, the 2.5 runs at cruise on that
  leg — the same regime as the box at 2.6 (cruise above ~2.5 on lead 2),
  so the hairpin course's table on this recipe reads 2.3 reliable, 2.5
  three-in-five, 2.7 dead, and chain T's lead-policy answer applies here
  too. Host at the time:
  `corespotlightd` at 60 % of a core (7.7 h old) plus `mds` and a fresh
  set of `mdworker`s — Spotlight is off for `/` (`mdutil -s`) but its
  CoreSpotlight side is still hot, 2 cores of the machine. Chain I
  still measures the alias path's table at the solver input, because "the
  two are bit-for-bit the same lead" was a claim, and a cheap one to
  check.
  **03:31 — the speed-scheduled lead is DEPLOYED** (`deploy_host.sh`,
  loads and starts; previous binary kept as `mit_ctrl_sim.pre_sched_lead`)
  and proven to fire on its first sprint: `[SCHED] table lead 2 -> 1
  adopted at a cycle wrap (v=2.71, band sprint)` as the ramp crossed the
  threshold, run 4882 PASS with an 8.2° peak. Chain H verifies it against
  the pinned default on the sprint (N = 6), wkc 2.2 (N = 5) and hp_gap20
  1.9 (N = 5); the suite gains `dash_trotting_30` (fast tier) so a default
  cannot regress the sprint silently again.
  **03:42 — sprint verification: scheduled 6/6 (peaks 8.2–11.1°, roll
  under 5°), pinned default 0/6** (Fisher p = 0.001), interleaved on the
  same binary. wkc 2.2 and hp_gap20 1.9 follow — there the schedule must
  reproduce the pinned default, since it never leaves the slow band.
  **04:37 — verified on all three cells, same binary, interleaved:**

  | cell | scheduled lead | pinned default (knob 2) | switch lines in the scheduled arm |
  |---|---|---|---|
  | `dash:100` at 3.0 | **6/6**, pitch median 9.1°, roll 3.0° | **0/6**, pitch 34.2° | one per run, at v = 2.71 |
  | `wkc_finals` at 2.2 | 3/5 | 4/5 | 0 |
  | `hp_gap20` at 1.9 | 5/5, roll 15.0° | 5/5, roll 15.3° | 0 |

  In the slow band the schedule IS the default (no switch fires, and the
  peaks match to a degree); in the sprint band it is the fix. **CLOSED**:
  the regression was the contact-table lead default introduced with the
  09-10 rework, exposed by two defaults that were each tuned on a slow
  cell and never probed at the top of the envelope (velocity aiding since
  08-28, the lead since 09-10); the fix is the speed-scheduled lead, shipped
  03:31, and the suite now carries `dash_trotting_30` in its fast tier.
  *(Superseded 2026-09-11 15:22: lead 1 is pinned everywhere — see the
  OPEN-28 notes for chains T, V and W; the schedule was the right answer
  for one afternoon on the pre-slew recipe.)*
  Left in the record, not in the code: aiding-off is an equally effective
  sprint fix that costs more hairpin roll and the galloping scale; the two
  together are destructive (0/10); the 3-dog framing this item opened with
  was a control-arm error. **05:40 — the fleet re-measured on the
  scheduled lead: 6 reps of the 3-dog `dash:100` at 3.0, cap held at 3,
  18/18 dog-runs PASS** against 1/18 on the previous default the day
  before, on the same host with the same instruments. That also closes
  the CLAUDE.md line from the August campaign ("trotting 3.0, 100 m dash,
  3 in parallel 0/6 - CAUSE NOT ISOLATED"): the parallel dash was never a
  fleet effect, it was the sprint sitting on the wrong side of the
  contact-table lead, and the fleet just sampled it three times per rep. Chain I's solver-input probe (04:45,
  `STM32MP1_MPC_IN=2`, two runs each at wkc 2.2): the lead of the solver's
  step-0 table over the gait's own segment has the SAME distribution on
  the old alias path (60 / 20 / 20 % at lead 0 / 1 / 2 over 8,444 solves)
  as on the fixed knob 2 (60 / 20 / 20 % over 5,711) — "knob 2 fixed =
  knob 1 + alias" holds at the solver input, and the hour-to-hour swings at
  2.2 were the course's coin flip, not the implementation.

- **CLOSED (was OPEN-32) · The estimator trusts the schedule, not the foot: A/B the
  sensorless contact gate** — closed 2026-09-11 00:35, measured null. `SIM_CONTACT_GATE=1`
  (`PositionVelocityEstimator.cpp`) has existed since 2026-09-04 with its
  label-scored numbers (the schedule calls a foot planted while it is in the
  air 12.8 % of the time; requiring foot speed < 0.15 m/s cuts that to
  5.8 % at the same overall accuracy) and was never run against a verdict.
  Done = interleaved A/B, gate off vs on, both arms with `SIM_ESTERR=1` so
  the estimator's forward/lateral velocity error against truth is the
  measured quantity (`item5_score.py`), on hp_gap20 at 1.9 (N = 8), the
  100 m dash at 3.0 (N = 6) and wkc_finals at 1.9 (N = 6); the gate's own
  `vetoed X of Y` line proves the arm fired. Ship on only if the error
  falls and no course regresses; otherwise record which way it moved and
  leave it off. Campaigns `item5_contactgate*`.
  **hp_gap20 result (14:09, 8 reps, interleaved, `item5_score.py`)** — a
  NULL on the quantity it exists to improve:

  | arm | PASS | \|dvx\| mean / p90 (m/s) | \|dvy\| | \|dvz\| | vetoes |
  |---|---|---|---|---|---|
  | gate off | 7/8 | 0.041 / 0.044 | 0.026 | 0.170 | — |
  | gate on | 8/8 | 0.041 / 0.043 | 0.024 | 0.170 | **18.3 %** of scheduled-stance samples |

  The gate fired on nearly a fifth of the schedule's stance samples and the
  estimator's velocity error against truth did not move to three decimals,
  in any axis. The reading that fits: the KF's own trust ramp already
  discounts a foot at the ends of stance, which is exactly where a
  scheduled-stance foot is still moving faster than 0.15 m/s — the gate
  vetoes samples the filter was already ignoring. The 2026-09-04 label
  score (false stance 12.8 % → 5.8 %) measured schedule against contact
  truth, not against the filter's effective weight, which is why it
  promised more than this delivers. Verdicts 7/8 vs 8/8 (the one fall is
  the off arm's rep 6 at the first corner, loop clean). Peak pitch at the
  reversal exit is +1.9° with the gate on (higher in 6 of 7 passing pairs,
  one pair −2.0°; not significant at N = 7) — if anything the wrong
  direction. Also measured, both arms alike: the vertical velocity estimate
  is off by **0.17 m/s** on average during a 1.9 m/s trot while forward
  is off by 0.04 — a fact about the LinearKF worth its own look (the MPC
  reads vz, weight 0.1; the height governor reads dz/dt). The dash (3.0)
  arm was stopped at 0/4 (both arms) because the sprint itself is marginal
  on this build (OPEN-34) — it carries no information about the gate; the
  wkc_finals arm (chain D, 00:33, 6 pairs) says the same: 6/6 vs 6/6,
  |dvx| 0.040 vs 0.040, |dvy| 0.025 vs 0.024, |dvz| 0.174 vs 0.171,
  20.4 % of scheduled-stance samples vetoed, peak pitch +1.2° (4/6, not
  significant). **Closed as measured-null**: the gate demonstrably fires
  and the estimator's error against truth does not move on either course;
  the KF's own trust ramp already discounts the samples the gate removes.
  `SIM_CONTACT_GATE` stays OFF and stays in the tree as the arm that
  answered the question. What the campaign did establish is the LinearKF's
  vertical-velocity error itself: 0.17 m/s mean, zero bias, twice the
  spread of the truth and only weakly correlated with it (r = 0.22) — a
  noisy vz, not a wrong one, worth its own item if the height governor or
  the MPC's vz row ever needs it.

- **CLOSED (was OPEN-33) · Sim-fidelity gaps that were never A/B'd: foot friction and a
  noise-free orientation** — closed 2026-09-10 22:20, measured. Filed 2026-09-10 from
  CLAUDE.md's "known places the sim is more generous than reality" table.
  Corrected on filing: that table's **joint velocity "unbounded" row was a
  record error** — every joint in `go1_speedway.sdf` carries
  `<velocity>30.1</velocity>` (12 of 12, re-checked) and the hip reaches it in
  fast swings; what the sim still lacks is the torque-speed (back-EMF)
  derating, which is the actuator-dynamics row. What is left to measure:
  (1) **foot friction** — the proto ships feet at μ 0.6 against a ground
  plane with no friction element (default 1.0), and DART combines a pair by
  the MINIMUM (`ContactSurface.cpp` v6.13.2, read from source), so the
  shipped pair is exactly the URDF's 0.6 - not a flattering number; the
  `concrete` surface kind sets
  BOTH sides to 0.90 (`apply_terrain` + `apply_surface_feet`), which is the
  rubber-on-concrete figure. Arm `flat` vs `concrete`, interleaved on
  hp_gap20 at 1.9. (2) **orientation noise** — `VectorNavOrientationEstimator`
  forwards the sim's exact pose; the bridge now perturbs the quaternion with
  a 0.5° RMS bias random walk (τ ≈ 5 s) plus 30 % white noise
  (`BRIDGE_ORI_NOISE_DEG`, applied in `send_sensor`, printed on startup).
  Arm 0 vs 0.5°, interleaved. Done = both A/Bs run at N ≥ 6 per arm with
  the verdicts, peak attitude and loop-max columns compared; an effect
  either way is recorded as an envelope fact (not tuned away), a null is
  recorded as a null. Campaigns `item6_friction`, `item6_orinoise`
  (chain B, `campaign_chain_20260910b.sh`, after chain A was stopped).
  **Friction result (14:54, hp_gap20 at 1.9, 6 pairs interleaved) — NOT a
  null, and in the uncomfortable direction:**

  | surface | pair μ | PASS | peak roll, median (max) | peak pitch, median | body yaw rate at the reversal apex |
  |---|---|---|---|---|---|
  | shipped `flat` | 0.6 | 6/6 | 14.8° (16.4) | 13.8° | −1.2 rad/s |
  | `concrete` | 0.9 | 6/6 | **24.4° (27.0)** | 17.3° | −1.5 to −1.7 rad/s |

  Concrete's peak roll is higher in 6 of 6 pairs (+4.4 to +13.4°, mean
  +9.6°; sign test p = 0.016) and its worst run sat **1.6° under the
  28.65° orientation ESTOP**. Every peak is the same event, the apex of
  the 180° reversal at t ≈ 40.6 s, and the trace says why: with grip the
  feet do not skate, the body actually turns at the rate the follower
  asks (−1.5 to −1.7 rad/s against −1.2 on the shipped surface), and the
  lateral load that comes with it rolls the body. The shipped surface has
  been making the hairpin SAFER by slipping. Consequence for the envelope:
  every hp_gap / wkc reversal number in this tracker was measured at
  μ 0.6; on the real rubber-on-concrete figure the 1.9 hairpin has almost
  no roll margin, and the pre-planner's lateral budget (`a_lat_max` 2.5,
  friction-capped only below μ 0.28) does not know that more grip means
  more roll. Not tuned away here — recorded. The right follow-ups are a
  reversal speed ladder ON concrete and a yaw-rate cap that the body
  actually obeys on a grippy surface.
  **Concrete ladder (2026-09-11 06:45, hp_gap20 on μ 0.9, 5 reps × 3
  speeds interleaved, on the scheduled-lead binary):** 1.7 → **5/5**
  (roll median 18.0°), 1.9 → **5/5** (roll 23.8°, max 24.6°), 2.1 →
  **3/5** — the two falls are at the FIRST 90° corner (wp1), one by roll
  (33.5°) and one by pitch (30.5°), and the passes carry 25.6–26.6° of
  pitch. So on a real surface the hairpin course's envelope is 1.9 with
  ~5° of roll margin and nothing to spare at 2.1, against 2.0 on the
  shipped μ 0.6. Note what the speed does: at 2.1 the failure moves from
  the reversal to the first corner and splits between roll and pitch,
  which is what a course with no margin anywhere looks like.
  **Orientation-noise result (22:17, hp_gap20 at 1.9, 6 pairs)**: 0.5° RMS
  bias random walk + 30 % white on the IMU quaternion (the bridge announces
  it; the control arm has no such line): **6/6 vs 6/6**, peak roll
  18.7° vs 15.5° median (+2.5° mean, higher in 5 of 6 pairs, sign test
  p = 0.22 — suggestive, not significant), pitch and yaw rate null,
  loop-max equal. So a VectorNav-class error budget costs the hairpin a
  couple of degrees of roll margin at most; the Madgwick-on-DroneCAN
  path the real machine will run is a larger dose (1.5–3°), which is the
  next rung if this is revisited.
  **Dose run 2026-09-11 07:15 (hp_gap20 at 1.9, 5 reps × 0 / 1.5 / 3.0°
  RMS, interleaved):** 0° → 5/5 (pitch median 13.9°); **1.5° → 5/5** but
  two passes peaked at 25.3° and 26.6° of pitch and yaw saturation counts
  climb (0–24 per run); **3.0° → 3/5**, both falls at the FIRST corner
  (pitch 40.0° / roll 30.9°). So the envelope on orientation error is
  roughly: 0.5° free, 1.5° eats most of the pitch margin, 3° fails a
  fifth-to-half of runs on the hairpin course at 1.9. A DroneCAN/Madgwick
  orientation on the real machine needs to be measured against that
  before any 1.9 course result is believed on hardware; a VectorNav-class
  unit is inside the free band. **Closed as measured**: friction is an
  envelope fact (recorded, not tuned away), orientation noise a null at
  0.5°, and the joint-velocity row was a record error, corrected.

### CLOSED (was OPEN-27) · The finish-line fall was zero-velocity locomotion, not the brake and not the settle

**Symptom.** The Westminster course (`course:wkc_finals`) at 1.9 m/s reached
all 16 waypoints and then fell over. Roughly half the time — 18/37 across
three independent control blocks (47 %, 50 %, 50 %).

**First cause, wrong.** Attributed to braking harder than the body can brake;
`decelerateAndConfirmStopped` was given a bounded-deceleration ramp
(`WP_ADEC`). Two interleaved campaigns found nothing, for three separate
reasons, each of which had to be found the hard way:

1. The first A/B (`dash:30` @ 2.5) never applied the treatment to a failing
   run — all 10 falls were mid-dash (6.5-23.8 m), none printed
   `[stop] shedding`. "harsh 8/12 vs bounded 6/12" was dash variance.
2. Its `peak_pitch_deg` column was stale on every PASS row: it read the newest
   `*FALL.json` by mtime, and a passing run writes none. **Never key a
   measurement on mtime.**
3. On a real course the ramp seed is **zero** — the planner's own end brake
   (CLOSED-19) has already stopped the dog, so every finish-line trace logs
   `shedding 0.00 m/s`. `WP_ADEC` is dead code there.

**Second cause, also wrong, and the anomaly that corrected it.** Reading
run3326 from the stop rather than the descent put the fall 1.3 s *after* a
clean four-feet-down stop, inside the blind `sleep_for(1500)` that follows
every `setControlMode(3)`. A watched settle cut finish-line falls from 47 % to
14 %, replicated exactly (2/14 in two separate rounds) — so the effect was
real. But a three-arm dose test refused to order: the **stricter** 5° bail
fired *less often* than the 8° one (4 vs 11), which is impossible if the
threshold is the operative variable. Reading the `[settle]` lines rather than
the totals: almost every bail fires **0.02-0.03 s in, already at 34-171°**.
The robot was arriving at `BALANCE_STAND` already toppling.

**Actual cause.** Between the planner's end brake and `K_BALANCE_STAND` the
robot stands **stopped, in `LOCOMOTION`, at zero commanded velocity** for up
to ~2.6 s — the regime `ConvexMPCLocomotion::zeroVelHold`'s own note measures
as taking the oval's stop from ~1-in-3 tips to 7-of-8. The shape is nearly
deterministic across 29 runs: physically stopped **0.27-0.30 s** after the
last upright sample at speed, past SafetyChecker's 28.65° limit **1.15-1.27 s**
later, 6 of 6 with no exceptions. Nothing shortens that dwell, because the
wait after the ramp watches **speed only** and a toppling body is not still —
`vBody` never drops under 0.15, so it burns its full 2 s backstop.
`SafetyChecker` cannot catch it either: its orientation trip is suspended
across stop windows (see the 8.7 s blind spot, CLOSED). `op_mode` stays 0 to
the end.

**Fix (both now default-on).**
- `decelerateAndConfirmStopped`'s confirm loop ends on **attitude** as well as
  speed — `WP_STOP_ATT_BAIL=0`, `WP_STOP_ATT_DEG` to restore or tune.
- `settleOnFeet()` replaces the blind 1.5 s sleep at all four
  `K_BALANCE_STAND` sites: leave when actually calm, bail to the damped
  lie-down at 8° — `WP_SETTLE_WATCH=0`, `WP_SETTLE_BAIL_DEG`.

**Evidence.** Round 4, 42 runs, interleaved every rep, ramp intact:

| arm | reached wp16 | fell at the finish | PASS |
|---|---|---|---|
| `BASE` (neither) | 8 | 4 (50 %) | 2/14 |
| `FIX` (both) | 10 | **0 (0 %)** | **9/14** |
| `SKIP` (`FIX` + zero-seed ramp skip) | 8 | 0 (0 %) | 0/14 |

`FIX` 0/10 against the pooled three-block control 18/37 — **p = 0.0076**.
All treated arms pooled, 4/38 vs 18/37 — **p = 0.0003**. Whole-run PASS in the
same interleaved block, `FIX` 9/14 vs `BASE` 2/14 — **p = 0.0183**. Median
peak attitude after the stop: 48.5° → 16.7° pitch, 37.8° → 8.8° roll; runs
past the safety limit 7/8 → 1/10.

**A thing I broke and had to un-break.** The zero-seed ramp skip went in
unconditional, applied to every arm of round 3, and took the *untreated* rate
from 6/12 to 12/12. The ramp is not idle when the seed is zero — its steered
branch keeps the follower's steering live through the first two thirds, which
is exactly what CLOSED-19 added to stop the oval tipping sideways out of a
turn. **Shedding no speed is not the same as doing nothing.** Now off by
default (`WP_STOP_SKIP_NOOP_RAMP=1`), and round 4 confirms it: `SKIP` prevents
the finish-line fall but passes **0/14**.

**Not fixed by this.** Total falls only move 9/14 → 4/14 (p = 0.128), because
the mid-course mode is untouched — see OPEN-28.

**Tools.** `gazebo/tools/wkc_settle_ab.sh` (arms as `NAME:ENV` pairs),
`gazebo/tools/stopfix_score.py` (peak attitude after the stop, window taken
from the trace's own velocity — two clocks live in these traces and on a long
course the `[nav]` lines run ~18 s behind the records, so a log-keyed window
scores the wrong seconds; E-stop halts excluded, since that halt is a fall in
progress rather than a stop).

### CLOSED (was OPEN-26) · The "level collapse" was a pitch-triggered safety E-STOP, and the classifier was reading the corpse

**Symptom.** 68% of every `[FALL]` in the archives classified as "level
collapse" — |roll| < 10°, |pitch| < 15°, body height 0.035–0.09 m — a
mode this project had never characterised and could not explain.

**Cause.** There is no such mode. The classification was taken from the
attitude in each trace's LAST tick, which is the pose *after* the robot
has finished falling. The real sequence, measured:

1. the robot pitches past `SafetyChecker::checkSafeOrientation()`'s
   0.5 rad = **28.65°** limit;
2. `ControlFSM::safetyPreCheck()` sets **ESTOP**;
3. every leg command is zeroed — `tauFeedForward` goes to exactly
   0.000, measured;
4. it sinks under gravity on limp legs and settles **flat at ~1.5°**,
   which is what the classifier saw.

**Evidence.** 19 of 20 falls E-stop this way; **0 of 14 passes ever trip
it**; the E-stop precedes the body's descent by **0.41 s**. The tripping
attitude is real, not estimator error: **36.9° estimated vs 36.4° gz
truth** at the instant it fires.

**And the limit is not what binds capability.** Four-point dose-response
on matched cells (28.6 / 45 / 60 / 90°, where 90 disables the check in
practice): trotting flat @3.5 → 62% / 50% / 38% / 75%; walking flat @3.5
→ 88% / 94% / 88% / 100%. **No ordering; 28.6 vs 90 gives Fisher
p = 1.000 on both.** Independently, only **9 of 67** excursions past
28.65° recover when allowed to (13.4%, 95% CI ≈ 5–22%). So the E-stop
aborts runs that were going to fail anyway, ~0.4 s early. **Every
capability ceiling in this record is a real dynamic limit.**

**Fix.** No behaviour change. `CTRL_ORIENT_LIMIT_DEG` was added (default
28.65, unchanged) so the question can be re-asked cheaply if the
controller changes, and `SafetyChecker` now announces its limit and hold
once per run. The real fix is to the METHOD, and it is recorded in
SKILL.md: **classify a failure at the triggering event, never at the end
of the trace.**

**What it cost, and what that bought.** Two days, a `Record` layout
change (98 → 150 bytes), and five hypotheses built and killed against a
phenomenon that was not there — foot kinematics, orientation error,
contact loss as a cause, a contact gate as a fix, and the estimator
itself. `op_mode` was already in the trace the whole time and would have
answered it on day one; nobody had plotted it. The full investigation
trail is kept below, because each dead end leaves a number that stops it
being re-run.

**The trail, as it was written (OPEN-26's own log):**


- **OPEN-26 · This port fails by FOLDING, not tipping — 2:1, and it has
  never been characterised** — `CLOSED 2026-09-10`. The "level collapse"
  mode was a classification made at the END of the trace: the body was
  flat because `SafetyChecker::checkSafeOrientation()` had tripped at
  28.65° of pitch 0.4 s earlier and the zeroed legs let it settle (the
  09-04 finding recorded further down, and the memory "classify at the
  event, not the end"). The trigger of that pitch excursion during cruise
  was found under OPEN-28: a 40 ms freeze of the command stream by the
  sim's loopback UDP path, landing on a stance exchange — not a property
  of the robot. What survives from this entry is the measurement method
  and the estimator-vs-truth numbers; the characterisation question is
  answered by OPEN-28's closure. Classifying
  every `[FALL]` in the archives by attitude: **68% are "level collapse"**
  (|roll| < 10°, |pitch| < 15°, body height 0.035–0.09 m — belly on the
  deck, body level), 24% are tip-overs, 8% mixed. Not new and not a
  regression: 67% on 08-30 (n=98), 56% on 08-31 (n=36), 56% on 09-02
  (n=16), 68% today (n=249).
  **Why it matters for the record**: CLAUDE.md's standing fall analysis
  ("THE ATOM'S FAILURE IS PITCH, AND THE a_lon DEFAULT IS BACKWARDS",
  pitch 30.5–36.9° with roll in the teens) studied the **minority mode**.
  Today's three `atom` failures are level collapses (roll/pitch ≤ 4°,
  z = 0.037–0.053) — a different mode from the one that entry explains, so
  its `a_lon` reasoning does not transfer to them.
  **Dead hypothesis, recorded so it is not re-run**: torque saturation or
  the legs going slack. A naive window said falls command a third of the
  torque of passes (median 22 vs 65 Nm) — that was a **phase confound**:
  the last seconds of a PASS include the hard braking arrival at the final
  waypoint. Phase-matched by 5-second bucket from launch, falls and passes
  are indistinguishable (LEVEL/PASS ratio 0.91–1.31 across every bucket,
  n = 845 vs 1500 samples at 15–20 s). `|imu_az|` (19 vs 21) and baro
  descent in the matched window do not separate them either. The legs are
  doing what they do in a passing run right up to the failure.
  **Also possibly not a defect at all**: level collapses occur in cells
  beyond a gait's validated envelope. A quadruped folding when pushed past
  its speed limit is plausible physics, not necessarily a bug — but which
  it is has never been established, and every capability ceiling this
  project measures is really "where the fold starts".
  **CHARACTERISED (2026-09-03, from 413 archived
  `shm_trace/*_FALL.json` snapshots — 20,000 per-tick records each, no new
  rig time).** Healthy body height is 0.28 m; the detector fires below
  0.10 m. Measuring the descent from the last tick above 0.20 m down to
  0.10 m (n=47 LEVEL, 14 TIP):

  | | duration | drop | mean rate | reported `vz` min |
  |---|---|---|---|---|
  | LEVEL COLLAPSE | 0.33 s | 0.102 m | 0.31 m/s | **−0.12 m/s** |
  | TIP-OVER | 0.44 s | 0.101 m | 0.23 m/s | −0.14 m/s |

  Two findings.
  **(1) It is a SINK, not a drop.** Free fall over that 0.10 m would take
  ~0.14 s and reach ~−1.4 m/s; the real descent is 2–3× slower. The legs
  are bearing load the whole way down — consistent with the phase-matched
  torque result above. Nothing lets go; the robot settles.
  **(2) The estimator's own position and velocity disagree by ~2.5×.**
  The trace logs `z = _stateEstimate.position[2]` (world) and
  `vz = _stateEstimate.vBody[2]` (body frame) — and in a LEVEL collapse
  the body is level by definition, so those frames nearly coincide and
  should agree. The position block descends at 0.31 m/s while the velocity
  block never reports worse than −0.12 m/s. Both are the same KF state
  vector. That is the documented velocity-covariance collapse
  (`PositionVelocityEstimator.cpp`: P's velocity block falls to ~1e-3
  within a second, gains to ~0.007) showing up as a **measured**
  inconsistency during the failure rather than as a covariance printout —
  the filter cannot see the body sinking.
  **c21 (2026-09-03): the contact-schedule lead is DEAD, and the field
  that named it is not what its name says.** ISSUES had named the contact
  *schedule* as the next instrument. The fall archive only ever dumps on
  FALL, so there was no PASS baseline to compare against; c21 ran the
  validated flat/walking@2.0 cell 10x (9 PASS) dumping the ring after
  every run. Phase-matched by 5 s bucket from launch, against 87 archived
  LEVEL collapses:

  | t from launch | PASS | LEVEL collapse |
  |---|---|---|
  | 10-15 s | 0.37 | 0.38 |
  | 15-20 s | 1.65 | 1.64 |
  | 20-25 s | 1.01 | 1.01 |
  | 25-30 s | 1.01 | 1.01 |
  | 30-35 s | 0.97 | 1.01 |

  Identical at every phase. The gait schedule is not what differs.

  **And it CANNOT differ, which is the actually interesting part.**
  `ContactEstimator::run()` (common/include/Controllers/ContactEstimator.h)
  is one statement: it copies `contactPhase` into `contactEstimate`. There
  is no contact estimation in this controller at all. The "contact
  estimate" the Kalman filter trusts is the gait scheduler's **expected**
  contact phase - open-loop, by construction identical in a fall and a
  pass. `PositionVelocityEstimator.cpp:165` even calls it `phase` when it
  reads it, and uses it to set each foot's measurement `trust`.

  **My own misreading, recorded so the number is not quoted.** I first
  reported "only ~1.0 of 4 feet in stance, in falls AND in normal
  walking" as a surprise. It is not a finding about the robot: the four
  values sum to 1.008 +/- 0.30 per tick and ramp smoothly (0.67 -> 0.76
  while a neighbour goes 0.07 -> 0.16). It is a normalised schedule
  phase, so thresholding it at 0.5 and counting "feet in stance" measures
  nothing. The identical-at-every-bucket result above stands; the "1 foot
  down" framing does not.

  **Where that leaves the hypothesis.** Every piece of evidence now fits
  one story: the controller keeps pushing on legs it *believes* are
  planted, because nothing in the loop can tell it otherwise. Torque is
  indistinguishable from a pass (phase-matched). The schedule is
  indistinguishable from a pass (open-loop). The KF trusts each foot as a
  fixed world point on the schedule's say-so, so a sinking body
  contradicts its own measurements and it resolves that by believing the
  feet - which is exactly the observed 2.5x disagreement between its
  position and velocity blocks. **Hypothesis, not yet established.**

  **c22 RAN IT, and it refutes the hypothesis above.** (2026-09-03,
  8 runs flat/walking@3.5 -- over the validated 2.5 so it folds rather
  than tips -- shadowed by a standalone `pose_feed.py` at 200 Hz for gz
  ground truth. Rows in `gazebo/conductor/data/c22_open26_truth_vs_estimator.csv`.)
  The two series have no common clock, so nothing is aligned by time:
  each is measured against ITS OWN healthy plateau, which also cancels
  the constant 0.02 m frame offset between gz's model origin (0.307) and
  the estimator's `position[2]` (0.288).

  **The control validates the instrument.** The three PASS runs end in a
  deliberate lie-down, and there the estimator tracks ground truth
  exactly -- descent rate 0.056-0.060 vs 0.057-0.059 m/s, ratio 0.98 /
  1.00 / 1.02, lead +0.00 s. Whatever goes wrong in a collapse is not
  the measurement.

  **In a collapse the estimator's height runs AHEAD of the body:**

  | | estimator z | gz truth | ratio |
  |---|---|---|---|
  | deliberate lie-down (n=3) | 0.057 m/s | 0.058 m/s | 1.00 |
  | collapse (n=5) | **0.174 m/s** | **0.098 m/s** | **1.79** |

  Measured as time, the estimator completes the same 0.12 m of descent
  **0.98 s before the body actually does** (per run: 0.49, 1.32, 1.21,
  1.39, 0.52 s; the control runs give 0.00).

  **And the velocity block reports ~ZERO through all of it.** `vz` at the
  bottom of each collapse: 0.00, -0.00, -0.01, -0.00, -0.00 m/s -- while
  the same state vector's position block is moving at 0.174 m/s and the
  real body at 0.098. In the control runs `vz` is small but nonzero and
  consistent with the slow descent (+0.06, -0.06, +0.07). So the filter
  is not merely imprecise during a fold; **its position and velocity
  blocks assert incompatible things**, and neither is right.

  **So the hypothesis I wrote above -- "the filter cannot see the body
  sinking" -- is wrong as stated.** The velocity block indeed cannot see
  it (0.00 against a real 0.098 m/s). But the position block over-reacts,
  registering a collapse ~1.8x faster and a full second earlier than it
  happens. "Blind" was the wrong word in one direction and the right one
  in the other.

  **Mechanism, offered as hypothesis and NOT established.** The KF
  corrects body position from foot kinematics, trusting each foot as a
  fixed world point on the *schedule's* say-so (see c21 above: there is
  no contact estimation). If the legs buckle while the feet stay planted,
  the leg's kinematic height shrinks and the filter reads that as the
  body having dropped -- faster than the body really moved. The same
  planted-foot assumption pins the velocity measurement at zero. One
  assumption, both symptoms.

  **CORRECTION (same evening, before this went anywhere).** I first wrote
  here that "the fall detector fires on the estimator's `z`", and told the
  operator the rig had been declaring falls a second early. **That is
  wrong, and it was wrong when I wrote it.** `RobotRunner.cpp:429` prefers
  `kinZ` -- a height built from leg forward-kinematics and the IMU
  orientation (`-min over legs of (rBody^T (hip + p))[2]`) -- and falls
  back to the estimator's `position[2]` only when FK returns nothing. The
  code even carries the comment "it cannot drift". The trace's `z` field,
  which is what c22/c23 measured, is `rs.position[2]`: the estimator, not
  the detector's input. I read a field name in the trace and assumed the
  detector consumed it.

  **What survives the correction:** the measurement itself. The
  estimator's `position[2]` does lead ground truth by 0.86 s in a level
  collapse (p = 3.1e-5 against controls) and its `vz` does read ~0 through
  the same window. Those are properties of the state estimate, and the
  state estimate is what the CONTROLLER balances on, whatever the fall
  detector reads.

  **What the correction opens, and it is a sharper question.** `kinZ` is
  leg kinematics plus orientation -- exactly the quantity the
  foot-buckling hypothesis says goes wrong first. It is not in the trace,
  so nothing here can say whether it leads, lags, or tracks. Adding it is
  a `Record` layout change (98 bytes, and the Python reaper's struct
  format must change in the same commit -- the `static_assert` says so)
  plus a rebuild with the DOCUMENTED configure, not Release.

  **c23 CONFIRMED it on a second cell, and shrank the effect.**
  (2026-09-03, 20 runs, arms alternated EVERY run: flat/walking@3.5
  against rough/walking@2.5. Rows in
  `gazebo/conductor/data/c23_open26_truth_vs_estimator.csv`; pooled
  analysis is `gazebo/tools/open26_pool.py`.) Pooled with c22 --
  **11 collapses, 17 controls, two terrains**:

  | | n | estimator | gz truth | ratio | lead |
  |---|---|---|---|---|---|
  | CONTROL, deliberate lie-down | 17 | 0.056 | 0.052 | 1.07 | **+0.03 s** (sd 0.18) |
  | collapse, all | 11 | 0.188 | 0.119 | 1.58 | **+0.79 s** (sd 0.40) |
  | -- LEVEL collapses | 8 | 0.196 | 0.112 | **1.74** | **+0.86 s** |
  | -- TIP / MIXED | 3 | 0.169 | 0.137 | 1.23 | +0.62 s |

  Mann-Whitney on the lead, control vs collapse: **p = 3.1e-5**; every one
  of the 11 collapses leads by more than 16 of the 17 controls do.

  **The effect is strongest in exactly the mode this issue is about.**
  LEVEL collapses -- 66% of the 425-fall archive -- give ratio 1.74;
  tip-overs give 1.23. That is the direction the foot-kinematics
  hypothesis predicts (a tip is not legs buckling under planted feet),
  and at n=3 tips it is suggestive, not established.

  **Honest revision of the size.** c22 alone read ratio 1.79 / lead
  0.98 s; c23 alone reads 1.46 / 0.63 s. Pooled: 1.58 / 0.79 s. The first
  look was the high end, as small first samples on this rig usually are --
  the DIRECTION replicated on a second terrain, the magnitude came down.
  Quote the pooled numbers, not c22's.

  **One caveat the control itself surfaces.** On rough, the PASS controls
  read ratio 1.14 rather than flat's 1.02 -- a systematic ~12% artefact of
  measuring against a plateau when the ground is not flat. It is far
  smaller than the 1.74 it would have to explain, but it is there, and any
  future rough-terrain number from this method should carry it.

  **Caveats, stated plainly.** 11 collapses over two terrains, but ONE
  GAIT (walking) and two speeds, both deliberately over their measured
  caps. 8 of 11 classify LEVEL by end attitude, matching the archive's
  dominant mode -- but it is still not established that an over-speed
  fold is the same phenomenon as the archive's 279 level collapses,
  most of which happened at speeds inside their envelope. Trotting is
  unmeasured here. The estimator-vs-truth method itself carries the
  rough-terrain artefact noted above.

  **c27 (2026-09-03) answered BOTH open questions, and the second answer
  retires a worry I raised on this page.** 16 runs, walking alternated
  with trotting every run at flat@3.5, gz ground truth alongside, and
  `kin_z` now in the trace. Rows:
  `gazebo/conductor/data/c27_open26_kinz.csv`; tools:
  `gazebo/tools/open26_three_way.py`, `open26_firetime.py`.

  **(1) It is gait-independent.** All 5 collapses were trotting (walking
  went 8/8 that hour). Trotting: estimator **1.86x** truth, lead
  **+0.88 s** -- against walking's 1.74x / +0.86 s from c22/c23. The same
  effect, in a gait with a completely different contact schedule.
  Controls: 11 passing runs, lead +0.00 to +0.03 s, all three signals
  agreeing within 1 mm.

  **(2) The DETECTOR's height leads too -- its comment is wrong.**
  `kin_z` descends at **1.79x** truth and leads by **+0.94 s**, slightly
  MORE than the estimator it was introduced to be safer than.
  `RobotRunner.cpp` says of it "it cannot drift"; measured against gz,
  during a fold, it does.

  **(3) AND THE DETECTOR IS STILL NOT FIRING EARLY. My worry was wrong.**
  That +0.94 s is measured over the band plateau-0.03 to plateau-0.15
  (0.288 -> 0.138 m). **A lead over one band does not transfer to a
  threshold at a different height**, and here it does not survive the
  trip: at the detector's actual `SIM_FALL_Z` of 0.10 m, `kin_z` reaches
  the threshold only **0.05 s** before the body truly does (per run:
  0.01, 0.02, 0.01, 0.21, 0.01 s). Both signals have converged by the
  bottom. Add the 0.5 s `SIM_FALL_HOLD_S` and the `[FALL]` declaration
  lands **0.45 s AFTER** the body has actually arrived. The detector is
  slightly LATE, not early.

  **So: the ceilings in this record are not detector artefacts.** I
  raised that possibility twice on this page -- first on a wrong premise
  (that the detector reads the estimator's z; it does not), then on a
  right premise with a number that does not apply at the threshold. It is
  now closed with a measurement of the exact quantity at the exact
  threshold. Every capability ceiling this project has measured counts
  robots that were genuinely on the deck.

  **What still stands, and it is about control, not detection.** The
  state estimate the CONTROLLER balances on is wrong by ~1.8x through the
  middle of a fold, in both gaits, while its velocity block reports ~0.
  Nothing above softens that; it just moves the consequence from "we
  mis-count falls" to "the controller is fed a bad picture precisely when
  it most needs a good one."

  **The foot-kinematics mechanism is NOT supported by its own test
  (2026-09-04).** `gazebo/tools/open26_mechanism.py` asks the falsifiable
  question the hypothesis implies: during a collapse, does the estimator's
  height track `kin_z` (pure leg geometry + orientation) more closely than
  it tracks the real body? Over c27's 5 collapses:

  | | RMS distance |
  |---|---|
  | estimator z <-> leg-kinematic height | 0.0256 m |
  | estimator z <-> gz ground truth | 0.0280 m |

  **1.1x. Essentially no discrimination**, and 2 of the 5 runs go the
  other way (0.9x, 0.7x). Note the test was rigged IN the hypothesis's
  favour: `z` and `kin_z` come from the same record, on the same clock, so
  that distance is exact, while the truth comparison carries alignment
  error from two independent clocks. Even with that advantage the
  hypothesis does not separate.

  **What that kills, and what it opens.** It kills "the estimator is
  effectively reading the leg geometry" -- the two signals diverge by
  ~2.6 cm RMS during a fold, having agreed to ~1 mm in normal walking.
  So `z` and `kin_z` both run ~1.8x fast against truth while NOT tracking
  each other. Two signals erring in the same direction independently
  suggests a COMMON INPUT, and they share exactly one: the estimated
  ORIENTATION. `kin_z` is `-min over legs of (rBody^T (hip + p))[2]`;
  the position block integrates in the world frame the same attitude
  defines. A pitched-or-rolled attitude estimate makes every derived
  height shrink, in both signals, without either tracking the other.
  **Untested**, because `pose_feed.py` emits only `[x, y, z, yaw]` -- there
  has never been ground truth for roll and pitch on this rig.
  n=5; the overnight queue is building the sample to re-run this on.

  **THE ORIENTATION MECHANISM IS ALSO DEAD (2026-09-04, n=14).** With
  roll/pitch truth finally available: mean |pitch error| **0.02 deg**,
  |roll error| 1.45 deg, and tilt accounts for **20%** of the height gap --
  driven entirely by two outliers, while several runs show 0.00 deg of
  attitude error alongside a 4-6 cm height gap. Attitude is not it.

  **What was actually happening: the divergence is a MID-DESCENT
  TRANSIENT, and I had been sampling the wrong instant.** Two analyses
  disagreed -- the rate work said kin_z runs 1.8x fast, the ground work
  said kin_z tracks truth to 7 mm -- so I printed one collapse's series
  aligned, with no cleverness:

  | dt | est_z | kin_z | truth |
  |---|---|---|---|
  | 0.00 s | 0.258 | 0.260 | 0.260 |
  | 0.40 s | 0.054 | 0.081 | **0.139** |
  | 0.80 s | **-0.008** | 0.038 | 0.037 |

  The whole discrepancy lives in a ~0.3 s window in the MIDDLE of the
  descent and has closed again by the deck. The ground analysis was not
  wrong, it sampled the bottom -- after convergence. (Its first version
  was separately broken: it aligned truth to the estimator by matching
  DEPTH, which is circular. Fixed to align on descent onset, the
  alignment a 17-run control had already validated at lead 0.00 s.)

  **Measured across 14 collapses** (`gazebo/tools/open26_divergence.py`):

  | | peak below truth |
  |---|---|
  | estimator `z` | 0.058 m |
  | detector `kin_z` | **0.154 m** |

  and the estimator's height goes **below zero** -- a belly underground --
  in **4/14 (29%)** of collapses. That is not a bias; it is the filter
  losing the state.

  **The detector is SOUND, now measured at the point that decides it**
  (`gazebo/tools/open26_firecheck.py`, n=15). `kin_z`'s 0.154 m error
  happens at heights well above the threshold and has closed by the time
  it matters: at the tick `kin_z` crosses 0.10, the true body height is
  **0.096 m**; 0.5 s later when the hold expires it is **0.054 m**. Runs
  where the robot was still standing (>0.20 m) at declaration: **0/15**.
  The detector question is closed for the third and last time, and this
  time at the value it triggers on.

  **A PHYSICAL finding, not an estimator one.** `truth_z - kin_z` IS the
  lowest foot's height above ground (kin_z is body-above-lowest-foot;
  truth_z is body-above-ground). Its 0.154 m peak therefore says that
  mid-fold **all four feet are ~15 cm off the deck** while the body is
  still at ~0.20 m. The robot is not collapsing onto its legs -- it is
  pulling every foot off the ground and coming down with them tucked.
  That reframes what "the fold" is, and it is the first mechanism
  candidate here that is about the ROBOT rather than about the filter.

  **THE OPERATOR WAS RIGHT: contact IS inferable without a sensor, and it
  beats the schedule where it counts** (2026-09-04). Operator: *"I don't
  understand how with an IMU and your own known gait patterns how you
  can't infer that a foot landed from it's position, and the force of the
  body on the joints... It is seemingly obvious when a leg has made
  contact."* Measured, against gz foot-contact sensors used purely as
  labels (his EDU dog has none, so nothing here may become a control
  input), 8 runs, ~100 Hz samples, all four legs:

  | predictor | accuracy | FALSE STANCE | false swing |
  |---|---|---|---|
  | gait schedule (`contactPhase > 0`) | 75.1% | **12.8%** | 12.0% |
  | world foot speed < 0.15 m/s | 75.1% | **6.0%** | 18.9% |
  | schedule AND slow (0.15) | 74.8% | **5.8%** | 19.5% |
  | schedule AND slow (0.10) | 74.4% | **3.9%** | 21.7% |

  **False stance is the error that matters** and the others are not
  interchangeable with it: when the filter believes a foot is planted it
  pins that foot as a fixed world point and corrects the body against it.
  If the foot is really in the air, the filter is being lied to. A false
  SWING only withholds information - the filter is left more conservative,
  which is the safe direction. So "same accuracy, 2.2x fewer false
  stances" is a straight win, and 3.3x fewer for 0.7 points of accuracy
  is a good trade.

  The signal is `v_foot_world = R^T (vBody + omega x r_foot + v_rel)` -
  IMU plus joint encoders, exactly what the EDU dog has. No force, no
  contact sensor: `SpiData` carries no torque field, `tauEstimate` is
  assembled from commands rather than measured, and the EDU dog has no
  foot sensors regardless.

  **Three measurement bugs had to be cleared first, each of which made
  every predictor look like a coin flip** - recorded because each is the
  kind that produces a confident wrong answer:
  1. `contactPhase > 0.5` as the stance test. It is progress THROUGH
     stance, not a flag; `PositionVelocityEstimator.cpp:165` reads it as
     `phase` and ramps trust over [0,0.2] and [0.8,1]. The right test is
     `> 0`. Scoring `> 0.5` measured my predicate, not the schedule.
  2. Foot velocity taken relative to the BODY. At a 3 m/s cruise a planted
     foot sweeps backwards through the body frame at 3 m/s; the body's own
     motion has to be added back. Relative speed separated stance from
     swing 1.4x; world speed separates it usefully.
  3. Clock alignment. `contact_feed` starts ~7 s after the controller, so
     anchoring both series at their first sample compared signals seven
     seconds apart - schedule accuracy read 52%. Now aligned by
     cross-correlating BODY HEIGHT (trace `z` against pose truth `z`):
     the same physical quantity in both, and neither is a contact
     predictor, so the offset cannot tilt the comparison toward either.

  **THE CONTACT GATE IS HARMFUL. Measured, and it stays dead.**
  (2026-09-04, interleaved, same binary both arms, only
  `SIM_CONTACT_GATE` differing, marginal cell trotting flat @3.5, N=20
  each.) Rows in `gazebo/conductor/data/gate_ab_contact_gate.csv`:

  | arm | PASS |
  |---|---|
  | gate OFF | **8/20** |
  | gate ON (0.15 m/s) | **1/20** |

  **Fisher two-sided p = 0.020.** Not a null - a significant harm, 40% to
  5%.

  **Why the offline win did not transfer, which is the lesson.** The
  offline scoring optimised FALSE STANCE and treated false swing as the
  cheap error: 12.8% -> 5.8% false stance at equal label accuracy looked
  like a free improvement. Closed-loop it is not free. Zeroing `trust`
  does not merely withhold a bad measurement, it discards a whole foot's
  contribution to the velocity update, and the gate does that on ~20% of
  samples (false swing rose 12.0% -> 19.5%). A filter this dependent on
  foot measurements for velocity is hurt more by losing good ones than by
  occasionally believing a bad one. **A label-accuracy improvement is not
  a control improvement, and only the A/B could tell the difference.**

  What survives: the finding that contact IS inferable from the IMU and
  the encoders alone stands (that measurement was against ground-truth
  labels and is unaffected). What is dead is using it as a hard veto on
  KF foot trust. A soft derate, or using it to REWEIGHT rather than
  discard, is untested - but it is a new hypothesis, not a rescue of this
  one, and it needs its own interleaved A/B before anyone believes it.

  **"The robot lets go, then falls" is DEAD - the precursor fires on
  healthy runs too.** (2026-09-04, `gazebo/tools/open26_precursor.py`,
  built on the operator's suggestion to use contact inference as an
  INSTRUMENT rather than a control input - which is also where it works on
  the real EDU dog.) Every run is read twice: inferred contact (world foot
  speed < 0.15 m/s, IMU + encoders only) and the gz contact sensors as
  labels.

  | | n | fired | mean lead before descent |
  |---|---|---|---|
  | FELL, inferred | 3 | 3 | 0.71 s |
  | **PASSED, inferred** | 3 | **3** | **1.89 s** |
  | FELL, true sensor | 3 | 3 | **0.07 s** |
  | PASSED, true sensor | 1 | 1 | 1.04 s |

  Two reasons this is dead. **All four feet leaving the ground happens on
  runs that PASS, with a LONGER lead than on runs that fall** - so it
  predicts nothing. And by the true sensor the falls' contact loss leads
  the descent by 0.07 s, which is simultaneity, not causation: the feet
  come up because the body is already going down. Consequence, not cause.

  Recorded because the story was good: the 0.154 m kin_z gap really does
  mean the legs tuck, retraction really does reach 0.285 m against a
  0.288 m standing height, and none of that separates a fall from a pass.
  The control arm is the only reason this is not now written up as the
  mechanism. n=3 per arm; the queue is raising it.

  **SOLVED (2026-09-04). THE "LEVEL COLLAPSE" MODE DOES NOT EXIST. It is
  a pitch-triggered safety E-STOP, and the classifier was reading the
  corpse.**

  Every number in this issue rested on classifying each fall by the
  attitude in its LAST tick: |roll| < 10 and |pitch| < 15 at the end meant
  "level collapse - belly on the deck, body level", 66-69% of all falls,
  "not new and not a regression... never characterised". That is the pose
  AFTER the robot has finished falling.

  Instrumenting the commanded side (`track_err[]` carrying commanded
  `kin_z`, joint tracking error and feed-forward torque) showed the real
  sequence in a single fall, and the population confirms it:

  | | n | E-STOPPED | peak pitch before E-stop | pitch at END |
  |---|---|---|---|---|
  | FALLS | 20 | **19 (95%)** | **33.3 deg** | **1.5 deg** |
  | PASSES | 14 | **0 (0%)** | - | - |

  Perfect separation, and the E-stop **precedes the descent by 0.41 s**.
  `SafetyChecker::checkSafeOrientation()` trips at 0.5 rad = **28.6 deg**;
  these falls reach 33.3. `ControlFSM::safetyPreCheck()` then sets
  `ESTOP`, leg commands are zeroed - `tauFeedForward` goes to exactly
  0.000, measured - and the robot sinks under gravity with limp legs and
  settles flat at 1.5 deg, which is what the classifier saw.

  **Everything that looked mysterious follows from that, and nothing else
  was needed:**
  * torque "indistinguishable from a pass right up to the failure" - it
    was, until the E-stop zeroed it;
  * the descent is a SINK, 2-3x slower than free fall - limp legs with
    joint damping still resist;
  * contact is lost at the same instant, not before - the legs go slack;
  * the estimator runs 1.8x fast and `vz` reads ~0 - its foot
    measurements are garbage once the legs stop tracking;
  * and the four hypotheses that died (foot kinematics, orientation
    error, contact as a cause, the contact gate as a fix) were all
    explaining a phenomenon that was not there.

  **A correction to this project's own record.** CLAUDE.md's standing
  entry "THE ATOM'S FAILURE IS PITCH, AND THE a_lon DEFAULT IS BACKWARDS"
  was right. On 2026-09-03 I wrote here that its pitch reasoning "does not
  transfer" to level collapses because those show roll/pitch <= 4 deg.
  That was wrong, and wrong for exactly the reason above: I was reading
  the attitude at the end. **Pitch is the failure mode, in both.**

  **What OPEN-26 becomes.** Not "why does this port fold?" but a sharper
  and far more useful question: **is 28.6 deg the right limit?** Every
  capability ceiling this project has measured is the speed at which the
  robot first pitches past that threshold - a SAFETY PARAMETER, not a
  dynamic limit. Whether it could recover from 33 deg if not E-stopped is
  untested and is the obvious next experiment;
  `CTRL_ORIENT_HOLD_MS` and the threshold are both tunable, and
  `setOrientTripEnable()` already exists to gate it.

  **AND THE 28.6 DEG LIMIT IS APPROXIMATELY RIGHT - the ceilings in this
  record are real** (2026-09-04). Having found that every fall is a
  pitch-triggered E-stop, the obvious worry was that the ceilings are an
  artefact of a conservative safety parameter. They are not.

  Pass/fail said little: interleaved, same binary, marginal cell, N=25
  each - **28.6 deg 13/25 (52%), 45 deg 17/25 (68%)**, +16 points but
  **Fisher p = 0.387**. A power check showed why chasing that is futile:
  at those rates even N=100 per arm reaches p<0.05 only 58% of the time.
  One bit per run is too blunt.

  So the endpoint changed instead. With the limit raised, every crossing
  of 28.65 deg held past the 60 ms hold is an experiment the shipped limit
  would have killed - and a run contains several. 80 runs, **every one
  dumped, pass and fail**, so recovery is not conditioned on falling
  (`gazebo/tools/open26_excursion.py`):

  | limit | traces | runs with an excursion | excursions | RECOVERED |
  |---|---|---|---|---|
  | 28.6 (shipped) | 16 | 4 | 4 | 0 |
  | 45 | 16 | 6 | 8 | 1 |
  | 45 (2nd cell) | 16 | 6 | 6 | 0 |
  | 45 (rough) | 16 | 3 | 5 | 1 |
  | 60 | 16 | 8 | 9 | 1 |

  **Pooled over the raised arms: 3 of 28 excursions recovered - 11%.**
  **Confirmed at scale (2026-09-04, 160 traces, 96 further runs): 9 of 67
  recovered - 13.4%, 95% CI roughly 5-22%.** Peak attitude in those
  excursions averaged 43.5 deg.
  Once this robot pitches past ~29 deg it is going down 89% of the time,
  whatever the safety checker does. That is why the pass-rate difference
  was small and insignificant: the limit is not what is binding.

  **So the conclusion is the reassuring one, and it was worth proving
  rather than assuming.** The capability ceilings this project has spent
  weeks measuring are close to real dynamic limits, not artefacts of a
  conservative parameter. `CTRL_ORIENT_LIMIT_DEG` stays at its default;
  it exists now so the question can be re-asked cheaply if the controller
  ever changes. The +16 points at 45 deg is a real but unproven effect
  worth about 1 recovered excursion in 9 - if someone wants it, it needs
  its own campaign, not this one's leftovers.

  **FINAL, AND IT RETRACTS MY OWN EARLIER READING: the orientation limit
  does not bind capability AT ALL.** (2026-09-04, 4-point dose-response on
  matched cells, 28.6 / 45 / 60 / 90 deg. At 90 deg the attitude E-stop is
  effectively disabled.)

  | limit | trotting flat @3.5 | walking flat @3.5 |
  |---|---|---|
  | 28.6 (shipped) | 5/8 (62%) | 7/8 (88%) |
  | 45 | 12/24 (50%) | 15/16 (94%) |
  | 60 | 6/16 (38%) | 14/16 (88%) |
  | 90 (no E-stop) | 6/8 (75%) | 8/8 (100%) |

  **No ordering. 28.6 vs 90 deg: Fisher p = 1.000 on both cells.** The
  trotting column bounces 38-75% across arms that ought to be monotonic,
  which is this rig's familiar +/-20 point block noise showing up between
  arms.

  **So the "+16 points at 45 deg" reported earlier (13/25 vs 17/25,
  p = 0.387) was noise, and I am retracting it as an effect.** A
  four-point dose-response is what settled it: two arms could not, because
  two arms cannot show you that the ordering is absent. That is the
  cheapest lesson here - when an A/B is suggestive but not significant,
  another arm is worth more than more N in the same two.

  **The conclusion is now stronger than "approximately right".** Raising
  the threshold, or removing it, does not buy capability: 87% of
  excursions past it are unrecoverable (9/67), and the pass rate does not
  move even with the check effectively off. The E-stop is nearly free - it
  aborts runs that were going to fail anyway, ~0.4 s earlier. **Every
  capability ceiling in this record is a real dynamic limit.**

  **Next, in order.**
  1. **Log `kinZ` and find out whether the DETECTOR's height leads too.**
     The estimator's does, by 0.86 s; the detector uses a different
     quantity that nothing has ever measured. If `kinZ` also leads, then
     some fraction of every ceiling in this record is a detector artefact
     rather than a robot limit, and that is the highest-value question
     here. If it tracks truth, the detector is sound and the finding stays
     confined to the estimator -- which still matters, because the
     controller balances on the estimator.
  2. **Does it hold for trotting?** Every number above is walking.
  3. **Confirm the mechanism.** The foot-kinematics story predicts the
     error grows with leg-angle change during the fold. **CORRECTION: I
     wrote that this was testable against the archived traces with no rig
     time. It is not.** `Record` carries only rpy, omegaBody, vBody, z,
     period, contact phase, op_mode -- no leg angles, no foot positions,
     nothing kinematic. 438 archived snapshots cannot answer it. The test
     requires the same `Record` change as (1), which should therefore
     carry `kinZ` AND enough leg state to check the prediction, in one
     layout change rather than two.
  Two things that would have been the next test and are now answered:
  which block is wrong (both, in opposite directions -- c22/c23) and
  whether the contact schedule differs (it cannot -- c21).

  Note the constraint that makes this hard: `SIM_KF_VFLOOR`, the flag
  designed for exactly this, was measured **harmful** at N=30 (CLOSED, was
  OPEN-17) at two independent sane floors. So the diagnosis is now
  concrete and the obvious remedy is already disproven; whatever fixes
  this is not a covariance floor.
  Not pursued: the contact fields are `contactEstimate` (a probability,
  not a schedule), so "how many feet were down" cannot be read from them
  with a threshold — that question needs the schedule itself.

  **Superseded** — 1 Hz bridge summaries cannot resolve this. The
  ShmTrace ring buffer holds per-tick state and `shm_reaper.py` already
  dumps a snapshot on a confirmed crash; the question is what the contact
  schedule, per-leg force and commanded body height do in the 200 ms
  before z crosses 0.10 — one archived snapshot per class would answer it
  without new rig time.



### CLOSED · The fall detector was blind for 8.7 s around every commanded crouch

**Operator, 2026-09-03:** "make it aware of the mode, and IF the dog has
been told to lay down, STOP detecting for a 'fall' like you normally do.
IF you are gonna keep detecting in the moment, use new logic, obviously."

**Half of it already existed.** `setFallZEnable(false)` has wrapped every
commanded crouch since 2026-08-24, for the reason the operator gave: the
z test cannot tell a commanded lie-down from a collapse, both being a
level body descending through 0.10 m.

**The other half did not, and it cost 8.7 s of blindness.** The
suspension ran from the PASSIVE hop all the way past `K_LOCOMOTION` --
20+2500+1200+20+2500+1500+1000 ms -- and a LEVEL fold inside that window
is invisible. Level folds are 66% of this port's falls (OPEN-26).

**The obvious replacement is DISPROVEN, and the measurement is why.** A
descent-rate test looks right and fails: over 26 commanded lie-downs and
11 collapses (c21/c22/c23 traces), peak descent rate over a 0.2 s window
is **0.625-0.748 m/s for the commanded lie-down** against **0.297-1.074
for a collapse** -- the commanded crouch is FASTER than the weakest
collapse. Sweeping the window from 0.2 s to 2.0 s, no width separates the
populations. The crouch is slow on average with a fast segment. My first
instinct was a 0.12 m/s threshold, which would have fired on every
lie-down in the archive.

**Fix: use what the rate signal does not have -- the mission knows what
it commanded.** After 2.5 s of `STAND_UP` and 1.5 s of `BALANCE_STAND`
the robot is standing (~0.28) or it is not (~0.08). That gap is wide
enough that the 1.74x estimator error measured in OPEN-26 cannot close
it, which is worth stating explicitly because the same estimate is
untrustworthy at a 0.10 threshold and perfectly adequate at this one.
So `mit_sim_main.cpp` now logs a `[stand-check]` verdict there and
**re-arms the z test at that point instead of after `LOCOMOTION`**,
returning ~2.5 s of the window. `$SIM_STAND_OK_Z` (default 0.15) tunes it.

**Evidence.** Both branches exercised live: `[stand-check] back up:
z=0.286 m` on a healthy run, and the failure branch forced with
`SIM_STAND_OK_Z=0.5`. c24: 8/10 over the interlude cell, checkpoint
present on all 8 runs that reached it; both failures fell BEFORE the
interlude (collapse 28.7 s, tip-over 56.3 s) so the change was not
implicated. c25, **interleaved, binary swapped every run**, N=8 each:
**NEW 8/8, OLD 6/8, Fisher p = 0.467** --
`gazebo/conductor/data/c25_standcheck_ab.csv`.

**Explicitly NOT a claimed improvement.** 8/8 against 6/8 is the kind of
number this rig produces by chance, and the change *cannot* raise a pass
rate: it adds a log line and makes the detector fire EARLIER, which can
only turn passes into failures, never the reverse. What c25 establishes
is that it does not COST anything, and the `stand-check lines` column
(1 on every NEW run, 0 on every OLD) doubles as machine proof that the
A/B actually swapped the binary it claimed to.


### CLOSED · A finished campaign wedged in its own teardown and the rig sat idle for six hours

**Symptom.** At 17:15 the operator asked what had been solved overnight.
The last commit was 10:55. Campaign c20 had been "running" since then.

**What was actually true.** c20 finished **all 16 runs** at 11:08:49 and
wrote every row of its CSV -- the measurement was complete and correct
(it is the OPEN-23 table above). Four seconds later, at 11:08:53, its
`trap ... EXIT` fired `restart_with 10 0.1`, which restarts the
conductor. The shell never returned from that trap. `sample` on the live
pid showed the leaf frame:

    __wait4  (in libsystem_kernel.dylib) + 8

It was waiting on the conductor it had just spawned -- a process designed
to run forever. `lsof` confirmed fd 1 was `/dev/null`, which is the
trap's own `>/dev/null`, so the wedge is located exactly.

**Why it cost six hours and not four seconds.** The waiter was
`bash /tmp/c19.sh 8 | grep -E "...|C19_DONE"`. **A pipe closes when the
last writer exits**, and the wedged shell was still a writer. So `grep`
never saw EOF, the chain gated on `grep`, and `chain_c20.sh` never
printed `C20_DONE` or queued anything after it. Nothing else was
watching. Total idle: **6 h 06 m**, with a finished measurement stranded
in `/tmp`.

**What I could not establish, recorded as unknown.** I could not
reproduce the `__wait4` wedge synthetically. Two of my repro attempts
appeared to hang and I nearly wrote up a mechanism from them -- they were
contaminated by leftover children of *earlier* repro attempts, and a
third "fix confirmed" run was a false pass (`setsid` does not exist on
this Mac, so the command failed and there was simply no child to wait
for; I only caught it by also asserting the child was still alive). The
`sample` output on the real pid is the one piece of hard evidence about
the mechanism. Why bash waited there and not on the 16 identical
`restart_with` calls that preceded it is **not known**.

**Fix -- remove the dependence rather than claim the mechanism.**
`gazebo/tools/campaign_lib.sh`:

* `campaign_done <name>` writes `$CAMPAIGN_DIR/<name>.done` the
  instant the data is complete and **before any teardown runs**;
* `wait_for_campaign <name> <deadline>` polls that **file**, never a
  pipe, and **returns 1 on a deadline** instead of waiting forever;
* `gazebo/tools/rig_watchdog.sh` answers the question nobody was asking:
  every 5 min, has `run_id` advanced or is a mission in flight? After 30
  idle minutes it writes to `$LOG_DIR/rig_idle.log` *with the list of
  campaign shells still alive*, so the cause is captured at the moment it
  happens rather than reconstructed six hours later.

`unittests/test_launcher_detaches.sh` asserts the deadline actually
fires, that the marker is a file, and that nothing but `start.sh`
launches the conductor.

**The transferable rule.** A campaign's completion signal must not depend
on the campaign process exiting, and no waiter may block without a
deadline. The measurement was never in danger here -- only the signal was
-- and that is the failure mode to design against, because a wedged
signal looks exactly like a long campaign.


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
**The "rebuild" suspicion is dead with a number** (10:54): HEAD vs
the Aug 30 controller source (`48d26dc`), built with the documented
configure, interleaved every run on the same flat `corner:25:120`
walking @2.5 cell, N=6 each — **HEAD 1/6, Aug 30 0/6, Fisher p = 1.0**.
Both binaries fail that cell; the 02:59 rebuild had nothing to do with
it, and the cell is simply beyond the robot on the anchored course, as
this entry already concluded from the conductor log. Nothing further is
owed here — the operator's Time Machine snapshots are no longer needed.

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
