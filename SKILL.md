# SKILL.md — STM32MP1 Cheetah port: the commands

Companion to `CLAUDE.md` (rules, traps, and the full running history) and
`TODO.md` (the current open backlog — check it before starting new work, and
update it when you close something).

---

# 🛑 RULE ZERO: THERE IS NO CHEATER MODE. DO NOT USE `SIM_CHEATER`. EVER.

**Not `SIM_CHEATER=1`. Not `SIM_CHEATER=0`. Not "just to bisect a bug." NEVER.**

The variable no longer does anything — the code path that fed sim ground truth
into the state estimator has been **DELETED**, not disabled. If you find
yourself typing `SIM_CHEATER`, stop: you are about to produce a number that
describes a robot that does not exist.

This rule exists because it was violated three separate ways, each time
producing confident, internally-consistent, completely invalid results:

1. `getenv("SIM_CHEATER")` is non-null for `"0"`, so `SIM_CHEATER=0` **enabled**
   cheater mode. Every "real estimator" figure in this project's history was a
   ground-truth run.
2. After that was found, documented and retracted in `CLAUDE.md`, the sweep
   harnesses (`dash_sweep.sh`, `refine_maxspeed.sh`, `pace_isolate.sh`) were
   still hardcoding `SIM_CHEATER=1` in their `COMMON=` env block — so a fresh
   100 m dash table was measured, read off and reported as a record without
   anyone opening the script that produced it.
3. Fixing the code and writing the lesson down changed nothing, because the
   thing that actually runs was never audited against the lesson.

Ground truth from Gazebo is legitimate for **MEASURING** — `dash_trace.py` reads
the sim's pose to compute distance, and instrumentation may log truth alongside
the estimate to quantify estimator error. Truth must **NEVER** enter the control
loop. Measuring with truth is science; controlling with truth is fiction.

**Before reporting ANY sweep number: read the harness's env block.** Not the
number first and the invocation only if something looks odd — nothing looked
odd, and the table was still worthless.

---

# 🛑 RULE ONE: NEVER LEAVE THE RIG IDLE. A FINITE CHAIN IS A BUG.

**Before you start waiting on anything, ask: what runs after this finishes?
If the answer is "nothing", you have already failed.**

The operator leaves this machine running for hours specifically so work
happens while they are away. A batch script that ends is not "done" - it is
a rig that has gone dark, and the time between it finishing and someone
noticing is time that produced nothing at all. This has now cost two
separate multi-hour blocks:

- a terrain sweep finished at 00:49 and the panel sat on its last run for
  **55 minutes** until the operator asked why nothing had moved;
- an overnight campaign scoped for five hours finished in **82 minutes** and
  the rig sat idle from 04:46 until the operator woke at 09:50 - and the
  conductor had died in the meantime, so the idle time was invisible.

Both times the individual work was fine. Both times the failure was the
same: **the queue ended and nothing was behind it.**

The rules that follow from that:

1. **Size the queue to the absence, then double it.** "Back in an hour" and
   "going to bed for five hours" are both statements about how much work to
   have queued. A campaign that can finish early MUST have another stage
   behind it. Over-queueing costs nothing - a resumable sweep skips measured
   cells - while under-queueing costs the whole remaining window.
2. **Never block on a monitor as your only plan.** Waiting on a background
   watcher is not work. If the thing you are waiting for might end, the next
   stage must already be chained to it in the same script, not in your head.
3. **NEVER GATE A STAGE ON `pgrep -f <script name>`.** Two separate
   40-minute stalls came from this, and the bracket trick does NOT save you:
   * `until ! pgrep -f "script.py"` matches the watcher's OWN command line,
     so it never exits. `[s]cript.py` fixes that one.
   * `[m]ain2.sh` still matches a THIRD process - the shell wrapper that
     created the script, whose argv contains the entire heredoc including
     the literal string `main2.sh`. Bracketing does nothing about that, and
     the queue deadlocked with a live conductor, an idle rig, and two
     scripts sleeping on each other.
   The fix is structural, not a better pattern: **chain stages inside ONE
   script**, sequentially, in one process. If stages genuinely must be
   separate processes, gate on an ARTEFACT the finishing stage writes (a
   DONE marker file, a CSV row count) - never on the existence of a process
   whose name appears in somebody else's argv.
   And when you DO count processes, `grep -v grep` and count with `wc -l`,
   not `grep -c`: the grep's own command line contains the pattern, so
   `grep -c` reports one more than is running. Measured in this repo's own
   status tool: 6 reported against 3 real.
5b. **A SUPERVISOR CAN ONLY REAP WHAT IT SPAWNED.** Asked how to stop
   leaving processes running and corrupting data, the honest answer is that
   the supervisor was never the problem: `self.procs` + `_watch_child` +
   `_reap_and_confirm` own every child faithfully. The camera leak that
   degraded `/api/state` from 0.0005 s to 4.7 s over 12 hours - and cost
   campaign c3 eighteen of its sixty launches - was an IN-PROCESS
   `gz.transport13.Node()`, "released" by setting a list to `[]`. Dropping
   a Python reference is not a teardown; the C++ threads never unwound.
   So the rule is not "remember to clean up", it is: **every long-lived
   resource is a CHILD PROCESS the supervisor owns, and teardown VERIFIES
   rather than hopes.** If something cannot be a subprocess, it needs a
   machine-checked invariant (see `Fleet.audit_threads()`), because a leak
   that only a human can notice will run for weeks.

5c. **A REFUSED LAUNCH IS NOT A FAILED RUN.** A Time Machine backup began
   mid-campaign, the launch gate correctly refused, `mission_runner.py`
   exited 1 - the same code as a real failure - and the harness recorded 15
   refusals as ordinary reps in 51 seconds, turning an N=20 stage into N=5
   with nothing in the log admitting it. Any harness that consumes a rep
   must first establish that a mission actually LAUNCHED. Use
   `--wait-for-gate` and check for `LAUNCH_REFUSED_EXIT` (5); on a refusal,
   retry the same rep and write no telemetry row.

6. **LIVENESS MEANS "WORK IS HAPPENING", AND ONLY ONE SIGNAL SAYS THAT.**
   Two plausible liveness checks were tried here and BOTH were wrong, in
   opposite directions:
   * *"is the producer process alive?"* - FALSE NEGATIVE. It reported
     `night2.sh IS GONE` while pid 98594 was plainly running, and the same
     check re-run by hand answered correctly. Process-name checks are
     unreliable even with `grep -v grep`.
   * *"is the stage's log fresh?"* - FALSE POSITIVE. A stage still queued
     behind another has an untouched log BY DEFINITION; it read 69 minutes
     stale while the rig was busy the whole time.
   The signal that actually means work is happening is the **conductor's run
   number advancing**. It is independent of which stage owns the queue, of
   how the stages were spawned, and of whether any particular log is being
   written. Watch that, with a generous window (a cell can take 150 s), and
   nothing else.
7. **ASSERT THE MECHANISM IS LIVE BEFORE SWEEPING ITS GAIN.** A null result
   from an inert feature is not a null result about the feature. Measured
   here: an 11-run sweep of `WP_RELIEF_K` came back uniformly FAIL and
   looked like a clean negative - the conductor was generating the DEM
   profile and passing `WP_TERRAIN_PROFILE` correctly, but the DEPLOYED
   BINARY predated the loader, so every cell was identical by construction.
   The build was clean; the deploy had been waiting for an idle gap that
   never got taken. Every parameter sweep must PROVE the parameter reaches
   the code first - look for the feature's own log line, and refuse to sweep
   if it is absent. This is the same family as RULE ZERO's "read the
   harness's env block before reporting a number": verify what actually ran,
   not what you intended to run.
8. **NEVER EDIT A RUNNING SHELL SCRIPT.** bash reads the file
   INCREMENTALLY, by byte offset - it does not slurp the whole thing at
   start. Patching a campaign script in place while it executes shifts
   every byte after the edit, and the shell resumes mid-token on its next
   read. Measured here: adding a heartbeat to a live `/tmp/go.sh` killed it
   with `line 31: | grep -E "VERDICT|METRICS" || echo ...` and the rig went
   quiet with nothing obviously wrong. Every change to a running campaign
   gets a NEW FILE.
9. **NEVER WAIT ON A MARKER WHOSE WRITER YOU MIGHT KILL.** A watcher looping
   `until grep -q DONE <log>` outlives the job when the job is killed,
   replaced, or crashes - one sat for THREE HOURS on a marker that could
   never appear because the script that would have written it had been
   deliberately replaced. Every wait needs a second exit condition: bail
   when the producer is gone (`ps ... | grep -v grep` for it), and bound the
   loop with a maximum iteration count so the worst case is a stale watcher
   for minutes, not for the rest of the session.
4. **Check elapsed time against the clock before diagnosing a stall.** Twice
   in one night a 14-second-old launch was investigated as a wedge because
   `ps` etime was misread. `date` first, then the log's own timestamp.
5. **A queue that survives a dead conductor.** Every stage must be gated on
   the server actually answering (`server_healthy()` in `corner_sweep.py` /
   `terrain_envelope.py`) and must RECYCLE rather than record. Otherwise the
   rig is not idle - it is worse, busily writing fiction.

The check that catches all of this in one line, before walking away:

```bash
# what is queued behind the thing currently running?
tail -3 /tmp/<campaign>.log && grep -c '^say\|^######' /tmp/<campaign>.sh
```

---

## The repeatable 100 m STAR MISSION (42.6 s, 13/13)

```bash
cd host-run && env DYLD_LIBRARY_PATH=. \
  SIM_GAIT=9 SIM_VX=2.0 SIM_VX_DELAY_S=4 SIM_VX_RAMP_S=8 \
  WP_MISSION=star:10.514:5 WP_ACCEPT=1.5 WP_MAX_YAWRATE=1.2 WP_PLANNER=1 \
  ./mit_ctrl_sim 127.0.0.1 stm32mp1-defaults.yaml mc-mit-ctrl-user-parameters.yaml
```

13/13, 42.5-42.7 s, ends with a judged `[mission] RESULT: PASS` (settles
upright, then lies down under damping). `SIM_VX=2.5` is 1 s faster and fails
1 run in 3 - the trade is recorded in `CLAUDE.md`, and 2.0 is the answer for
anything that has to be repeatable.

Star geometry: `star:<r>:5` gives legs of `1.9021*r` and a course of `9.5106*r`.
r=10.514 -> 5 legs of 20.0 m -> 100 m.

**NEVER `cp` into host-run/ by hand - use `gazebo/deploy_host.sh`.**
Overwriting a Mach-O in place invalidates its signature and macOS SIGKILLs it at
exec with ZERO output, which reads as the robot failing instantly on every run.

## The fastest known configuration (100 m in 32.2 s at 3.46 m/s)

`trotting` on the real state estimator, Mac SITL, confirmed 5/5 (32.2/32.2/32.3/32.4/32.7 s),
zero falls, zero safety trips, body height 0.287 m against a 0.300 m reference:

```bash
# host-run/solv_oases.yaml is the shipped user-parameters yaml with use_jcqp: 0
cd host-run && env DYLD_LIBRARY_PATH=. \
  SIM_ZEROVEL_HOLD_GAIT=1 SIM_HEADING_HOLD=1 SIM_MPC_ASYNC=0 SIM_WBC_DECIM=1 \
  SIM_MPC_HORIZON=10 SIM_SWING_H=0.11 SIM_VX_DELAY_S=4 SIM_VX_RAMP_S=12 \
  SIM_GAIT=9 SIM_VX=3.1 SIM_MPC_MS=22 \
  ./mit_ctrl_sim 127.0.0.1 stm32mp1-defaults.yaml solv_oases.yaml
```

The two settings that matter, both previously on the "ruled out" list:

- **`use_jcqp: 0`** (qpOASES). JCQP returns ~1/5 of the required ground reaction
  force under a moving gait, so the robot walks crouched at z=0.13 and sinks.
  NOT a universal win: `walking2` fails at every speed with it. And qpOASES cost
  198-218 ms on the board's A7, so the board needs `SIM_MPC_ASYNC=1` or a
  re-measured reduced solve — do not assume this transfers.
- **`SIM_MPC_MS` is SPEED-DEPENDENT.** 22 ms at 3.0 m/s; 26 ms (the default) at
  2.0-2.75. Sharp optimum in both directions — at 3.0, both 18 and 26 fail.

Speeds confirmed by repetition: 3.1 (5/5), 3.0 (3/3), 2.75 (2/2), 2.5 (3/3), 2.0 (4/4).
Above 3.1 it goes STOCHASTIC: 3.15 crosses 2 of 5, 3.2 fails. Repeat before believing.
The ceiling is ~3.5 m/s ACHIEVED; commanding above 3.1 overshoots it. See `CLAUDE.md`.

---

Board IP and Mac IP move with DHCP — set them once. See `CLAUDE.md` for why/traps.

```bash
BOARD=192.168.0.90     # STM32MP1 (re-check the lease)
MAC=192.168.0.75       # this Mac's LAN IP (Gazebo/bridge host)
```

## Controller tuning lives in `ctrl_tuning.yaml`

```bash
host-run/ctrl_tuning.yaml          # the shipped defaults, with the reasoning
stm32mp1/deploy_pkg/ctrl_tuning.yaml   # the board's copy
```

Resolution order is **environment > `ctrl_tuning.yaml` > code default**, so
every sweep harness keeps working unchanged (they all export `CTRL_*`), and
the DEFAULT is finally somewhere you can read it. The controller prints
`[ctrl_tuning] loaded N values from <path>` at boot - if that line names the
wrong file or says "not found", the robot is not running the config you
think it is. `CTRL_TUNING_YAML=<path>` points it elsewhere.

Values commented out in the file are AT their code default; an uncommented
line is a decision about the shipped robot. The file also records the levers
that were measured and REJECTED (`CTRL_BANK` null at 5/8 vs 8/11,
`CTRL_CORNER_CROUCH` harmful at 0/2) so they do not get re-derived.

## One-time setup (Mac)
```bash
brew tap messense/macos-cross-toolchains
brew install arm-unknown-linux-gnueabihf          # arm-unknown-linux-gnueabihf-{gcc,g++,...}
brew install gz-harmonic                          # Gazebo Sim 8 (for the SITL)
```

## Cross build
```bash
stm32mp1/build.sh                 # -> mp1-build/robot/{jpos_ctrl, jpos_ctrl_sim, stand_sim}
stm32mp1/build.sh clean           # wipe + reconfigure (needed after Eigen-align / ABI changes)
stm32mp1/tools/build_tools.sh     # -> stm32mp1/tools/bin/{unitree_probe, imu_probe}
```

## Deploy to the board
```bash
stm32mp1/deploy.sh push $BOARD /usr/local/cheetah-mp1     # build + stage + scp (self-contained, $ORIGIN rpath)
stm32mp1/deploy.sh                                        # stage only, no board access
# quick single-binary refresh:
arm-unknown-linux-gnueabihf-strip mp1-build/robot/stand_sim -o /tmp/stand_sim
scp /tmp/stand_sim $BOARD:/usr/local/cheetah-mp1/
```

## THE GO1'S REAL SPEED ENVELOPE (do not design against the spec sheet)

    practical max   3.5 - 3.7 m/s
    peak spec       4.7 m/s (17 km/h)

The dash table reaches 4.69 m/s with trotRunning and that number is real, but
it is the PEAK, held on flat ground in a straight line with a 12 s ramp. Do not
size a course, a cruise speed, or a lateral budget against it - a mission that
needs 4.0 m/s sustained is asking for something the robot does not have. Design
to 3.5 and treat anything above as headroom, not as a target.

## METHOD: ground truth only, measured on an idle machine

Every real finding in this port came from a measurement. Every wrong turn came
from reasoning ahead of the data and then looking for confirmation. These rules
are written from actual mistakes made here, not from principle.

### The only four sources of truth
1. **The Unitree decompile** (`docs/LEGGED_SPORT_REVERSE.md`) - the factory
   controller is the same MIT codebase, so its constants and structure are
   authoritative for a Go1.
2. **The URDF / SDK headers** - but **assume they may be wrong**. They disagree
   with each other and with the binary: three different joint-limit sets exist
   (URDF +-49.5 deg, Legged_sport's operational +-55, `go1_const.h` +-60), and
   this port had used the most permissive as a PHYSICAL stop.
3. **MIT's source** - upstream intent, including its bugs and its TODOs
   (`ContactEstimator` is an admitted pass-through, not a detector).
4. **Your own instrumentation** - logs, traces, and printed internals.

If a claim cannot be traced to one of those four, it is a hypothesis, and it
gets labelled as one.

### Rules

**Never reason ahead of the data.** State a falsifiable prediction BEFORE the
run, then let it stand or die. The cycle-time theory for the ~1 m/s wall
predicted that a 16 ms segment would clear 1.4 m/s; it failed at the same
distance as 26 ms, so the theory was dropped. That is the pattern to repeat.

**Verify the knob took effect before interpreting the result.** A change that
silently does nothing looks exactly like a change that does not help.
- the MPC contact-table patch fired (`6/10 horizon steps ... forced to stance`)
  and still did nothing - checking told us the mechanism was wrong, not absent;
- the GPS aiding "engaged" (origin printed) but its correction was inert,
  because MIT caps the x,y covariance so the Kalman gain was ~0;
- `SIM_MPC_MS` was verified as `dtMPC: 0.026 -> 0.020` before its result was
  believed.

**Measure the quantity the hypothesis is about.** Distance-walked comes from
Gazebo TRUTH, so it cannot show estimator error: a run scored a clean 83 m while
the estimate was 4.50 m wrong the whole way. Absolute position is NOT observable
from leg odometry (relative) + IMU - only GPS closes it.

**Repeat every marginal cell before claiming it.** This port has documented
run-to-run variance and it caught three claims in one session. Bounding at
1.0 m/s went "speed-responsive" (1 run) -> "broken" (2 runs) -> bimodal, ~50%
(4 runs: 5.23 / 0.04 / 0.03 / 5.52).

**A stop-at-first-success ladder is biased DOWNWARD.** It reported trotting at
0.6 m/s; 0.9 actually holds 176 m. Always re-test the speed ABOVE a reported
ceiling. And a gait whose viable speed is below your lowest rung reads as
"never crosses" - `walking` did, until 0.8 was tried.

**Re-measure documented numbers before building on them.** The recorded
real-estimator figure was 0.65 m and was quoted all day as a caveat; the actual
value was 57.6 m. Documentation written earlier is a hypothesis about the
present.

**Default new behaviour OFF until measured.** The flight-phase cost gate shipped
default-ON and cut solved MPC force from 39-42 N/foot to 6.1 N/foot.

### Your instrument is part of the system: verify it before believing a FAILURE

Four failures in one session came from the measuring apparatus, not the robot.
A wrong instrument that reports SUCCESS gets caught eventually; one that reports
FAILURE manufactures a finding that stops the investigation dead.

| what broke | how it lied |
|---|---|
| `getenv("SIM_CHEATER")` non-null for `"0"` | `SIM_CHEATER=0` still enabled cheater mode, so every "real estimator" result was a ground-truth run. Identical cheater/real numbers were read as "the estimator is fine" instead of "the flag does nothing". |
| fall detector threshold 0.15 m | The robot WALKS at ~0.175 estimated and dips to 0.149 on the stand-up transient, so the detector aborted every real-estimator run while Gazebo truth showed 0.197 and still walking. Nearly reported as "walking2 fails on the real estimator". |
| flight-phase cost gate | Shipped default-ON; cut solved MPC force 39-42 -> 6.1 N/foot. |
| debug print in the wrong branch | An early `continue` skipped it, so a firing correction reported as "never fires". |

Rules that follow:
- **Parse env VALUES, never test the pointer.** `getenv("X")` is non-null for
  `X=0`. Use `getenv("X") && atoi(getenv("X")) != 0`.
- **`SIM_CHEATER` is NEVER set to `0`. It is UNSET.** The code now parses the
  value correctly, but this rule stands regardless of any future code change:
  do not rely on parsing being correct, rely on the variable being ABSENT for
  the real estimator. Every "real estimator" claim in this port's history was
  wrong for exactly the opposite mistake - do not risk it recurring.
- **Audit the HARNESS env block, not just the source.** A retraction was
  written into CLAUDE.md saying every dash time was a cheater number - and
  `dash_sweep.sh` went on setting `SIM_CHEATER=1` in its `COMMON=` line for
  weeks afterward, producing a fresh "record" table that was read off and
  reported before anyone opened the script. Fixing the CODE and documenting the
  LESSON does nothing while the runner still exports the flag. Before believing
  any sweep result, `grep` the harness for the variables the result depends on,
  and launch with `env -u` for anything that must be absent.
- **A result is only as clean as the process that produced it.** Read the
  invocation before reporting the number, every time - not the number first and
  the invocation only when something looks odd. Nothing looked odd here; the
  table was internally consistent, reproducible to ~1%, and entirely invalid.
- **When a foundational bug turns up, every negative result measured before it
  is VOID.** `use_jcqp: 1` returned ~1/5 of the required ground reaction force
  under any moving gait, and this file's "ruled out, do not spend time on these
  again" list - cycle time, stance stiffness, Q weights, force cap, flight-phase
  gating - was compiled entirely on top of it. A lever that does nothing while
  the robot is being handed a fifth of the force it needs has not been tested;
  it has been masked. Re-run the ruled-out list after any fix to the foundation,
  and treat the old conclusions as unmeasured rather than as settled.
- **Diagnose AT the speed that fails, not at the speed that works.** Laddering
  down to whatever survives measures success and never measures the failure.
  Running straight at the 2.0 m/s target and instrumenting the collapse found in
  one run what months of tuning at 0.6-1.0 m/s had not: the robot was not
  tipping over, it was SINKING (`roll=0 pitch=-0 z=0.028`), which points at a
  force budget rather than at stability, and from there at the solver.
- **Measure the quantity the physics requires, not the one that is easy.** To
  hold height, a gait with stance duty `d` must command `m*g/d` while its feet
  are down. Printing that ratio ([MPCZ]) turned "the robot falls over" into "the
  MPC asks for 0.25x bodyweight when it needs 2.00x", which has exactly one
  plausible cause. Force in newtons alone would not have shown it - the number
  has to be divided by what the gait actually demands.
- **When a ladder rung fails, check what the robot was ACHIEVING before you
  sweep parameters at that rung.** Commanded speed and achieved speed differ
  here (3.0 commanded cruises at 3.46 m/s), so commanding 3.5 asks for ~3.9 -
  past the machine's ceiling. Roughly 20 configurations were then swept at that
  rung, every one of them doomed by the command rather than by the parameter,
  and the uniformly flat response read exactly like "none of these levers do
  anything". Walking the command up finely instead found the edge sits between
  3.15 and 3.2. A target the machine cannot reach makes every lever look inert.
- **NEVER `cp` a binary into `host-run/` by hand - use `deploy_host.sh`.**
  Overwriting a Mach-O IN PLACE invalidates its code signature; macOS caches the
  signature against the inode and the loader then **SIGKILLs the binary at exec
  with exit 137 and ZERO output**. Every mission reports 0/5 with an empty log,
  which is indistinguishable from the robot dying instantly. It cost a full
  verification sweep, a single-run bisect, and a "regression" that was diagnosed
  and REVERTED on the strength of a 0/5 that had never run. The three defences
  are all required: `rm` the target first (fresh inode), re-sign
  (`codesign -f -s -`), and PROVE it loads by running it briefly and requiring
  non-empty output before any sweep depends on it.
  **An empty log is an infrastructure failure until proven otherwise.** A robot
  that fails always prints something first.
- **After killing a sweep, WAIT FOR THE MACHINE TO GO CLEAN.** Stragglers live
  for several seconds and the next sweep's first batch inherits them - a bridge
  still holding a port, a controller still driving, topics from a dead
  partition. Measured symptom: three dogs failing identically, one with the
  estimator diverging 250 times and another stuck at N=0.00 E=0.00 for 170 s.
  That read exactly like a controller regression and cost a baseline
  investigation to disprove. `sweep_lock.sh` now refuses to start until
  `pgrep gz sim|mit_ctrl_sim|bridge` returns nothing.
- **A transient NON-FINITE is NOT a failed run.** Passing runs routinely blip
  once or twice at startup and the estimator reinitialises; 12/12 good runs at
  39.2-39.7 s included several with two NaN events each. A gate that rejects any
  NaN throws away good data - and if the blip correlates with one arm, it
  biases the A/B against that arm. Gate on whether the dog GOT GOING, not on
  whether a warning printed.
- **YOUR OWN INTERACTIVE COMMANDS ARE A SECOND SWEEP.** The lock stops scripts
  colliding with scripts; it does nothing about an ad-hoc `pkill -9 -f "gz sim"`
  typed while a sweep is running. That killed a whole A/B mid-flight - the log
  filled with `Terminated: 15` and `Killed: 9` and reported `wp=0/93` as data.
  While a sweep holds the lock, READ ONLY: cat, grep, tail. No pkill, no
  sim_up.sh, no one-off verification run.
- **`pkill -f "gz sim"` MATCHES YOUR OWN SHELL.** The pattern is a literal
  substring of the command line running it, so the killer kills itself - exit
  144 with no output, before anything is flushed, which looks like the command
  mysteriously producing nothing. Same self-match trap as `pgrep -f` waiters.
  Use a bracket to break the literal (`pkill -f 'gz[ ]sim'`) or kill by PID.
- **A harness "failed with exit code N" does NOT mean the process died.** Both
  a `nohup ... &` wrapper and a killed parent have reported failure while the
  sweep carried on running and writing results. Check with
  `ps -ax -o pid,etime,command | grep <script>` before concluding anything, and
  certainly before starting a replacement.
- **The two-sweeps rule is now ENFORCED, because discipline failed twice.**
  `source gazebo/sweep_lock.sh` at the top of every sweep script; it
  takes an exclusive lock and refuses to start if another sweep holds it. The
  second collision destroyed an atom ladder (three runs reported 0/143
  waypoints - the sim was killed under them) AND the fall-signature collection
  that killed it (runs truncated at 25 samples), and the wreckage was reported
  as data in both directions. A rule that has to be remembered every time is
  not a rule, it is a hope.
- **NEVER run two sweeps at once.** Every harness here starts with
  `pkill -9 -f "gz sim"` and then brings up its own simulator, so two sweeps
  running together repeatedly kill each other's world. The symptom does not look
  like a process collision at all - it looks like a catastrophic controller bug:
  250 consecutive "STATE ESTIMATE WENT NON-FINITE", `z=nan`, `roll=180`, and a
  robot that never stood up. A whole corner sweep and the tail of a transition
  batch were thrown away chasing that before the cause turned out to be a second
  sweep I had launched myself. Check `ps aux | grep -c "[s]weepname"` before
  launching, and treat any batch that overlapped another as CONTAMINATED rather
  than trying to salvage the rows that look plausible.
- **A waiter that polls for a completion marker must be BOUNDED.** `while ! grep
  -q DONE; do sleep; done` never terminates if the producer dies - and worse,
  `while pgrep -f "foo.sh"` matches the waiter's OWN command line, so it waits
  forever on itself. Two of these sat for seven hours. Use a fixed iteration
  count (`for i in $(seq 1 60)`) so the wait always ends.
- **Never launch a sweep as `nohup script.sh &` from a backgrounded shell.** The
  wrapper exits immediately, the harness reports "completed, exit 0", and the
  sweep is in fact still running - so the next thing you do starts a SECOND
  sweep on top of it, which is the two-sweeps-at-once failure above wearing a
  disguise. Launch the script itself in the background (no `&`, no `nohup`) and
  wait on its completion marker.
- **A detector must not depend on the quantity most likely to be wrong.** The
  fall detector reads ESTIMATED height, so it inherits estimator error. Keep a
  wide margin below the true operating value (walking ~0.175 -> trip at 0.10,
  not 0.15), and prefer attitude/kinematics over an estimated scalar.
- **When two configurations give IDENTICAL results, suspect the switch before
  concluding "no effect."** 57.64 vs 57.66 m and 4.50 vs 4.50 m were both the
  flag doing nothing.
- **After renaming an env variable, grep the HARNESSES, not just the source.**
  The SIM_ -> CTRL_ rename left seven dead variables in `dash_sweep.sh`'s
  COMMON block. They looked like a pinned configuration and were in fact
  nothing - every run since had used compiled defaults. This is the
  SIM_CHEATER trap's mirror image: there the harness silently ADDED something
  the source did not show, here it silently STOPPED adding something the
  harness did show. Check with
  `grep -rl '"SIM_FOO"' user/ robot/ common/` for each name a harness sets;
  an empty result means the line is decoration.
- **Warning time before a THRESHOLD is not warning time before the POINT OF NO
  RETURN.** The roll-out signature gives 160-1700 ms between |roll| = 0.30 and
  the 0.5 rad safety trip, which reads like plenty of time to act. It is not:
  by 0.30 rad the fall is already committed, and a guard that successfully
  arrests the roll (measured - it holds the peak within 0.03 rad and recovers
  in 0.1-0.7 s) still loses the run to pitch instead. Before building an
  actuator response, check whether the trigger fires while the outcome is still
  reversible, not merely before the alarm.
- **A course is an INSTRUMENT - pick it for the failure mode you want to see.**
  The star (polygon: sharp vertices, long recovery straights) and the atom
  (one closed stroke, continuous moderate curvature, no recovery) fail the
  robot in completely different ways at the same 2.5 m/s: height departure on
  a straight vs a roll-out under three seconds of sustained yaw. A fix
  measured on one says nothing about the other - the height governor goes
  7/7 on the star and 0/3 on the atom, correctly, because on the atom the
  height never departs and there is nothing for it to detect.
- **When entering a closed curve from inside it, join where the tangent is
  RADIAL.** Joining at a lobe tip puts a 90-degree corner at the entry of an
  otherwise corner-free course, and the planner reports the artefact as the
  course's tightest radius (0.27 m measured against a true 2.14 m) and brakes
  for it. Solve `p x v = 0, p . v > 0` and rotate that point onto the spawn
  heading.
- **Health is the TAIL, not the maximum.** `maxPeriod` is a max over ~20,000
  control ticks, so one scheduling hiccup flags a healthy run and a genuinely
  starved run looks the same. Measured: p50 never moved (2.47-2.48 ms) across a
  pass at 0.0% overruns, a pass at 1.3%, and a FAILURE at 10.9%. Report the
  fraction of intervals over threshold, and p95. Gate at 5%.
- **A missed control deadline is a physics change, not a slow computer.** The
  loop targets 2.0 ms and computes forces for that interval; if the scheduler
  wakes it at 9 ms those forces are applied 4.5x too long and the dog goes over
  for no reason visible in the result. Name it a deadline miss, measure it per
  run, and re-run anything that missed - it is an INSTRUMENT failure, not data.
- **Identical degradation across independent instances means the HOST.** Two
  dogs in separate engines, separate processes, cannot affect each other. When
  both failed in the same rep with tails identical to 0.1% (14.1% and 14.1%),
  the only shared thing is the Mac. That inference is how you tell a machine
  problem from a robot problem - and a run that samples the top non-simulation
  process while it runs can name the culprit instead of guessing.
- **Every arm gets EQUAL, VALID runs.** Not just equal counts - equal counts of
  runs that passed their acceptance criteria. A run that missed deadlines, or
  whose config never took effect, is re-run, never dropped, or the arms drift
  apart while looking matched. `partune.sh` enforces this.
- **Three dogs in parallel is free; four is not.** Verified 12/12 and 9/9 at
  identical times and zero overruns. At four, every dog hits STATE ESTIMATE
  NON-FINITE before standing and the cause is NOT isolated - not RTF, not loop
  starvation, not sensor wiring, not a startup race, not settling time.
- **Re-check the BASELINE choice, not just the treatment.** Every star result
  in this project was measured on trotting because an early table said trotting
  was the best all-rounder. It is not: trotRunning goes 32/32 across 2.5-3.3
  where trotting cannot finish 2.7 at all. Months of A/Bs were run against the
  wrong control. When a course, speed or controller changes, the gait ranking
  can invert - and it does, in both directions (trotting wins on the atom by
  the same margin trotRunning wins on the star).
- **Every arm of a comparison gets the SAME number of runs.** Sweeps here have
  mixed 5-run and 3-run arms (atomfall x5 vs atompass x3; sprawl off x4 vs
  false-fire x3). Unequal arms are not comparable, they quietly weight the
  better-sampled side, and they are how a 3/8-vs-7/7 headline survived long
  enough to be reported when the true stock rate was 8/13. Set N once per
  block and use it for every arm in that block.
- **An A/B where the treatment never FIRES is not a test of the treatment.**
  The first height-governor A/B came back "no effect" over twelve runs. It had
  no effect because the trigger threshold sat below the robot's normal
  operating band, so both arms ran an identical controller: over six armed runs
  the reference moved 6 mm and the derate never engaged once. Always log a
  "did it act" quantity (min scale, max reference, a `fired()` flag) alongside
  the outcome, and check it BEFORE interpreting the outcome. Otherwise
  "the idea does not help" and "the idea never ran" are indistinguishable.
- **A reactive controller must be faster than the event it reacts to - so
  measure the event FIRST.** The collapse here is not a slow droop: the robot
  cruises for tens of seconds then loses 8 cm in ~0.6 s. A loop built with a
  150 ms error filter and a 350 ms output slew cannot act inside that, and no
  amount of gain tuning fixes a reaction chain longer than the event. Log the
  failure at 5 Hz and read the time course before choosing a single constant.
- **De-bob before you differentiate.** Any rate taken from the body state
  contains the gait's own oscillation. Raw dh/dt at 2.5 m/s trotting swings
  +/-0.25 m/s from the step bob alone, which a 0.3 s lead term turned into a
  phantom 0.08 m "collapse" on a run that PASSED. Low-pass at roughly one gait
  period first. This costs lead time and there is no way around it - below the
  gait period the signal is not there.
- **Import the PRINCIPLE from biology, not the number.** The cheetah runs at
  0.55-0.57 x leg length; the Go1's two-segment leg cruises at 0.63. Wiring the
  animal's ratio in as a setpoint made the controller crouch the robot for
  entire runs and measured WORSE than stock. What transferred was the paper's
  actual finding - that stance height is a regulated variable - not its value
  for a different morphology.
- **Before reporting a negative result, disable your own instrumentation and
  re-run.** `SIM_FALL_EXIT=0` was what revealed the robot had been fine all along.


### CPU hygiene: measurements need an idle machine
On the board, building and running happen on different machines. On the Mac they
compete, and the SITL is sensitive to it.

- **Never compile, run r2/objdump analysis, or start a second sweep while a
  measurement is in flight.** A `make -j8` during a sweep turned a known-good
  21.23 m run into 3.54 m; the tell was the worst control-loop time going
  1.0 -> 4.5 ms.
- Check the loop time in every result. If `MAXLOOP` is far above ~2 ms, the run
  is contaminated and must be discarded, not interpreted.
- Kill stale `gz sim` with `pkill -9` between runs - a soft kill leaves a zombie
  holding the sensor systems, which silently produces a dead-sensor world.
- **Restore any harness edit immediately.** Pointing `host_sweep.sh` at an
  experimental yaml would have silently contaminated every later run.
- `pgrep -f "<pattern>"` MATCHES ITS OWN COMMAND LINE - a waiting shell reports
  the thing it is waiting for as already running. Use
  `ps -Ao pid,command | grep ... | grep -v "bash -c"`, or check for an output
  file.


## Mac-first workflow (develop here, then cross-compile for the board)

The board is ~11x slower and its eth0 flaps under load, so the math gets banged
out natively on the Mac first. Same source, same code paths - only the ISA and
the two Linux-only drivers differ.

```bash
cmake -B host-build -DSTM32MP1_HOST=ON -DSTM32MP1_MIT=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build host-build -j8 --target mit_ctrl_sim
cp host-build/user/MIT_Controller/mit_ctrl_sim host-run/
# run from host-run/ - the RPATH is $ORIGIN (a Linux-ism), so on macOS:
cd host-run && DYLD_LIBRARY_PATH=. ./mit_ctrl_sim 127.0.0.1 \
    stm32mp1-defaults.yaml mc-mit-ctrl-user-parameters.yaml
```

**Never build while a measurement is running** - on the board, compiling and
running are on different machines; here they compete. A `make -j8` during a
sweep turned a known-good 21.23 m run into 3.54 m (worst control-loop time
1.0 -> 4.5 ms is the tell).

## Measurement harnesses (Mac)

```bash
gazebo/host_sweep.sh <configfile> [secs]   # many env configs, one fresh stack each
gazebo/dash_sweep.sh                       # 100 m dash: fastest speed per gait
gazebo/refine_maxspeed.sh <results> [secs]  # bisect/push bracketed speed ceilings
gazebo/summarize_runs.py <results...>       # consolidate into the result tables
gazebo/dash_trace.py <max_s> [target_m]     # 10 Hz pose trace, times the line crossing
```

Config line format: `<label>  KEY=VAL KEY=VAL ...`; the label must carry the
speed (`walk2_v10` -> 1.0 m/s) for the scorer to bin it.

## Reverse-engineering `Legged_sport` (see docs/LEGGED_SPORT_REVERSE.md)

```bash
tools/reversing/extract_consts.py <addr>    # mov/movk + fmov float immediates in a function
tools/reversing/extract_pool.py   <addr>    # resolve adrp+ldr -> .rodata constant pool
tools/reversing/find_qweights.py            # locate the MPC cost vector in .rodata
```

## Key runtime knobs

| env | default | what |
|---|---|---|
| `SIM_GAIT` | yaml | gait id: 9 trot, 20 walking, 21 walking2, 8 pacing, 1 bound, 2 pronk, 22 gallop, 5 trotRunning |
| `SIM_VX` / `SIM_VX_RAMP_S` | - / 3 | commanded speed and ramp. **A 3 s ramp masquerades as a speed ceiling** - use 12 s to measure a gait |
| `SIM_HEADING_HOLD` | 1 | integrate the yaw reference instead of re-slaving it to the measurement (0 = stock MIT) |
| `SIM_YAW_ERR_MAX` | 0.40 | how far the heading reference may lead; competes with body height |
| `SIM_MPC_MS` | 36 | MPC segment ms. **26-30 is the working range**; 36 was a board-compute compromise that capped the walk |
| `SIM_MPC_ASYNC` | 0 | solve on a worker thread; costs nothing at 26 ms and lets a slow board use a fast segment |
| `SIM_FALL_EXIT` / `SIM_FALL_DEG` / `SIM_FALL_Z` | 1 / 50 / 0.15 | end a run on tip-over OR collapse |
| `SIM_WBC_DECIM` | 1 | run WBIC every Nth tick, caching its outputs between |
| `SIM_CHEATER` | 0 | feed sim ground truth to the estimator |
| `WP_MISSION` | - | `star:<r>:<n>` / `circle:<r>:<n>` / `outback:<m>` |
| `WP_YAW_SIGN` / `WP_TURN_FLOOR` / `WP_MAX_YAWRATE` | 1 / 0.65 / 1.2 | nav: measured turn sign, arc-vs-pivot floor, turn authority |


## Run on real hardware (board)
```bash
ssh $BOARD 'cd /usr/local/cheetah-mp1 && sudo ./board_setup.sh'   # bring up can0, list ttySTM*
ssh $BOARD 'cd /usr/local/cheetah-mp1 && sudo ./run.sh'           # jpos_ctrl (SCHED_FIFO as root)
# subsystem probes (bring-up, before the control loop):
ssh $BOARD 'cd /usr/local/cheetah-mp1 && ./imu_probe can0 -1'     # CAN IMU: gyro/accel/quat + Hz
ssh $BOARD 'cd /usr/local/cheetah-mp1 && ./unitree_probe /dev/ttySTM1 4000000 0'  # one motor (needs RS485)
```

## Gazebo Go1 SITL
```bash
# Mac: Gazebo (headless) + bridge, one command (Ctrl-C stops both):
gazebo/run_gazebo_sim.sh            # --gui to watch the Go1
# Board: the controller, pointing at the Mac:
ssh $BOARD "cd /usr/local/cheetah-mp1 && ./stand_sim $MAC"       # holds/squats a Go1 stance (clean demo)
ssh $BOARD "cd /usr/local/cheetah-mp1 && ./jpos_ctrl_sim $MAC"   # JPos sine sweep
```

### Stale bridge/controller processes: check for them at BOTH ends, always

Neither `cheetah_gazebo_bridge.py` nor `rt_gazebo.cpp` sets `SO_REUSEADDR` on
their UDP ports (9100/9101 + 10*instance), on purpose - a stale occupant is
meant to be caught, not silently shared. A bridge left running from an
earlier manual test, with no `gz sim` behind it any more, has actually done
real damage: it kept answering a fresh controller with frozen/stale sensor
data while looking, from the outside, like a genuine and reproducible
physics failure - it took hours to trace and invalidated an entire pronking
speed-ladder sweep before the stale pid on port 9100 was finally found by
hand with `lsof`.

**This is now checked automatically in two places, and both should stay in
place - do not remove either on the assumption the other one covers it:**
1. **At bridge start** - `cheetah_gazebo_bridge.py`'s `_clear_stale_port()`
   runs before its own `sock.bind()`, `lsof -ti udp:<port>`s its own CMD_PORT
   and kills whatever it finds (this process starting up is authoritative for
   that port; anything already on it is leftover, never a peer).
2. **At test/launch start** - `server.py`'s `launch()` sweeps every port pair
   for every dog slot about to launch (`9100+10*i` / `9101+10*i`) and kills
   any stale occupant, BEFORE building the fleet world or starting any
   process for the new run. This is the one place that knows the full port
   list for the fleet up front, and it also catches a stale `mit_ctrl_sim`
   squatting `SENSOR_PORT`, which the bridge-side check cannot see.

If you ever bring a dog up by hand OUTSIDE the conductor (`sim_up.sh`,
`sim_up_multi.sh`, a direct `mit_ctrl_sim` invocation), these two checks do
not run for you automatically the same way - `lsof -i :9100 -i :9101` (and
the `+10*i` ports for any instance beyond 0) before trusting a "frozen
state"/"identical failure every run" result from a manual session.
Regenerate the world after editing `make_world.py`:
```bash
cd gazebo
gz sdf -p unitree_ros/robots/go1_description/urdf/go1.urdf > /tmp/go1_raw.sdf
python3 make_world.py /tmp/go1_raw.sdf        # -> worlds/go1.sdf
```
Inspect gz topics live:
```bash
export GZ_SIM_RESOURCE_PATH=$PWD/unitree_ros/robots
gz sim -s -r worlds/go1.sdf &
gz topic -l | grep -E 'imu|air_pressure|navsat|joint_state|cmd_force'
```

## Debug a crash on the board
```bash
ssh $BOARD 'dmesg -c >/dev/null'                                  # clear first (traps only)
# capture a core (systemd Storage=none, so route it ourselves):
ssh $BOARD 'cd /usr/local/cheetah-mp1
  echo /tmp/jc-core.%p > /proc/sys/kernel/core_pattern; ulimit -c unlimited
  for i in $(seq 1 15); do timeout 4 ./jpos_ctrl.dbg ... >/dev/null 2>&1; [ $? = 139 ] && break; done
  gdb --batch -nx -ex "bt 30" ./jpos_ctrl.dbg /tmp/jc-core.*'
# push the UNSTRIPPED binary for symbols:  scp mp1-build/robot/jpos_ctrl $BOARD:/usr/local/cheetah-mp1/jpos_ctrl.dbg
```

## Board admin
```bash
ssh $BOARD 'systemctl stop ninjapilot && systemctl disable ninjapilot'   # free the cores (kills the CAN IMU stream)
ssh $BOARD 'free -m; uptime; ip -br addr'
```

## Gaits and waypoint missions (Go1 SITL)

The bridge must run in the MIT abstract joint convention for every gait below:

```bash
cd gazebo
export GZ_SIM_RESOURCE_PATH="$PWD/unitree_ros/robots:$PWD/models:/path/to/NinjaPilot/ground/gazebo_bridge/models"
gz sim -s -r worlds/go1_farm.sdf &                       # headless server (farm world, solid buildings)
BRIDGE_CONV=mit python3 cheetah_gazebo_bridge.py &       # gz python bindings: use the OpenPilot venv
gz sim -g &                                              # OPTIONAL GUI - only for watching, never for batches
```

Always `chrt -f 80` on the board, and force the route over ethernet:

```bash
ssh $BOARD "/sbin/ip route replace $MAC/32 dev eth0; cd /usr/local/cheetah-mp1 && \
  SG_VX=0.2 SG_T=1.0 SG_H=0.26 chrt -f 80 ./static_gait_sim $MAC"
```

### Statically-stable crawl — `static_gait_sim` (the reliable one)
`SG_VX` m/s · `SG_T` cycle s · `SG_H` body height m · `SG_SHIFT` lateral CoM shift m ·
`SG_LIFT` foot lift m · `SG_TURN` differential-stride turn bias

### Dynamic trot — `trot_sim` (the fast one, still being tuned)
`TR_V` m/s · `TR_T` cycle s · `TR_DUTY` stance fraction (>0.5 = double support) ·
`TR_H` height · `TR_LIFT` · `TR_KV` Raibert gain · `TR_KP_ROLL/KD_ROLL/KP_PITCH/KD_PITCH`
attitude feedback · `TR_KP_YAW/KD_YAW` heading hold · `TR_KP_J/KD_J` joint PD

### MIT convex MPC + WBC — `mit_ctrl_sim`
`SIM_MODE` final FSM mode (1 stand, 3 balance, 4 locomotion) · `SIM_STAND_S/SIM_BAL_S/SIM_LOCO_S`
stage times · `SIM_SKIP_BAL=1` go 1->4 directly · `SIM_VX` + `SIM_VX_RAMP_S` velocity ramp ·
`SIM_BODY_H` MPC body height · `SIM_MPC_MS` gait segment ms · `SIM_CHEATER=1` feed the
estimator sim ground truth · `STM32MP1_EST_DBG=1` 20 Hz `[EST]`/`[LEG]` dumps

### Waypoint missions (OpenPilot path planner)
```bash
ssh $BOARD "/sbin/ip route replace $MAC/32 dev eth0; cd /usr/local/cheetah-mp1 && \
  WP_MISSION=circle:3:8 WP_ACCEPT=0.5 SG_VX=0.25 SG_T=0.9 SG_H=0.26 \
  chrt -f 80 ./static_gait_sim $MAC"
```
`WP_MISSION=circle:<radius_m>:<points>` or `outback:<metres>` · `WP_ACCEPT` acceptance
radius m · `WP_LOOP` repeat forever. Progress prints as `[nav] reached wpNN ...`.

## Batch gait testing (do not hand-run sweeps)
```bash
cat > /tmp/cfg.txt <<'CFG'
label-a | static_gait_sim | SG_VX=0.2 SG_T=1.0
label-b | trot_sim        | TR_V=0.5 TR_DUTY=0.70
CFG
RUN_S=25 gazebo/batch_test.sh /tmp/cfg.txt
```
Prints outcome (UPRIGHT / fell at t / never stood), end pose, distance, mean speed
and yaw drift per config. It starts the world+bridge once, resets between runs, and
verifies each reset actually took effect (see CLAUDE.md for why that check exists).

## Headless video capture
```bash
python3 gazebo/record_video.py out.mp4 25 /chase_cam
```
Records the `chase_cam` sensor on the robot's trunk straight into ffmpeg. Needs no
GUI and does not spam "Saved image to:" toasts the way `/gui/screenshot` polling did.

## Run numbers - use them

Every launch gets a monotonic run number, persisted across server restarts
(`/tmp/cheetah_conductor/run_seq.txt`). It appears in the panel, on every
orchestration log line (`run47 dog2: ...`), and inside each dog's own
controller log via `$SIM_RUN_ID`. Quote it when reporting behaviour - "run
47's atom did X" is unambiguous, "the atom fell" is not.

## Harness rules learned the hard way (2026-08-25)

**A harness that can emit a false PASS is worse than no harness.** A
verification script reported the dash as PASS with numbers byte-identical
to the previous run: the launch had crashed, no new log was written, and
the reporter read the PREVIOUS run's file. Only an md5 of the logs caught
it. Every harness here now `rm -f /tmp/cheetah_conductor/ctrl_*.log`
BEFORE each run, and treats a missing log as "never launched", never as a
result.

**Do not touch gz while a measurement is running.** A `pkill -9 -f "gz sim"`
issued to probe something else killed the simulator out from under a test
that was mid-flight and turned a valid rep into INCOMPLETE. Same rule as
the two-sweeps trap, but it applies to ad-hoc diagnostics too - if a batch
is in flight, READ ONLY.

**Check the failure AXIS before picking a lever.** The atom's trips were
pitch (30-37 deg) with roll in the teens; the first fix lowered the LATERAL
budget, which governs roll, and the very next run rejected it with pitch
36.9 again. Roll <- a_lat, pitch <- a_lon. One glance at which number is
large would have saved a whole cycle.

**Detect, do not mitigate, while you are still learning the failure.** A
host-stall "safe hold" (pause, lie down, stand up) tripped on 4.8 ms of
ordinary scheduler jitter, zeroed a 3.00 m/s command in one tick and
flipped the dog - destroying both a good run and the evidence. A mitigated
run tells you nothing about what the hazard would have done.

**Use `mission_runner.py`, not a bespoke `.sh` script, to launch and verify
a run.** Per direct instruction to stop writing throwaway shell scripts
that can hang forever: `gazebo/conductor/mission_runner.py` talks
only to the conductor's REST API, has a genuine `--timeout` AND a
`--stall-timeout` (no new orchestration log line for N seconds = abort and
call `/api/stop` itself, rather than sitting in a fixed `sleep` loop until
a caller notices), and its own two harness bugs from the session that
wrote it are worth not repeating in a future one: `/api/state`'s `log`
array is a sliding 60-line window, not a growing log, so a length-diff
across polls silently skips or reprints lines - track the last line
printed and resync off its most recent position instead; and the verdict
must match the CURATED wording (`"dog%d: mission result: %s"`), not the
RAW controller log's `"[mission] RESULT: %s"` that only exists inside
`ctrl_%d.log` - the two look interchangeable and are not, and guessing at
the wrong one silently turns every PASS into a reported FAIL. A third: a
healthy run can go 60s+ between curated log lines with the control loop
perfectly fine the whole time (the atom's tightest corner, R~1.89m, has
the dog visibly hold one N/E for several seconds) - `--stall-timeout`
defaults to 100s for exactly this reason, and a TIMEOUT verdict should be
checked against the raw log before it is trusted as a real wedge.

```bash
python3 gazebo/conductor/mission_runner.py --slot "dash:100" --timeout 90
python3 gazebo/conductor/mission_runner.py \
  --slot "star:10.514:5" --slot "oval:40:5.0" --slot "atom:9.0:6" \
  --dash 100 --dash 100 --dash 100 --timeout 300
```

## The Conductor fleet panel (`gazebo/conductor/`)

```bash
cd gazebo/conductor && python3 server.py            # serves on :8420
open http://127.0.0.1:8420                                   # panel; /docs is the REST reference
```

### The full mission catalog (all verified PASS - see CLAUDE.md for tuning story)

```
star:<r>:<points>            # e.g. star:10.514:5 - 5-point star
oval:<straight_m>:<radius_m> # e.g. oval:40:5.0 - stadium
atom:<outer_r>:<lobes>       # e.g. atom:9.0:6 - epitrochoid rosette
dash:<metres>                # straight sprint, ends at the final waypoint
circle:<r>:<points>          # N-GON, named honestly now: circle:9:8 is
                              #   an OCTAGON (45deg/vertex, the discrete-
                              #   corner probe); circle:9:36 is the
                              #   functionally-smooth real circle. Both
                              #   share the "circle" recipe.
corner:<leg_m>:<angle_deg>   # e.g. corner:25:45 - ONE isolated corner
                              #   with a real approach and exit; the
                              #   per-angle cornering probe (sweep the
                              #   angle: 25:90, 25:135, ...). Verified
                              #   45/90/135 all PASS on one wide tuning.
sector:<leg_m>:<reps>        # e.g. sector:15:3 - SAR sector search
parallel:<width_m>:<height_m>:<passes>  # e.g. parallel:30:5:8 - SAR lawnmower
expsquare:<step_m>:<legs>    # e.g. expsquare:5:12 - SAR expanding square
lissajous:<A_m>:<wx>:<wy>    # e.g. lissajous:15:5:7 - integer ratio, any
                              #   ratio works but higher ones (11:9+) are
                              #   genuinely long courses - see the timeout
                              #   note in CLAUDE.md before assuming a stall
spiro:<outer_r>:<lobes>      # e.g. spiro:9.0:8 - Spirograph rosette,
                              #   makeAtom's own formula at k=lobes (not
                              #   lobes-1) and depth near 1.0 - see
                              #   CLAUDE.md for the parameter search
```

The six newest (circle/sector/parallel/expsquare/lissajous) all started
from ONE shared base tuning - `gait=walking, speed=1.5 m/s,
extra="WP_ACCEPT=1.5 WP_CORRIDOR_MIN=0.1 WP_ALON=0.4"` - already baked
into their RECIPES entries; six of these seven fell at bare default and
passed clean with it. Start any FUTURE new mission there.

That base alone leaves corners CUT, not tight - the corridor-grading
ramp (`turn_soft`/`turn_hard`) defaults to bracketing star's own two
angles (144/162deg) and does little or nothing for a gentler turn. Every
non-lissajous SAR pattern (circle 45deg, sector 120-147.5deg, parallel/
expsquare 90deg) needed its OWN `WP_TURN_SOFT`/`WP_TURN_HARD` narrowed
around ITS angle, plus `WP_CORRIDOR_MIN=0.07` (not the shared 0.1) - see
CLAUDE.md's tightening entries for the exact values and the
`planner_probe.cpp` methodology (compute the mission's own turn angle
FIRST, sweep candidate tunings against it, THEN touch the recipe) before
adding a new angular pattern. Lissajous is a smooth parametric curve, not
a fillet-through-vertices course, and does not need this treatment.

circle/sector/parallel/expsquare also call `shiftFirstToOrigin()` (a
WaypointNav.cpp/.hpp change, not a recipe knob) so the robot spawns
exactly ON waypoint 0 instead of walking there from a separate local
origin - star/oval/atom/dash/lissajous deliberately do NOT do this (star
specifically depends on the old convention; see CLAUDE.md). A brand new
angular SAR-style pattern probably wants this too; a smooth/parametric
one (like lissajous) probably does not, since there is no single
"first waypoint" any more special than another point on the curve.

### Running a 3-dog fleet with random mission types

```bash
python3 -c "
import random
catalog = ['star:10.514:5','oval:40:5.0','atom:9.0:6','spiro:9.0:8','dash:100',
           'circle:9:8','circle:9:36','corner:25:45','sector:15:3',
           'parallel:30:5:8','expsquare:5:12',
           'lissajous:15:1:2','lissajous:15:5:7','lissajous:15:11:9']
print(' '.join(f'--slot \"{m}\"' for m in random.sample(catalog, 3)))
"
# paste the three --slot args into:
python3 gazebo/conductor/mission_runner.py --slot "..." --slot "..." --slot "..." \
  --timeout 600 --stall-timeout 200
```

Omit `--gait`/`--speed`/`--extra` and the runner resolves each slot's own
recipe defaults automatically. Size `--timeout` above the SLOWEST mission
drawn — `lissajous:15:11:9` alone is a ~560s course, so a draw containing it
needs 700s+ (see `CLAUDE.md`'s Lissajous stall-timeout note).

**If all three dogs fall at the exact same wall-clock second, that is a HOST
stall, not a mission or code regression** — confirmed by grepping the raw
per-dog ctrl logs (`GET /api/logs/{i}?full=1`) for `[STALL] control period`
right around the failure timestamp; if all three show a period spike well
past their 8ms limit at the same instant, that's the shared Gazebo physics
thread or another process (Time Machine has been the culprit before — check
`tmutil status`) stealing the host, not the controller. This class of failure
is already tracked in `TODO.md` under "genuine unsolved mysteries" (4+ dog
fleets, and the trotting-dash-fails-in-parallel case) — don't re-diagnose it
as a new bug without first ruling out the host the way `CLAUDE.md`'s fleet
sections already did repeatedly.

Everything the browser can do has a REST route — `GET /docs` is a live,
clickable reference (Play + curl + response) for all of them, kept in sync
with `docs.js`'s `ENDPOINTS` array as the single source of truth. Useful ones
for debugging without the browser:

```bash
curl -s http://127.0.0.1:8420/api/state | python3 -m json.tool   # full snapshot: phase, live per-dog, curated log
curl -s "http://127.0.0.1:8420/api/logs/0?tail=200"               # raw ctrl log text for dog 0, last 200 lines
curl -s "http://127.0.0.1:8420/api/logs/0?kind=bridge&full=1"     # the bridge's log instead, in full
```

`/api/state`'s `log` array is a CURATED feed (regex-matched, one line per
init/stand/gait-change/dash/safety event) — good for watching a run live, not
for diagnosing an exact-tick failure. `/api/logs/{i}` is the raw thing
`mit_ctrl_sim` actually printed; reach for it once the curated feed has told
you roughly WHEN something went wrong and you need to see every line around
that timestamp.

**Kill stragglers the same way as any other sweep**: `pkill -9 -f
"server.py|gz sim|mit_ctrl_sim|cheetah_gazebo_bridge"` before relaunching. A
stale `server.py` serving code from before your last edit is the single most
common "why didn't my fix take effect" here — it looks exactly like a
regression.

**The browser can be stale too, and worse than the server about it.**
`Handler.end_headers()` sends `Cache-Control: no-store` on every response
(static files included) specifically because a browser was seen running
old `app.js` through a hard reload, a brand-new tab, and a fresh preview
in a row — restarting `server.py` alone does not fix a page that already
loaded the JS before your edit. If a browser-side fix looks like it did
not take effect, close the tab and open a genuinely new one before
suspecting the code.

**Any panel control that fires an async request should disable/relabel
itself SYNCHRONOUSLY on click, before the `fetch`, and restore itself on
failure** — not rely on the next `poll()` tick (every 400 ms) to notice
and disable it. Two real bugs shipped from skipping this: a slot's
remove button raced a second click against a still-in-flight DELETE and
could act on a now-stale index, and the launch button's up-to-400ms gap
let a second click fire a redundant `/api/launch`, whose refusal surfaced
as a blocking `alert()` that froze the whole page (including the poll
loop) until dismissed — which reads exactly like "needs clicked several
times to react," even though the first click had already worked. Both
are fixed in `app.js`; match this pattern for any new button.

**Test the FULL sequence, not just the piece you changed.** A fix verified in
isolation (e.g. an open-loop yaw-sweep for the cornering cap) still needs to
be re-run through the WHOLE mission end to end before it is trusted — the
cornering fix above was solid in isolation and in a dash=0 star, but the same
build's loop-to-dash interlude still fails on the full dash=30 run. Isolating
a fix is for finding it; the full sequence is for believing it.

**When a maneuver fails, check the speed it was ENTERED at before debugging
the maneuver itself.** The "fall during lie-down" was chased through three
real-but-downstream fixes (FSM transition legality, edamp coverage, ramp
shape) while the raw log's own arrival line said `v=3.50` — the dog was
reaching every stop point at full cruise because the profile only braked for
corners, never for stops. A sequence validated at one cruise speed (2.0-era
stop code) silently becomes a crash-stop when the recipes move to 3.5. The
planner now brakes for stops (`addStopXY`/end-stop, `WP_END_BRAKE=0` to
disable for A/B); if a stop maneuver regresses, FIRST confirm the arrival
speed in the `[nav]` lines is actually low.

**Restart the conductor with `conductor.sh` (or its venv python), never bare
`python3`.** `server.py` imports gz.transport13 + protobuf at module level;
system python lacks them and the server dies at import — which presents as
"the whole UI stopped working". The venv interpreter path is pinned inside
`conductor.sh`. A stale server running pre-edit code is the same trap as a
stale `gz sim` — kill it and relaunch after ANY server.py change.

## Measurement gotcha: gz pose/info is PARENT-relative below model level

`/world/<w>/pose/info` publishes MODEL poses in the world frame but LINK
poses relative to their MODEL (and visuals relative to their link). Reading
a link's z as world-z without adding the model's own z produced the false
"feet 10-17 cm under the floor at settle" record (real figure: within
±1 cm, matching Unitree's documented startup pose - source:
wiki.cci.arts.ac.uk/books/robotics-lab/page/using-go1-edu-robot-dog-by-unitree).
Always compose child poses up the chain before comparing against world
geometry - `base z = +0.0000` exactly is the tell that you are reading a
relative pose.

## Records of decisions and facts

See CLAUDE.md's "RULE: records of decisions and facts" - operator
decisions must be quoted verbatim with when/where, facts must state a
source, and even sourced facts get re-checked before being used as a
constraint. Correcting a broken implementation of X is NOT rejecting X.
