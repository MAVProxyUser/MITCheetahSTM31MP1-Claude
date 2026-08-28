# Session notes — 2026-08-27 evening

Running notes for the hour the operator is away. Newest work at the top of
each section. Everything claimed here is either measured or explicitly
labelled as not-yet-verified.

---

## COMPLETED (was "in flight")

### 1. Definitive regression suite against the HEAD binary — **8/8 PASS** ✅
Ran 21:18–21:30. Log: `/tmp/regress_head.log`.

    star 94.6s   atom 94.6s   oval 80.5s   dash_trotRunning 72.6s
    dash_long_duration 182.8s   circle@trotting 58.6s
    circle@bounding 76.6s   circle@galloping 90.6s

This is item #2 from the backlog audit and the reason it mattered: the
previous 8/8 suite ran 18:19–18:32, but the `x_comp_integral` **reset**
only landed at 18:24 and was not deployed until 19:59. So the suite had
never actually run against the binary that now ships. Before launching I
confirmed a clean tree (no uncommitted `.cpp/.h/.hpp`), rebuilt from HEAD,
redeployed via `deploy_host.sh`, and checked `tmutil status` was idle.

**First run with BOTH `close_leg` and the windup reset deployed together.**

### 2. Dash-slot end-of-mission defaults — **VERIFIED** ✅
Operator request: on the dash slot, default "100m dash when done" OFF and
"close final leg" OFF; every other mission keeps its dash finish.

Implemented as `server.py::kind_slot_defaults(kind)` — one source of truth,
used by the initial draft, "+ Add dog", and the mission-change branch of
`draft_set_slot`. `app.js` mirrors it on the dropdown change purely for
instant feedback and sends both values explicitly so the two cannot
disagree. Applies only on an actual kind CHANGE, so re-picking the same
mission never stomps a deliberate override.

Worth being precise about what this does and does not fix: both fields are
**already no-ops** for a 1-waypoint dash — `appendDash()` returns at
`_n < 2` and `closeFinalLeg()` skips `_n < 2`. So this is not repairing
live misbehaviour; it makes the checkboxes honest (they claimed two things
that never happened) and removes the reliance on that coupling, so a future
dash variant with 2+ waypoints cannot silently become an
out-and-back-plus-sprint.

**Verified live after the suite freed the panel:**

    (a) default fleet   star/oval/atom   dash=100  close_leg=True
    (b) switch to dash  dash:100         dash=0    close_leg=False
    (c) switch back     circle:9:8       dash=100  close_leg=True

(c) is the one that mattered - switching BACK restores the loop defaults,
so a slot that was briefly a dash does not stay crippled.

---

## BACKLOG — ground truth as of 21:18

Ordered by my read of value, not by age.

| # | item | state |
|---|---|---|
| 1 | Windup clamp OFF by default | **DONE** - defaults to 1.0 (the value every measured pass used). Suite **8/8** at 22:49 with it on. The fix now SHIPS. |
| 2 | Suite vs HEAD binary | **DONE 21:30 - 8/8**, re-confirmed 8/8 at 22:36 after the force-cap fix |
| 3 | Contention experiment | tool + model built and committed; never run |
| 4 | `corner:` mission | no RECIPES entry (that absence *was* the WP_PLANNER bug); still no clean PASS |
| 5 | Spawn pose | operator-flagged; two fixes reverted; feet still 10–17 cm under at settle |
| 6 | Oval gait-switch fall | **PARTIAL** - found+fixed a real force-cap bug (below); reach wp33 -> wp44; fast oval still falls |
| 7 | Residual ~10 % estimator scale under-read (galloping) | new tonight, real, separate from the windup, uninvestigated |
| 8 | `parallel` closed-leg | never re-measured; largest gap in the catalog (46.1 m) + a 90→49.4° closing corner |
| 9 | Chase camera position not live | flagged, deliberately not built pending a decision on per-tick cost |
| 10 | walking2 cornering at 90° / 120–147.5° | documented coverage gap |

**#1 remains the single highest-value item.** Tonight's headline — all 8
gaits completing the 100 m dash — is not in anyone's hands until the clamp
is on by default. #2 (running now) is its prerequisite: promote only after
the suite is green against exactly what ships.

---

## DONE this evening (all committed and pushed to `mp1repo`)

- **Root-caused the backward walk**: `x_comp_integral` windup in stock MIT's
  `ConvexMPCLocomotion` (2019 import, `c54e50b`). A never-reset integrator
  divided a persistent height error by *current* velocity, so as velocity
  fell the increment grew, feeding `A(11,9) = x_drag` in the MPC's own
  linearised dynamics. Commanded forward force went **+21.9 N → −110.2 N**:
  the dog was being commanded backward at ~bodyweight, not drifting.
- **All 8 gaits now complete the 100 m dash** — pronking 107.7 s, galloping
  114.3 s, bounding 84.8 s, walking 60.5 s, trotting 46.1 s, trotRunning
  149.6/148.3 s, pacing 149.8 s, walking2 235.1 s @0.5. Three of those had
  never crossed 100 m in this project's history.
- **Retired the "three different mechanisms" framing** — pronking's flat
  collapse, bounding's tip-over and galloping's silent drift were one shared
  bug presenting differently, not three mechanisms.
- **Corrected the galloping estimator finding's causal direction.** The
  29–34 m divergence was real and truth-verified, but happened while the
  robot was stationary/reversing — leg odometry faithfully integrating feet
  sweeping under a body the controller had stopped. Re-measured on the
  now-passing run: truth 99.92 m vs estimate 90.19 m, a ~10 % scale
  under-read with velocity tracked closely.
- **Confirmed the clamp is MIT's own treatment, not an invention**: `rpy_int`
  100 lines up is the identical construct (error integral ÷ velocity, same
  divide-by-zero guard) and MIT clamps it to ±0.25 *and* zeroes it. Added
  the missing reset too, in `firstRun`.
- **"Close final leg" checkbox**, default ON. Measured A/B: +6.8 % (circle
  @trotting), +7.8 % (bounding), +8.8 % (galloping), +11 % (expsquare) —
  same walk home each time, so the slower the gait the larger the fraction.
- **Timeouts derived from geometry × speed** instead of hand-picked numbers.
- **GPS_HZ → 50** (u-blox NEO-M9V, spec verified from the datasheet).
- **Host-load budget model + tool**; measured that per-tick cost is FLAT
  (2.48–2.49 ms) across every mission kind, so duration — not geometric
  complexity — is the load variable. Inverts the intuition: `dash:100 @0.6`
  costs 392 dog-seconds vs star's 129.
- **Four duplicated-source-of-truth drifts fixed at the root**: the decel
  ramps, the default oval slot (was launching the config proven broken), the
  gait dropdown (3 of 8 gaits unselectable), and the recipe notes.
- **Dog 0 deletable + "delete all"** button.
- `[RUNID]` stamped on every ctrl log line.

---

## Self-inflicted problems worth not repeating

- **`pgrep -f "script.py"` pollers match their own command line** and never
  exit. Cost ~86 min of dead wall-clock tonight: the suite finished at 18:32
  and I did not notice until 19:58 because two self-matching watchers still
  said "running". Hit it, killed them, then wrote the identical pattern
  again minutes later. Bracket the pattern (`"[s]cript.py"`), or trust the
  job's own state (`/api/state` phase, the log file) over a watcher.
  Recorded in memory.
- Restarting `server.py` needs the venv interpreter from `conductor.sh`
  (`PYBIN`), not system `python3` — and always `/api/stop` first.

---

## FOUND WHILE WAITING: an untested interaction I introduced

`closeFinalLeg()` runs **before** `appendDash()` (deliberately - so the dash
appends to a closed course). But that changes WHICH branch of `appendDash`
fires for courses that were previously open:

- `star` — already closed before tonight, so it took `appendDash`'s
  "already closes" branch (append ONE sprint point). Unchanged.
- `circle` (6.89 m gap), `sector` (15.0), `expsquare` (18.0),
  `parallel` (46.1) — were OPEN, so they took the TWO-point branch
  (explicit return to wp00, then the sprint). With `close_leg` ON they are
  now closed, so they take the ONE-point branch instead. **Changed.**

Both branches are correct by design, but the switch is a behaviour change
on four courses that has not been exercised: searching the last 40 archived
ctrl logs finds **no loop+dash run at all** on the current binary. Every
suite case and every sweep tonight ran `--dash 0`.

**Queued test (directly answers "ensure all other missions still execute
the dash when done"):** `circle:9:8` with dash=100 and close_leg ON, and
confirm the closing leg fires, the interlude fires (loop complete → stop →
lie down → stand back up → sprint), and the mission PASSES. Running as soon
as the suite frees the panel.

**Statically verified while waiting — no degeneracy.** The worry was that
with `close_leg` the last waypoint equals wp00 *exactly*, so a sprint
direction computed as `wp[last] -> wp[0]` would be a zero vector. It is
not: the already-closed branch uses the final LEG's heading
(`_wp[_n-1] - _wp[_n-2]`) and guards `l2 < 1e-3f`. For a closed circle that
is "the direction the dog was travelling as it arrived home", so the sprint
continues the shape - which is the stated intent. The live test below is
still worth running, but the interaction is safe by construction.

### 3. "All other missions still execute the dash when done" — **VERIFIED** ✅

The untested interaction above, exercised end to end. `circle:9:8` with
`close_leg` ON and a 100 m dash finish, from the raw ctrl log:

    [nav] closing the final leg: wp08 back to home (N=0.00 E=0.00), 6.89 m
    [nav] dash finish appended: course already closes at wp08,
          100.0 m sprint onward to wp09  N=92.39  E=38.27
    [nav] reached wp08 (N=0.00 E=0.00) dist=1.50      <- walked home
    [nav] reached wp09 (N=92.39 E=38.27) dist=1.47    <- ran the sprint
    [nav] MISSION COMPLETE t=108.7s  (10 waypoints)   -> PASS

Confirms all three links: the closing leg fires, `appendDash` takes the
**changed** "already closes" branch (one sprint point, not the old
two-point return), and the full interlude runs - "loop complete -
stopping, lying down before the dash finish" then "back on its feet -
dashing the final leg". Sprint length checks out exactly:
`hypot(92.39, 38.27) = 100.00 m`, aimed along the heading the dog arrived
home on.

---

## #6 OVAL — what actually happened (partial, honest)

**The oval was never broken.** `oval:40:5.0` at trotting @2.4 passes the
regression suite every single time (80.5 s). What fails is the FAST
config - trotRunning @3.5 with `WP_ANALYZER=1` mid-course gait switching,
the one worth ~30 s instead of ~46 s.

**Found and fixed a real bug on the way.** `setup_problem()`'s per-foot
force cap was passed at FOUR call sites with THREE values, and
`applySchedule()` - which re-runs on any MPC segment-timing change, i.e.
any gait or speed switch - passed a hardcoded **120 N**, mini-cheetah's
number. So every mid-course gait switch silently cut the cap 175 -> 120
N/foot: 350 -> 240 N across two feet, against the **315 N** trotRunning
needs at 3.5 m/s. Measured with `$SIM_MPCZ`: `Fz` pinned at exactly 350.0
while height fell and `vz` stayed negative. `$CTRL_F_MAX` did not help
because it only ever reached the constructor. Now one `mpcForceCap()`
accessor at all four sites. **Suite 8/8 after.**

That also invalidates this file's "trotRunning genuinely cannot hold this
curve" - it was measured on a build handing the gait 76 % of the force it
needed the instant the analyzer acted.

**Measured negatives, so nobody repeats them:**
- x_drag fully disabled (`CTRL_XDRAG_CLAMP=0`): still falls. The oval
  fall is NOT the windup.
- Effective 260 N/foot (verified reaching the solver, `Fz` hit 442.9):
  still falls.

**Net:** reach went wp33 -> wp44. It now completes the FULL analyzer
cycle (5->9 into the curve, 9->5 out) and fails in the SECOND curve. The
cap was load-bearing; at least one more cause sits behind it.

**Trap corrected mid-investigation:** `vx` in `[MPCZ]` is `vWorld[0]`, the
estimator's initial-heading axis - on an oval's RETURN leg a negative
`vx` is CORRECT, not a backward-walk symptom. I misread it once.

## A FALSE-PASS GENERATOR found and fixed on the way

`shm_reaper --tail-text` replayed a **dead** writer's SHM ring into the
new run's freshly truncated `ctrl_N.log`: ShmTrace only `shm_unlink()`s at
startup, and server.py spawns the reaper microseconds before the
controller, which needs ~1 s to boot - so the first attach lands on the
PREVIOUS run's full ring essentially every time. Its final
"MISSION COMPLETE"/"RESULT: PASS" lines then landed in the new log and
`_start_poller` believed them.

It produced one: **run430 was declared "COMPLETE t=212.8s PASS" nine
seconds after launch**, reporting run429's parallel-course time, on an
oval test. Caught only because the `[RUNID]` stamp added earlier tonight
said `run=429` in a file claiming to be run430 - the exact contamination
that stamp was added for, paying for itself the same evening. Fixed: skip
history from a writer whose pid is not alive.
