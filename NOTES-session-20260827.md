# Session notes — 2026-08-27 evening

Running notes for the hour the operator is away. Newest work at the top of
each section. Everything claimed here is either measured or explicitly
labelled as not-yet-verified.

---

## IN FLIGHT right now

### 1. Definitive regression suite against the HEAD binary — RUNNING
Launched 21:18, ~15 min for 8 cases. Log: `/tmp/regress_head.log`.

This is item #2 from the backlog audit and the reason it mattered: the
previous 8/8 suite ran 18:19–18:32, but the `x_comp_integral` **reset**
only landed at 18:24 and was not deployed until 19:59. So the suite had
never actually run against the binary that now ships. Before launching I
confirmed a clean tree (no uncommitted `.cpp/.h/.hpp`), rebuilt from HEAD,
redeployed via `deploy_host.sh`, and checked `tmutil status` was idle.

**First run with BOTH `close_leg` and the windup reset deployed together.**

### 2. Dash-slot end-of-mission defaults — CODE DONE, NOT YET TESTED
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

**Still to do:** needs a server restart to take effect (cannot restart
mid-suite), then verify (a) a dash slot shows both boxes off, (b) star/oval/
atom still show dash=100 + close_leg on, (c) a loop mission still actually
runs its dash finish end to end.

---

## BACKLOG — ground truth as of 21:18

Ordered by my read of value, not by age.

| # | item | state |
|---|---|---|
| 1 | **Windup clamp is OFF by default** (`CTRL_XDRAG_CLAMP` defaults to -1) | every tonight result needed it passed explicitly; a default panel launch still has the bug |
| 2 | Suite vs HEAD binary | **in flight now** |
| 3 | Contention experiment | tool + model built and committed; never run |
| 4 | `corner:` mission | no RECIPES entry (that absence *was* the WP_PLANNER bug); still no clean PASS |
| 5 | Spawn pose | operator-flagged; two fixes reverted; feet still 10–17 cm under at settle |
| 6 | Oval gait-switch fall | exposed by my own SIM_GAIT fix; falls ~300 ms after the 9→5 switch; never chased |
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
