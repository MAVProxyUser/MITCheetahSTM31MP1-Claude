#!/usr/bin/env python3
"""
Regression suite over this port's ESTABLISHED, validated mission/gait/speed
combinations - the ones CLAUDE.md documents as the reliable baseline after
real measurement, not guesses. Each case below is a real Gazebo SITL run
through the conductor (gazebo/conductor/), not a synthetic/mocked
test - there is no meaningful way to unit-test this controller without the
sim, and pretending otherwise would just move the lie somewhere less visible.

WHY THIS FILE EXISTS: every one of these combinations was arrived at by
burning real wall-clock time (and, some nights, real money) discovering
which gait/speed/course pairings actually work and which look plausible but
aren't. Re-deriving that from CLAUDE.md's prose every time a change might
have regressed something is exactly the kind of work an agent should not
have to redo from scratch. Run this file after any change that touches the
estimator, the WBC/MPC gains, the gait scheduler, or the waypoint
planner/follower - a clean pass does not prove nothing broke (SITL variance
is real and documented at length in CLAUDE.md), but a case that used to be
rock-solid and now fails on a clean re-run is a real signal, not noise.

USAGE
    python3 unittests/test_validated_missions.py                 # everything
    python3 unittests/test_validated_missions.py --only star atom  # by name
    python3 unittests/test_validated_missions.py --repeats 3        # more confidence

Talks ONLY to the conductor's REST API via mission_runner.py, exactly the
same way a human operator would - it cannot corrupt sim/controller state by
construction (see mission_runner.py's own docstring for why that matters).

Requires the conductor server already running:
    python3 gazebo/conductor/server.py &
"""
import argparse
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
RUNNER = REPO_ROOT / "stm32mp1" / "gazebo" / "conductor" / "mission_runner.py"

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_history  # noqa: E402 - ring-buffer archive of every run, see run_history.py


@dataclass
class Case:
    name: str
    slots: list          # mission specs, one per dog (1-3 dogs)
    gaits: list
    speeds: list
    dashes: list = field(default_factory=list)
    extras: list = field(default_factory=list)
    min_s: float = 1.0   # a real completion below this is suspicious (too fast = didn't run)
    max_s: float = 400.0 # generous outer bound; mission_runner's own baseline logic does the real timing
    why: str = ""
    # "fast" = the quick regression gate (~15 min, the tier every code
    # change has been validated against all along). "full" = the long-
    # course catalog (SAR patterns, lissajous, spiro) - recipe-driven,
    # ~35 more minutes, lissajous:15:11:9 alone is ~9.5 of them. The
    # DEFAULT invocation runs BOTH tiers (per direct instruction: "add
    # the full suite"); --fast restores the quick gate for iteration.
    tier: str = "fast"


# ---------------------------------------------------------------------------
# THE VALIDATED BASELINE. Every entry cites the CLAUDE.md section it comes
# from so a failure can be checked against the ACTUAL prior evidence rather
# than just re-argued from scratch.
# ---------------------------------------------------------------------------
CASES = [
    Case(
        name="star",
        slots=["star:10.514:5"], gaits=["trotRunning"], speeds=[3.5],
        # dashes=[100] is LOAD-BEARING, not decoration: this case's own why
        # has always described "full loop, stop, lie down, stand up, 100 m
        # dash" - but the case ran DASH-LESS, so the interlude (the only
        # consumer of the mid-path-stop machinery) had zero suite coverage.
        # Two real bugs hid behind that gap and were found by an operator
        # UI click, not the suite: the phase gate's into-standing exemption
        # adopting mid-FLIGHT (run599: contacts [0,0,0,0] at the 5->4), and
        # addStopXY resolving the loop closure to s=0 after
        # shiftFirstToOrigin made it coincide with the path start (run602:
        # arrival at vx=+2.98, crash-stop, face-plant). A case's config
        # must exercise what its why CLAIMS it exercises.
        dashes=[100],
        min_s=90, max_s=150,
        why=(
            "CLAUDE.md 'BASELINE, END OF 2026-08-24' and every full-catalog "
            "regression sweep since: star at trotRunning 3.5 m/s is THE "
            "flagship mission - full loop, stop, lie down, stand up, 100 m "
            "dash. Historically 60.3-69.8 s depending on exact build/dash "
            "config. Falling here means something load-bearing broke, not "
            "a marginal edge case - this is the single most-repeated PASS "
            "in the whole project history."
        ),
    ),
    Case(
        name="atom",
        slots=["atom:9.0:6"], gaits=["trotting"], speeds=[2.1],
        extras=["WP_ALON=0.4"],
        min_s=55, max_s=140,
        why=(
            "CLAUDE.md 'THE ATOM'S FAILURE IS PITCH, AND THE a_lon DEFAULT "
            "IS BACKWARDS': the atom's continuous curvature needs trotting "
            "(NOT a flight gait - pronking/galloping fail this course's "
            "sustained turning), 2.1 m/s (the flight-gait-vs-trotting gait "
            "table shows trotting wins here), and WP_ALON=0.4 explicitly "
            "(the a_lon auto-select picks 1.5 for slow cruises, which is "
            "backwards for a course that is ALWAYS turning - this override "
            "is not optional). Historically 58.97-124 s depending on config; "
            "widened here because this port's own atom-in-fleet fragility "
            "notes mean a slow but eventual pass is still a pass."
        ),
    ),
    Case(
        name="oval",
        slots=["oval:40:5.0"], gaits=["trotRunning"], speeds=[3.5],
        extras=["WP_ANALYZER=1 WP_VSUS=2.4 WP_GAIT_CORNER=5"],
        min_s=25, max_s=70,
        why=(
            "THE FAST OVAL (2026-08-28): trotRunning @3.5 with the "
            "analyzer's speed cap (WP_VSUS=2.4) governing the sustained "
            "R=5 curves and the gait SWITCH disabled (WP_GAIT_CORNER=5 "
            "makes the sustained-segment gait trotRunning itself). 4/4 "
            "PASS at 37.0-37.1s, 0.1s spread. History, in order: (1) the "
            "milestone-era 'analyzer oval PASS' never actually switched - "
            "the pre-fix SIM_GAIT override silently discarded the "
            "analyzer's cmpc_gait writes, so what it validated WAS "
            "cap-only trotRunning; (2) once the gait fix made switches "
            "real, EVERY switching config fell at a switch boundary - "
            "first from the phase-misaligned contact table (fixed: "
            "phase-gated adoption in ConvexMPCLocomotion), then from "
            "hot-entry dynamics no lead/cap combination survived; (3) "
            "'trotRunning cannot hold this curve' was measured UNCAPPED - "
            "capped at 2.4 it holds fine. The conservative fallback is "
            "trotting @2.4 WP_ANALYZER=0 (~46s course time, long "
            "validated). If this case ever regresses, check the phase "
            "gate and the analyzer cap before blaming the gait."
        ),
    ),
    Case(
        name="dash_trotRunning",
        slots=["dash:100"], gaits=["trotRunning"], speeds=[2.5],
        min_s=25, max_s=60,
        why=(
            "CLAUDE.md 'CORRECTION, same night: the dash failure is NOT "
            "gait-specific' - trotRunning's dash is KNOWN TO FAIL at higher "
            "commanded speeds (0.6-1.0 m/s in the real-estimator regime, and "
            "the backward-walk/estimator-divergence bug is NOT root-caused "
            "as of this writing). 2.5 m/s here is deliberately chosen from "
            "the cornering-envelope sweep, where trotRunning at 2.5-3.5 m/s "
            "on a straight-ish course was NOT the failure mode being "
            "chased (that was cornering, not straight-line dash) - if this "
            "case starts failing, check CLAUDE.md's estimator-divergence "
            "section before assuming a new bug; it may be the same one."
        ),
    ),
    Case(
        name="dash_long_duration",
        slots=["dash:100"], gaits=["trotRunning"], speeds=[0.6],
        extras=["CTRL_XDRAG_CLAMP=1.0"],
        min_s=120, max_s=260,
        why=(
            "CLAUDE.md 'SOLVED: the backward-walk is x_comp_integral windup'. "
            "THIS CASE EXISTS BECAUSE THE SUITE MISSED THAT BUG ENTIRELY, and "
            "the reason is worth keeping: dash_trotRunning above runs at 2.5 "
            "m/s and finishes in 25-60 s, but the windup needs ~35-60 s of "
            "UNINTERRUPTED cruise before it dominates - so the fast dash "
            "structurally cannot reach the failing regime, and neither can "
            "star/oval/atom (their corners drop speed through the |vx|>0.3 "
            "accumulation gate every few seconds). A bug that only appears "
            "after a sustained duration needs a case that actually SPENDS "
            "that duration; covering a mission shape is not the same as "
            "covering its failure modes. 0.6 m/s over 100 m is ~150 s of "
            "continuous cruise, which is what reproduces it. Archive tally "
            "before the fix: 20 m dashes 4/4 completed, 100 m dashes 2/20. "
            "After (CTRL_XDRAG_CLAMP=1.0): PASS t=149.6 s with Gazebo truth "
            "confirming 99.82 m of 100 m genuinely travelled. If this case "
            "fails, check whether the clamp is still being passed BEFORE "
            "hunting a new bug - without it this is expected to fail, since "
            "the clamp is deliberately opt-in (unset = stock MIT)."
        ),
    ),
    Case(
        name="octagon_recipe",
        slots=["circle:9:8"], gaits=[], speeds=[],
        extras=[],
        min_s=20, max_s=80,
        why=(
            "The octagon (circle:9:8 - 8 vertices, 45deg each; named "
            "honestly in the panel since 2026-08-28) at its OWN recipe "
            "(walking @1.5, graded corridor, WP_TURN_SOFT=0.3/HARD=0.79 "
            "bracketing its 45deg vertices). Operator-asked coverage gap: "
            "the three envelope cases below run this course deliberately "
            "OFF-recipe (trotting/bounding/galloping) to probe cornering, "
            "so the shipping recipe config itself was never suite-covered. "
            "Empty gaits/speeds/extras = the recipe drives, exactly like a "
            "panel launch - which also regression-tests the recipe-fallback "
            "path in mission_runner/launch()."
        ),
    ),
    Case(
        name="circle_smooth_36gon",
        slots=["circle:9:36"], gaits=[], speeds=[],
        extras=[],
        min_s=20, max_s=80,
        why=(
            "The REAL circle: 36 vertices, ~10deg each - functionally "
            "smooth at this port's corridor/lookahead scale, selectable in "
            "the panel since the octagon/circle naming fix. Shares the "
            "'circle' recipe; its turn-grading is a structural no-op here "
            "(10deg sits below turn_soft=17deg), so this exercises the "
            "smooth-arc path where the octagon exercises discrete 45deg "
            "corners. Verified PASS 33.7s on its first selectable run; "
            "suite-covered per operator request ('including octagon and "
            "circle')."
        ),
    ),
    Case(
        name="corner_probe_90deg",
        slots=["corner:25:90"], gaits=[], speeds=[],
        extras=[],
        min_s=25, max_s=90,
        why=(
            "The per-angle cornering probe at 90deg, on its own recipe "
            "(walking @1.5, WIDE turn-grading window 0.3-2.0 rad - kept "
            "wide on purpose so angle sweeps measure the robot, not a "
            "per-angle tuning). corner: had NO recipe at all until "
            "2026-08-28 and was carried as broken because of it (pitch "
            "53.8deg at settle with zero tuning); with the recipe it "
            "passed 45/90/135 first try (61.3/54.9/47.8s). 90deg chosen "
            "for the suite as the angle no other single-corner case "
            "covers at recipe config."
        ),
    ),
    Case(
        name="corner_octagon_45deg",
        slots=["circle:9:8"], gaits=["trotting"], speeds=[2.5],
        min_s=15, max_s=40,
        why=(
            "CLAUDE.md 'CORNERING ENVELOPE, CONSOLIDATED' (2026-08-27): "
            "trotting cleanly passes every discrete corner angle in this "
            "catalog (45 through 162 degrees) at up to 2.5 m/s; the "
            "ceiling was bracketed to 2.5 PASS / 3.0 FAIL for angles >= "
            "120 degrees. circle:9:8 is a REGULAR OCTAGON (8 vertices, 45 "
            "degrees each by construction) - not a smooth circle, despite "
            "the mission name; see the 'Naming correction' note in "
            "CLAUDE.md before assuming this tests continuous curvature."
        ),
    ),    Case(
        name="oval_real_switch",
        slots=["oval:40:5.0"], gaits=["trotRunning"], speeds=[3.5],
        extras=["WP_GAIT_CORNER=9"],
        min_s=25, max_s=75,
        why=(
            "REAL mid-motion gait switching (5->9 into the sustained curve, "
            "9->5 out of it), the mechanism that had never passed on any "
            "build until 2026-08-28. Exists to keep all three switch "
            "transients pinned: (1) phase-gated adoption (contact tables "
            "disagree on 40% of the trot-pair cycle), (2) the analyzer "
            "settle lead (arrive at the arc at plan speed, not hot), (3) "
            "the phase-origin rebase (changing iterationsBetweenMPC "
            "mid-count teleports the segment index - the 26->22ms schedule "
            "change ~1s after adoption was the last killer). 3/3 PASS at "
            "38.6-38.7s the night it first worked. The SHIPPING oval "
            "recipe is deliberately cap-only (same time, fewer moving "
            "parts); this case is the switching machinery's own regression "
            "guard, not the recommended config. Check the [SCHED] adoption "
            "lines before believing anything about this case."
        ),
    ),
    Case(
        name="sector_recipe", tier="full",
        slots=["sector:15:3"], gaits=[], speeds=[], extras=[],
        min_s=100, max_s=280,
        why=(
            "SAR sector search at its own recipe (walking @2.0, star-tight "
            "turn-grading 0.8/2.0, WP_FINAL_ACCEPT=0.3). Validated at "
            "141.5/141.8s leg-open; close_leg (default ON) adds the 15.0m "
            "walk home - its sharpest angle is UNCHANGED by closing (32.5deg "
            "either way, the course already had one that tight). First "
            "suite-covered 2026-08-28 with the full-catalog tier."
        ),
    ),
    Case(
        name="parallel_recipe", tier="full",
        slots=["parallel:30:5:8"], gaits=[], speeds=[], extras=[],
        min_s=140, max_s=330,
        why=(
            "SAR parallel-track (lawnmower) at its own recipe (walking "
            "@1.5). The largest close-leg cost in the catalog - 46.1m gap "
            "plus a 90->49.4deg closing corner - measured PASS 212.8s "
            "closed vs 181-182s open (+17%, monotonic in gap size across "
            "circle/expsquare/parallel). Bounds sized for the CLOSED "
            "course, which is what the recipe now runs."
        ),
    ),
    Case(
        name="expsquare_recipe", tier="full",
        slots=["expsquare:5:12"], gaits=[], speeds=[], extras=[],
        min_s=80, max_s=230,
        why=(
            "SAR expanding square at its own recipe (walking @1.5). "
            "Closed-leg measured PASS 121.3s (+11% over 109s open); the "
            "closing corner is the SHARPEST on the course (90->33.7deg) "
            "and the turn-grading was never measured against it - it "
            "passes, but if this case regresses at the very END of the "
            "course, suspect that corner before the course itself."
        ),
    ),
    Case(
        name="lissajous_1_2", tier="full",
        slots=["lissajous:15:1:2"], gaits=[], speeds=[], extras=[],
        min_s=60, max_s=190,
        why=(
            "Lissajous 1:2 at recipe (walking @1.5) - the short smooth "
            "parametric course, 96.1-96.2s across every prior sweep. "
            "Closes itself by construction (ends 0.00m from home), so "
            "close_leg is a structural no-op here - a shift in its time "
            "cannot be blamed on the closing leg."
        ),
    ),
    Case(
        name="lissajous_5_7", tier="full",
        slots=["lissajous:15:5:7"], gaits=[], speeds=[], extras=[],
        min_s=250, max_s=480,
        why=(
            "Lissajous 5:7 at recipe - 345.9s baseline, twice matched "
            "exactly. Historically the course that exposed BOTH the silent "
            "'timeout 240' SIGKILL (its controller vanished mid-run with a "
            "signature indistinguishable from a crash) and the false "
            "stall-timeout class - if this case times out, read the "
            "module docstring before believing the verdict."
        ),
    ),
    Case(
        name="lissajous_11_9", tier="full",
        slots=["lissajous:15:11:9"], gaits=[], speeds=[], extras=[],
        min_s=420, max_s=760,
        why=(
            "Lissajous 11:9 at recipe - the longest course in the catalog "
            "(902m path, 561.7-562.0s baseline). THE duration stress case: "
            "~9.5 minutes of continuous low-speed cruising, which is also "
            "the deepest sustained exercise of the x_comp_integral clamp "
            "outside the dash cases. The whole reason the full tier is "
            "opt-out for iteration (--fast) instead of this being trimmed."
        ),
    ),
    Case(
        name="spiro_recipe", tier="full",
        slots=["spiro:9.0:8"], gaits=[], speeds=[], extras=[],
        min_s=80, max_s=210,
        why=(
            "Spirograph 8-lobe rosette at recipe (trotting @1.8) - "
            "makeAtom's own formula at k=lobes, depth 0.99. 118.9-119.2s "
            "across three prior runs including one 3-dog fleet. Ends "
            "0.05m from home (periodic curve), close_leg no-op."
        ),
    ),

]

FLIGHT_GAIT_CASES = [
    Case(
        name="bounding_octagon_45deg",
        slots=["circle:9:8"], gaits=["bounding"], speeds=[1.0],
        min_s=15, max_s=40,
        why=(
            "CLAUDE.md cornering-envelope table: bounding is clean at 45 "
            "and 144-162 degrees, but FAILS at 90 and 120-147.5 degrees - a "
            "real, reproducible, non-monotonic mid-band vulnerability, "
            "refuted as both a corner-density effect and a leg-sync-"
            "symmetry effect (tested directly against pacing). Do not "
            "expand this case to sector/parallel and expect it to pass -  "
            "that is the KNOWN failure, not a bug in the test."
        ),
    ),
    Case(
        name="galloping_octagon_45deg",
        slots=["circle:9:8"], gaits=["galloping"], speeds=[0.8],
        min_s=20, max_s=50,
        why=(
            "CLAUDE.md 'GALLOPING'S REAL CAUSE, CONFIRMED': galloping's "
            "cornering behaviour is fine at 45/144-162 degrees; its real, "
            "confirmed failure mode is a state-ESTIMATOR divergence on long "
            "UNINTERRUPTED straights (34 m of ground-truth-verified error "
            "over a 171 s dash), not cornering and not a swing-leg "
            "placement bug. This case exists to confirm cornering still "
            "works, not to exercise the dash failure - see dash_trotRunning "
            "and the GPS-velocity-aiding section for that thread."
        ),
    ),
]

ALL_CASES = {c.name: c for c in CASES + FLIGHT_GAIT_CASES}


SETTLE_S = 0.0  # see the false-positive note below - the actual race this
                # guarded against is now closed IN THE SERVER
                # (Fleet._teardown_done in conductor/server.py), which
                # structurally blocks a new launch until the previous
                # fleet's teardown is CONFIRMED complete, not just reported
                # "done". Verified directly: star immediately followed by
                # atom with ZERO delay between the two mission_runner.py
                # calls now gives atom a real ~60s completion, not the
                # bogus 10.4s this file used to need a client-side sleep to
                # avoid. Left as a named constant (not deleted outright) in
                # case a future, DIFFERENT race is ever found here - but the
                # fix belongs in the server, not in every caller, and now
                # it is.


def _tail(proc, n):
    """Show STDERR as well as stdout.

    The suite printed only proc.stdout, and mission_runner reports its
    hardest failures through SystemExit - which writes to STDERR. So a run
    that died before it ever launched showed a tail consisting of one
    unrelated line ("cameras forced OFF...") and nothing else, and an 18-of-19
    failure was completely undiagnosable from the suite's own output. A
    harness that hides the reason for a failure is barely better than one
    that reports the wrong verdict.
    """
    out = (proc.stdout or "").splitlines()[-n:]
    err = (proc.stderr or "").splitlines()[-n:]
    parts = []
    if out:
        parts.append("\n".join(out))
    if err:
        parts.append("  --- stderr ---\n" + "\n".join(err))
    return "\n".join(parts) if parts else "  (no output at all)"


def run_case(case: Case, repeats: int) -> bool:
    ok = True
    for rep in range(1, repeats + 1):
        cmd = [sys.executable, str(RUNNER)]
        for s in case.slots:
            cmd += ["--slot", s]
        for g in case.gaits:
            cmd += ["--gait", g]
        for sp in case.speeds:
            cmd += ["--speed", str(sp)]
        for d in case.dashes:
            cmd += ["--dash", str(d)]
        for e in case.extras:
            cmd += ["--extra", e]
        cmd += ["--timeout", str(int(case.max_s) + 120)]  # generous; auto-baseline also applies

        label = f"[{case.name} {rep}/{repeats}]"
        print(f"{label} launching: {' '.join(case.slots)} @ {case.gaits}/{case.speeds}")
        t0 = time.time()
        proc = subprocess.run(cmd, cwd=str(REPO_ROOT), capture_output=True, text=True)
        elapsed_wall = time.time() - t0

        run_history.record(
            case.name,
            {"slots": case.slots, "gaits": case.gaits, "speeds": case.speeds,
             "dashes": case.dashes, "extras": case.extras},
            proc.returncode, elapsed_wall, proc.stdout,
        )

        if proc.returncode == 0:
            print(f"{label} PASS (wall {elapsed_wall:.1f}s)")
        elif "NOFEED" in proc.stdout:
            # OPEN-21: the conductor's pose feed decays with accumulated
            # launches; a NOFEED verdict means the DOG's result is unknown
            # because the panel's own telemetry died - infrastructure, not
            # a robot verdict. The sanctioned response is a safe server
            # recycle (conductor_ctl - /api/stop first, never a bare kill)
            # and ONE retry on the fresh feed. A second NOFEED fails the
            # case so a permanently broken feed cannot silently eat the
            # suite.
            print(f"{label} NOFEED - pose feed dead; recycling conductor and retrying")
            sys.path.insert(0, str(REPO_ROOT / "gazebo/conductor"))
            import conductor_ctl
            conductor_ctl.restart_server("NOFEED in " + case.name)
            proc = subprocess.run(cmd, cwd=str(REPO_ROOT), capture_output=True, text=True)
            if proc.returncode == 0:
                print(f"{label} retry PASS on fresh feed")
            else:
                print(f"{label} retry exit {proc.returncode} - FAILING the case")
                ok = False
        elif proc.returncode == 2:
            # A harness timeout is not a mission verdict - but it is NOT a
            # pass either, and treating it as one hid a real frozen-at-spawn
            # regression behind a 12/12 SUMMARY (the aiding-default star tip,
            # 2026-08-28: the dog ESTOPped at engagement, the stall-timeout
            # fired, exit 2 fell through this branch without touching `ok`,
            # and the case printed PASS). One retry absorbs the legitimate
            # exit-2 causes (finish-line races, a tight budget on a slow but
            # healthy run); a SECOND inconclusive in a row on the same case
            # is treated as the failure it almost certainly is.
            print(f"{label} HARNESS TIMEOUT - not a mission verdict; retrying once")
            print(f"{label} tail of output:\n" + _tail(proc, 15))
            proc = subprocess.run(cmd, capture_output=True, text=True)
            if proc.returncode == 0:
                print(f"{label} retry PASS")
            else:
                ok = False
                print(f"{label} retry exit {proc.returncode} - FAILING the case "
                      f"(two inconclusive/failed attempts in a row)")
                print(f"{label} tail:\n" + _tail(proc, 20))
        else:
            ok = False
            print(f"{label} FAIL (exit {proc.returncode})")
            print(f"{label} why this case is expected to pass:\n  {case.why}")
            print(f"{label} tail of output:\n" + _tail(proc, 25))

        # THE FALSE-POSITIVE THIS FIXES (found running this suite for real,
        # first night it existed): launching a case immediately after the
        # previous one's mission_runner.py process exits is NOT the same as
        # waiting for the SERVER to finish tearing the previous fleet down -
        # server.py's teardown (terminate() -> sleep(1) -> kill() on gz/
        # bridge/controller) runs on a background thread and can still be
        # in flight for a second or more after mission_runner.py has already
        # printed a verdict and returned. A launch that lands in that window
        # can read a few lines of the PREVIOUS run's tail into the new run's
        # ctrl log before truncation catches up - reproduced directly:
        # star (PASS, 6 waypoints) immediately followed by an atom case
        # returned a bogus "PASS" in 10.4s (atom's real baseline is 55+ s)
        # whose log was actually star's tail content, not a real atom run.
        # The identical atom launch, given a natural pause, passed correctly
        # in 60.3s. A harness that can emit a false PASS is worse than no
        # harness (this project's own established lesson, from the exact
        # same class of bug found in the standalone-dash mission earlier) -
        # so this sleep is not a nicety, it is load-bearing for trusting any
        # PASS this file reports.
        time.sleep(SETTLE_S)
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", nargs="*", help="run only these case names")
    ap.add_argument("--fast", action="store_true",
                     help="run only the fast tier (~15 min quick gate) - the default "
                          "runs the FULL suite including the long-course catalog "
                          "(~50 min; lissajous:15:11:9 alone is ~9.5)")
    ap.add_argument("--repeats", type=int, default=1, help="repeats per case (default 1 - SITL has real run-to-run variance, see CLAUDE.md; use 3+ before trusting a single failure)")
    ap.add_argument("--list", action="store_true", help="list cases and their tribal-knowledge rationale, then exit")
    ap.add_argument("--history", nargs="?", const="__all__", help="print the ring-buffer run history (optionally for one case name) and exit, without running anything")
    args = ap.parse_args()

    if args.list:
        for name, c in ALL_CASES.items():
            print(f"\n=== {name} ===\n{c.why}")
        return 0

    if args.history:
        run_history.summarize(None if args.history == "__all__" else args.history)
        return 0

    if args.only:
        names = args.only
    elif args.fast:
        names = [n for n, c in ALL_CASES.items() if c.tier == "fast"]
    else:
        names = list(ALL_CASES.keys())
    unknown = [n for n in names if n not in ALL_CASES]
    if unknown:
        print(f"Unknown case name(s): {unknown}. Known: {list(ALL_CASES.keys())}")
        return 2

    results = {}
    for name in names:
        results[name] = run_case(ALL_CASES[name], args.repeats)

    print("\n" + "=" * 60)
    print("SUMMARY")
    for name, ok in results.items():
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    return 0 if all(results.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
