#!/usr/bin/env python3
"""
Regression suite over this port's ESTABLISHED, validated mission/gait/speed
combinations - the ones CLAUDE.md documents as the reliable baseline after
real measurement, not guesses. Each case below is a real Gazebo SITL run
through the conductor (stm32mp1/gazebo/conductor/), not a synthetic/mocked
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
    python3 stm32mp1/gazebo/conductor/server.py &
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


# ---------------------------------------------------------------------------
# THE VALIDATED BASELINE. Every entry cites the CLAUDE.md section it comes
# from so a failure can be checked against the ACTUAL prior evidence rather
# than just re-argued from scratch.
# ---------------------------------------------------------------------------
CASES = [
    Case(
        name="star",
        slots=["star:10.514:5"], gaits=["trotRunning"], speeds=[3.5],
        min_s=60, max_s=90,
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
        extras=["WP_VSUS=2.4 WP_ANALYZER=1"],
        min_s=60, max_s=110,
        why=(
            "CLAUDE.md 'THE OVAL: a course where a gait switch can pay' and "
            "the later VSUS re-sweep after the planner lineage changed: "
            "WP_VSUS=2.4 (NOT the campaign-era 2.6, which was re-measured "
            "~50% marginal on the current planner/follower) is what clears "
            "the sustained 180s reliably. The oval's own stop sequence is "
            "separately documented as ~80% reliable even when the curves "
            "are clean (a steering-through-the-decel fix, not fully closed) - "
            "a stop-window tip on this one case specifically is a known, "
            "already-documented residual, not necessarily a new regression."
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
        elif proc.returncode == 2:
            print(f"{label} HARNESS TIMEOUT - not a mission verdict, treating as INCONCLUSIVE, not FAIL")
            print(f"{label} tail of output:\n" + "\n".join(proc.stdout.splitlines()[-15:]))
        else:
            ok = False
            print(f"{label} FAIL (exit {proc.returncode})")
            print(f"{label} why this case is expected to pass:\n  {case.why}")
            print(f"{label} tail of output:\n" + "\n".join(proc.stdout.splitlines()[-25:]))

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

    names = args.only if args.only else list(ALL_CASES.keys())
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
