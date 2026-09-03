#!/usr/bin/env python3
"""Pre-planning host-load budget for a fleet - what it costs the MACHINE,
computed before anything launches.

WHY THIS EXISTS
---------------
Per direct instruction: "based on the complexity and length of the mission
you can preemptively calculate the load budget on the CPU / simulator as
part of pre-planning too if we are gonna imply things about that host
contention theory of yours. You could use that value as a weight, and see
if any specific combined weight triggers it."

That is the right correction. This project has repeatedly attributed
multi-dog failures to "host contention" AFTER the fact, qualitatively,
which is unfalsifiable as stated - and has been wrong about it at least
three times (see CLAUDE.md's meta-lesson on multi-dog-batch artifacts:
pacing's "~50% coin-flip", walking's "mid-band softness", part of
bounding's "bimodal" all evaporated under solo re-testing). A number
computed up front, that predicts which combinations should break, is
testable; a story told afterwards is not.

WHAT DRIVES THE LOAD - MEASURED, NOT ASSUMED
--------------------------------------------
The intuition worth checking was "dash is the least complex", i.e. that a
geometrically simple course is cheaper per tick. Checked against 120
archived ctrl logs, median control-loop period by mission kind:

    atom   2.49 ms      circle 2.49 ms      dash   2.48 ms
    oval   2.48 ms      star   2.48 ms      (corner 2.94, unvalidated)

**Per-tick cost is flat across every validated mission kind.** That is
what the architecture predicts once stated plainly: the convex-MPC solve
is a fixed-size QP (horizon 10, 12 decision vars) that knows nothing about
course geometry, BodyPathPlanner::follow()'s nearest-index search is
forward-only from _lastIdx (amortised O(1), not O(path)), and its
lookahead scan is bounded by Ld/resample_step, a constant. Gazebo's own
physics step is per-DOG, not per-course.

So course "complexity" is NOT the load variable. **Duration is.** A dog's
cost to the host is (how long it runs) x (a per-dog constant), and the
interesting consequence is counter-intuitive:

    dash:100 @0.6 m/s   ~150 s  -> MORE total host load
    star @3.5 m/s        ~60 s  -> LESS

The simplest course in the catalog is the EXPENSIVE one, because it is
slow. "More dashes than sectors" is therefore not automatically a lighter
fleet - it depends entirely on the speeds.

THE UNIT
--------
dog-seconds: one dog running for one second. A fleet's budget is the sum
over its dogs, and its PEAK CONCURRENCY is the number of dogs actually
overlapping (which is what a shared physics thread and a 10-core host
actually contend over). Both are reported, because they are different
questions: total dog-seconds is how much work, peak concurrency is how
much at once.

USAGE
    load_budget.py "star:10.514:5@3.5" "dash:100@0.6" "atom:9.0:6@2.1"
    load_budget.py --equal-load 3 dash:100 star:10.514:5

The second form answers the experiment design question directly: what
speeds make N dogs cost the SAME, so a contention test varies concurrency
and nothing else.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from mission_geometry import mission_waypoints  # noqa: E402

# Measured constants, from this project's own archived runs - not guesses.
# See the module docstring for the per-kind median-period table these come
# from. TICK_MS is the control period the loop actually holds (target 2.0,
# achieved 2.48-2.49 median on this host), and it is FLAT across mission
# kinds, which is the whole finding.
TICK_MS = 2.485
# Fixed per-run sequence that is not distance-scaled: boot-limp, stand,
# balance-stand settle, LOCOMOTION entry, gait-engage hold, and at the end
# the decel ramp, settle check and lie-down. Same figure mission_runner.py
# uses for its own timeout derivation, for the same reason.
BOOT_OVERHEAD_S = 75.0
# Slack on pure length/speed: the planner brakes for every corner and ramps
# from a standstill, so no mission averages its commanded cruise.
PLAN_SLACK = 1.9


def path_length_m(spec):
    """Real planned polyline length, including the origin -> wp0 leg the
    dog actually walks. Measured from the same generator the controller
    uses rather than parsed out of the spec - dash:100 is 100 m but
    atom:9.0:6 is ~127 m from a 9 m radius, and only the geometry knows."""
    pts = mission_waypoints(spec)
    if not pts:
        return 0.0
    total, prev = 0.0, (0.0, 0.0)
    for p in pts:
        total += math.hypot(p[0] - prev[0], p[1] - prev[1])
        prev = p
    return total


def dog_seconds(spec, speed):
    """This dog's own host-load footprint, in dog-seconds."""
    if not speed or speed <= 0.05:
        return None
    return path_length_m(spec) / float(speed) * PLAN_SLACK + BOOT_OVERHEAD_S


def ticks(spec, speed):
    """Control-loop ticks, i.e. the actual unit of work the host does.
    Reported alongside dog-seconds because it is the quantity the flat
    per-tick cost finding is about."""
    s = dog_seconds(spec, speed)
    return None if s is None else int(s * 1000.0 / TICK_MS)


def speed_for_load(spec, target_dog_seconds):
    """Inverse: what commanded speed makes this mission cost exactly
    target_dog_seconds? Used to build equal-load fleets, so a contention
    experiment varies CONCURRENCY and holds work-per-dog fixed instead of
    confounding the two the way every past multi-dog batch here did."""
    travel = target_dog_seconds - BOOT_OVERHEAD_S
    if travel <= 0:
        return None
    return path_length_m(spec) * PLAN_SLACK / travel


def main(argv):
    if len(argv) > 2 and argv[1] == "--equal-load":
        n = int(argv[2])
        specs = argv[3:]
        if not specs:
            print("need at least one mission spec", file=sys.stderr)
            return 2
        # Anchor on the first spec at a sane default so the target is a real
        # runtime rather than an arbitrary number.
        target = dog_seconds(specs[0], 2.0)
        print("equal-load fleet of %d, target %.0f dog-seconds each "
              "(%.0f total, anchored on %s @2.0 m/s)\n"
              % (n, target, target * n, specs[0]))
        print("%-22s %10s %10s" % ("mission", "speed", "path_m"))
        for s in specs:
            v = speed_for_load(s, target)
            print("%-22s %10s %10.1f"
                  % (s, ("%.2f m/s" % v) if v else "impossible", path_length_m(s)))
        return 0

    entries = []
    for a in argv[1:]:
        if "@" in a:
            spec, sp = a.rsplit("@", 1)
            entries.append((spec, float(sp)))
        else:
            entries.append((a, 2.0))
    if not entries:
        print(__doc__)
        return 2

    print("%-22s %8s %9s %11s %10s" %
          ("mission", "speed", "path_m", "dog-seconds", "ticks"))
    total = 0.0
    for spec, sp in entries:
        ds = dog_seconds(spec, sp)
        total += ds or 0.0
        print("%-22s %8.2f %9.1f %11.0f %10s"
              % (spec, sp, path_length_m(spec), ds or 0,
                 "{:,}".format(ticks(spec, sp)) if ds else "n/a"))
    print("-" * 64)
    print("fleet total        %27.0f dog-seconds" % total)
    print("peak concurrency   %27d dogs" % len(entries))
    print("\nPeak concurrency is what a shared physics thread and a fixed core")
    print("count actually contend over; total dog-seconds is how much work the")
    print("fleet asks for overall. A contention threshold, if one exists, should")
    print("track the FIRST of those - vary it while holding per-dog load equal")
    print("(--equal-load) so the two are not confounded, which is exactly what")
    print("every past multi-dog batch in this project failed to do.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
