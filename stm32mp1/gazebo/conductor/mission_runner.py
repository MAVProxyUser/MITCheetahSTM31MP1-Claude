#!/usr/bin/env python3
"""Conductor mission launcher/verifier - replaces the ad-hoc .sh test scripts.

Talks only to the conductor's REST API (never touches gz/bridge/controller
processes directly), so it can never itself cause the "harness kills a
legit run" class of bug documented in CLAUDE.md. Every wait is bounded -
overall --timeout AND a --stall-timeout on "no new log line" - so a wedged
server or a hung sim ends the script with a clear TIMEOUT verdict instead
of hanging the terminal forever.

Usage:
  mission_runner.py --slot "dash:100" --timeout 90
  mission_runner.py --slot "star:10.514:5" --slot "oval:40:5.0" \\
                     --slot "atom:9.0:6" --gait trotRunning --gait trotRunning \\
                     --gait trotting --speed 3.5 --speed 3.5 --speed 2.1 \\
                     --dash 100 --dash 100 --dash 100 --timeout 300

Each --slot is just the mission spec; --gait/--speed/--dash/--extra are
repeated flags matched to --slot BY POSITION (the i-th --gait applies to
the i-th --slot). Omitted ones fall back to that mission's own recipe
default.

Exit codes are DELIBERATELY distinct, not a single pass/fail bit:
  0 = every dog reported PASS
  1 = the MISSION reported a real verdict this disagrees with (FAIL, FELL,
      server phase=error) - the sim/controller's own doing
  2 = THIS SCRIPT gave up - its own --timeout or --stall-timeout fired and
      killed a run that may have been perfectly healthy. NOT the same claim
      as exit 1, and printed as an unmissable banner for exactly that
      reason: collapsing the two let a run that had already printed
      RESULT: PASS get reported as a bare, generic failure once tonight
      (see below).

The stall-timeout no longer watches the curated orchestration log alone -
it ALSO resets on any change to state["status"][i]["waypoints"]/["text"]
(updated ~1/s straight from the raw per-tick ctrl log, regardless of
mission shape). This matters more than it sounds: a single-gait, non-
analyzer, no-dash mission (e.g. any of the lissajous specs) can go its
ENTIRE middle - "nav taking the stick" to "settled on its feet" at the
very end, ~550s on lissajous:15:11:9 - without EVENT_PATTERNS matching a
single line, because there is no gait change (needs $WP_ANALYZER, which
that recipe does not set), no dash, no fall, nothing to log. A log-only
stall-timeout of ANY size less than the whole mission will eventually
false-positive on a course exactly this shape; the fix is watching actual
progress, not raising the number further. (Measured getting this wrong:
200s and 250s stall-timeouts each killed a perfectly healthy
lissajous:15:11:9 mid-run, with the ctrl log growing continuously and the
bridge reporting live, changing telemetry the whole time - looked exactly
like a wedged sim or host stall from the outside, and was neither; the
same run went on to PASS at 561.7s, matching its own established
baseline, once given a timeout it could not possibly trip.) Confirm any
future suspected wedge against the raw log (GET /api/logs/{i}) before
trusting a TIMEOUT verdict either way.

Every run - PASS, FAIL, or TIMEOUT - writes a report to
/tmp/cheetah_conductor/reports/run<N>_report.{txt,png}: the full
orchestration log plus a planned-vs-flown path plot, per dog, drawn from
the exact same world-frame data the live panel's own canvas uses (not a
re-derivation from the raw log, which could silently show a different
picture than what was actually on screen).
"""
import argparse
import json
import math
import os
import re
import sys
import time
import urllib.error
import urllib.request

# shm_reaper has no gz dependency (plain ctypes/mmap/struct), unlike
# trail_daemon/mission_geometry above - safe to import unconditionally.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import shm_reaper  # noqa: E402 - "even the launcher itself can reap"

BASE = "http://127.0.0.1:8420"
REPORT_DIR = "/tmp/cheetah_conductor/reports"

# Same hues as trail_daemon.py's DOG_HUES / the panel's own app.js HUES -
# (dim, bright) per dog index, so a report image reads as the same colours
# the operator saw live. Kept as a plain tuple here rather than imported:
# trail_daemon.py pulls in gz.transport13 at module level, which this
# script must not require just to write a report after the run is over.
DOG_HUES = [
    ((1.00, 0.55, 0.10, 0.55), (1.00, 0.75, 0.25, 1.0)),   # amber
    ((0.20, 0.55, 1.00, 0.55), (0.35, 0.80, 1.00, 1.0)),   # blue
    ((0.30, 0.90, 0.35, 0.55), (0.50, 1.00, 0.55, 1.0)),   # green
]


def api(method, path, body=None, timeout=10):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(BASE + path, data=data, method=method,
                                  headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read())
    except urllib.error.URLError as e:
        raise SystemExit("conductor unreachable at %s: %s "
                          "(is server.py running?)" % (BASE, e))


def render_report(run_id, slots, state, all_lines):
    """Write a self-contained post-mission report: the orchestration log
    plus a planned-vs-flown path plot for every dog.

    Uses state["planned"]/state["positions"][i]["trail"] - the EXACT same
    world-frame data server.py freezes at launch and updates live, that the
    browser panel's own canvas draws from (see app.js's ctx.strokeStyle =
    hue.dim / hue.bright). This is not a reconstruction from the raw log;
    it is a snapshot of what was actually on screen, which is the whole
    point - a description built by re-deriving the geometry could silently
    diverge from what the operator saw and reintroduce the exact
    stale/ambiguous-screenshot confusion this feature exists to avoid.
    Called even on a TIMEOUT/stall abort, using whatever state was last
    polled - that is often the most useful case to have a report for.
    """
    os.makedirs(REPORT_DIR, exist_ok=True)
    base = os.path.join(REPORT_DIR, "run%s" % run_id)

    # Per-dog verdict, attributed by the "dogN: ..." / "dogN FELL" prefix
    # server.py's poller _note()s with - the same curated lines the
    # aggregate PASS/FAIL/FELL counts below are drawn from, just split out
    # per index instead of summed across the whole fleet.
    per_dog = {}
    for i in range(len(slots)):
        # log lines carry a "[HH:MM:SS] runN " prefix ahead of the "dogN:
        # ..." text _note() was actually called with, so this has to be a
        # substring check, not startswith. "dogN:"/"dogN " (with the colon
        # or space right after the digit) can't false-match a different
        # dog's line - "dog1:" is not a substring of "dog10:", and this
        # fleet caps at 3 dogs anyway.
        mine = [l for l in all_lines if ("dog%d:" % i) in l or ("dog%d " % i) in l]
        if any("mission result: PASS" in l for l in mine):
            per_dog[i] = "PASS"
            # GROUND-TRUTH GATE (operator-prescribed, 2026-08-28, after a
            # whole terrain matrix of false PASSes): "checking the path
            # actual vs path traveled trails should have told you that."
            # A PASS whose FLOWN trail (world-frame gz poses, the same data
            # the panel canvas and this report plot) is a tiny fraction of
            # its PLANNED path length means the dog's own belief completed
            # the course while its body went nowhere - the estimator
            # hallucinating over slipping feet (frozen GPS, legs cycling in
            # place). The judge inside the controller trusts the estimator,
            # so the gate must live out here, against gz truth.
            def _plen(pts):
                return sum(math.hypot(pts[k + 1][0] - pts[k][0],
                                       pts[k + 1][1] - pts[k][1])
                           for k in range(len(pts) - 1)) if pts and len(pts) > 1 else 0.0
            plan_pts = (state.get("planned") or {}).get(str(i))
            trail_pts = ((state.get("positions") or {}).get(str(i)) or {}).get("trail")
            plan_len = _plen(plan_pts)
            flown_len = _plen(trail_pts)
            if plan_len > 3.0 and flown_len < 0.3 * plan_len:
                # Distinguish "the dog's belief flew while its body stood"
                # (INVALID - trail shows the body parked) from "the panel's
                # own pose feed died" (NOFEED - trail EMPTY/near-empty while
                # the run claims full completion; run748 flew the whole oval
                # per bridge-GPS range while the dead feed produced ratio
                # 0.00 and a false INVALID). A dead feed is infrastructure,
                # not a robot verdict - cross-check bridge GPS before
                # trusting either way.
                per_dog[i] = "NOFEED" if len(trail_pts or []) < 5 else "INVALID"
            # FRICTION/DEVIATION METRICS, per run (operator: "start keeping
            # track of the friction vibe in each world and how much it
            # makes the runs deviate"). xtrack_max = worst distance from
            # any flown point to the planned polyline - the how-wide-did-
            # it-slide number; ratio = flown/planned length. One greppable
            # line; terrain_sweep.py accumulates these into a CSV.
            def _xtrack(trail, plan):
                if not trail or not plan or len(plan) < 2:
                    return -1.0
                worst = 0.0
                for px, py in trail[:: max(1, len(trail) // 200)]:
                    best = float("inf")
                    for k in range(len(plan) - 1):
                        ax, ay = plan[k]; bx, by = plan[k + 1]
                        dx, dy = bx - ax, by - ay
                        L2 = dx * dx + dy * dy
                        t = 0.0 if L2 == 0 else max(0.0, min(1.0,
                            ((px - ax) * dx + (py - ay) * dy) / L2))
                        ex, ey = ax + t * dx - px, ay + t * dy - py
                        best = min(best, ex * ex + ey * ey)
                    worst = max(worst, best)
                return worst ** 0.5
            print("[runner] METRICS dog%d terrain=%s mission=%s flown=%.1f "
                  "plan=%.1f ratio=%.2f xtrack_max=%.2f verdict=%s"
                  % (i, state.get("terrain"), slots[i], flown_len, plan_len,
                     (flown_len / plan_len) if plan_len > 0 else 0.0,
                     _xtrack(trail_pts, plan_pts), per_dog[i]), flush=True)
        elif any("FELL" in l for l in mine):
            per_dog[i] = "FELL"
        elif any("mission result: FAIL" in l for l in mine):
            per_dog[i] = "FAIL"
        else:
            per_dog[i] = "incomplete"

    txt_path = base + "_report.txt"
    with open(txt_path, "w") as f:
        f.write("Cheetah Conductor post-mission report - run %s\n" % run_id)
        f.write("phase at report time: %s\n" % state.get("phase"))
        f.write("=" * 60 + "\n")
        for i, spec in enumerate(slots):
            f.write("dog%d: %-24s -> %s\n" % (i, spec, per_dog[i]))
        f.write("=" * 60 + "\n")
        f.write("orchestration log:\n")
        f.write("\n".join(all_lines) + "\n")
    print("[runner] report: %s" % txt_path, flush=True)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[runner] matplotlib not installed - skipping path plot "
              "(the text report above still has the full log)", flush=True)
        return per_dog

    planned = state.get("planned") or {}
    positions = state.get("positions") or {}
    fig, ax = plt.subplots(figsize=(9, 9), facecolor="#0c0e10")
    ax.set_facecolor("#0c0e10")
    ax.grid(True, color="#1a2020", linewidth=0.8)
    ax.set_aspect("equal")

    any_pts = False
    all_x, all_y = [], []
    for i, spec in enumerate(slots):
        dim, bright = DOG_HUES[i % len(DOG_HUES)]
        p = planned.get(str(i))
        if p and len(p) > 1:
            # closed, matching app.js's ctx.closePath() on the planned line
            xs = [pt[0] for pt in p] + [p[0][0]]
            ys = [pt[1] for pt in p] + [p[0][1]]
            ax.plot(xs, ys, color=dim, linewidth=2,
                    label="dog%d planned (%s)" % (i, spec))
            all_x += xs; all_y += ys
            any_pts = True
        trail = (positions.get(str(i)) or {}).get("trail")
        if trail and len(trail) > 1:
            # NOT closed, matching app.js's flown-trail line
            xs = [pt[0] for pt in trail]
            ys = [pt[1] for pt in trail]
            ax.plot(xs, ys, color=bright, linewidth=2.5,
                    label="dog%d flown (%s)" % (i, per_dog[i]))
            all_x += xs; all_y += ys
            any_pts = True

    if not any_pts:
        plt.close(fig)
        print("[runner] no planned/flown points in state yet - skipping path plot "
              "(run likely aborted before launch finished placing the fleet)",
              flush=True)
        return per_dog

    # Fixed padding around the data bbox, same convention as app.js's own
    # canvas framing (pad=6, span floored at 1) - without this, aspect=
    # 'equal' on a near-straight course (e.g. dash, near-zero east extent)
    # crushes one axis to a sliver and its tick labels collide into an
    # unreadable smear.
    pad = 6.0
    minx, maxx = min(all_x) - pad, max(all_x) + pad
    miny, maxy = min(all_y) - pad, max(all_y) + pad
    if maxx - minx < 1.0:
        cx = (minx + maxx) / 2.0; minx, maxx = cx - 0.5, cx + 0.5
    if maxy - miny < 1.0:
        cy = (miny + maxy) / 2.0; miny, maxy = cy - 0.5, cy + 0.5
    ax.set_xlim(minx, maxx)
    ax.set_ylim(miny, maxy)

    ax.set_title("run %s  -  %s" % (run_id, ", ".join(slots)), color="#e6ebe6")
    ax.set_xlabel("east (m)", color="#7a8580")
    ax.set_ylabel("north (m)", color="#7a8580")
    ax.tick_params(colors="#7a8580")
    for spine in ax.spines.values():
        spine.set_color("#2a3330")
    ax.legend(facecolor="#121517", edgecolor="#2a3330", labelcolor="#e6ebe6",
              fontsize=8, loc="best")
    png_path = base + "_report.png"
    fig.savefig(png_path, dpi=130, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("[runner] report: %s" % png_path, flush=True)
    return per_dog


# Exit codes, DISTINCT on purpose:
#   0 = every dog PASSED
#   1 = the MISSION reported a real verdict this script disagrees with
#       (FAIL, FELL, or server phase=error) - the sim/controller's own doing
#   2 = THIS SCRIPT gave up and killed a run that may have been perfectly
#       healthy - its own --timeout or --stall-timeout fired, not anything
#       the mission itself reported
# Collapsing 1 and 2 into one exit code is exactly what let a run that had
# ALREADY printed "[mission] RESULT: PASS" get reported as a plain, generic
# failure tonight (expsquare:5:12, --timeout 150 fired in a race against the
# mission's own finish) - indistinguishable from a real FELL/FAIL without
# opening the raw log by hand, which is precisely the trap CLAUDE.md already
# documents this whole script existing to avoid. It happened four times in
# one night before this fix, always because a --timeout/--stall-timeout
# picked for an EARLIER, untightened baseline turned out too tight once a
# course's own tuning made it legitimately slower - never because the sim
# was actually broken. See the module docstring's lissajous:15:11:9 story.
HARNESS_TIMEOUT_EXIT = 2

_NAV_RE = re.compile(r"\[nav\] wp\d+/\d+ N=([-\d.]+) E=([-\d.]+)")

# Generous, per-MISSION-KIND expected solo completion times, in seconds -
# not the fastest recorded run, the slowest one this project has actually
# seen pass, rounded well up. Used two ways: (1) to auto-size the overall
# timeout from whatever's actually IN a fleet instead of one hand-picked
# number covering the worst case across the whole catalog every time
# (previously: a fleet containing only fast missions still had to be
# launched with a --timeout sized for lissajous:15:11:9, or a draw
# CONTAINING 11:9 needed remembering to raise it by hand - either way a
# human had to get it right per launch), and (2) to print an early,
# NON-FATAL "past its own baseline" note - per direct instruction, a slow
# dog that is still genuinely progressing should never be killed just for
# missing this number. Keyed by mission-kind prefix; lissajous is further
# keyed by its wx:wy ratio since 1:2 (~96s), 5:7 (~346s) and 11:9 (~562s)
# differ by nearly 6x on the exact same recipe.
BASELINE_S = {
    "star": 120, "oval": 100, "atom": 130, "spiro": 150, "dash": 60,
    "circle": 60, "sector": 160, "parallel": 210, "expsquare": 130,
}
_LISSAJOUS_RATIO_S = {(1, 2): 120, (5, 7): 400, (11, 9): 650}

# Fixed, NOT distance-scaled, sequence every mission pays regardless of
# course length: boot-limp, stand up, balance-stand settle, LOCOMOTION
# entry, the gait-engage hold before nav takes the stick, then at the end
# the decel ramp, the settle check, and the lie-down. Measured off real
# orchestration logs (the gap between "initialising" and "nav taking the
# stick" is ~20 s on its own; the end-of-mission sequence adds ~15 s),
# rounded up to leave room for a slow launch.
BOOT_OVERHEAD_S = 75.0

def mission_path_length_m(spec):
    """Actual planned path length in metres, from the mission's OWN geometry
    (mission_geometry.mission_waypoints, the same generator the controller
    and the panel overlay use), or None if it can't be computed.

    Deliberately measures the real waypoint polyline rather than parsing a
    size parameter out of the spec string: "dash:100" is 100 m but
    "atom:9.0:6" is ~128 m from a 9 m radius, and only the geometry knows.
    """
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
        from mission_geometry import mission_waypoints
        pts = mission_waypoints(spec)
    except Exception:
        return None
    if not pts:
        return None
    # The robot starts at the local-frame origin, so the first leg is
    # origin -> wp0 (for courses that do not shiftFirstToOrigin, that leg
    # is real distance the dog actually walks; for those that do, wp0 IS
    # the origin and this term is zero either way).
    total, prev = 0.0, (0.0, 0.0)
    for p in pts:
        total += math.hypot(p[0] - prev[0], p[1] - prev[1])
        prev = p
    return total


def mission_baseline_s(spec, speed=None):
    """Expected solo completion time for one mission, in seconds.

    DERIVED, not looked up, whenever the geometry and the commanded speed
    are both available: path_length / speed, times a slack factor for the
    accel/decel ramps and per-corner braking the planner imposes, plus the
    fixed boot/stand/settle/lie-down overhead every mission pays.

    WHY THIS IS COMPUTED RATHER THAN TABULATED: BASELINE_S used to be a
    flat per-KIND constant, so "dash" was 60 s whether it was dash:20 at
    0.8 m/s (~25 s of nav) or dash:100 at 0.6 m/s (~150 s+). Everything
    needed to size that correctly - distance and commanded speed - was
    already known at launch, and using a hand-picked round number instead
    is what produced repeated harness timeouts on runs that were healthy
    and still progressing, each of which then had to be re-run and
    re-confirmed by hand before its result could be trusted at all.

    The table stays as a FALLBACK for the case where geometry or speed is
    genuinely unavailable (an unknown future mission kind, or a caller
    that never resolved a speed), and the lissajous ratio table stays
    because those three ratios differ ~6x on identical geometry parameters.
    """
    kind = spec.split(":", 1)[0]
    kind = "dash" if kind in ("outback", "dash") else kind

    if speed and speed > 0.05:
        length_m = mission_path_length_m(spec)
        if length_m and length_m > 0:
            # 1.9x slack on the pure length/speed time: the planner brakes
            # for every corner and ramps from a standstill, so no mission
            # ever averages its commanded cruise. Measured against real
            # runs this project has on record - e.g. atom:9.0:6 (~128 m)
            # at 2.1 m/s is 61 s of ideal cruise against a 62-124 s
            # observed range, and dash:100 at 0.6 is 167 s ideal against
            # a 149.6 s observed (a straight beats the factor, as it
            # should). Plus BOOT_OVERHEAD_S for the fixed stand/settle/
            # gait-engage/lie-down sequence that is not distance-scaled.
            return length_m / float(speed) * 1.9 + BOOT_OVERHEAD_S

    if kind == "lissajous":
        parts = spec.split(":")
        try:
            ratio = (int(parts[2]), int(parts[3]))
        except (IndexError, ValueError):
            ratio = None
        return _LISSAJOUS_RATIO_S.get(ratio, 700)
    return BASELINE_S.get(kind, 300)


def find_zombies(slots):
    """Distinguish a genuinely WEDGED dog from a slow-but-healthy one at the
    moment a timeout fires, instead of leaving that to a manual grep every
    time (which is how the lissajous:15:5:7 zombie below was actually found
    - this automates exactly those two checks).

    A dog is flagged a "zombie" when its raw ctrl log's tail BOTH shows
    "Operating Mode: ESTOP" (MIT's FSM forced it to PASSIVE - motors cut,
    zero debounce, see CLAUDE.md's SafetyCheck section) AND its last few
    [nav] lines report the IDENTICAL (N, E) - nav is still issuing a
    velocity command into a robot that physically cannot move. Confirmed
    reproducing this exact signature live (3-dog fleet, run162,
    lissajous:15:5:7): ESTOP fired at nav handoff, then 280+ seconds of
    "wp0/366 N=0.00 E=0.00 ... v=0.25 w=0.00" with no [FALL] ever printed
    (a zombie doesn't topple, so the mission's own fall detector never
    catches it) - only a stall-timeout on waypoint progress does. This is
    the same "stuck dog" class CLAUDE.md's orientation-hold A/B already
    named (dog0 in hold60_1 sat at wp6/7 for 160s, ESTOPed, never falling,
    never finishing) - that investigation found the DEBOUNCE LENGTH is not
    the fix (Fisher p~0.6, noise on an interleaved A/B), so this only
    diagnoses the symptom, it does not attempt to prevent it.
    """
    zombies = []
    for i in range(len(slots)):
        try:
            resp = api("GET", "/api/logs/%d?tail=60" % i)
        except SystemExit:
            continue
        text = resp.get("text", "") if isinstance(resp, dict) else ""
        if "Operating Mode: ESTOP" not in text:
            continue
        coords = _NAV_RE.findall(text)
        if len(coords) >= 3 and len(set(coords[-3:])) == 1:
            zombies.append(i)
    return zombies


def harness_timeout(reason, run_id, slots, state, all_lines):
    """Called ONLY when THIS SCRIPT's own bound (--timeout/--stall-timeout)
    fired - never when the mission itself reported a result. Loud and
    unmissable on purpose: everything printed here is about to look exactly
    like a wedged sim or a real fall from the outside (see the exit-code
    comment above for why that reads as more dangerous than it is), and the
    single most common REAL cause, measured repeatedly in one night, is that
    the bound itself was picked too tight for this specific course - not
    that anything is actually broken. It can ALSO mean a dog genuinely
    wedged (see find_zombies above) - this function checks for that instead
    of leaving it to a manual log read every time.
    """
    zombies = find_zombies(slots)
    print("", flush=True)
    print("=" * 70, flush=True)
    print("[runner] ##  THIS TEST HARNESS TIMED OUT - NOT A MISSION VERDICT  ##", flush=True)
    print("[runner] %s" % reason, flush=True)
    if zombies:
        print("[runner] AUTO-DIAGNOSIS: dog(s) %s appear ZOMBIED - ESTOP in the"
              % zombies, flush=True)
        print("[runner] raw log with zero position change over the last few [nav]", flush=True)
        print("[runner] lines. This IS a real failure (a fallen-motors freeze the", flush=True)
        print("[runner] mission's own [FALL] detector cannot see), just not one the", flush=True)
        print("[runner] mission itself reported - see find_zombies()'s docstring.", flush=True)
        # Archive each zombie's shm trace NOW - it is the run's own oracle
        # entry, and the process (or its next launch on the same
        # SIM_INSTANCE) could still be alive and about to overwrite the
        # ring the moment this script's own /api/stop lands.
        for i in zombies:
            try:
                path = shm_reaper.dump_snapshot(i, "zombie_estop", run_id=run_id)
                if path:
                    print("[runner] dog%d: shm trace archived -> %s" % (i, path), flush=True)
            except Exception as e:  # noqa: BLE001 - archiving must never mask
                print("[runner] dog%d: shm archive failed: %r" % (i, e), flush=True)
                # the timeout diagnosis already printed above
    else:
        print("[runner] This does NOT mean the mission failed, fell, or the sim", flush=True)
        print("[runner] hung - it means THIS SCRIPT stopped waiting. The most", flush=True)
        print("[runner] likely explanation, by far: --timeout/--stall-timeout was", flush=True)
        print("[runner] too tight for this course. CONFIRM before trusting this as", flush=True)
        print("[runner] a real failure: GET /api/logs/{i} and check whether the", flush=True)
        print("[runner] mission had already reached MISSION COMPLETE / RESULT: PASS.", flush=True)
    print("[runner] Exiting %d (harness timeout), NOT 1 (mission-reported fail)."
          % HARNESS_TIMEOUT_EXIT, flush=True)
    print("=" * 70, flush=True)
    print("", flush=True)
    render_report(run_id, slots, state, all_lines)
    api("POST", "/api/stop")
    sys.exit(HARNESS_TIMEOUT_EXIT)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--slot", action="append", default=[], metavar="MISSION",
                     help="mission spec for one dog slot, e.g. star:10.514:5 "
                          "(repeat for a multi-dog fleet, up to 3)")
    ap.add_argument("--gait", action="append", default=[],
                     help="gait for the matching --slot by position "
                          "(default: recipe default)")
    ap.add_argument("--speed", action="append", default=[], type=float,
                     help="cruise speed for the matching --slot by position")
    ap.add_argument("--dash", action="append", default=[], type=float,
                     help="finish-dash metres for the matching --slot by position (0=none)")
    ap.add_argument("--extra", action="append", default=[],
                     help="raw KEY=VALUE env override(s) for the matching --slot, space-separated")
    ap.add_argument("--timeout", type=float, default=None,
                     help="overall wall-clock budget in seconds. Default: auto-derived "
                          "per-dog from BASELINE_S/mission_baseline_s (each dog's own "
                          "known-slowest mission+ratio, x3, +120s boot overhead) rather "
                          "than one fixed number - a fleet of fast missions no longer "
                          "has to inherit a --timeout sized for lissajous:15:11:9, and a "
                          "draw that DOES contain it doesn't need remembering to raise "
                          "this by hand. This is the FINAL backstop, not the thing that "
                          "usually fires: a dog missing its own baseline only gets a "
                          "printed warning (see --stall-timeout) as long as it is still "
                          "genuinely progressing; only ALL-dogs-stalled or this absolute "
                          "ceiling ends the run early. Pass explicitly to override.")
    ap.add_argument("--stall-timeout", type=float, default=200,
                     help="PER-DOG: seconds of zero change in that dog's own "
                          "state[\"status\"] entry (waypoint index/distance/speed, "
                          "updated ~1/s straight from its raw ctrl log regardless of "
                          "mission shape) before it is flagged as genuinely stalled "
                          "(default 200). This does NOT end the run by itself - other "
                          "still-progressing dogs keep running; the whole fleet is only "
                          "torn down once EVERY dog still running is simultaneously "
                          "flagged, or --timeout's absolute ceiling is hit. Exceeding a "
                          "mission's own expected BASELINE_S is a separate, non-fatal "
                          "warning - a dog that is slow but still moving is never killed "
                          "for that alone. See mission_baseline_s()'s docstring for why "
                          "the two are tracked separately.")
    ap.add_argument("--poll", type=float, default=2.0,
                     help="state-poll interval in seconds (default 2)")
    ap.add_argument("--terrain", default=None,
                     help="terrain/surface kind for this launch (flat, rolling, "
                          "rough, concrete, asphalt, grass, dirt, gravel, sand, "
                          "mud, rock, ice); omitted = the server draft's kind")
    ap.add_argument("--no-chase", action="store_true",
                     help="run camera-dark (chase is ON by default per "
                          "operator, 2026-08-28: 'if there is no cost to "
                          "enabling it, please keep the chase cam enabled' - "
                          "measured cost of one chase feed: zero control-loop "
                          "impact, a few %% GPU)")
    ap.add_argument("--chase", action="store_true",
                     help="spawn + stream each dog's chase camera (front/nadir "
                          "stay off). Since body-launch camera flags FAIL DARK, "
                          "--keep-cameras alone can no longer turn a camera ON "
                          "for an automated run - this can, and is the way to "
                          "WATCH a scripted run live (operator-requested for "
                          "the terrain matrix, 2026-08-28).")
    ap.add_argument("--keep-cameras", action="store_true",
                     help="don't force cameras off before launch (default: force off)")
    args = ap.parse_args()

    if not args.slot:
        ap.error("need at least one --slot")
    if len(args.slot) > 3:
        ap.error("max 3 slots")

    st = api("GET", "/api/state")

    # AUTO-SIZED TIMEOUTS, from each mission's own geometry and the speed it
    # will actually be launched at - resolved HERE, after /api/state is
    # available, because the speed may come from the recipe rather than the
    # command line and mission_baseline_s cannot derive anything without it.
    # See that function's docstring for why this is computed instead of
    # looked up in a per-kind table.
    _recipes_for_baseline = st.get("recipes", {})

    def _resolved_speed(i, spec):
        if i < len(args.speed):
            return args.speed[i]
        k = spec.split(":", 1)[0]
        k = "dash" if k in ("outback", "dash") else k
        return _recipes_for_baseline.get(k, {}).get("speed")

    dog_baseline = [mission_baseline_s(m, _resolved_speed(i, m))
                    for i, m in enumerate(args.slot)]
    if args.timeout is None:
        args.timeout = max(dog_baseline) * 2.0 + 60
        print("[runner] --timeout not given - auto-derived %.0fs from this fleet's own "
              "geometry x speed (per-dog: %s, x2 + 60s)"
              % (args.timeout,
                 {m: "%.0fs" % b for m, b in zip(args.slot, dog_baseline)}), flush=True)

    gaits = st["gaits"]                       # name -> numeric SIM_GAIT id
    gait_name_by_id = {v: k for k, v in gaits.items()}
    recipes = st["recipes"]                   # RECIPES stores gait as that
                                               # SAME numeric id, not a name -
                                               # has to be converted back
                                               # before it can be POSTed to
                                               # /api/slots/{i}, which only
                                               # accepts a name (checks
                                               # `fields["gait"] in GAITS`,
                                               # GAITS's KEYS being names).

    # --- build the slot list LOCALLY and launch with an explicit body ----
    # This script used to reconcile the server's DRAFT slot-by-slot
    # (add/remove to match the count, then POST each slot's fields) and
    # launch with an empty body, i.e. "launch whatever the draft shows".
    # That worked, but it meant every automated run REWROTE THE OPERATOR'S
    # OWN PANEL: after any suite/sweep, the draft held the last test's
    # mission/gait/speed - e.g. circle:9:8 @ galloping 0.8 sitting in slot
    # 0 - and the panel then flagged it with "not this course's validated
    # combo" warnings the operator never caused (operator-reported, twice).
    # /api/launch has always accepted an explicit "slots" body that skips
    # the draft entirely, and launch() resolves every omitted field from
    # the mission's own recipe with the same code path the draft uses - so
    # automation now uses that, and the draft belongs to the human again.
    slots_body = []
    for i, mission in enumerate(args.slot):
        kind = mission.split(":", 1)[0]
        kind = "dash" if kind in ("outback", "dash") else kind
        recipe = recipes.get(kind, {})
        slot = {"mission": mission}
        if i < len(args.gait):
            g = args.gait[i]
            if g not in gaits:
                raise SystemExit("unknown gait %r - choices: %s" % (g, sorted(gaits)))
            slot["gait"] = g
        elif "gait" in recipe:
            slot["gait"] = gait_name_by_id.get(recipe["gait"], recipe["gait"])
        if i < len(args.speed):
            slot["speed"] = args.speed[i]
        elif "speed" in recipe:
            slot["speed"] = recipe["speed"]
        # --dash has no recipe concept (RECIPES carries gait/speed/extra/
        # note only) - explicit 0 rather than an omission, so nothing is
        # left to a default that might change underneath this script.
        slot["dash"] = args.dash[i] if i < len(args.dash) else 0
        # ADDITIVE, not a replacement: launch() ALWAYS prepends the
        # recipe's own extra ("recipe['extra'] + ' ' + s['extra']"), so
        # this is a genuine additional override (env A=1 A=2 keeps the
        # last, so an explicit key here wins over the recipe's).
        slot["extra"] = args.extra[i] if i < len(args.extra) else ""
        if not args.keep_cameras:
            slot["cam_front"] = slot["cam_nadir"] = slot["cam_chase"] = False
        if args.chase or not args.no_chase:
            slot["cam_chase"] = True
        slots_body.append(slot)

    if not args.keep_cameras:
        # Say so out loud: runs launched this way have NO camera sensors in
        # the world, so the panel's camera checkboxes are inert for the whole
        # run (a sensor absent from the SDF cannot be enabled mid-run) - which
        # read as "dead checkboxes" to the operator (2026-08-28) when suite
        # runs were on screen. The panel now greys them out; this line makes
        # the same fact visible in the runner's own transcript.
        print("[runner] cameras forced OFF for this launch (GPU fail-dark; "
              "--keep-cameras to keep the slot's own camera flags)")

    # --- launch -------------------------------------------------------
    body = {"slots": slots_body}
    if args.terrain:
        body["terrain"] = args.terrain
    r = api("POST", "/api/launch", body)
    if not r.get("ok"):
        raise SystemExit("launch refused: %s" % r.get("message"))

    st = api("GET", "/api/state")
    run_id = st.get("run_id")
    print("[runner] launched run %s: %s" % (run_id, ", ".join(args.slot)), flush=True)

    # /api/state only ever carries the last 60 orchestration lines (a
    # sliding window, not a growing log with a stable offset), so we can't
    # page through it by length. Instead remember the last line we already
    # printed and find its most recent occurrence in the new window; print
    # only what comes after it. If it has aged out of the window entirely
    # (a burst of >60 lines between polls), fall back to printing the whole
    # window rather than silently dropping lines.
    last_line = None
    all_lines = []   # every line ever streamed, for the verdict - the
                      # snapshot's own log[] is capped to the last 60 and
                      # can scroll an early dog's result line out of view
                      # before we read the final state.
    # PROGRESS, not just LOG LINES. Found the hard way on lissajous:15:11:9
    # (a single-gait, non-analyzer, no-dash mission): EVENT_PATTERNS has no
    # entry at all for routine waypoint advancement, gait changes never fire
    # without $WP_ANALYZER, and there is no dash/HGOV/fall to log either -
    # so the orchestration log produces LITERALLY ZERO new lines between
    # "nav taking the stick" and "settled on its feet" at the very END of
    # the mission. On that course that gap is the ENTIRE ~550s middle of an
    # otherwise perfectly healthy run. No stall-timeout value that still
    # deserves the name "stall timeout" can paper over a gap that long by
    # just being bigger - the fix is to stop relying on the sparse curated
    # log alone. state["status"][i]["waypoints"]/["text"] (wp index, d=.../
    # v=...) IS updated every ~1s straight from the raw per-tick ctrl log
    # regardless of mission shape (see server.py's _start_poller) - treating
    # a CHANGE there as progress too catches "the robot is still actually
    # moving" even through a mission-shape that never emits a single curated
    # event for minutes at a time. Verified: the same mission that a 200s/
    # 250s log-only stall-timeout falsely killed twice ran to completion
    # (PASS 561.7s) once given a timeout it could not possibly hit - this
    # is what makes that unnecessary in the first place.
    # PER-DOG tracking, not one shared clock. A shared clock means the
    # FASTEST dog in the fleet resets everyone's grace period, which can
    # mask a genuinely dead dog for as long as anyone else is moving, and
    # a single dog whose own mission is legitimately slower than another
    # slot's has no way to get more room than whatever number was picked
    # for the whole run. Each dog gets its own start time, its own
    # baseline (mission_baseline_s), its own stall clock, and its own
    # one-shot "past baseline" warning - a dog missing its baseline is
    # flagged, never killed, as long as ITS OWN status keeps changing.
    n_dogs_total = len(args.slot)
    dog_last_progress = [time.monotonic()] * n_dogs_total
    dog_last_status = [None] * n_dogs_total
    dog_warned_slow = [False] * n_dogs_total
    dog_flagged_stall = [False] * n_dogs_total
    dog_done = [False] * n_dogs_total
    # Baselines were measured as each mission's OWN nav-clock "COMPLETE
    # t=Xs" (README/CLAUDE.md), not wall-clock-since-script-launch - every
    # dog spends ~25-35s on world build + stand + balance + settle before
    # nav ever takes the stick, which is real but has nothing to do with
    # that mission's own speed. Comparing against launch time made a
    # dash:100 (33.3s nav-clock) trip its 60s baseline warning at 60s
    # wall-clock even though only ~28s of that was the mission actually
    # running - caught live on the very first real 2-dog run this shipped
    # on. None is "hasn't started yet" - no warning is possible before that.
    dog_nav_start = [None] * n_dogs_total
    start = time.monotonic()
    final_state = None
    while True:
        now = time.monotonic()
        if now - start > args.timeout:
            harness_timeout("ABSOLUTE CEILING (--timeout %.0fs) elapsed - this is the "
                             "final backstop, not a per-mission budget" % args.timeout,
                             run_id, args.slot, st, all_lines)

        st = api("GET", "/api/state")
        log = st.get("log", [])
        if log:
            if last_line in log:
                idx = len(log) - 1 - log[::-1].index(last_line)
                new_lines = log[idx + 1:]
            else:
                new_lines = log
            for line in new_lines:
                print("[run%s] %s" % (run_id, line), flush=True)
                all_lines.append(line)
                for i in range(n_dogs_total):
                    # "dog0: mission result: PASS" and "dog0 FELL" are both
                    # real formats server.py emits (see EVENT_PATTERNS) -
                    # neither reliably has a colon, so match the dog token
                    # on a word boundary rather than assuming one.
                    if re.search(r"\bdog%d\b" % i, line) and \
                            ("mission result:" in line or "FELL" in line):
                        dog_done[i] = True
                    if dog_nav_start[i] is None and re.search(r"\bdog%d\b" % i, line) and \
                            "nav taking the stick" in line:
                        dog_nav_start[i] = now
            last_line = log[-1]

        for s in st.get("status", []):
            i = s.get("index")
            if i is None or i >= n_dogs_total or dog_done[i]:
                continue
            key = (s.get("waypoints"), s.get("text"))
            if key != dog_last_status[i]:
                dog_last_status[i] = key
                dog_last_progress[i] = now
                dog_flagged_stall[i] = False

        still_running = [i for i in range(n_dogs_total) if not dog_done[i]]
        for i in still_running:
            if not dog_warned_slow[i] and dog_nav_start[i] is not None and \
                    now - dog_nav_start[i] > dog_baseline[i]:
                dog_warned_slow[i] = True
                print("[runner] dog%d: past its own expected baseline (%.0fs since nav "
                      "took the stick, now %.0fs) for %s - still progressing, NOT "
                      "killing it"
                      % (i, dog_baseline[i], now - dog_nav_start[i], args.slot[i]), flush=True)
            if not dog_flagged_stall[i] and now - dog_last_progress[i] > args.stall_timeout:
                dog_flagged_stall[i] = True
                print("[runner] dog%d: NO PROGRESS for %.0fs (stall-timeout %.0fs) - "
                      "flagging as stalled; other dogs keep running"
                      % (i, now - dog_last_progress[i], args.stall_timeout), flush=True)

        if still_running and all(dog_flagged_stall[i] for i in still_running):
            harness_timeout("ALL REMAINING DOGS STALLED: %s - nothing left to wait for"
                             % still_running, run_id, args.slot, st, all_lines)

        phase = st.get("phase")
        if phase in ("done", "error", "idle"):
            final_state = st
            break
        time.sleep(args.poll)

    # --- verdict ------------------------------------------------------
    # These strings must match server.py's OWN curated wording exactly
    # (EVENT_PATTERNS' "mission result: %s" formatter and the "dogN FELL"
    # note next to it) - not the raw controller log's "[mission] RESULT:
    # PASS", which never appears verbatim in /api/state's log array. A
    # prior version of this script guessed at that raw-log string and
    # reported every passing run as FAIL.
    log_text = "\n".join(all_lines)
    n_dogs = len(args.slot)
    passes = log_text.count("mission result: PASS")
    fails = log_text.count("mission result: FAIL")
    fell = sum(1 for line in all_lines if "FELL" in line)
    world_fail = "world build FAILED" in log_text

    per_dog = render_report(run_id, args.slot, final_state, all_lines)
    # The gate can demote a claimed PASS to INVALID (flown trail a tiny
    # fraction of the planned path - the dog's belief finished, its body
    # did not). Re-derive the counts from the GATED verdicts, not the raw
    # log strings, or a hallucinated run still exits 0.
    passes = sum(1 for v in per_dog.values() if v == "PASS")
    invalid = sum(1 for v in per_dog.values() if v == "INVALID")

    print("=" * 60)
    print("[runner] run %s phase=%s  dogs=%d  PASS=%d FAIL=%d FELL=%d INVALID=%d"
          % (run_id, final_state.get("phase"), n_dogs, passes, fails, fell, invalid))
    if invalid:
        print("[runner] ## INVALID: %d dog(s) claimed PASS with essentially no"
              " real-world travel (flown << planned) - false positive, do NOT"
              " count this as a result ##" % invalid)
    if world_fail:
        print("[runner] WORLD BUILD FAILED - nothing launched")
        sys.exit(1)
    if final_state.get("phase") == "error":
        # A LAUNCH THAT NEVER RAN IS NOT A MISSION VERDICT. The server aborts
        # a launch when the world build fails or the dogs never advertise
        # sensors (the gz-transport discovery failure) - and it now does so
        # in ~25 s, cleanly, instead of half-starting and orphaning the sim.
        # That made the failure FASTER, which is good, but it also made it
        # look exactly like a quick robot failure to every caller: the
        # overnight sweep recorded several of these as FAIL at ~29 s. Exit 3
        # says "infrastructure, re-run" the way exit 2 says "harness timeout,
        # not a verdict" - callers that grade cells must treat it that way.
        print("[runner] ## LAUNCH ABORTED BY THE SERVER (phase=error) - "
              "the mission never ran. This is INFRASTRUCTURE, not a verdict: "
              "do NOT record it as a failure, re-run the cell. ##")
        sys.exit(3)
    if passes == n_dogs and fails == 0 and fell == 0 and invalid == 0:
        print("[runner] VERDICT: PASS (%d/%d)" % (passes, n_dogs))
        sys.exit(0)
    print("[runner] VERDICT: FAIL (%d/%d passed)" % (passes, n_dogs))
    sys.exit(1)


if __name__ == "__main__":
    main()
