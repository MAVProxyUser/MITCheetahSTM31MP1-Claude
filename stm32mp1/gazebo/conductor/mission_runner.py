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
import os
import re
import sys
import time
import urllib.error
import urllib.request

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
        return

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
        return

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


def mission_baseline_s(spec):
    """Generous expected-completion estimate for one mission spec string -
    see BASELINE_S's comment for what "generous" means here. Unknown kinds
    (a future mission nobody's added yet) get a large flat guess rather
    than a crash, printed as a guess so it doesn't read as a measurement."""
    kind = spec.split(":", 1)[0]
    kind = "dash" if kind in ("outback", "dash") else kind
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
    ap.add_argument("--keep-cameras", action="store_true",
                     help="don't force cameras off before launch (default: force off)")
    args = ap.parse_args()

    if not args.slot:
        ap.error("need at least one --slot")
    if len(args.slot) > 3:
        ap.error("max 3 slots")

    dog_baseline = [mission_baseline_s(m) for m in args.slot]
    if args.timeout is None:
        args.timeout = max(dog_baseline) * 3 + 120
        print("[runner] --timeout not given - auto-derived %.0fs from this fleet's "
              "own missions (baselines: %s, x3 + 120s boot overhead)"
              % (args.timeout, dict(zip(args.slot, dog_baseline))), flush=True)

    st = api("GET", "/api/state")
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

    # --- reconcile draft slot COUNT to len(args.slot) --------------------
    for _ in range(20):
        cur = len(st["draft_slots"])
        if cur == len(args.slot):
            break
        if cur < len(args.slot):
            r = api("POST", "/api/slots/add")
            if not r.get("ok"):
                raise SystemExit("could not add draft slot: %s" % r.get("message"))
        else:
            r = api("DELETE", "/api/slots/%d" % (cur - 1))
            if not r.get("ok"):
                raise SystemExit("could not remove draft slot: %s" % r.get("message"))
        st = api("GET", "/api/state")
    if len(st["draft_slots"]) != len(args.slot):
        raise SystemExit("could not reconcile draft slot count (have %d, want %d)"
                          % (len(st["draft_slots"]), len(args.slot)))

    # --- populate each slot ----------------------------------------------
    # /api/slots/{i} only OVERWRITES the fields you pass it - a bare
    # {"mission": ...} leaves gait/speed/dash/extra at whatever that draft
    # slot already held (server.py's draft_set_slot() has no "look up this
    # mission's recipe" fallback of its own; that lookup lives in the
    # BROWSER's JS, which this script does not run). Relying on the slot
    # to already hold the right mission's leftover values is exactly the
    # kind of silent state carryover this harness exists to avoid - caught
    # here after a server restart reset slot 0 to its hard-coded star
    # default (trotRunning/3.5/dash=100) and a bare --slot sector:15:3 ran
    # star's gait/speed/dash against sector's waypoints. So: look up each
    # mission's own recipe by kind (mirroring server.py's mission_kind())
    # and apply it explicitly whenever the matching --gait/--speed/--extra
    # flag was not given. --dash has no recipe concept at all (RECIPES
    # carries gait/speed/extra/note only) - default it to 0 (no finish
    # dash) rather than trust the slot's leftover value.
    for i, mission in enumerate(args.slot):
        kind = mission.split(":", 1)[0]
        kind = "dash" if kind in ("outback", "dash") else kind
        recipe = recipes.get(kind, {})
        body = {"mission": mission}
        if i < len(args.gait):
            g = args.gait[i]
            if g not in gaits:
                raise SystemExit("unknown gait %r - choices: %s" % (g, sorted(gaits)))
            body["gait"] = g
        elif "gait" in recipe:
            body["gait"] = gait_name_by_id.get(recipe["gait"], recipe["gait"])
        if i < len(args.speed):
            body["speed"] = args.speed[i]
        elif "speed" in recipe:
            body["speed"] = recipe["speed"]
        if i < len(args.dash):
            body["dash"] = args.dash[i]
        else:
            body["dash"] = 0
        if i < len(args.extra):
            # ADDITIVE, not a replacement: server.py's launch() ALWAYS
            # prepends the recipe's own extra to the slot's extra field
            # ("recipe['extra'] + ' ' + s['extra']", server.py:598), so
            # this is for a genuine additional override beyond the
            # recipe's own tuning (env A=1 A=2 keeps the last, so this
            # wins over the recipe on a shared key).
            body["extra"] = args.extra[i]
        else:
            # Explicitly clear rather than omit the key: server.py's
            # draft_set_slot() only touches "extra" when the key is
            # present at all, so a bare omission here would leave
            # whatever override a PRIOR /api/slots/{i} call on this same
            # slot happened to leave behind - the exact silent-carryover
            # bug this whole fallback exists to close. The recipe's own
            # tuning still applies either way, via the same launch()
            # prepend.
            body["extra"] = ""
        if not args.keep_cameras:
            body["cam_front"] = body["cam_nadir"] = body["cam_chase"] = False
        r = api("POST", "/api/slots/%d" % i, body)
        if not r.get("ok"):
            raise SystemExit("slot %d rejected: %s" % (i, r.get("message")))

    # --- launch -------------------------------------------------------
    r = api("POST", "/api/launch", {})
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

    render_report(run_id, args.slot, final_state, all_lines)

    print("=" * 60)
    print("[runner] run %s phase=%s  dogs=%d  PASS=%d FAIL=%d FELL=%d"
          % (run_id, final_state.get("phase"), n_dogs, passes, fails, fell))
    if world_fail:
        print("[runner] WORLD BUILD FAILED - nothing launched")
        sys.exit(1)
    if final_state.get("phase") == "error":
        print("[runner] server phase=error")
        sys.exit(1)
    if passes == n_dogs and fails == 0 and fell == 0:
        print("[runner] VERDICT: PASS (%d/%d)" % (passes, n_dogs))
        sys.exit(0)
    print("[runner] VERDICT: FAIL (%d/%d passed)" % (passes, n_dogs))
    sys.exit(1)


if __name__ == "__main__":
    main()
