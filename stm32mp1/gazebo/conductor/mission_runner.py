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
default. Exits 0 if every dog in the run reported PASS, 1 otherwise (FAIL,
FELL, error, or timeout).

Pick --stall-timeout generously - a mission can go 60s+ between curated
orchestration log lines with the controller perfectly healthy the whole
time (e.g. the atom's tightest corner, R~1.89m, has the dog visibly creep
for several seconds with no discrete event to log). A short stall-timeout
does not distinguish that from a genuine wedge; it will kill a healthy run
and report a false FAIL. Confirm suspected wedges against the raw log
(GET /api/logs/{i}) before trusting a TIMEOUT verdict.

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
    ap.add_argument("--timeout", type=float, default=300,
                     help="overall wall-clock budget in seconds (default 300)")
    ap.add_argument("--stall-timeout", type=float, default=200,
                     help="abort if no new orchestration log line appears for this many "
                          "seconds - catches a wedged launch, not just a slow one (default "
                          "200; a healthy run CAN go well over 100s silent through a course "
                          "with many short legs and gentle braking and no gait-change/mission "
                          "event to log in between - measured on the sector-search mission, "
                          "which killed a perfectly healthy run at 100s. See the module "
                          "docstring.)")
    ap.add_argument("--poll", type=float, default=2.0,
                     help="state-poll interval in seconds (default 2)")
    ap.add_argument("--keep-cameras", action="store_true",
                     help="don't force cameras off before launch (default: force off)")
    args = ap.parse_args()

    if not args.slot:
        ap.error("need at least one --slot")
    if len(args.slot) > 3:
        ap.error("max 3 slots")

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
    last_progress = time.monotonic()
    start = time.monotonic()
    final_state = None
    while True:
        now = time.monotonic()
        if now - start > args.timeout:
            print("[runner] TIMEOUT after %.0fs (overall budget) - stopping run"
                  % args.timeout, flush=True)
            render_report(run_id, args.slot, st, all_lines)
            api("POST", "/api/stop")
            sys.exit(1)
        if now - last_progress > args.stall_timeout:
            print("[runner] TIMEOUT: no new log line for %.0fs - run appears wedged, stopping"
                  % args.stall_timeout, flush=True)
            render_report(run_id, args.slot, st, all_lines)
            api("POST", "/api/stop")
            sys.exit(1)

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
            if new_lines:
                last_progress = now
            last_line = log[-1]

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
