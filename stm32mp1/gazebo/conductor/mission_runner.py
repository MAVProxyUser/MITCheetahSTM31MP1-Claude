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
"""
import argparse
import json
import sys
import time
import urllib.error
import urllib.request

BASE = "http://127.0.0.1:8420"


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
    ap.add_argument("--timeout", type=float, default=240,
                     help="overall wall-clock budget in seconds (default 240)")
    ap.add_argument("--stall-timeout", type=float, default=100,
                     help="abort if no new orchestration log line appears for this many "
                          "seconds - catches a wedged launch, not just a slow one (default "
                          "100; a healthy run CAN go 60s+ silent through a slow corner, "
                          "e.g. the atom's tightest turn - see the module docstring)")
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
    gaits = st["gaits"]

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
    for i, mission in enumerate(args.slot):
        body = {"mission": mission}
        if i < len(args.gait):
            g = args.gait[i]
            if g not in gaits:
                raise SystemExit("unknown gait %r - choices: %s" % (g, sorted(gaits)))
            body["gait"] = g
        if i < len(args.speed):
            body["speed"] = args.speed[i]
        if i < len(args.dash):
            body["dash"] = args.dash[i]
        if i < len(args.extra):
            body["extra"] = args.extra[i]
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
            api("POST", "/api/stop")
            sys.exit(1)
        if now - last_progress > args.stall_timeout:
            print("[runner] TIMEOUT: no new log line for %.0fs - run appears wedged, stopping"
                  % args.stall_timeout, flush=True)
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
