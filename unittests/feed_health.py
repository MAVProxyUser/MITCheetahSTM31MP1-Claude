#!/usr/bin/env python3
"""OPEN-21 metric: how healthy is the pose feed, per SERVER PROCESS LIFETIME?

The failure being tracked is "the feed decays as launches accumulate inside
one server process", so the only meaningful denominator is launches SINCE
THE LAST SERVER START - a count across restarts hides exactly the effect.
This reads the conductor's own stdout log, splits it at each server start,
and reports the last segment (or all of them with --all).

Signals, all of them lines the server already writes - nothing added to the
hot path:
  "building fleet world:"        one per launch          -> the denominator
  "POSE FEED STALLED"            heartbeat saw >5 s gap
  "NOFEED"                       a run demoted for a dead feed
  "pose feed is LYING"           the bridge-GPS arbiter caught a false accusation
  "pose feed: restarted"         self-heal fired
  "pose feed up (per-run"        the OPEN-21 root fix is in play for that run
"""
import argparse
import re
import sys

LOG = "/tmp/conductor_server.log"
SIGNALS = [
    ("launches",   re.compile(r"building fleet world:")),
    ("stalled",    re.compile(r"POSE FEED STALLED")),
    ("nofeed",     re.compile(r"NOFEED")),
    ("arbiter",    re.compile(r"pose feed is LYING")),
    ("healed",     re.compile(r"pose feed: restarted|resubscribed")),
    ("subproc",    re.compile(r"pose feed up \(per-run")),
    ("inproc",     re.compile(r"pose subscriber up")),
    ("discovery",  re.compile(r"gz-transport discovery failure")),
]
START = re.compile(r"Conductor on http://127\.0\.0\.1:")


def segments(path):
    segs, cur = [], []
    for line in open(path, errors="ignore"):
        if START.search(line):
            if cur:
                segs.append(cur)
            cur = []
        cur.append(line)
    if cur:
        segs.append(cur)
    return segs


def report(seg, label):
    counts = {name: 0 for name, _ in SIGNALS}
    for line in seg:
        for name, rx in SIGNALS:
            if rx.search(line):
                counts[name] += 1
    n = counts["launches"]
    mode = ("per-run SUBPROCESS" if counts["subproc"] else
            ("in-process Node" if counts["inproc"] else "unknown"))
    print("%s: %d launches, feed=%s" % (label, n, mode))
    print("   stalled=%d  nofeed=%d  self-heals=%d  discovery-failures=%d  "
          "arbiter-catches=%d" % (counts["stalled"], counts["nofeed"],
                                   counts["healed"], counts["discovery"],
                                   counts["arbiter"]))
    if n:
        bad = counts["stalled"] + counts["nofeed"]
        print("   feed trouble per launch: %.3f  (%d events / %d launches)"
              % (bad / n, bad, n))
    return counts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--all", action="store_true",
                    help="report every server lifetime, not just the current")
    ap.add_argument("--log", default=LOG)
    args = ap.parse_args()
    segs = segments(args.log)
    if not segs:
        print("no server segments in", args.log)
        return 1
    if args.all:
        for i, seg in enumerate(segs):
            report(seg, "server #%d" % (i + 1))
            print()
    else:
        report(segs[-1], "current server")
    return 0


if __name__ == "__main__":
    sys.exit(main())
