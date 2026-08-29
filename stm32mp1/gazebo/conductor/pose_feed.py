#!/usr/bin/env python3
"""pose_feed - the fleet's world-pose subscription, in its OWN process.

OPEN-21, root fix. The conductor used to subscribe to
`/world/<world>/dynamic_pose/info` from inside `server.py` on a
`gz.transport13.Node` it kept alive for the life of the fleet. Measured
consequence: the feed DEGRADES as launches accumulate inside one server
process - partial trails first, then a dead feed - and the in-process
self-heal (drop the Node, make a fresh one) is only ever transient. By
~20-25 launches it stays dead, and a dead feed does not merely lose the
drawn trail: every world-motion instrument on the panel reads it, so a
healthy dog gets accused of hallucinating (see the bridge-GPS arbiter).

A subscription cannot outlive a process that has exited, so the fix is to
give it a process that exits: this one is started per RUN and dies with
it, taking every scrap of gz-transport discovery state, socket and
subscriber bookkeeping with it. The server's own long-lived process never
touches gz-transport again.

Contract - deliberately dumb, so the server's downstream logic (trail
decimation, speed EMA, the feed heartbeat) stays EXACTLY as it was and
only its SOURCE changes:

    stdout, one JSON object per line, flushed:
      {"t": <float, time.time()>, "p": {"<index>": [x, y, z, yaw], ...}}

Poses are emitted at most --rate times a second (gz publishes far faster
than any panel needs), and a line is emitted only when at least one known
model is present in the message.
"""
import argparse
import json
import math
import sys
import time

import gz.transport13 as transport
from gz.msgs10.pose_v_pb2 import Pose_V


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True)
    ap.add_argument("--names", required=True,
                    help="comma list of model=index, e.g. go1_0=0,go1_1=1")
    ap.add_argument("--rate", type=float, default=20.0,
                    help="max lines per second (default 20)")
    args = ap.parse_args()

    name_to_index = {}
    for pair in args.names.split(","):
        if not pair.strip():
            continue
        n, _, i = pair.partition("=")
        name_to_index[n.strip()] = int(i)

    min_dt = 1.0 / max(args.rate, 0.1)
    state = dict(last_emit=0.0)

    def on_pose(msg):
        now = time.time()
        if now - state["last_emit"] < min_dt:
            return
        out = {}
        for p in msg.pose:
            idx = name_to_index.get(p.name)
            if idx is None:
                continue
            o = p.orientation
            yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                             1.0 - 2.0 * (o.y * o.y + o.z * o.z))
            out[str(idx)] = [round(p.position.x, 4), round(p.position.y, 4),
                             round(p.position.z, 4), round(yaw, 4)]
        if not out:
            return
        state["last_emit"] = now
        try:
            sys.stdout.write(json.dumps({"t": now, "p": out}) + "\n")
            sys.stdout.flush()
        except (BrokenPipeError, ValueError):
            # server went away; nothing left to feed
            raise SystemExit(0)

    node = transport.Node()
    topic = "/world/%s/dynamic_pose/info" % args.world
    ok = node.subscribe(Pose_V, topic, on_pose)
    # One status line on STDERR so it lands in the run's log without ever
    # being mistaken for a pose record on stdout.
    sys.stderr.write("[pose_feed] subscribe %s -> %s\n"
                     % (topic, "ok" if ok else "FAILED"))
    sys.stderr.flush()
    if not ok:
        raise SystemExit(2)
    # Nothing else to do: gz-transport delivers on its own thread. Park.
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    main()
