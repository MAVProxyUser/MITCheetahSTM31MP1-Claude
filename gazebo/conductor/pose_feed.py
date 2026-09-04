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
    ap.add_argument("--warmup", type=float, default=8.0,
                    help="seconds to wait for the FIRST message before "
                         "declaring the subscription deaf (default 8)")
    ap.add_argument("--deaf-after", type=float, default=10.0,
                    help="seconds without a message mid-run before exiting "
                         "so the parent restarts the feed (default 10)")
    args = ap.parse_args()

    name_to_index = {}
    for pair in args.names.split(","):
        if not pair.strip():
            continue
        n, _, i = pair.partition("=")
        name_to_index[n.strip()] = int(i)

    min_dt = 1.0 / max(args.rate, 0.1)
    state = dict(last_emit=0.0, last_msg=0.0, last_raw=0.0, count=0)

    def on_pose(msg):
        now = time.time()
        # COUNT MATCHED POSES, NOT RAW MESSAGES. Counting messages was
        # wrong and the data caught it: run1081 logged "receiving (13 msgs
        # in warmup)" and still produced an EMPTY trail, because
        # /dynamic_pose/info carries whatever moved - a message can arrive
        # with not one of OUR models in it. A liveness check that passes on
        # traffic we cannot use is worse than none: it reports healthy while
        # the panel has nothing. So the counter below moves only when a
        # message actually contains a model we placed.
        state["last_raw"] = now
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
            # ROLL AND PITCH, appended 2026-09-04 for OPEN-26.
            #
            # This rig has never had ground truth for attitude - only yaw was
            # derived, because only yaw was needed for the panel's heading.
            # OPEN-26 now needs the other two: the estimator's height and the
            # detector's kin_z both run ~1.8x fast during a fold WITHOUT
            # tracking each other, and the one input they share is the
            # estimated ORIENTATION (kin_z is -min over legs of
            # (rBody^T (hip + p))[2]; the position block integrates in the
            # world frame that same attitude defines). A tilted attitude
            # estimate shrinks every derived height in both signals at once.
            # There was no way to test that without truth for roll/pitch.
            #
            # APPENDED, never inserted: every existing consumer indexes this
            # list positionally (p[2] is z, p[3] is yaw), so adding at the end
            # cannot break one. Standard ZYX quaternion -> Euler; pitch is
            # clamped because asin() of a value a hair outside [-1,1] from
            # rounding throws, and a robot at +-90 deg pitch is a fall, not a
            # measurement to lose.
            sinr = 2.0 * (o.w * o.x + o.y * o.z)
            cosr = 1.0 - 2.0 * (o.x * o.x + o.y * o.y)
            roll = math.atan2(sinr, cosr)
            sinp = 2.0 * (o.w * o.y - o.z * o.x)
            sinp = 1.0 if sinp > 1.0 else (-1.0 if sinp < -1.0 else sinp)
            pitch = math.asin(sinp)
            out[str(idx)] = [round(p.position.x, 4), round(p.position.y, 4),
                             round(p.position.z, 4), round(yaw, 4),
                             round(roll, 4), round(pitch, 4)]
        if not out:
            return
        state["last_msg"] = now      # a pose WE can use
        state["count"] += 1
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

    # A SUBSCRIPTION THAT SAYS "ok" AND THEN DELIVERS NOTHING IS THE ACTUAL
    # FAILURE. Measured 2026-08-29 on this very process: `subscribe -> ok`
    # in a brand-new process with a fresh Node, and zero messages ever
    # arrived - the run's trail was 0.0 m of a 71.2 m plan while bridge GPS
    # showed the whole course flown. So moving the subscription into a
    # per-run process was necessary and NOT sufficient: the failure is not
    # accumulated in-process state, it is gz-transport's discovery handshake
    # silently not connecting this subscriber to the publisher - very likely
    # the same underlying defect as OPEN-22, which presents as "no topics
    # visible" instead of "subscribed but deaf".
    #
    # It cannot be fixed here, but it CAN be detected here, which is what
    # turns a silent failure into a retried one: if nothing arrives within
    # --warmup seconds of a successful subscribe, say so and exit non-zero
    # so the parent can restart us on a fresh process. Same afterwards - a
    # feed that goes deaf mid-run exits rather than pretending.
    warm_deadline = time.time() + args.warmup
    while state["count"] == 0 and time.time() < warm_deadline:
        time.sleep(0.2)
    if state["count"] == 0:
        raw = "some traffic, but none of our models" if state["last_raw"] \
              else "no traffic at all"
        sys.stderr.write("[pose_feed] SUBSCRIBED BUT USELESS: no pose for any "
                         "of [%s] in %.1fs on %s (%s). Exiting so the parent "
                         "can retry on a fresh process.\n"
                         % (args.names, args.warmup, topic, raw))
        sys.stderr.flush()
        raise SystemExit(3)
    sys.stderr.write("[pose_feed] receiving (%d usable poses in warmup)\n"
                     % state["count"])
    sys.stderr.flush()

    # Alive. Watch for going deaf mid-run, and exit if it happens.
    while True:
        time.sleep(1.0)
        gap = time.time() - state["last_msg"]
        if gap > args.deaf_after:
            sys.stderr.write("[pose_feed] WENT DEAF: %.1fs since the last "
                             "message - exiting so the parent can restart "
                             "the feed.\n" % gap)
            sys.stderr.flush()
            raise SystemExit(4)


if __name__ == "__main__":
    main()
