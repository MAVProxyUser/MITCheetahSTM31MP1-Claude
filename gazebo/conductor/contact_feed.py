#!/usr/bin/env python3
"""contact_feed.py - GROUND-TRUTH foot contact, for labelling only.

Subscribes to the four gz foot contact sensors and writes one JSON line per
sample:

    {"t": <time.time()>, "c": [FL, FR, RL, RR]}      # 1 = touching, 0 = not

WHY THIS IS A SEPARATE PROCESS, AND WHY IT MUST STAY ONE:
  The operator's Go1 EDU has no foot contact sensors - only the expensive
  variants do. So these readings exist for exactly one purpose: to LABEL what
  really touched the ground, so a SENSORLESS contact estimator (IMU + joint
  encoders, which the EDU dog does have) can be scored against truth rather
  than against the gait schedule - and the schedule is precisely what OPEN-26
  suspects, which makes it worthless as a label.

  Nothing in the control path may read this. A controller that works in sim
  because it reads a contact sensor is a controller that does not work on the
  real dog. Keeping it in its own process, writing to its own file, is what
  makes that violation impossible rather than merely discouraged - the same
  arrangement pose_feed.py uses for ground-truth pose (and the same reason
  OPEN-19/21 made those subprocesses: a process that exits takes its
  gz-transport state with it).

  Needs the gz env server.py sets on itself: GZ_PARTITION, GZ_IP, GZ_RELAY.
"""
import argparse, json, sys, time, threading

import gz.transport13 as transport
from gz.msgs10.contacts_pb2 import Contacts

LEGS = ("FL", "FR", "RL", "RR")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True)
    ap.add_argument("--model", default="go1_0")
    ap.add_argument("--rate", type=float, default=200.0,
                    help="max lines per second (default 200)")
    ap.add_argument("--stale", type=float, default=0.02,
                    help="a leg with no contact message for this long is "
                         "treated as NOT touching (default 0.02 s = 10 "
                         "sensor periods at the 500 Hz sensor rate)")
    ap.add_argument("--warmup", type=float, default=30.0,
                    help="seconds to wait for the first message before "
                         "declaring the subscription deaf")
    args = ap.parse_args()

    # PER-LEG staleness, and it is the whole trick. A gz contact sensor
    # publishes only WHEN something is touching it - a foot in swing sends
    # nothing at all. A first version latched each leg's last value, and read
    # "all four feet touching" on 99.9% of samples of a 3 m/s trot, which is
    # impossible. So absence of a message IS the signal: a leg with no contact
    # message for --stale seconds is not touching.
    state = {"c": [0, 0, 0, 0], "seen": [0.0, 0.0, 0.0, 0.0],
             "last_msg": 0.0, "count": 0, "last_emit": 0.0}
    lock = threading.Lock()
    min_dt = 0.9 / max(args.rate, 0.1)

    def make_cb(idx):
        def cb(msg):
            # A contact sensor publishes a Contacts message every update; the
            # foot is touching iff that message carries at least one contact.
            now = time.time()
            with lock:
                if len(msg.contact) > 0:
                    state["seen"][idx] = now
                state["last_msg"] = now
                state["count"] += 1
        return cb

    # EMIT ON A TIMER, NOT FROM THE CALLBACKS. Decaying and emitting inside a
    # callback means nothing is emitted during a FLIGHT PHASE - when no foot is
    # touching, no sensor publishes, no callback fires, and the last (contact)
    # sample is what the log keeps. That biases the labels toward contact
    # exactly when the interesting thing is happening. A timer samples the
    # world uniformly whether anything is touching or not.
    def emitter():
        while True:
            time.sleep(min_dt)
            now = time.time()
            with lock:
                c = [1 if (now - state["seen"][k]) <= args.stale else 0
                     for k in range(4)]
                state["c"] = c
            try:
                sys.stdout.write(json.dumps({"t": now, "c": c}) + "\n")
                sys.stdout.flush()
            except Exception:
                return
    threading.Thread(target=emitter, daemon=True).start()

    node = transport.Node()
    subscribed = 0
    for i, leg in enumerate(LEGS):
        topic = "/world/%s/model/%s/link/%s_calf/sensor/%s_foot_contact/contact" % (
            args.world, args.model, leg, leg)
        if node.subscribe(Contacts, topic, make_cb(i)):
            subscribed += 1
        else:
            sys.stderr.write("[contact_feed] subscribe FAILED: %s\n" % topic)
    sys.stderr.write("[contact_feed] subscribed to %d/4 foot sensors\n" % subscribed)
    if subscribed == 0:
        sys.stderr.write("[contact_feed] nothing subscribed - is SIM_FOOT_CONTACT "
                         "enabled in the world? (gazebo/tools/add_foot_contacts.py)\n")
        return 2

    t0 = time.time()
    while True:
        time.sleep(0.5)
        with lock:
            n, last = state["count"], state["last_msg"]
        if n == 0 and time.time() - t0 > args.warmup:
            sys.stderr.write("[contact_feed] SUBSCRIBED BUT USELESS: no contact "
                             "message in %.1fs. Exiting so the parent can retry.\n"
                             % (time.time() - t0))
            return 3


if __name__ == "__main__":
    sys.exit(main() or 0)
