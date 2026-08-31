#!/usr/bin/env python3
"""cam_feed - every camera subscription for one run, in its OWN process.

OPEN-19, root fix - and the same shape as pose_feed.py's OPEN-21 fix, for
the same reason. `server.py` used to create one `gz.transport13.Node()`
per camera per launch and "release" them at teardown with
`self._gz_cam_nodes = []`. Dropping a Python reference is a HOPE, not a
teardown: gz-transport's discovery/reception threads are C++ and do not
unwind on refcount-zero, and nothing ever called unsubscribe. Measured
consequence on a 12-hour-old conductor (2026-08-31):

    fresh server:   4 threads, /api/state TTFB 0.0005-0.0019 s
    after 12 h:    54 threads, /api/state TTFB 4.66-4.90 s

...a ~2500x regression that grows about +1.2 threads per launch, with the
main thread spending ~20% of its samples in take_gil. That is not merely a
choppy panel: campaign c3 lost 18 of its 60 launches to "(no verdict)",
clustered at the END of each block, because the harness's own polling of a
GIL-saturated /api/state timed out. A leak in the video path was
corrupting mission data.

A subscription cannot outlive a process that has exited, so this gets a
process that exits. It also moves the JPEG encode OUT of the server: at
3 dogs x 3 cameras x 10 Hz that was 90 PIL encodes/second competing for
the server's GIL with every HTTP request.

Contract - binary framing on stdout, one frame per record:

    {"i":0,"c":"chase_cam","n":18234,"w":480,"h":270,"t":1756...}\\n
    <exactly n bytes of JPEG>

stdin (optional, one JSON object per line) carries the live mute set, so
unchecking a camera in the panel still skips the ENCODE rather than just
hiding the tile:

    {"mute": ["0:front_cam", "1:nadir_cam"]}
"""
import argparse
import io
import json
import sys
import threading
import time

import gz.transport13 as transport
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.boolean_pb2 import Boolean as GzBoolean
from PIL import Image as PILImage


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cams", required=True,
                    help="comma list of index:sensor, e.g. 0:chase_cam,1:front_cam")
    ap.add_argument("--quality", type=int, default=60,
                    help="JPEG quality (default 60, matches the old in-server encode)")
    ap.add_argument("--rate", type=float, default=30.0,
                    help="max frames/sec PER CAMERA (default 30; the sensor's "
                         "own update_rate is the real ceiling)")
    ap.add_argument("--max-width", type=int, default=0,
                    help="downscale frames wider than this (0 = never scale)")
    ap.add_argument("--warmup", type=float, default=12.0,
                    help="seconds to wait for the FIRST frame on ANY camera "
                         "before declaring the subscriptions deaf")
    ap.add_argument("--set-pose-service", default="",
                    help="gz service for chase-camera following, e.g. "
                         "/world/go1_world/set_pose. When given, this process "
                         "also owns the follow requests - see the module "
                         "docstring on why the server must not.")
    ap.add_argument("--follow-dt", type=float, default=0.1,
                    help="seconds between chase-camera set_pose requests")
    ap.add_argument("--deaf-after", type=float, default=0.0,
                    help="seconds with no frame on any camera mid-run before "
                         "exiting so the parent restarts us (0 = never exit; "
                         "default, because a muted fleet is legitimately silent)")
    args = ap.parse_args()

    cams = []
    for pair in args.cams.split(","):
        if not pair.strip():
            continue
        i, _, name = pair.partition(":")
        cams.append((int(i), name.strip()))
    if not cams:
        sys.stderr.write("[cam_feed] no cameras requested\n")
        raise SystemExit(2)

    min_dt = 1.0 / max(args.rate, 0.1)
    out = sys.stdout.buffer
    wlock = threading.Lock()          # serialises whole frames onto stdout
    state = dict(count=0, last=0.0)
    muted = set()
    last_emit = {}
    want = {}                      # chase-cam name -> latest desired pose
    pose_lock = threading.Lock()

    def on_image(msg, idx, camname):
        key = "%d:%s" % (idx, camname)
        now = time.time()
        # Mute gate BEFORE the encode - unchecking a camera in the panel
        # must stop the work, not just hide the tile (the in-server version
        # gated here too, and that property is worth keeping).
        if key in muted:
            return
        if now - last_emit.get(key, 0.0) < min_dt:
            return
        try:
            img = PILImage.frombytes("RGB", (msg.width, msg.height), msg.data)
            if args.max_width and img.width > args.max_width:
                h = int(round(img.height * args.max_width / float(img.width)))
                img = img.resize((args.max_width, h), PILImage.BILINEAR)
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=args.quality)
            payload = buf.getvalue()
        except Exception as e:  # noqa: BLE001 - one bad frame must not kill the feed
            sys.stderr.write("[cam_feed] %s encode error: %r\n" % (key, e))
            sys.stderr.flush()
            return
        hdr = json.dumps({"i": idx, "c": camname, "n": len(payload),
                          "w": img.width, "h": img.height,
                          "t": round(now, 3)}).encode("ascii")
        last_emit[key] = now
        state["count"] += 1
        state["last"] = now
        try:
            with wlock:
                out.write(hdr + b"\n")
                out.write(payload)
                out.flush()
        except (BrokenPipeError, ValueError):
            # server went away; nothing left to feed
            raise SystemExit(0)

    node = transport.Node()
    ok_any = False
    for idx, camname in cams:
        topic = "/go1_%d/%s" % (idx, camname)
        ok = node.subscribe(
            GzImage, topic,
            lambda m, i=idx, c=camname: on_image(m, i, c))
        ok_any = ok_any or ok
        sys.stderr.write("[cam_feed] subscribe %s -> %s\n"
                         % (topic, "ok" if ok else "FAILED"))
    sys.stderr.flush()
    if not ok_any:
        raise SystemExit(2)

    # Live mute updates from the parent. Same lesson as pose_feed: a
    # subscription that reports "ok" and then delivers nothing is the real
    # failure mode, so the warmup check below is not optional.
    def stdin_reader():
        for line in sys.stdin:
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            if "mute" in msg:
                muted.clear()
                muted.update(msg["mute"] or [])
            if "setpose" in msg:
                # LATEST WINS, never a queue - same rule as the frames. The
                # reader must not issue the request itself: node.request()
                # is blocking (no async variant in this gz.transport13
                # build), so a stalled service call would stop us reading
                # stdin and back the server's own follow loop up behind it.
                with pose_lock:
                    for item in (msg["setpose"] or []):
                        want[item["name"]] = item
    threading.Thread(target=stdin_reader, daemon=True).start()

    if args.set_pose_service:
        def follower():
            """Chase-camera following, in THIS process rather than the
            server's. The server keeps every line of the geometry (it owns
            the poses and the draft slots); all that moves here is the
            blocking gz call, because a `gz.transport13.Node()` held in the
            long-lived server is precisely the leak OPEN-19 was about. The
            server's Node for this was created once per run and never
            released - measured at +0.56 threads/run AFTER the camera
            subscriptions had already been moved out, which is how it was
            found: the leak canary kept firing on a server that was
            supposed to be clean."""
            while True:
                with pose_lock:
                    batch = list(want.values())
                for item in batch:
                    req = GzPose()
                    req.name = item["name"]
                    req.position.x, req.position.y, req.position.z = item["p"]
                    (req.orientation.w, req.orientation.x,
                     req.orientation.y, req.orientation.z) = item["q"]
                    try:
                        node.request(args.set_pose_service, req, GzPose,
                                     GzBoolean, 100)
                    except Exception:  # noqa: BLE001 - one missed tick is invisible
                        pass
                time.sleep(args.follow_dt)
        threading.Thread(target=follower, daemon=True).start()
        sys.stderr.write("[cam_feed] chase follower on %s every %.3fs\n"
                         % (args.set_pose_service, args.follow_dt))
        sys.stderr.flush()

    deadline = time.time() + args.warmup
    while state["count"] == 0 and time.time() < deadline:
        time.sleep(0.2)
    if state["count"] == 0:
        sys.stderr.write("[cam_feed] SUBSCRIBED BUT USELESS: no frame on any "
                         "of [%s] in %.1fs. Exiting so the parent can retry "
                         "on a fresh process.\n" % (args.cams, args.warmup))
        sys.stderr.flush()
        raise SystemExit(3)
    sys.stderr.write("[cam_feed] receiving (%d frames in warmup)\n" % state["count"])
    sys.stderr.flush()

    while True:
        time.sleep(1.0)
        if args.deaf_after > 0:
            gap = time.time() - state["last"]
            if gap > args.deaf_after:
                sys.stderr.write("[cam_feed] WENT DEAF: %.1fs since the last "
                                 "frame - exiting.\n" % gap)
                sys.stderr.flush()
                raise SystemExit(4)


if __name__ == "__main__":
    main()
