#!/usr/bin/env python3
"""
record_video.py - headless video capture from the Go1's chase camera.

Subscribes to the gz camera sensor topic (/chase_cam, declared on the trunk in
worlds/go1_farm.sdf) and writes frames straight into ffmpeg. This replaces the
old /gui/screenshot polling loop, which popped a "Saved image to:" toast per
frame and stalled the GUI - and needed a GUI running at all.

  record_video.py <out.mp4> <seconds> [topic]

Needs ffmpeg on PATH and the gz python bindings (use the OpenPilot venv).
"""
import subprocess
import sys
import threading
import time

import gz.transport13 as transport
from gz.msgs10.image_pb2 import Image

out_path = sys.argv[1] if len(sys.argv) > 1 else "out.mp4"
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
topic = sys.argv[3] if len(sys.argv) > 3 else "/chase_cam"

FPS = 20
state = {"proc": None, "w": 0, "h": 0, "n": 0, "lock": threading.Lock()}


def on_image(msg: Image):
    with state["lock"]:
        if state["proc"] is None:
            state["w"], state["h"] = msg.width, msg.height
            state["proc"] = subprocess.Popen(
                ["ffmpeg", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                 "-s", f"{msg.width}x{msg.height}", "-r", str(FPS), "-i", "-",
                 "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23",
                 out_path],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f"[rec] {msg.width}x{msg.height} -> {out_path}", flush=True)
        try:
            state["proc"].stdin.write(msg.data)
            state["n"] += 1
        except (BrokenPipeError, ValueError):
            pass


def main():
    node = transport.Node()
    if not node.subscribe(Image, topic, on_image):
        print(f"[rec] subscribe failed: {topic}", flush=True)
        sys.exit(1)
    print(f"[rec] recording {topic} for {duration:.0f}s", flush=True)
    time.sleep(duration)
    with state["lock"]:
        p = state["proc"]
        n = state["n"]
    if p:
        p.stdin.close()
        p.wait(timeout=30)
        print(f"[rec] wrote {n} frames to {out_path}", flush=True)
    else:
        print("[rec] NO FRAMES - is the sensors system running and the world loaded?",
              flush=True)
        sys.exit(2)


if __name__ == "__main__":
    main()
