#!/usr/bin/env python3
"""
mission_viz.py - draw the PLANNED mission track and the ACTUAL ground track.

Both are spawned as real world entities (not GUI markers), so they show up in
the GUI *and* in camera-sensor recordings - GUI markers only exist client-side
and would be missing from the MP4.

  mission_viz.py star:5:5      [duration_s]
  mission_viz.py circle:3:8    [duration_s]

  green posts + green ribbon = planned waypoints and legs
  red breadcrumbs             = where the robot actually went

The waypoint math mirrors WaypointNav.cpp exactly (same "+1 vertex" offset), so
what you see is what the controller is chasing.
"""
import math
import os
import subprocess
import sys
import time

import gz.transport13 as transport
from gz.msgs10.pose_v_pb2 import Pose_V

WORLD = "go1_world"
MODEL = "go1"


def spawn(name, x, y, z, sx, sy, sz, rgba, box=True):
    """Spawn a static, collisionless marker so it cannot disturb the robot."""
    geom = (f"<box><size>{sx} {sy} {sz}</size></box>" if box
            else f"<sphere><radius>{sx}</radius></sphere>")
    r, g, b, a = rgba
    sdf = (f"<?xml version='1.0'?><sdf version='1.9'><model name='{name}'>"
           f"<static>true</static><pose>{x} {y} {z} 0 0 0</pose><link name='l'>"
           f"<visual name='v'><geometry>{geom}</geometry><material>"
           f"<ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse>"
           f"<emissive>{r*0.6} {g*0.6} {b*0.6} 1</emissive>"
           f"</material></visual></link></model></sdf>")
    subprocess.run(
        ["gz", "service", "-s", f"/world/{WORLD}/create",
         "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
         "--timeout", "2000", "--req", f"sdf: \"{sdf}\""],
        capture_output=True)


# Shared with trail_daemon.py and mission_runner.py's report generator -
# see mission_geometry.py's docstring. NOTE: this file's own former copy
# of mission_waypoints() was missing the atom and oval cases entirely
# (would have raised "unknown mission spec" if anyone ever ran a viz on
# either) - the shared version fixes that gap as a side effect of
# de-duplicating.
from mission_geometry import mission_waypoints


def draw_planned(wps):
    """Posts at each waypoint, plus a flat ribbon along each leg.
    Gazebo world is ENU: world x = east, world y = north."""
    for i, (n, e) in enumerate(wps):
        spawn(f"plan_wp_{i}", e, n, 0.5, 0.09, 0.09, 1.0, (0.1, 0.9, 0.2, 1))
    prev = (0.0, 0.0)                      # the robot starts at the local origin
    for i, (n, e) in enumerate(wps):
        pn, pe = prev
        dn, de = n - pn, e - pe
        length = math.hypot(dn, de)
        if length > 0.05:
            # one thin slab per leg, laid on the ground, split into segments so
            # it stays axis-aligned (no rotation needed in the spawn helper)
            steps = max(2, int(length / 0.4))
            for s in range(steps):
                f = (s + 0.5) / steps
                spawn(f"plan_leg_{i}_{s}", pe + de * f, pn + dn * f, 0.02,
                      0.12, 0.12, 0.02, (0.1, 0.75, 0.2, 1))
        prev = (n, e)
    print(f"[viz] planned track drawn: {len(wps)} waypoints", flush=True)


def main():
    spec = sys.argv[1] if len(sys.argv) > 1 else "star:5:5"
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 600.0

    wps = mission_waypoints(spec)
    draw_planned(wps)

    node = transport.Node()
    latest = [None]

    def cb(msg: Pose_V):
        for p in msg.pose:
            if p.name == MODEL:
                latest[0] = (p.position.x, p.position.y)

    node.subscribe(Pose_V, f"/world/{WORLD}/dynamic_pose/info", cb)

    t0 = time.time()
    last = None
    n = 0
    while time.time() - t0 < duration:
        time.sleep(0.25)
        if latest[0] is None:
            continue
        x, y = latest[0]
        if last is None or math.hypot(x - last[0], y - last[1]) > 0.30:
            spawn(f"trail_{n}", x, y, 0.03, 0.05, 0, 0, (0.95, 0.15, 0.1, 1), box=False)
            last = (x, y)
            n += 1
    print(f"[viz] actual track: {n} breadcrumbs", flush=True)


if __name__ == "__main__":
    main()
