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


def mission_waypoints(spec):
    """Same geometry as WaypointNav.cpp."""
    kind, *rest = spec.split(":")
    if kind == "star":
        r = float(rest[0]); n = int(rest[1])
        step = 2 if n % 2 == 1 else 1
        wps = []
        for i in range(n):
            v = ((i + 1) * step) % n
            a = 2 * math.pi * v / n
            wps.append((r * math.cos(a), r * math.sin(a)))   # (north, east)
        return wps
    if kind == "circle":
        r = float(rest[0]); n = int(rest[1])
        return [(r * math.sin(2 * math.pi * (i + 1) / n),
                 r * (1 - math.cos(2 * math.pi * (i + 1) / n))) for i in range(n)]
    if kind == "outback":
        d = float(rest[0])
        return [(d, 0.0), (0.0, 0.0)]
    if kind == "dash":
        d = float(rest[0])
        return [(d, 0.0)]
    if kind == "sector":
        leg = float(rest[0])
        reps = int(rest[1]) if len(rest) > 1 else 3
        offset = math.pi / 9
        bearing = 0.0
        n = e = 0.0
        out = []
        for r in range(reps):
            for k in range(1, 7):
                D = leg if (k - 1) % 2 == 0 else 0.5 * leg
                n += D * math.cos(bearing)
                e += D * math.sin(bearing)
                # Skip the cycle-closing waypoint except on the true last
                # leg - see WaypointNav.cpp's makeSectorSearch for why
                # (every intermediate cycle returns to the exact same
                # physical point, which confuses the follower).
                if k != 6 or r == reps - 1:
                    out.append((n, e))
                bearing += 2 * math.pi / 3
            bearing += offset
        return out
    if kind == "parallel":
        width = float(rest[0])
        height = float(rest[1]) if len(rest) > 1 else 5.0
        passes = int(rest[2]) if len(rest) > 2 else 6
        n = e = 0.0
        north = True
        out = []
        for p in range(passes):
            n += width if north else -width
            out.append((n, e))
            if p + 1 < passes:
                e += height
                out.append((n, e))
            north = not north
        return out
    if kind == "expsquare":
        step = float(rest[0])
        legs = int(rest[1]) if len(rest) > 1 else 12
        bearing = 0.0
        n = e = 0.0
        out = []
        for k in range(1, legs + 1):
            length = step * ((k - 1) // 2)
            if length > 1e-3:
                n += length * math.cos(bearing)
                e += length * math.sin(bearing)
                out.append((n, e))
            bearing += math.pi / 2
        return out
    if kind == "lissajous":
        A = float(rest[0])
        wx = int(rest[1]) if len(rest) > 1 else 1
        wy = int(rest[2]) if len(rest) > 2 else 2
        step = float(os.environ.get("WP_LISS_DS", 1.5))
        px = lambda t: A * math.sin(wx * t + math.pi / 2)
        py = lambda t: A * math.sin(wy * t)
        SUB = 20000
        dt = 2 * math.pi / SUB
        out = [(px(0.0), py(0.0))]
        pn, pe = out[0]
        acc = 0.0
        for i in range(1, SUB + 1):
            t = dt * i
            n, e = px(t), py(t)
            acc += math.hypot(n - pn, e - pe)
            pn, pe = n, e
            if acc >= step:
                acc -= step
                out.append((n, e))
        out.append((px(0.0), py(0.0)))
        return out
    raise SystemExit(f"unknown mission spec: {spec}")


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
