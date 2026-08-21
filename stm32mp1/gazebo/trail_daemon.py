#!/usr/bin/env python3
"""Draw PLANNED vs FLOWN mission tracks, OpenPilot-style.

This is the Cheetah port of NinjaPilot's ground/gazebo_bridge/tools/trail_daemon.py,
and it keeps that file's hard-won details rather than reinventing them:

  * gz MARKERS, not spawned models. An earlier version of this script spawned a
    model per trail segment, which littered the entity tree with hundreds of
    "plan_leg_*" entries and sat under the farm's grass visual. Markers are
    scene-only: nothing to clean up, nothing in the entity tree.
  * LINE_STRIP renders as a 1-pixel line and is effectively invisible, so every
    segment is a CYLINDER (a "tube") spanning point A to B.
  * the /marker service replies with gz.msgs.Empty and frequently reports
    ok=False even when the marker registered - so the reply is ignored. Asking
    for a Boolean instead makes every call look like a failure and the trails
    silently never appear.
  * SEPARATE PROCESS. Each /marker call is a blocking service request; doing
    them inline stalls whatever loop they live in (in OpenPilot that cost a 9 Hz
    guidance loop). Here the bridge is what must never stall, so trails get
    their own process and their own core.
  * own namespace, so a DELETE_ALL from elsewhere cannot wipe these.

usage: trail_daemon.py <mission-spec> [duration_s]
       mission-spec: star:<radius>:<points> | circle:<radius>:<points> | outback:<m>
"""
import math
import sys
import time

import gz.transport13 as transport
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.pose_v_pb2 import Pose_V

WORLD = "go1_world"
MODEL = "go1"
NS = "cheetah_trail"
SEG_MIN = 0.30          # m between flown-trail segments
TRAIL_DIAMETER = 0.10
TRAIL_Z = 0.12          # above the grass, so the track is never buried in it

PLAN_RGBA = (1.00, 0.72, 0.00, 0.85)    # amber - the planned mission track
FLOWN_RGBA = (0.10, 0.75, 0.95, 0.85)   # cyan  - where the dog actually went
WP_RGBA = (0.15, 0.95, 0.25, 0.95)      # green - the waypoints themselves


def marker_base(mid, rgba):
    m = Marker()
    m.ns = NS
    m.id = mid
    m.action = Marker.ADD_MODIFY
    m.type = Marker.LINE_STRIP
    for tgt in (m.material.ambient, m.material.diffuse, m.material.emissive):
        tgt.r, tgt.g, tgt.b, tgt.a = rgba
    return m


def marker_send(node, m):
    # reply is Empty and often reports ok=False even on success - ignore it
    try:
        node.request("/marker", m, type(m), Empty, 60)
    except Exception:
        pass


def marker_tube(mid, a, b, rgba, diameter=TRAIL_DIAMETER):
    """Cylinder spanning a->b (gz cylinders are Z-axis, so rotate +Z onto the
    segment direction)."""
    dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    m = marker_base(mid, rgba)
    m.type = Marker.CYLINDER
    m.scale.x = m.scale.y = diameter
    m.scale.z = max(length, 0.01)
    m.pose.position.x = (a[0] + b[0]) / 2.0
    m.pose.position.y = (a[1] + b[1]) / 2.0
    m.pose.position.z = (a[2] + b[2]) / 2.0
    if length > 1e-6:
        ux, uy, uz = dx / length, dy / length, dz / length
        if uz > 0.99999:
            q = (1.0, 0.0, 0.0, 0.0)
        elif uz < -0.99999:
            q = (0.0, 1.0, 0.0, 0.0)
        else:
            ax, ay = -uy, ux
            n = math.sqrt(ax * ax + ay * ay)
            ax, ay = ax / n, ay / n
            half = math.acos(uz) / 2.0
            s = math.sin(half)
            q = (math.cos(half), ax * s, ay * s, 0.0)
        (m.pose.orientation.w, m.pose.orientation.x,
         m.pose.orientation.y, m.pose.orientation.z) = q
    return m


def marker_sphere(mid, p, rgba, d=0.34):
    m = marker_base(mid, rgba)
    m.type = Marker.SPHERE
    m.scale.x = m.scale.y = m.scale.z = d
    m.pose.position.x, m.pose.position.y, m.pose.position.z = p
    return m


def marker_clear(node):
    m = Marker()
    m.ns = NS
    m.action = Marker.DELETE_ALL
    marker_send(node, m)


def mission_waypoints(spec):
    """Mirrors WaypointNav.cpp exactly. Returns [(north, east), ...]."""
    kind, *rest = spec.split(":")
    if kind == "star":
        r, n = float(rest[0]), int(rest[1])
        step = 2 if n % 2 == 1 else 1
        out = []
        for i in range(n):
            v = ((i + 1) * step) % n
            a = 2 * math.pi * v / n
            out.append((r * math.cos(a), r * math.sin(a)))
        return out
    if kind == "circle":
        r, n = float(rest[0]), int(rest[1])
        return [(r * math.sin(2 * math.pi * (i + 1) / n),
                 r * (1 - math.cos(2 * math.pi * (i + 1) / n))) for i in range(n)]
    if kind == "outback":
        d = float(rest[0])
        return [(d, 0.0), (0.0, 0.0)]
    raise SystemExit(f"unknown mission spec: {spec}")


def main():
    spec = sys.argv[1] if len(sys.argv) > 1 else "star:5:5"
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 600.0

    node = transport.Node()
    marker_clear(node)

    # ---- planned track: amber tubes, green waypoint spheres ----
    # world is ENU: world x = east, world y = north
    wps = mission_waypoints(spec)
    mid = 1
    prev = (0.0, 0.0)
    for i, (n, e) in enumerate(wps):
        a = (prev[1], prev[0], TRAIL_Z)
        b = (e, n, TRAIL_Z)
        marker_send(node, marker_tube(mid, a, b, PLAN_RGBA, 0.07)); mid += 1
        marker_send(node, marker_sphere(mid, (e, n, TRAIL_Z), WP_RGBA)); mid += 1
        prev = (n, e)
    print(f"[trail] planned track drawn: {len(wps)} waypoints ({spec})", flush=True)

    # ---- flown track: cyan tubes, appended as the dog moves ----
    latest = [None]

    def cb(msg: Pose_V):
        for p in msg.pose:
            if p.name == MODEL:
                latest[0] = (p.position.x, p.position.y)

    node.subscribe(Pose_V, f"/world/{WORLD}/dynamic_pose/info", cb)

    flown_id = 10000
    last = None
    t0 = time.time()
    while time.time() - t0 < duration:
        time.sleep(0.05)
        p = latest[0]
        if p is None:
            continue
        if last is None:
            last = p
            continue
        if math.dist(last, p) < SEG_MIN:
            continue
        marker_send(node, marker_tube(flown_id, (last[0], last[1], TRAIL_Z),
                                      (p[0], p[1], TRAIL_Z), FLOWN_RGBA))
        flown_id += 1
        last = p
    print(f"[trail] flown track: {flown_id - 10000} segments", flush=True)


if __name__ == "__main__":
    main()
