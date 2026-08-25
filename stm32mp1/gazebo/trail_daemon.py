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

MULTI-DOG. With several dogs in one engine each needs its own model name,
its own marker namespace (so one dog's DELETE_ALL cannot wipe another's), its
own colour, and its planned track offset to ITS lane - the mission is drawn in
the dog's local frame, but the dog is spawned at y = index * spacing, so an
un-offset plan would sit under dog 0 for everybody.

usage: trail_daemon.py <mission-spec> [duration_s] [dog_index] [lane_spacing_m]
       mission-spec: star:<r>:<n> | atom:<R>:<lobes> | oval:<straight>:<R>
                   | circle:<r>:<n> | outback:<m>
"""
import math
import sys
import time

import gz.transport13 as transport
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.pose_v_pb2 import Pose_V

import os
WORLD = os.environ.get("SIM_WORLD", "go1_world")
MODEL = os.environ.get("SIM_MODEL", "go1")
NS = "cheetah_trail_" + MODEL          # per-dog namespace
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
        # rotated so wp00 is due north - MUST match WaypointNav::makeStar
        a0 = 2 * math.pi * (step % n) / n
        out = []
        for i in range(n):
            v = ((i + 1) * step) % n
            a = 2 * math.pi * v / n - a0
            out.append((r * math.cos(a), r * math.sin(a)))
        return out
    if kind == "circle":
        r, n = float(rest[0]), int(rest[1])
        return [(r * math.sin(2 * math.pi * (i + 1) / n),
                 r * (1 - math.cos(2 * math.pi * (i + 1) / n))) for i in range(n)]
    if kind == "outback":
        d = float(rest[0])
        return [(d, 0.0), (0.0, 0.0)]
    if kind == "dash":
        # Mirrors WaypointNav::makeDash - a single straight leg due north,
        # ending AT that waypoint (no return leg, unlike outback).
        d = float(rest[0])
        return [(d, 0.0)]
    if kind == "atom":
        # Mirrors WaypointNav::makeAtom, including the tangential entry: the
        # curve is JOINED where its tangent is radial (roots of p x v = 0 with
        # p . v > 0), rotated so that point lies due north. Drawing the plan
        # from t=0 instead would put the whole track at the wrong phase.
        R = float(rest[0]); lobes = int(rest[1]) if len(rest) > 1 else 6
        A = float(os.environ.get("WP_ATOM_DEPTH", 0.8))
        k = lobes - 1; S = R / (1.0 + A)
        rx = lambda t: S * (math.cos(t) + A * math.cos(k * t))
        ry = lambda t: S * (math.sin(t) - A * math.sin(k * t))
        vx = lambda t: S * (-math.sin(t) - k * A * math.sin(k * t))
        vy = lambda t: S * (math.cos(t) - k * A * math.cos(k * t))
        t0, prev = 0.0, rx(0) * vy(0) - ry(0) * vx(0)
        for i in range(1, 20001):
            t = 2 * math.pi * i / 20000
            cr = rx(t) * vy(t) - ry(t) * vx(t)
            if (cr < 0) != (prev < 0) and rx(t) * vx(t) + ry(t) * vy(t) > 0:
                t0 = t; break
            prev = cr
        rot = -math.atan2(ry(t0), rx(t0))
        c, sr = math.cos(rot), math.sin(rot)
        px = lambda t: rx(t) * c - ry(t) * sr
        py = lambda t: rx(t) * sr + ry(t) * c
        step = float(os.environ.get("WP_ATOM_DS", 1.2))
        out, acc = [(px(t0), py(t0))], 0.0
        pn, pe = px(t0), py(t0)
        for i in range(1, 40001):
            t = t0 + 2 * math.pi * i / 40000
            n, e = px(t), py(t)
            acc += math.hypot(n - pn, e - pe); pn, pe = n, e
            if acc >= step:
                acc -= step; out.append((n, e))
        return out
    if kind == "oval":
        # Mirrors WaypointNav::makeOval: north straight, right 180, south
        # straight, right 180 back to the start.
        S = float(rest[0]); R = float(rest[1]) if len(rest) > 1 else 3.0
        arc = math.pi * R; total = 2 * S + 2 * arc
        step = float(os.environ.get("WP_OVAL_DS", 1.2))
        out = []
        d = step
        while d <= total:
            if d <= S:                      out.append((d, 0.0))
            elif d <= S + arc:
                a = (d - S) / R;            out.append((S + R * math.sin(a), R - R * math.cos(a)))
            elif d <= 2 * S + arc:          out.append((S - (d - S - arc), 2 * R))
            else:
                a = (d - 2 * S - arc) / R;  out.append((-R * math.sin(a), R + R * math.cos(a)))
            d += step
        if not out or math.hypot(out[-1][0], out[-1][1]) > 0.6 * step:
            out.append((0.0, 0.0))
        return out
    if kind == "sector":
        # Mirrors WaypointNav::makeSectorSearch: alternating full/half legs
        # at 120 deg, bearing 0 = north, (north,east) = (cos,sin)(bearing).
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
        # Mirrors WaypointNav::makeParallelTrack: sweep runs north/south,
        # step runs east, first pass heads north.
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
        # Mirrors WaypointNav::makeExpandingSquare: outward spiral, 90 deg
        # turns, leg k has length step*floor((k-1)/2) (k=1,2 are zero-length
        # and skipped).
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
        # Mirrors WaypointNav::makeLissajous: X=A*sin(wx*t+pi/2) (north),
        # Y=A*sin(wy*t) (east), t in [0, 2*pi) - integer wx,wy close the
        # curve in exactly one sweep (see the C++ comment for why NOT
        # 2*pi*lcm(wx,wy), a mistake caught before it shipped).
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


# One hue per dog, so three tracks side by side are readable at a glance.
# Planned is the dimmer shade, flown the brighter, in the SAME hue - the eye
# then reads "how far is bright from dim" as the error, per dog, without
# having to remember a legend.
DOG_HUES = [
    ((1.00, 0.55, 0.10, 0.55), (1.00, 0.75, 0.25, 1.0)),   # amber
    ((0.20, 0.55, 1.00, 0.55), (0.35, 0.80, 1.00, 1.0)),   # blue
    ((0.30, 0.90, 0.35, 0.55), (0.50, 1.00, 0.55, 1.0)),   # green
    ((0.90, 0.30, 0.75, 0.55), (1.00, 0.50, 0.90, 1.0)),   # magenta
    ((0.95, 0.85, 0.20, 0.55), (1.00, 0.95, 0.45, 1.0)),   # yellow
    ((0.20, 0.85, 0.80, 0.55), (0.40, 1.00, 0.95, 1.0)),   # teal
]

def main():
    spec = sys.argv[1] if len(sys.argv) > 1 else "star:5:5"
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 600.0
    dog = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    lane = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    # Marker IDs must not collide between dogs sharing one scene.
    id_base = 1 + dog * 100000
    plan_rgba, flown_rgba = DOG_HUES[dog % len(DOG_HUES)]

    node = transport.Node()
    marker_clear(node)

    # ---- planned track: amber tubes, green waypoint spheres ----
    # world is ENU: world x = east, world y = north
    wps = mission_waypoints(spec)
    # BOUND THE MARKER COUNT. Every /marker call is a blocking service request,
    # and the dense courses are 93 (oval) to 108 (atom) waypoints - at two
    # markers each that is over 200 blocking calls per dog, times N dogs, all
    # before the run starts. The planned track only has to convey SHAPE, so
    # decimate to a fixed budget: the drawing cost is then independent of how
    # finely the course happens to be sampled, and of how many dogs there are.
    PLAN_MAX = int(os.environ.get("TRAIL_PLAN_MAX", "44"))
    if len(wps) > PLAN_MAX:
        stride = math.ceil(len(wps) / PLAN_MAX)
        wps = wps[::stride] + [wps[-1]]
    mid = id_base
    prev = (0.0, 0.0)
    for i, (n, e) in enumerate(wps):
        # world is ENU (x=east, y=north); shift east by this dog's lane
        a = (prev[1] + lane, prev[0], TRAIL_Z)
        b = (e + lane, n, TRAIL_Z)
        marker_send(node, marker_tube(mid, a, b, plan_rgba, 0.07)); mid += 1
        marker_send(node, marker_sphere(mid, (e + lane, n, TRAIL_Z), WP_RGBA)); mid += 1
        prev = (n, e)
    print(f"[trail] dog{dog} ({MODEL}) planned: {len(wps)} wp ({spec}) lane y={lane:+.0f}",
          flush=True)

    # ---- flown track: cyan tubes, appended as the dog moves ----
    latest = [None]

    def cb(msg: Pose_V):
        for p in msg.pose:
            if p.name == MODEL:
                latest[0] = (p.position.x, p.position.y)

    node.subscribe(Pose_V, f"/world/{WORLD}/dynamic_pose/info", cb)

    flown_id = id_base + 50000
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
                                      (p[0], p[1], TRAIL_Z), flown_rgba))
        flown_id += 1
        last = p
    print(f"[trail] flown track: {flown_id - 10000} segments", flush=True)


if __name__ == "__main__":
    main()
