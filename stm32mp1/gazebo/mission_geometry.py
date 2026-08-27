"""Pure-math mirror of WaypointNav.cpp's mission generators.

No gz/network imports on purpose: trail_daemon.py and mission_viz.py need
gz.transport13 to actually draw markers in a running world, but the
GEOMETRY itself doesn't - and a third caller (mission_runner.py's
post-run report) needs the waypoint list without wanting a live world or
the gz Python bindings (which only exist in specific venvs on this Mac,
see CLAUDE.md). Previously this exact function was duplicated between
trail_daemon.py and mission_viz.py; a third copy for the report would
have been one too many, so it lives here once and both import it.
"""
import math
import os


def _shift_first_to_origin(pts):
    """Mirrors WaypointNav::shiftFirstToOrigin() exactly - translate the
    whole course so pts[0] becomes (0,0), the robot's own true local
    origin/spawn point. Applied to every mission kind except "dash" and
    "outback" (a dash's "first waypoint" IS the far end of the sprint by
    definition - there is no separate pattern centre to shift away from).
    Pure translation - every corner angle, leg length, and the whole
    corridor-grading tuning are unaffected. See the C++ field comment on
    shiftFirstToOrigin() (WaypointNav.hpp) for the full history, including
    why an earlier, narrower scoping here turned out to be overcautious
    rather than load-bearing once actually tested."""
    if not pts:
        return pts
    dn, de = pts[0]
    return [(n - dn, e - de) for (n, e) in pts]


# The proto go1 model's own base <pose> (worlds/go1_speedway.sdf) spawns
# every dog at SDF yaw = pi/2 - "body-x points north" - chosen back when
# every mission's OWN rotation logic (makeStar's "wp00 due north",
# makeAtom/makeLissajous's tangential-entry alignment) aligned the walk
# FROM spawn TO wp0 with that fixed heading. Once every mission spawns
# EXACTLY ON wp0 (see _shift_first_to_origin above), that walk no longer
# exists - what actually needs aligning is the first REAL leg, wp0->wp1,
# which for most of these courses is nowhere near due north (star's is a
# ~160 degree turn from it). A universal fixed spawn heading and a
# per-mission wp0->wp1 alignment cannot both hold at once any more, so
# the spawn heading itself now has to be per-mission.
_NORTH_YAW_RAD = math.pi / 2.0


def mission_opening_bearing_rad(spec):
    """Standard compass bearing (radians, 0=north, positive clockwise
    toward east) from wp0 to wp1 - the direction of the mission's first
    REAL leg. Returns 0.0 (north) for anything with fewer than 2
    waypoints (dash: nothing to aim at besides the one waypoint it
    already faces by construction) or that mission_waypoints() cannot
    parse. Both endpoints come from the SAME shifted frame
    WaypointNav.cpp's real generators use, so this is guaranteed to
    agree with whatever the robot actually has to do at spawn, not a
    separate approximation of it.

    This is the one number two independent things need to agree on: it
    sets the dog's SPAWN heading (mission_spawn_yaw_rad below, consumed
    by the world-builder) AND has to be fed to the controller itself as
    WP_SPAWN_BEARING_DEG (see mit_sim_main.cpp's heading-datum comment) -
    without the second half, nav's own heading math silently assumes
    spawn-heading-equals-true-north (true only when this bearing is 0),
    which is exactly the "confidently walks 180 degrees the wrong way"
    bug this was caught fixing.
    """
    try:
        pts = mission_waypoints(spec)
    except SystemExit:
        return 0.0
    if len(pts) < 2:
        return 0.0
    n0, e0 = pts[0]
    n1, e1 = pts[1]
    return math.atan2(e1 - e0, n1 - n0)


def mission_spawn_yaw_rad(spec):
    """SDF spawn yaw (radians) so the dog starts facing from wp0 straight
    at wp1 - no in-place hairpin before the mission's first real leg even
    begins.

    Derivation: the proto's yaw=pi/2 is defined as "facing due north"
    (bearing 0), and SDF yaw theta puts the body's local +X along world
    (cos theta, sin theta) in (east, north) - so theta = pi/2 - bearing
    reproduces that same convention for any compass bearing, not just
    north (checked: bearing=0 gives theta=pi/2, matching the proto
    exactly).
    """
    return _NORTH_YAW_RAD - mission_opening_bearing_rad(spec)


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
        return _shift_first_to_origin(out)
    if kind == "circle":
        r, n = float(rest[0]), int(rest[1])
        pts = [(r * math.sin(2 * math.pi * (i + 1) / n),
                r * (1 - math.cos(2 * math.pi * (i + 1) / n))) for i in range(n)]
        return _shift_first_to_origin(pts)
    if kind == "outback":
        d = float(rest[0])
        return [(d, 0.0), (0.0, 0.0)]
    if kind == "dash":
        # Mirrors WaypointNav::makeDash - a single straight leg due north,
        # ending AT that waypoint (no return leg, unlike outback).
        d = float(rest[0])
        return [(d, 0.0)]
    if kind == "corner":
        # Mirrors WaypointNav::makeCorner - one isolated corner of angle_deg
        # at the end of a leg_m approach, then a leg_m exit at that new
        # bearing. Built for the per-gait/per-angle cornering envelope.
        leg_m, angle_deg = float(rest[0]), float(rest[1])
        theta = math.radians(angle_deg)
        wp0 = (leg_m, 0.0)
        wp1 = (wp0[0] + leg_m * math.cos(theta), wp0[1] + leg_m * math.sin(theta))
        return [wp0, wp1]
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
        return _shift_first_to_origin(out)
    if kind == "spiro":
        # Mirrors WaypointNav::makeSpirograph - the SAME trochoid formula
        # as "atom" above, at a different point in its own parameter
        # space (k=lobes directly, not lobes-1; depth pushed near 1.0).
        # See that C++ function's own comment for why those two changes
        # move the curve from atom's own look to a classic Spirograph
        # rosette. Same tangential-entry join as atom, same reason.
        R = float(rest[0]); lobes = int(rest[1]) if len(rest) > 1 else 8
        A = float(os.environ.get("WP_SPIRO_DEPTH", 0.99))
        k = lobes; S = R / (1.0 + A)
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
        step = float(os.environ.get("WP_SPIRO_DS", 0.45))
        out, acc = [(px(t0), py(t0))], 0.0
        pn, pe = px(t0), py(t0)
        for i in range(1, 40001):
            t = t0 + 2 * math.pi * i / 40000
            n, e = px(t), py(t)
            acc += math.hypot(n - pn, e - pe); pn, pe = n, e
            if acc >= step:
                acc -= step; out.append((n, e))
        return _shift_first_to_origin(out)
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
        return _shift_first_to_origin(out)
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
        return _shift_first_to_origin(out)
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
        return _shift_first_to_origin(out)
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
        return _shift_first_to_origin(out)
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
        return _shift_first_to_origin(out)
    raise SystemExit(f"unknown mission spec: {spec}")
