#!/usr/bin/env python3
"""
make_multi_world.py <src.sdf> <out.sdf> <n> [spacing_m | mission_spec]

N dogs in ONE physics engine, each with its own sensor topics.

Why one engine rather than N processes: a separate `gz sim` per dog costs a
full physics engine each (~55 % of a core measured), and they cannot share
broadphase or the ground plane. One engine with N robots pays for physics once.

What has to be namespaced, and what already is:

  ALREADY SCOPED BY MODEL NAME - nothing to do
    /world/<world>/model/<model>/joint_state
    /model/<model>/joint/<joint>/cmd_force
    /world/<world>/dynamic_pose/info      (one topic, filtered by name)

  NOT SCOPED - collides, and this script fixes it
    <topic>imu</topic>            -> <topic>go1_<i>/imu</topic>
    <topic>air_pressure</topic>   -> <topic>go1_<i>/air_pressure</topic>
    <topic>navsat</topic>         -> <topic>go1_<i>/navsat</topic>
    <topic>chase_cam</topic>      -> <topic>go1_<i>/chase_cam</topic>

Dogs are spaced along +Y (north) so their courses cannot overlap. Pass a
MISSION SPEC instead of a number and the spacing is derived from that course's
actual NORTH extent via mission_bbox() - a hand-picked constant is either
wasteful or unsafe depending on which course you run.

CORRECTED 2026-08-23: the original formula used 2x the course's cross-track
radius for EVERY mission type, including the oval - which is wrong on the
oval's own spacing axis. Dogs here are spaced along north, and the oval's
40 m straight runs along north too, so its true north extent is S+2R (~50 m
for oval:40:5.0), not 2R (~10 m). At the old number, N ovals sharing one
engine would have overlapped by roughly 40 m. mission_bbox() below returns the
real extent on both axes so this cannot happen again, in either direction.

Navigation is unaffected by the offset: each dog's nav sets its local origin
from its own first GPS fix, so every mission is relative to wherever that dog
was spawned.
"""
import sys
import copy
import xml.etree.ElementTree as ET
from mission_geometry import mission_waypoints, mission_spawn_yaw_rad


def _numeric_bbox(spec):
    """Bbox computed directly from mission_waypoints() - the same shifted
    generator the real course uses - rather than a hand-derived closed
    form. Also includes (0,0): the robot's own local origin, which for
    the shiftFirstToOrigin() missions IS wp0 (already in the list, so this
    is a no-op there) but for anything else still needs to count, since
    the fleet spawn/clearance math cares about everywhere the robot
    actually stands, not just the waypoints it visits."""
    pts = mission_waypoints(spec) + [(0.0, 0.0)]
    ns = [p[0] for p in pts]; es = [p[1] for p in pts]
    return (min(ns), max(ns), min(es), max(es))


def mission_bbox(spec):
    """(north_min, north_max, east_min, east_max) metres, mirroring the exact
    geometry in WaypointNav.cpp. None if `spec` is not a mission spec (e.g. a
    bare number). Every bound here is a real extent, not a guess - verified
    against the probed C++ output for atom and oval (see mit_sim_main.cpp
    smoke tests) before this was trusted for robot placement.
    """
    try:
        kind, rest = spec.split(":", 1)
        f = [float(x) for x in rest.split(":")]
    except ValueError:
        return None
    if kind in ("star", "atom", "spiro"):
        # WaypointNav::makeStar/makeAtom/makeSpirograph all now call
        # shiftFirstToOrigin() (robot spawns ON wp0, always and forever,
        # per direct instruction), which re-centres each course around
        # wp0 instead of its own nucleus/centre. The old closed form
        # (-r,r,-r,r) was valid for "every point lies within radius r of
        # the CENTRE" - true before the shift, but the centre is no longer
        # at (0,0) once wp0 (a point ON the r-radius circle/curve) is.
        # Numeric, same rationale as circle/sector/parallel/expsquare
        # below: correct by construction from the same shifted generator
        # the real course uses, rather than re-deriving a closed form
        # that depends on exactly where wp0 landed on the old shape.
        return _numeric_bbox(spec)
    if kind == "circle":
        # WaypointNav::makeCircle now calls shiftFirstToOrigin() (robot
        # spawns ON wp0, per direct instruction), which re-centres the
        # course around wp0 instead of the old tangent-to-start point -
        # the old closed form (-r,r,0,2r) was relative to THAT origin and
        # is stale. Computed numerically from the same shifted generator
        # mission_waypoints() calls, rather than re-deriving a new closed
        # form by hand (which depends on n, not just r, once shifted) -
        # single source of truth, correct by construction.
        return _numeric_bbox(spec)
    if kind == "oval":
        # Was a closed form relative to the OLD (near-origin, not exactly
        # wp0) start point; now stale for the same shiftFirstToOrigin()
        # reason as circle/sector/parallel/expsquare below - wp0 was
        # already ~1.2m off true (0,0) before this file's own universal
        # shift, and is now made EXACTLY (0,0), moving the reference point
        # this closed form was built around. Numeric, same rationale.
        return _numeric_bbox(spec)
    if kind in ("outback", "dash"):
        # WaypointNav::makeDash is a single waypoint `d` metres due north;
        # makeOutAndBack adds the return leg along the same line. Both span
        # 0..d north and nothing east. (A dash slot that also carries a
        # WP_DASH finish extends further north, but the fleet layout adds
        # its own margin on top of the bbox, so this stays the course's own
        # extent rather than guessing at the finish.)
        d = f[0]
        return (0.0, d, 0.0, 0.0)
    if kind == "corner":
        # WaypointNav::makeCorner: one approach leg due north, one exit leg
        # at the turn angle. Not a shiftFirstToOrigin() mission (wp0 is ahead
        # of spawn, same convention as dash/star) - numeric, straightforward.
        return _numeric_bbox(spec)
    if kind == "sector":
        # Was a closed form relative to the OLD (tangent-to-start) origin;
        # now stale for the same shiftFirstToOrigin() reason as circle
        # above. Numeric, same rationale.
        return _numeric_bbox(spec)
    if kind == "parallel":
        # Same shift, same fix: was (0,width,0,(passes-1)*height) relative
        # to the OLD origin (the true spawn point); wp0 is now that origin,
        # so the old formula's own reference point moved.
        return _numeric_bbox(spec)
    if kind == "expsquare":
        # Same shift, same fix: the spiral's own size (step_m*legs/4 per
        # axis) is unchanged by a translation, but the OLD (-r,r,-r,r) was
        # centred on the spiral's start, not on wp0 (its first REAL,
        # non-zero-length leg) - no longer the same point.
        return _numeric_bbox(spec)
    if kind == "lissajous":
        # X=A*sin(...), Y=A*sin(...) are each exactly bounded in [-A,A]
        # relative to the curve's OWN centre - true regardless of the
        # frequency ratio, but that closed form is now stale for the same
        # shiftFirstToOrigin() reason as everything else on this list:
        # wp0 (at the curve's own (A,0) by construction) is now (0,0), so
        # the [-A,A]x[-A,A] box needs to move with it. Numeric rather than
        # hand-shifting the box, so a future phase/convention change to
        # the generator cannot silently desync this from it again.
        return _numeric_bbox(spec)
    return None


def mission_width(spec):
    """North (spacing-axis) extent of a course, metres - what this script's
    own N-copies-of-one-mission spacing needs. None if not a mission spec."""
    bb = mission_bbox(spec)
    if bb is None:
        return None
    n0, n1, _, _ = bb
    return n1 - n0


def retopic(model, name):
    """Namespace every bare sensor topic under the model's own name."""
    for s in model.iter("sensor"):
        t = s.find("topic")
        if t is not None and t.text and not t.text.startswith("/"):
            t.text = "%s/%s" % (name, t.text.strip())


def load_proto(src):
    """Parse `src`, pull out the go1 model as a reusable template, and hand
    back (tree, world_element, proto_model) - the shared pieces both this
    script and conductor/fleet_world.py need."""
    tree = ET.parse(src)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        sys.exit("no <world> in %s" % src)
    proto = None
    for m in world.findall("model"):
        if m.get("name") == "go1":
            proto = m
            break
    if proto is None:
        sys.exit("no <model name='go1'> in %s" % src)
    world.remove(proto)
    return tree, world, proto


def clone_dog(proto, index, north, east, height=0.08, yaw=None):
    """One namespaced, positioned copy of the go1 template.

    `yaw` (radians, SDF convention) overrides the proto's own spawn
    heading when given - None keeps whatever the proto already has (the
    universal "facing north" default, still correct for anything with no
    more specific heading to aim at). See
    mission_geometry.mission_spawn_yaw_rad's own docstring for why this
    exists: once every mission spawns ON wp0 rather than walking there,
    the direction worth aligning is wp0->wp1, which is mission-specific,
    not a fixed universal heading any more."""
    m = copy.deepcopy(proto)
    name = "go1_%d" % index
    m.set("name", name)
    pose = m.find("pose")
    if pose is None:
        pose = ET.SubElement(m, "pose")
        pose.text = "0 0 %g 0 0 0" % height
    v = [float(x) for x in pose.text.split()]
    # world is ENU: x = east, y = north (see trail_daemon.py)
    v[0] += east
    v[1] += north
    if yaw is not None:
        v[5] = yaw
    pose.text = " ".join("%g" % x for x in v)
    retopic(m, name)
    return m, name


def main():
    src, out = sys.argv[1], sys.argv[2]
    n = int(sys.argv[3])
    arg = sys.argv[4] if len(sys.argv) > 4 else "star:10.514:5"
    w = mission_width(arg)
    if w is None:
        spacing = float(arg)
        src_note = "explicit"
    else:
        # Margin for a dog that falls, overshoots or drifts: half the
        # course's own extent again, never less than 15 m.
        spacing = w + max(15.0, 0.5 * w)
        src_note = "%s (course %.1f m along the spacing axis)" % (arg, w)

    yaw = mission_spawn_yaw_rad(arg) if w is not None else None
    tree, world, proto = load_proto(src)
    for i in range(n):
        m, name = clone_dog(proto, i, north=i * spacing, east=0.0, yaw=yaw)
        world.append(m)

    tree.write(out, encoding="utf-8", xml_declaration=True)
    print("wrote %s: %d dogs, %.1f m apart [%s], topics namespaced per model"
          % (out, n, spacing, src_note))
    for i in range(n):
        print("  go1_%d  y=%+.0f m  topics go1_%d/{imu,air_pressure,navsat}"
              % (i, i * spacing, i))


if __name__ == "__main__":
    main()
