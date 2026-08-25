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
    if kind in ("star", "atom"):
        # Every point lies within radius f[0] of the nucleus/centre by
        # construction (star: vertices ON the circle, chords stay inside it;
        # atom: r(t) <= outer_radius_m at every t). Safe on both axes.
        r = f[0]
        return (-r, r, -r, r)
    if kind == "circle":
        # WaypointNav::makeCircle: north = r*sin(a) in [-r,r],
        #                          east  = r*(1-cos(a)) in [0,2r]
        r = f[0]
        return (-r, r, 0.0, 2.0 * r)
    if kind == "oval":
        # WaypointNav::makeOval: the straight runs along NORTH for S metres,
        # with a semicircular bulge of radius R at each end - one bulging
        # further north (to S+R), the other bulging south of the start
        # (to -R). East spans the two straights, 0 to 2R.
        S = f[0]
        R = f[1] if len(f) > 1 else 3.0
        return (-R, S + R, 0.0, 2.0 * R)
    if kind in ("outback", "dash"):
        # WaypointNav::makeDash is a single waypoint `d` metres due north;
        # makeOutAndBack adds the return leg along the same line. Both span
        # 0..d north and nothing east. (A dash slot that also carries a
        # WP_DASH finish extends further north, but the fleet layout adds
        # its own margin on top of the bbox, so this stays the course's own
        # extent rather than guessing at the finish.)
        d = f[0]
        return (0.0, d, 0.0, 0.0)
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


def clone_dog(proto, index, north, east, height=0.08):
    """One namespaced, positioned copy of the go1 template."""
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

    tree, world, proto = load_proto(src)
    for i in range(n):
        m, name = clone_dog(proto, i, north=i * spacing, east=0.0)
        world.append(m)

    tree.write(out, encoding="utf-8", xml_declaration=True)
    print("wrote %s: %d dogs, %.1f m apart [%s], topics namespaced per model"
          % (out, n, spacing, src_note))
    for i in range(n):
        print("  go1_%d  y=%+.0f m  topics go1_%d/{imu,air_pressure,navsat}"
              % (i, i * spacing, i))


if __name__ == "__main__":
    main()
