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

Dogs are spaced along +Y so their courses cannot overlap. Pass a MISSION SPEC
instead of a number and the spacing is derived from that course's actual
footprint rather than guessed - a hand-picked constant is either wasteful (the
oval is only 10 m wide) or dangerous (nothing checks it against the course).

    star:10.514:5   waypoints at radius r      -> 2r  = 21.0 m wide
    atom:9.0:6      lobe tips at radius R      -> 2R  = 18.0 m wide
    oval:40:5.0     straights at y=0 and y=2R  -> 2R  = 10.0 m wide
    circle:r:n                                 -> 2r
    outback:d       there and back on one line -> ~2 m

plus a margin for a dog that falls, overshoots, or drifts off course, so the
lanes never touch even in failure. Navigation is unaffected by the offset:
each dog's nav sets its local origin from its own first GPS fix, so every
mission is relative to wherever that dog was spawned.
"""
import sys, copy
import xml.etree.ElementTree as ET

src, out = sys.argv[1], sys.argv[2]
n = int(sys.argv[3])

def mission_width(spec):
    """Lateral (Y) extent of a course, metres. None if not a mission spec."""
    try:
        kind, rest = spec.split(":", 1)
        f = [float(x) for x in rest.split(":")]
    except ValueError:
        return None
    if kind in ("star", "circle", "atom"):
        return 2.0 * f[0]                    # waypoints/lobe tips at radius f[0]
    if kind == "oval":
        return 2.0 * (f[1] if len(f) > 1 else 3.0)   # straights at y=0 and y=2R
    if kind == "outback":
        return 2.0
    return None

arg = sys.argv[4] if len(sys.argv) > 4 else "star:10.514:5"
w = mission_width(arg)
if w is None:
    spacing = float(arg)
    src_note = "explicit"
else:
    # Margin for a dog that falls, overshoots or drifts: half the course width
    # again, never less than 15 m. Lanes must not touch even in failure.
    spacing = w + max(15.0, 0.5 * w)
    src_note = "%s (course %.1f m wide)" % (arg, w)

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

def retopic(model, name):
    """Namespace every bare sensor topic under the model's own name."""
    for s in model.iter("sensor"):
        t = s.find("topic")
        if t is not None and t.text and not t.text.startswith("/"):
            t.text = "%s/%s" % (name, t.text.strip())

for i in range(n):
    m = copy.deepcopy(proto)
    name = "go1_%d" % i
    m.set("name", name)
    pose = m.find("pose")
    if pose is None:
        pose = ET.SubElement(m, "pose"); pose.text = "0 0 0.08 0 0 0"
    v = [float(x) for x in pose.text.split()]
    v[1] += i * spacing                     # step along +Y
    pose.text = " ".join("%g" % x for x in v)
    retopic(m, name)
    world.append(m)

tree.write(out, encoding="utf-8", xml_declaration=True)
print("wrote %s: %d dogs, %.1f m apart [%s], topics namespaced per model"
      % (out, n, spacing, src_note))
for i in range(n):
    print("  go1_%d  y=%+.0f m  topics go1_%d/{imu,air_pressure,navsat}" % (i, i*spacing, i))
