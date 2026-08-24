#!/usr/bin/env python3
"""
fleet_world.py <src.sdf> <out.sdf> <slot0_mission> [<slot1_mission> ...]

Builds ONE Gazebo world holding up to 3 dogs running DIFFERENT missions side
by side - the thing make_multi_world.py cannot do, because it assumes N
copies of the SAME course and spaces them uniformly along north.

Here each slot gets its own mission (star / oval / atom / a straight dash),
and the slots are bin-packed along EAST instead, each given exactly the lane
width its own course needs (from make_multi_world.mission_bbox), plus a fixed
30 m margin. Packing along east rather than north matters specifically because
the oval's long axis (the 40 m straight) already runs along north - packing
along north would need lanes ~50 m apart per oval; packing along east needs
only the oval's own 10 m cross-track width plus margin, which is why this
script exists instead of just calling make_multi_world.py three times into
one file.

Also emits a <gui><camera> pose in the output world, roughly centred over the
combined layout and high enough to frame every slot at once - so the operator
does not have to hunt for the fleet with the mouse the moment the window opens.
"""
import sys
import os
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) + "/..")
from make_multi_world import mission_bbox, load_proto, clone_dog  # noqa: E402

MARGIN = 30.0  # m between adjacent slots' bounding boxes


def layout(missions):
    """[(mission_spec, spawn_north, spawn_east, (n0,n1,e0,e1) world bbox), ...]"""
    slots = []
    cursor_east = 0.0
    for spec in missions:
        bb = mission_bbox(spec)
        if bb is None:
            raise SystemExit("not a mission spec: %r" % spec)
        n0, n1, e0, e1 = bb
        spawn_east = cursor_east - e0
        slots.append((spec, 0.0, spawn_east, (n0, n1, spawn_east + e0, spawn_east + e1)))
        cursor_east = spawn_east + e1 + MARGIN
    return slots


def camera_pose(slots):
    """A top-down pose that frames every slot's bounding box with margin."""
    n_lo = min(s[3][0] for s in slots)
    n_hi = max(s[3][1] for s in slots)
    e_lo = min(s[3][2] for s in slots)
    e_hi = max(s[3][3] for s in slots)
    cx = 0.5 * (e_lo + e_hi)          # world X = east
    cy = 0.5 * (n_lo + n_hi)          # world Y = north
    span = max(e_hi - e_lo, n_hi - n_lo, 20.0)
    # default camera FOV ~1.05 rad; half-width at altitude h is h*tan(FOV/2).
    # Solve for h so that half-width covers span/2 with ~25% headroom.
    import math
    alt = (span * 0.5 * 1.25) / math.tan(1.05 / 2.0)
    alt = max(alt, 25.0)
    # pitch = pi/2 rotates the camera's forward axis (+X) down to -Z: top-down.
    return "%.1f %.1f %.1f 0 1.5708 0" % (cx, cy, alt)


def main():
    src, out = sys.argv[1], sys.argv[2]
    missions = sys.argv[3:]
    if not missions:
        raise SystemExit(__doc__)
    if len(missions) > 3:
        raise SystemExit("fleet_world supports at most 3 slots (asked for %d)" % len(missions))

    slots = layout(missions)
    tree, world, proto = load_proto(src)

    for i, (spec, north, east, bbox) in enumerate(slots):
        m, name = clone_dog(proto, i, north=north, east=east)
        world.append(m)

    gui = world.find("gui")
    if gui is None:
        gui = ET.SubElement(world, "gui")
    cam = gui.find("camera")
    if cam is None:
        cam = ET.SubElement(gui, "camera")
        cam.set("name", "fleet_view")
    pose = cam.find("pose")
    if pose is None:
        pose = ET.SubElement(cam, "pose")
    pose.text = camera_pose(slots)

    tree.write(out, encoding="utf-8", xml_declaration=True)
    print("wrote %s: %d-dog fleet, camera %s" % (out, len(slots), pose.text))
    for i, (spec, north, east, bbox) in enumerate(slots):
        print("  go1_%d  %-18s spawn n=%+.1f e=%+.1f  bbox n[%.1f,%.1f] e[%.1f,%.1f]"
              % (i, spec, north, east, bbox[0], bbox[1], bbox[2], bbox[3]))


if __name__ == "__main__":
    main()
