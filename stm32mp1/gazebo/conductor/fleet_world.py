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
import json
import math
import sys
import os
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) + "/..")
from make_multi_world import mission_bbox, load_proto, clone_dog  # noqa: E402
from mission_geometry import mission_spawn_yaw_rad  # noqa: E402
import terrain  # noqa: E402

MARGIN = 30.0  # m between adjacent slots' bounding boxes

DEFAULT_CAM_CFG = dict(front=True, nadir=True, chase=True,
                        distance=3.0, height=1.2, degree=90.0)


def strip_camera_sensor(model, sensor_name):
    """Remove one named <sensor> from wherever it lives in the model - a
    disabled feed costs real GPU/CPU (always_on=1 renders regardless of
    subscribers), so "off" means actually absent from the world, not just
    unsubscribed. ElementTree has no find-parent, so walk every link."""
    for link in model.findall("link"):
        for s in list(link.findall("sensor")):
            if s.get("name") == sensor_name:
                link.remove(s)


def make_chase_cam_model(index, distance, height, degree, north, east, spawn_height=0.08):
    """A FREE-FLOATING camera model for dog `index`, teleported live by
    server.py's Fleet._follow_chase_cams() via the /world/.../set_pose
    service instead of being rigidly attached to the dog's own body link.

    The original design (see git history) baked distance/height/degree into
    a body-mounted sensor's LOCAL <pose> at world-build time - free to run
    (rides with the dog with zero per-tick cost) but frozen for the life of
    the run: the browser's distance/height/degree fields could edit the
    DRAFT slot all they wanted, nothing changed until the next full
    relaunch. Per direct instruction, this is the fix: a standalone,
    <static>true</static> model (no physics needed, set_pose teleports a
    static model exactly the same as a dynamic one) carrying just the
    camera sensor, positioned every ~100ms by the server from that dog's
    OWN live pose (already tracked for the trail overlay) plus whatever
    distance/height/degree the draft slot holds AT THAT MOMENT - so a
    slider drag mid-run is visible within one follow-tick, not on the next
    launch.

    Topic is named EXACTLY as before ("go1_<i>/chase_cam") specifically so
    server.py's _subscribe_cameras() needs no changes at all - it already
    subscribes by this string, and has no idea (nor needs to know) whether
    the publisher behind it is a body-mounted sensor or a free one.

    The pose passed here is only a placeholder for the sliver of time
    between world load and the first live position update - the follow
    loop overwrites it on its very first tick, so it does not need to
    account for the dog's actual spawn heading (dogs spawn facing north;
    getting this exactly right would need duplicating that convention here
    for a pose nobody will see for more than ~100ms).
    """
    rad = math.radians(degree)
    ox = -distance * math.cos(rad)   # 0 deg = straight behind (-X, body fwd is +X)
    oy = distance * math.sin(rad)    # 90 deg = to the left (+Y)
    name = "go1_%d_chasecam" % index
    m = ET.Element("model", {"name": name})
    ET.SubElement(m, "static").text = "true"
    pose = ET.SubElement(m, "pose")
    # world is ENU: x = east, y = north (matches clone_dog's own convention)
    pose.text = "%.3f %.3f %.3f 0 0 0" % (east + ox, north + oy, spawn_height + height)
    link = ET.SubElement(m, "link", {"name": "link"})
    sensor = ET.SubElement(link, "sensor", {"name": "chase_cam", "type": "camera"})
    ET.SubElement(sensor, "always_on").text = "1"
    ET.SubElement(sensor, "update_rate").text = "10"
    ET.SubElement(sensor, "topic").text = "go1_%d/chase_cam" % index
    cam = ET.SubElement(sensor, "camera")
    ET.SubElement(cam, "horizontal_fov").text = "1.15"
    img = ET.SubElement(cam, "image")
    ET.SubElement(img, "width").text = "480"
    ET.SubElement(img, "height").text = "270"
    ET.SubElement(img, "format").text = "R8G8B8"
    clip = ET.SubElement(cam, "clip")
    ET.SubElement(clip, "near").text = "0.05"
    ET.SubElement(clip, "far").text = "300"
    return m


def apply_camera_config(model, cfg, index, north, east, spawn_height=0.08):
    """cfg: dict with front/nadir/chase bools and distance/height/degree for
    chase. Strips whichever feeds are unchecked. The body-mounted chase_cam
    sensor is ALWAYS stripped now - live repositioning needs the free-
    floating model below instead (see its docstring) - and this returns
    that model (or None if chase is off) for the caller to world.append()
    alongside the dog; it cannot be nested inside the dog's own model
    element the way the old body-mounted sensor was."""
    if not cfg.get("front", True):
        strip_camera_sensor(model, "front_cam")
    if not cfg.get("nadir", True):
        strip_camera_sensor(model, "nadir_cam")
    strip_camera_sensor(model, "chase_cam")
    if not cfg.get("chase", True):
        return None
    return make_chase_cam_model(index, float(cfg.get("distance", DEFAULT_CAM_CFG["distance"])),
                                 float(cfg.get("height", DEFAULT_CAM_CFG["height"])),
                                 float(cfg.get("degree", DEFAULT_CAM_CFG["degree"])),
                                 north, east, spawn_height)


def apply_terrain(world, kind, run_dir, slots=None):
    """Swap the ground_plane's flat <plane> for a procedural <heightmap>, or
    do nothing at all for "flat" - which leaves the EXACT geometry every
    campaign result in CLAUDE.md was measured on, byte-for-byte unchanged.
    A non-flat terrain is new, unvalidated ground and is opt-in for exactly
    that reason.

    `slots` (from layout()) sizes the flat spawn disc to just cover this
    fleet's actual spawn points, each (spawn_north, spawn_east) away from
    world origin - NOT a fixed fraction of the grid. A fixed 1/6-of-grid disc
    (65.6 m radius) was measured to fully contain the 100 m star (21 m east
    extent) and every other course this port has run, so "rolling terrain"
    launches were silently running on flat ground the whole time. Sizing per
    fleet still protects every dog's stance but stops swallowing missions
    whose own scale is close to the spawn-clearance radius."""
    if kind == "flat" or kind not in terrain.TERRAIN_TYPES:
        return
    gp = None
    for m in world.findall("model"):
        if m.get("name") == "ground_plane":
            gp = m
            break
    if gp is None:
        return
    spec = terrain.TERRAIN_TYPES[kind]
    if "surface" in spec and spec["surface"].get("skip_ground"):
        return
    if "surface" in spec:
        # SURFACE kind: same flat plane geometry (2D waypoints therefore sit
        # on the ground by construction), different contact physics + look.
        # The ground half of the pair; the foot half is apply_surface_feet()
        # on the proto, so the effective pair mu equals the surface value
        # under any engine combine rule.
        color = spec["surface"]["color"]
        for link in gp.findall("link"):
            for col in link.findall("collision"):
                for old in col.findall("surface"):
                    col.remove(old)
                col.append(ET.fromstring(terrain.surface_xml(spec)))
            for vis in link.findall("visual"):
                mat = vis.find("material")
                if mat is not None:
                    for tag in ("ambient", "diffuse"):
                        el = mat.find(tag)
                        if el is not None:
                            el.text = "%.2f %.2f %.2f 1" % color
        return
    flatten_radius_m = 2.0
    if slots:
        import math
        farthest = max(math.hypot(north, east) for _spec, north, east, _bb in slots)
        flatten_radius_m = farthest + 2.0
    png_path = os.path.join(run_dir, "terrain_%s.png" % kind)
    amp = terrain.generate(kind, png_path, flatten_radius_m=flatten_radius_m)
    for link in gp.findall("link"):
        for tag in ("collision", "visual"):
            hm_xml = terrain.build_heightmap_xml(png_path, amp, textured=(tag == "visual"))
            for el in link.findall(tag):
                geom = el.find("geometry")
                if geom is not None:
                    el.remove(geom)
                el.append(ET.fromstring("<geometry>%s</geometry>" % hm_xml))


def apply_surface_feet(proto, kind):
    """The FOOT half of a surface kind's contact pair: set every
    *foot_collision* mu/mu2 in the PROTO (before cloning, so all dogs
    inherit) to the surface's mu. The proto ships foot mu=0.6; with only
    the ground patched, the pair's effective friction would depend on the
    engine's combine rule (min? product? sqrt-product?) - setting both
    sides equal makes it the surface value under any of them. Everything
    else in the foot surface block (kp 1e6, kd 1) is left alone: ground
    compliance is the GROUND's job (apply_terrain), and the validated
    'flat' kind patches nothing at all."""
    spec = terrain.TERRAIN_TYPES.get(kind, {})
    if "surface" not in spec or spec["surface"].get("skip_feet"):
        return
    mu = spec["surface"]["mu"]
    n = 0
    for col in proto.iter("collision"):
        if "foot_collision" not in (col.get("name") or ""):
            continue
        for el in col.iter():
            if el.tag in ("mu", "mu2"):
                el.text = "%g" % mu
                n += 1
    if n != 8:   # 4 feet x (mu + mu2) - a proto change would break this silently
        raise SystemExit("apply_surface_feet: expected 8 mu edits, made %d" % n)


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
    args = sys.argv[1:]
    terrain_kind = "flat"
    cam_cfgs = None
    for a in list(args):
        if a.startswith("--terrain="):
            terrain_kind = a.split("=", 1)[1]
            args.remove(a)
        elif a.startswith("--cam_config="):
            cam_cfgs = json.loads(a.split("=", 1)[1])
            args.remove(a)
    src, out = args[0], args[1]
    missions = args[2:]
    if not missions:
        raise SystemExit(__doc__)
    if len(missions) > 3:
        raise SystemExit("fleet_world supports at most 3 slots (asked for %d)" % len(missions))

    slots = layout(missions)
    tree, world, proto = load_proto(src)
    apply_terrain(world, terrain_kind, os.path.dirname(os.path.abspath(out)), slots=slots)
    apply_surface_feet(proto, terrain_kind)

    for i, (spec, north, east, bbox) in enumerate(slots):
        yaw = mission_spawn_yaw_rad(spec)
        m, name = clone_dog(proto, i, north=north, east=east, yaw=yaw)
        cfg = cam_cfgs[i] if cam_cfgs and i < len(cam_cfgs) else DEFAULT_CAM_CFG
        chase_model = apply_camera_config(m, cfg, i, north, east)
        world.append(m)
        if chase_model is not None:
            world.append(chase_model)

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
