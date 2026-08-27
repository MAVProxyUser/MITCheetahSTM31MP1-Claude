#!/usr/bin/env python3
"""
Turn the URDF->SDF conversion of the Unitree Go1 (unitree_ros/go1_description)
into a gz-harmonic (Gazebo Sim 8) world the Cheetah bridge can drive:

  - strip the gazebo-classic / ROS plugins (gazebo_ros_control, unitree*, etc.)
  - add a gz IMU sensor on imu_link
  - add the JointStatePublisher system + one ApplyJointForce system per joint
    (torque control; the bridge computes the Unitree motor PD and applies force)
  - wrap the model in a world with physics @ 1 kHz, ground plane, sun, and the
    physics/scene/user-commands/imu/contact systems.

Input : /tmp/go1_raw.sdf   (gz sdf -p go1.urdf)
Output: stm32mp1/gazebo/worlds/go1.sdf
Meshes resolve via GZ_SIM_RESOURCE_PATH=.../unitree_ros/robots (model://go1_description/...)
"""
import os, sys, re
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
RAW = sys.argv[1] if len(sys.argv) > 1 else "/tmp/go1_raw.sdf"
OUT = os.path.join(HERE, "worlds", "go1.sdf")

JOINTS = [f"{leg}_{j}_joint" for leg in ("FR", "FL", "RR", "RL")
          for j in ("hip", "thigh", "calf")]
WORLD = "go1_world"

with open(RAW) as f:
    raw = f.read()
# gz's URDF->SDF converter emits custom <gz::...> tags (undeclared namespace)
# which stdlib ElementTree rejects. Strip them (all inside plugin blocks we drop).
raw = re.sub(r"<gz::[^>]*>.*?</gz::[^>]*>", "", raw, flags=re.S)
raw = re.sub(r"<gz::[^>]*/>", "", raw)
sdf = ET.fromstring(raw)
model = sdf.find("model")
if model is None:
    sys.exit("no <model> in converted SDF")

# 1) strip every classic/ROS <plugin> and any pre-existing <sensor> in the model
for parent in model.iter():
    for tag in ("plugin", "sensor"):
        for el in list(parent.findall(tag)):
            parent.remove(el)

# 2) one gz IMU sensor on the body link (imu_link merges into trunk/base on conversion)
links = {ln.get("name"): ln for ln in model.findall("link")}
imu_link = links.get("imu_link") or links.get("trunk") or links.get("base")
if imu_link is not None:
    s = ET.SubElement(imu_link, "sensor", {"name": "cheetah_imu", "type": "imu"})
    ET.SubElement(s, "always_on").text = "1"
    ET.SubElement(s, "update_rate").text = "500"
    ET.SubElement(s, "topic").text = "imu"
    ET.SubElement(s, "imu")  # default axes
    # Baro (air pressure) + GPS (navsat) on the body -- not consumed by Cheetah yet,
    # plumbed for future waypoint navigation.
    b = ET.SubElement(imu_link, "sensor", {"name": "cheetah_baro", "type": "air_pressure"})
    ET.SubElement(b, "always_on").text = "1"
    ET.SubElement(b, "update_rate").text = "50"
    ET.SubElement(b, "topic").text = "air_pressure"
    ap = ET.SubElement(b, "air_pressure")
    ET.SubElement(ap, "reference_altitude").text = "0"
    g = ET.SubElement(imu_link, "sensor", {"name": "cheetah_gps", "type": "navsat"})
    ET.SubElement(g, "always_on").text = "1"
    # GPS_HZ: was a flat 10 Hz with no citation - arbitrary, and the direct
    # cause of the staleness bug documented in CLAUDE.md's "GPS VELOCITY
    # AIDING" section (a 500 Hz control loop re-applying the same
    # zero-order-held reading ~49 times per real sample). Default now
    # matches a real, commonly-used fast GPS module rather than a guess:
    # the u-blox ZED-F9P's documented max navigation rate is 20 Hz (10 Hz
    # in RTK-fixed mode, 20 Hz standalone/DGNSS - this project has no RTK
    # base station modeled, so the standalone figure is what applies).
    # Kept selectable via GPS_HZ for regression against the old behavior,
    # or to model a cheaper/slower receiver deliberately.
    ET.SubElement(g, "update_rate").text = os.environ.get("GPS_HZ", "20")
    ET.SubElement(g, "topic").text = "navsat"

# helper to append a system plugin
def add_plugin(parent, filename, name, children=None):
    p = ET.SubElement(parent, "plugin", {"filename": filename, "name": name})
    for tag, txt in (children or []):
        ET.SubElement(p, tag).text = txt
    return p

# 3) model systems: joint state publisher + per-joint force
add_plugin(model, "gz-sim-joint-state-publisher-system",
           "gz::sim::systems::JointStatePublisher")
for jn in JOINTS:
    add_plugin(model, "gz-sim-apply-joint-force-system",
               "gz::sim::systems::ApplyJointForce", [("joint_name", jn)])

# 4) spawn pose: drop base ~0.45 m so it settles onto its legs
pose = model.find("pose")
if pose is None:
    pose = ET.SubElement(model, "pose")
pose.text = "0 0 0.45 0 0 0"

# ---- build the world ----
world_sdf = ET.Element("sdf", {"version": "1.9"})
world = ET.SubElement(world_sdf, "world", {"name": WORLD})

phys = ET.SubElement(world, "physics", {"name": "1ms", "type": "ignored"})
ET.SubElement(phys, "max_step_size").text = "0.001"
ET.SubElement(phys, "real_time_factor").text = "1.0"

for fn, nm in [
    ("gz-sim-physics-system", "gz::sim::systems::Physics"),
    ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
    ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
    ("gz-sim-imu-system", "gz::sim::systems::Imu"),
    ("gz-sim-air-pressure-system", "gz::sim::systems::AirPressure"),
    ("gz-sim-navsat-system", "gz::sim::systems::NavSat"),
    ("gz-sim-contact-system", "gz::sim::systems::Contact"),
]:
    ET.SubElement(world, "plugin", {"filename": fn, "name": nm})

# GPS needs a georeferenced world origin.
sc = ET.SubElement(world, "spherical_coordinates")
ET.SubElement(sc, "surface_model").text = "EARTH_WGS84"
ET.SubElement(sc, "world_frame_orientation").text = "ENU"
ET.SubElement(sc, "latitude_deg").text = "37.4275"
ET.SubElement(sc, "longitude_deg").text = "-122.1697"
ET.SubElement(sc, "elevation").text = "0"
ET.SubElement(sc, "heading_deg").text = "0"

# sun
light = ET.SubElement(world, "light", {"name": "sun", "type": "directional"})
ET.SubElement(light, "cast_shadows").text = "true"
ET.SubElement(light, "pose").text = "0 0 10 0 0 0"
ET.SubElement(light, "diffuse").text = "0.8 0.8 0.8 1"
ET.SubElement(light, "direction").text = "0.5 0.5 -1"

# ground plane
gp = ET.SubElement(world, "model", {"name": "ground_plane"})
ET.SubElement(gp, "static").text = "true"
gl = ET.SubElement(gp, "link", {"name": "link"})
for kind in ("collision", "visual"):
    el = ET.SubElement(gl, kind, {"name": kind})
    geom = ET.SubElement(el, "geometry")
    plane = ET.SubElement(geom, "plane")
    ET.SubElement(plane, "normal").text = "0 0 1"
    ET.SubElement(plane, "size").text = "50 50"

world.append(model)

os.makedirs(os.path.dirname(OUT), exist_ok=True)
ET.indent(world_sdf, space="  ")
ET.ElementTree(world_sdf).write(OUT, encoding="unicode", xml_declaration=False)
with open(OUT, "r+") as f:
    body = f.read(); f.seek(0); f.write("<?xml version='1.0'?>\n" + body)
print("wrote", OUT)
print("joints:", len(JOINTS), "| imu on:", imu_link.get("name") if imu_link is not None else "NONE")
