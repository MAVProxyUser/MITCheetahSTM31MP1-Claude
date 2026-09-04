#!/usr/bin/env python3
"""Inject foot contact sensors into an EXISTING world SDF. Sim-only labels.

WHY NOT make_world.py: that regenerates the world from the URDF, and the raw
SDF it needs is gone. Regenerating would risk a world that differs in some
unnoticed way from the one every measurement tonight was taken on, and a
changed world is a changed robot. This edits the existing file and touches
nothing else. (make_world.py has the same gated block for future regens.)

WHAT THESE ARE FOR, and the limit on them:
  The operator's EDU dog has no foot contact sensors. These exist only to
  LABEL what really touched the ground, so a sensorless contact estimator -
  IMU plus joint encoders, which the EDU dog does have - can be scored against
  truth. Scoring it against the gait schedule is worthless: the schedule is
  precisely what OPEN-26 suspects.

  NOTHING IN THE CONTROL PATH MAY READ THESE. A controller that works here
  because it reads a contact sensor is a controller that does not work on the
  real dog. The reader is a separate subscriber process, exactly as
  pose_feed.py is for ground-truth pose.

  add_foot_contacts.py <world.sdf> [--remove]
"""
import sys, re
import xml.etree.ElementTree as ET

path = sys.argv[1] if len(sys.argv) > 1 else "gazebo/worlds/go1.sdf"
remove = "--remove" in sys.argv

src = open(path).read()
head = ""
m = re.match(r"(<\?xml[^>]*\?>\s*)", src)
if m:
    head = m.group(1); src = src[len(head):]
root = ET.fromstring(src)

links = {}
for ln in root.iter("link"):
    links[ln.get("name")] = ln

n = 0
for leg in ("FL", "FR", "RL", "RR"):
    link = links.get("%s_calf" % leg)
    if link is None:
        sys.stderr.write("WARNING: no %s_calf link; that foot gets no sensor\n" % leg)
        continue
    for s in list(link.findall("sensor")):
        if s.get("type") == "contact":
            link.remove(s); n -= 1
    if remove:
        continue
    col = None
    for c in link.findall("collision"):
        if "foot" in (c.get("name") or ""):
            col = c.get("name"); break
    if col is None:
        sys.stderr.write("WARNING: no foot collision on %s_calf; no sensor\n" % leg)
        continue
    s = ET.SubElement(link, "sensor",
                      {"name": "%s_foot_contact" % leg, "type": "contact"})
    ET.SubElement(s, "always_on").text = "1"
    ET.SubElement(s, "update_rate").text = "500"
    cc = ET.SubElement(s, "contact")
    ET.SubElement(cc, "collision").text = col
    n += 1

ET.indent(root, space="  ")
open(path, "w").write(head + ET.tostring(root, encoding="unicode"))
if remove:
    print("removed foot contact sensors from", path)
else:
    sys.stderr.write("WARNING: %d FOOT CONTACT SENSORS ADDED to %s. Ground-truth "
                     "LABELS only - the real EDU dog has none, so no control "
                     "path may read them.\n" % (n, path))
