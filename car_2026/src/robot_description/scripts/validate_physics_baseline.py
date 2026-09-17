#!/usr/bin/env python3
from pathlib import Path
import xml.etree.ElementTree as ET
import math

root_dir = Path(__file__).resolve().parents[1]
urdf = root_dir / "urdf" / "robotcar.urdf"
root = ET.parse(urdf).getroot()
links = {e.attrib["name"] for e in root.findall("link")}
joints = root.findall("joint")
assert len(links) == 6, f"expected 6 links, got {len(links)}"
assert len(joints) == 5, f"expected 5 joints, got {len(joints)}"
for j in joints:
    p=j.find("parent").attrib["link"]
    c=j.find("child").attrib["link"]
    assert p in links and c in links, (j.attrib["name"],p,c)
for name in ["front_left","front_right","rear_left","rear_right"]:
    j=root.find(f"joint[@name='{name}_wheel_joint']")
    assert j is not None and j.attrib["type"] == "continuous"
    assert j.find("axis").attrib["xyz"] == "0 1 0"
    link=root.find(f"link[@name='{name}_wheel_link']")
    mass=float(link.find("inertial/mass").attrib["value"])
    assert mass > 0
print("robot_description v0.3 baseline checks: PASS")
