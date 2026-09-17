#!/usr/bin/env python3
from pathlib import Path
import math
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
URDF = PKG / 'urdf' / 'robotcar.urdf'
root = ET.parse(URDF).getroot()

links = {e.attrib['name'] for e in root.findall('link')}
joints = root.findall('joint')
children = {}
for j in joints:
    parent = j.find('parent').attrib['link']
    child = j.find('child').attrib['link']
    assert parent in links, (j.attrib['name'], 'missing parent', parent)
    assert child in links, (j.attrib['name'], 'missing child', child)
    assert child not in children, (child, 'has multiple parents')
    children[child] = parent

assert 'base_footprint' in links
assert 'base_footprint' not in children
for required in [
    'base_link',
    'front_left_wheel_link','front_right_wheel_link','rear_left_wheel_link','rear_right_wheel_link',
    'laser_link','camera_link','camera_optical_frame','imu_link']:
    assert required in links, required

# Mass total
mass = 0.0
for link in root.findall('link'):
    m = link.find('./inertial/mass')
    if m is not None:
        mass += float(m.attrib['value'])
assert abs(mass - 5.2) < 1e-9, mass

# Wheel speed from official max body speed.
r = 0.037230282
expected = 0.6 / r
for n in ['front_left','front_right','rear_left','rear_right']:
    j = root.find(f"./joint[@name='{n}_wheel_joint']")
    assert j is not None
    got = float(j.find('limit').attrib['velocity'])
    assert abs(got - expected) < 1e-9, (n, got, expected)

# Sensor horizontal positions.
expect_xyz = {
    'laser_joint': (0.09199999, 0.0, 0.122011877),
    'camera_joint': (0.13499999, 0.0, 0.060287877),
    'imu_joint': (-0.043, 0.0, 0.107799877),
}
for name, exp in expect_xyz.items():
    j=root.find(f"./joint[@name='{name}']")
    assert j is not None, name
    got=tuple(float(x) for x in j.find('origin').attrib['xyz'].split())
    assert all(abs(a-b)<1e-12 for a,b in zip(got,exp)), (name,got,exp)

opt=root.find("./joint[@name='camera_optical_joint']")
assert opt is not None
rpy=tuple(float(x) for x in opt.find('origin').attrib['rpy'].split())
assert abs(rpy[0]+math.pi/2)<1e-12 and abs(rpy[1])<1e-12 and abs(rpy[2]+math.pi/2)<1e-12

print(f'links={len(links)} joints={len(joints)} mass={mass:.12f} kg')
print(f'max_wheel_speed={expected:.12f} rad/s')
print('sensor TF tree: PASS')
print('v5 Windows baseline: PASS')
