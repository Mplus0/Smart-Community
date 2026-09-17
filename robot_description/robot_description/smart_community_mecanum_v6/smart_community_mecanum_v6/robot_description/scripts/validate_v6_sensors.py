#!/usr/bin/env python3
from pathlib import Path
import math
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / 'urdf' / 'robotcar.urdf'
XACRO = ROOT / 'urdf' / 'robotcar_sensor_plugins.xacro'

root = ET.parse(URDF).getroot()
links = {x.attrib['name'] for x in root.findall('link')}
assert {'laser_link','camera_link','camera_optical_frame','imu_link'} <= links

sensors = {}
for gazebo in root.findall('gazebo'):
    ref = gazebo.attrib.get('reference')
    sensor = gazebo.find('sensor')
    if sensor is not None:
        sensors[ref] = sensor

assert sensors['laser_link'].attrib['type'] == 'ray'
assert sensors['camera_link'].attrib['type'] == 'depth'
assert sensors['imu_link'].attrib['type'] == 'imu'

# Lidar official-consistency checks.
laser = sensors['laser_link']
rate = float(laser.findtext('update_rate'))
samples = int(laser.find('ray/scan/horizontal/samples').text)
max_range = float(laser.find('ray/range/max').text)
resolution_deg = 360.0 / samples
assert 5.0 <= rate <= 12.0
assert math.isclose(rate * samples, 5000.0, rel_tol=0, abs_tol=1e-9)
assert 0.43 <= resolution_deg <= 0.86
assert math.isclose(max_range, 10.0, rel_tol=0, abs_tol=1e-12)

lp = laser.find('plugin')
assert lp.attrib['filename'] == 'libgazebo_ros_laser.so'
assert lp.findtext('topicName') == 'scan'
assert lp.findtext('frameName') == 'laser_link'

cam = sensors['camera_link']
cp = cam.find('plugin')
assert cp.attrib['filename'] == 'libgazebo_ros_depth_camera.so'
assert cp.findtext('frameName') == 'camera_optical_frame'
assert float(cam.find('camera/clip/near').text) == 0.6
assert float(cam.find('camera/clip/far').text) == 8.0
assert int(cam.find('camera/image/width').text) <= 1280
assert int(cam.find('camera/image/height').text) <= 1024

imu = sensors['imu_link']
ip = imu.find('plugin')
assert ip.attrib['filename'] == 'libgazebo_ros_imu_sensor.so'
assert ip.findtext('topicName') == 'imu/data'
assert ip.findtext('frameName') == 'imu_link'
assert 0.1 <= float(ip.findtext('updateRateHZ')) <= 200.0

# Xacro must at least be well-formed XML and carry the same plugin names.
x = ET.parse(XACRO).getroot()
text = XACRO.read_text()
for plugin in ('libgazebo_ros_laser.so','libgazebo_ros_depth_camera.so','libgazebo_ros_imu_sensor.so'):
    assert plugin in text

print('robot_description v0.6 sensor baseline checks: PASS')
print(f'LiDAR: {samples} samples x {rate:g} Hz = {samples*rate:g} measurements/s, {resolution_deg:.3f} deg/sample')
print('Depth camera baseline: 640x480, 0.6-8.0 m; HFOV/rate are simulation baselines')
print('IMU baseline: 100 Hz; sensor noise remains uncalibrated')
