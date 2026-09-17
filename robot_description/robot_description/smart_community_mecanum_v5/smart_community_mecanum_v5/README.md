# Smart Community mecanum v5 — sensor TF + official physical baseline

This Windows-prepared bundle contains:

```text
robot_description/
robot_mecanum_control/
```

v5 builds on v4 and adds the real-robot sensor frame layout using the official STL plus user measurements.
No Linux / Gazebo execution has been performed yet.

## Confirmed / measured geometry

Robot frame convention:

```text
+X forward
+Y left
+Z up
```

STL envelope is the geometry baseline.

Sensor horizontal positions:

```text
laser_link  : x = +0.09199999 m, y = 0
camera_link : x = +0.13499999 m, y = 0
imu_link    : x = -0.04300000 m, y = 0
```

Sources:
- LiDAR center: ~73 mm behind robot front.
- Depth camera: ~30 mm behind robot front.
- IMU: ~103 mm ahead of robot rear, centered left/right.
- IMU is mounted on the same upper plane as the LiDAR base.
- Ultrasonic sensors are not used.

Vertical coordinates are currently derived from STL geometry and remain hardware-verification items.

## TF tree added in v5

```text
base_footprint
└── base_link
    ├── front_left_wheel_link
    ├── front_right_wheel_link
    ├── rear_left_wheel_link
    ├── rear_right_wheel_link
    ├── laser_link
    ├── camera_link
    │   └── camera_optical_frame
    └── imu_link
```

`camera_optical_frame` uses the standard ROS optical orientation.
`imu_link` is currently a logical simulation frame aligned to `base_link`; the physical IMU board-axis rotation still needs verification.

## Official hardware values incorporated

```text
empty mass        = 5.2 kg
payload            = 5.0 kg
max chassis speed  = 0.6 m/s
motion model       = mecanum
```

The v4 CAD-export total mass (0.486 kg) is no longer treated as the real robot mass.
Until individual wheel masses are measured, v5 uniformly scales the earlier CAD-based mass distribution to total 5.2 kg.

With wheel radius `0.037230282 m`, nominal straight-line wheel speed at 0.6 m/s is:

```text
16.115913385 rad/s
```

This replaces the previous 30 rad/s temporary limit.

## Files to inspect on Windows

- `robot_description/preview/robotcar_sensor_tf_skeleton.urdf`
- `robot_description/config/sensor_frames.yaml`
- `robot_description/config/official_sensor_specs.yaml`
- `robot_description/docs/sensor_frame_derivation.md`

## Still requires Linux / Gazebo / hardware verification

- Gazebo spawn and static contact stability
- mecanum friction (`mu1`, `mu2`, `fdir1`)
- four wheel joint sign convention under real simulation
- LiDAR scan zero-angle orientation
- exact LiDAR/camera vertical centers
- physical IMU board/chip axis rotation
- motor torque and gearbox ratio
- sensor Gazebo plugins and topic names
