# robot_description v0.6

Formal robot description for the Smart Community mecanum platform.

## Primary model

`urdf/robotcar.urdf.xacro`

It includes:

1. `robotcar_base.xacro` — official STL visual, simplified collision, 5.2 kg empty-mass baseline, four independent wheels.
2. `robotcar_sensors.xacro` — measured/derived LiDAR, depth-camera and IMU frames.
3. `robotcar_transmissions.xacro` — four velocity transmissions.
4. `robotcar_gazebo.xacro` — mecanum contact baseline and `gazebo_ros_control`.
5. `robotcar_sensor_plugins.xacro` — LiDAR, depth-camera and IMU Gazebo/ROS interfaces.

## Intended topics

- `/scan` (`laser_link`)
- `/camera/color/image_raw`
- `/camera/color/camera_info`
- `/camera/depth/image_raw`
- `/camera/depth/camera_info`
- `/camera/depth/points` (`camera_optical_frame`)
- `/imu/data` (`imu_link`)

The exact topic resolution and plugin runtime behavior must still be verified on Linux/Gazebo.

## Important distinction

Official hardware values and simulation-only defaults are separated in `config/sensor_simulation.yaml` and documented in `docs/sensor_simulation_baseline.md`.
