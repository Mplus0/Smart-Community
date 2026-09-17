# Smart Community Mecanum Robot — v0.6

This package set extends v0.5 with ROS Noetic / Gazebo Classic sensor plugins while preserving the measured mecanum geometry and sensor TF tree.

## Packages

- `robot_description`: URDF/Xacro, official STL, mecanum physical model, measured sensor frames, Gazebo sensor plugins.
- `robot_mecanum_control`: four-wheel mecanum IK/FK controller and wheel odometry prepared in v0.4/v0.5.

## v0.6 additions

- LiDAR `/scan` baseline derived from the official 5000 measurements/s specification.
- Depth-camera color, depth, camera-info and point-cloud ROS interfaces.
- IMU `/imu/data` at a 100 Hz simulation baseline.
- No ultrasonic sensor (explicitly disabled).
- Linux validation launch/checklist prepared, but runtime status remains **UNVERIFIED** until ROS Noetic + Gazebo Classic is available.

See `robot_description/docs/sensor_simulation_baseline.md` and `robot_description/docs/linux_sensor_test.md`.
