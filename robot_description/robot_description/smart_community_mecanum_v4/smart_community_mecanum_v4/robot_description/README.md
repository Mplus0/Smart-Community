# robot_description v0.4 — mecanum drive prepared

This package is the v0.4 continuation of the official Mowen CAD reconstruction.

Compared with v0.3 it adds:

- four wheel `VelocityJointInterface` transmissions;
- a `gazebo_ros_control` plugin declaration;
- X-layout anisotropic ODE contact baselines for the four wheel collisions;
- wheel velocity / effort safety limits;
- controller configuration in `config/mecanum_controllers.yaml`.

The official whole-robot STL remains the visual model. Independent wheel links remain invisible because the official STL already contains the wheel appearance.

## Important validation boundary

This version was prepared while Gazebo was unavailable. It has **not** been run in ROS Noetic / Gazebo Classic yet. In particular, the friction values and effective `fdir1` behavior remain tuning candidates, not validated hardware/physics parameters.

The matching `/cmd_vel` conversion and raw wheel-odometry package is provided separately as `robot_mecanum_control` in the same v4 bundle.
