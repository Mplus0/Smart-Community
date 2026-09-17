# Windows-stage status

This bundle was prepared without running ROS Noetic or Gazebo Classic.

## Completed without Gazebo
- official STL geometry retained;
- four independent wheel links/joints retained;
- X-layout handedness inferred from official STL geometry: FL/RR match, FR/RL match;
- four `VelocityJointInterface` transmissions added;
- `gazebo_ros_control` integration prepared;
- `/cmd_vel -> [FL, FR, RL, RR]` inverse kinematics implemented;
- raw `/wheel/odom` implemented from wheel joint velocities, without publishing TF;
- command watchdog and uniform wheel-speed saturation implemented;
- pure-Python IK/FK tests available on Windows.

## LINUX_GAZEBO_TEST_REQUIRED
- Gazebo plugin loading;
- static contact stability;
- actual forward / lateral / diagonal / rotational motion;
- effective ODE `fdir1` behavior while wheels rotate;
- friction coefficients (`mu1=1.0`, `mu2=0.05`);
- actual joint sign convention in Gazebo;
- wheel maximum speed / effort values;
- wheel-odometry accuracy and slip.
