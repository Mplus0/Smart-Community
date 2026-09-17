# Smart Community mecanum v4 — Windows-prepared / Gazebo-unverified

This bundle contains two ROS1 packages intended to be copied later into `car_2026/src/`:

```text
robot_description/
robot_mecanum_control/
```

Control chain:

```text
/cmd_vel (vx, vy, wz)
  -> X-layout mecanum inverse kinematics
  -> [FL, FR, RL, RR] rad/s
  -> velocity_controllers/JointGroupVelocityController
  -> 4 x VelocityJointInterface
  -> 4 continuous wheel joints
```

Geometry recovered from the official STL:
- wheel radius `0.037230282 m`
- half wheelbase `0.106000002 m`
- half track `0.125000000 m`
- `L + W = 0.231000002 m`

X-layout equations, wheel order `[FL, FR, RL, RR]`:

```text
w_FL = (vx - vy - (L+W)wz) / r
w_FR = (vx + vy + (L+W)wz) / r
w_RL = (vx + vy - (L+W)wz) / r
w_RR = (vx - vy + (L+W)wz) / r
```

Expected sign patterns:

```text
forward +X : + + + +
left    +Y : - + + -
CCW     +Z : - + - +
```

`/wheel/odom` is raw wheel odometry only. It intentionally does not publish `odom -> base_footprint`, leaving that transform for the later EKF layer.

No Gazebo execution has been performed on this v4 bundle. The anisotropic ODE contact parameters are prepared as a tuning baseline only.

Windows-only mathematical validation:

```text
python robot_mecanum_control/scripts/validate_mecanum_kinematics.py
```
