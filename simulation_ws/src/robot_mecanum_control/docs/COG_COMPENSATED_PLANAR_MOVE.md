# base_footprint Planar-Move Strategy

## Why this plugin exists

The competition robot preserves the official CAD-derived inertial parameters.
After URDF fixed-joint reduction, Gazebo's canonical chassis link is
`base_footprint`, and its inertial CoG is offset from the link-frame origin.

A stock model-level planar-motion plugin calls `Model::SetLinearVel()` and
`Model::SetAngularVel()`. For an articulated mecanum model, those model-level
setters also write velocities to the wheel links. This conflicts with the
wheel joints / wheel velocity controller and does not guarantee that the
`base_footprint` origin stays fixed during a nominal in-place turn.

## Current implementation

`robotcar_planar_move.cpp` therefore:

1. subscribes to `/cmd_vel`;
2. looks up the final Gazebo link `base_footprint`;
3. reads that link's CoG directly from its Gazebo inertial object;
4. treats `/cmd_vel` as the desired planar twist at the `base_footprint`
   frame origin;
5. converts the desired frame-origin velocity to the corresponding CoG
   velocity using

   `v_cog = v_base + omega x r_(base->cog)`;

6. applies linear and yaw velocity only to `base_footprint`;
7. never writes wheel-link velocities.

For the planar body-frame case:

- `v_cog_x = vx - wz * r_y`
- `v_cog_y = vy + wz * r_x`

The CoG is not hard-coded in the Xacro, so changing the chassis inertial data
will automatically change the compensation after the model is respawned.

## Intended architecture

```text
                    /cmd_vel
                       |
          +------------+-------------+
          |                          |
          v                          v
robotcar_planar_move            mecanum_cmd_vel.py
(base_footprint only)            true mecanum IK
          |                          |
          v                          v
 chassis planar motion      wheel_velocity_controller
                                     |
                                  4 wheels
```

The plugin deliberately publishes no odometry and no TF. `/wheel/odom` remains
the wheel-kinematic odometry source and can later be fused with IMU data.

## Validation sequence

Before starting `mecanum_control.launch`, test the plugin alone:

1. Confirm `/cmd_vel` has only `/gazebo` as subscriber.
2. Record `robotcar::base_footprint` with `/gazebo/get_link_state`.
3. Apply `vx=0`, `vy=0`, `wz=+0.30` for about 5 seconds.
4. Send a zero Twist.
5. Record `robotcar::base_footprint` again.
6. Verify XY drift is reduced to a small numerical / contact-level residual
   while yaw changes normally.

Only after that test passes should `mecanum_control.launch` be enabled and the
combined chassis-motion + wheel-rotation behaviour retested.
