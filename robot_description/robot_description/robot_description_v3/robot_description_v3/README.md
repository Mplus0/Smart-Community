# robot_description v0.3 — Gazebo physical baseline

This version advances the official-CAD reconstruction from a kinematic skeleton to a physically valid Gazebo baseline. It still **does not contain the mecanum controller**.

## What changed from v0.2

- keeps the official `base_link.STL` as the visual model;
- uses a simplified lower-chassis box for base collision instead of the 270k-triangle CAD mesh;
- gives all four wheel links cylinder collisions and non-zero inertial properties;
- keeps all four wheel joints as `continuous` with Y-axis rotation;
- adds light joint damping for numerical stability;
- adds ordinary isotropic wheel/ground friction for the static-contact test;
- corrects `base_footprint` so it lies on the true wheel/ground contact plane.

## Current tree

```text
base_footprint
└── base_footprint_joint [fixed]
    └── base_link
        ├── front_left_wheel_joint  [continuous] -> front_left_wheel_link
        ├── front_right_wheel_joint [continuous] -> front_right_wheel_link
        ├── rear_left_wheel_joint   [continuous] -> rear_left_wheel_link
        └── rear_right_wheel_joint  [continuous] -> rear_right_wheel_link
```

## Important: v0.3 is not yet a strafing mecanum simulation

The four wheel contacts currently use `mu1 = mu2 = 1.0`. This is deliberate: v0.3 is only for verifying that the robot falls, settles, and maintains stable four-wheel contact in Gazebo.

The next control version must add either an anisotropic mecanum contact approximation (roller directions / `fdir1`) or another explicitly selected mecanum-drive method, plus `vx/vy/wz -> wheel angular velocity` control. Those choices should not be hidden inside this physical-baseline step.

## Mass baseline

The official CAD URDF gives one lumped mass of `0.486219 kg`. The STL volume split suggests about `0.0193 kg` per wheel if mass were distributed only by mesh volume. Directly subtracting that much from the official lumped inertia creates a non-positive residual base inertia, so it is not physically consistent with the exported inertia tensor.

For v0.3 a conservative **simulation baseline** of `0.015 kg` per wheel is used. The remaining base mass is `0.426219 kg`, preserving the official total mass. The residual base inertia is positive definite. These values are not claimed to be measured hardware parameters and should be replaced if real mass/inertia data become available.

See `docs/physical_parameter_derivation.md`.

## Files to use

ROS/Xacro:

```text
urdf/robotcar.urdf.xacro
```

Plain URDF equivalent:

```text
urdf/robotcar.urdf
```

Collision/joint visual check without STL dependency:

```text
preview/robotcar_physics_debug.urdf
```

For the Gazebo static-contact test, follow `docs/gazebo_static_test.md`.
