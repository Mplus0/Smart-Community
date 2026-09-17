# robot_description v0.2 — mecanum kinematic reconstruction

This version upgrades the official single-mesh `mowen` export into a clean ROS robot-description hierarchy while preserving the official CAD appearance.

## Current URDF tree

```text
base_footprint
└── base_footprint_joint (fixed)
    └── base_link
        ├── front_left_wheel_joint  (continuous)
        │   └── front_left_wheel_link
        ├── front_right_wheel_joint (continuous)
        │   └── front_right_wheel_link
        ├── rear_left_wheel_joint   (continuous)
        │   └── rear_left_wheel_link
        └── rear_right_wheel_joint  (continuous)
            └── rear_right_wheel_link
```

## Why the official file had no wheel joints

`docs/official_reference/export.log` shows that the SolidWorks exporter configuration originally included four wheel links plus LiDAR, Camera and IMU links. It attempted to create all four wheel joints as continuous joints, but those wheel-joint export operations failed. The final `mowen.urdf` therefore collapsed to the single `base_link` mesh.

## CAD-derived wheel geometry

The official STL can be split into four symmetric wheel components. Their centres are:

```text
FL: +0.106, +0.125, 0.035791 m
FR: +0.106, -0.125, 0.035791 m
RL: -0.106, +0.125, 0.035791 m
RR: -0.106, -0.125, 0.035791 m
```

Approximate wheel envelope:

```text
radius = 0.037230 m
width  = 0.032000 m
wheelbase = 0.212000 m
track     = 0.250000 m
```

See `docs/derived_geometry.md` for the derivation and limitations.

## Frame normalization

The official STL's CAD origin is near ground level. For a cleaner mobile-base hierarchy this package now places:

- `base_footprint` at ground level;
- `base_link` at wheel-axis height, `z = 0.035791123 m`;
- the official STL visual is shifted by the opposite amount, so the visible robot remains in exactly the same physical place.

## Which file to use on the Windows visualizer

First use:

```text
preview/robotcar_kinematic_skeleton.urdf
```

It has no external mesh dependency and makes the four continuous joints easy to inspect.

Then use:

```text
preview/robotcar_mecanum_debug.urdf
```

Together with `meshes/base_link.STL`. It overlays four coloured debug cylinders on the official CAD wheels so you can verify that the reconstructed wheel centres line up with the real mesh.

For ROS use:

```text
urdf/robotcar.urdf.xacro
```

The plain-URDF equivalent is:

```text
urdf/robotcar.urdf
```

## Important scope of v0.2

This is a **kinematic/TF validation version**, not yet the final Gazebo wheel-contact model.

We intentionally have not guessed:

- wheel mass/inertia;
- wheel-ground friction;
- mecanum roller contact coefficients;
- motor/controller limits;
- Gazebo drivetrain plugin parameters.

After the four wheel joints are visually confirmed, the next version can add the physical collision/inertia layer and mecanum controller without changing the recovered wheel geometry.
