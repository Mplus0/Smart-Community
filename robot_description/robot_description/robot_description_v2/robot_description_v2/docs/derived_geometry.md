# Official CAD geometry recovered for the mecanum base

The following values are derived directly from connected components in the official `base_link.STL` and are used only where the mesh gives a clear symmetric result.

## Coordinate convention

The reconstructed package adopts ROS REP-103:

- +X: forward
- +Y: left
- +Z: up

The +X end of the official mesh is the tapered/front end, so this convention is consistent with the CAD geometry.

## Four wheel centres

| Wheel | CAD centre (x, y, z) m |
|---|---|
| Front left | `( +0.106000002, +0.125000000, 0.035791123 )` |
| Front right | `( +0.106000002, -0.125000000, 0.035791123 )` |
| Rear left | `( -0.106000002, +0.125000000, 0.035791123 )` |
| Rear right | `( -0.106000002, -0.125000000, 0.035791123 )` |

Derived centre-to-centre dimensions:

- wheelbase: `0.212000004 m`
- track width: `0.250000000 m`

## Wheel envelope

Each isolated wheel component has approximately the same bounding envelope:

- X extent: `0.074378170 m`
- Y extent: `0.032000005 m`
- Z extent: `0.074460564 m`

For the simplified wheel cylinder used in the kinematic skeleton:

- nominal radius: `0.037230282 m`
- nominal width: `0.032000005 m`
- axle: Y axis

This cylinder represents only the main wheel envelope. It does not simulate individual 45-degree mecanum rollers.

## Evidence from official export log

The SolidWorks URDF exporter configuration explicitly contained these intended child links:

- `front_left_wheel`
- `front_right_wheel`
- `back_left_wheel`
- `back_right_wheel`
- `laser_link`
- `camera_Link`
- `IMU_link`

It also recorded the wheel joints as `continuous`. During export, creation of the four wheel joints failed. This explains why the final official `mowen.urdf` contains only `base_link` even though the CAD/export configuration had a multi-link robot structure.

## Not yet treated as final simulation parameters

The following are deliberately not inferred from geometry alone:

- individual wheel mass and inertia
- chassis mass after separating wheel masses
- wheel/ground friction coefficients
- mecanum roller friction model
- controller effort/velocity limits
- motor reduction ratio
- encoder parameters

These belong to the Gazebo dynamics/controller stage and should be confirmed from hardware data or tuned explicitly as simulation parameters.
