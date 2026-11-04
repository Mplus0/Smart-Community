# Sensor frame derivation — v0.5

Coordinate convention:

- `+X`: robot forward
- `+Y`: robot left
- `+Z`: robot up
- `base_footprint`: ground plane
- `base_link`: wheel-axis plane

## Horizontal positions

Official STL envelope:

- front edge: `X = +0.16499999 m`
- rear edge: `X = -0.14600000 m`

User measurements:

- LiDAR center is about `73 mm` behind the robot front.
- Depth camera center is about `30 mm` behind the robot front.
- IMU is `103 mm` ahead of the robot rear and lies on the lateral centerline.
- LiDAR, camera, and IMU are centered laterally (`Y ~= 0`) based on the real-robot views / measurement.
- IMU is mounted on the same upper plane as the LiDAR base.
- Ultrasonic sensors are not used in the current robot.

Therefore:

```text
laser_link X  = +0.16499999 - 0.073 = +0.09199999 m
camera_link X = +0.16499999 - 0.030 = +0.13499999 m
imu_link X    = -0.14600000 + 0.103 = -0.04300000 m
```

All three use `Y = 0` for the current baseline.

## Vertical positions

The user has not supplied direct height measurements, so v0.5 does not claim measured Z values.
They are recovered from the official STL geometry:

- LiDAR component center CAD Z ~= `0.157803 m`
- upper sensor mounting plane CAD Z ~= `0.143591 m`
- front camera assembly CAD Z ~= `0.096079 m`
- CAD wheel-center Z = `0.035791123 m`

Because `base_link` is defined on the wheel-center plane, the frame Z values are:

```text
laser_link Z  ~= 0.157803 - 0.035791123 = 0.122011877 m
imu_link Z    ~= 0.143591 - 0.035791123 = 0.107799877 m
camera_link Z ~= 0.096079 - 0.035791123 = 0.060287877 m
```

These Z values are **STL-derived estimates**, not direct measurements.

## Orientation status

- `laser_link`: aligned with robot frame for now; 0-degree scan direction must be verified in Linux / real driver.
- `camera_link`: aligned with robot frame, facing forward.
- `camera_optical_frame`: standard ROS optical transform (`+Z forward, +X right, +Y down`).
- `imu_link`: logical simulation frame aligned with `base_link`. The physical PCB/chip axis rotation is not yet known and must be verified from the real IMU module / driver documentation.

## Mass update

The official parameter table states empty mass = `5.2 kg`.
The old CAD export contained a lumped mass of only `0.486218814966626 kg`; this is no longer used as the total robot mass.

Until the four individual wheel masses are measured, v0.5 scales the previous CAD-based mass distribution uniformly so that the total is exactly `5.2 kg`:

- each wheel: `0.160421599492 kg`
- base body: `4.558313602032 kg`
- total: `5.2 kg`

This preserves the prior relative distribution but remains a simulation approximation.

## Speed update

With official maximum chassis speed `0.6 m/s` and STL wheel radius `0.037230282 m`:

```text
max nominal wheel angular speed ~= 0.6 / 0.037230282
                               ~= 16.115913385 rad/s
```

This is now the default wheel-speed limit. Motor torque / gear-ratio data are still unavailable.
