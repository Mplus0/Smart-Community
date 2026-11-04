# v0.6 Gazebo / ROS sensor baseline

This file separates official hardware specifications from simulation-only choices.
Nothing in the baseline column should be reported as a measured hardware property.

## LiDAR

Official inputs:

- maximum range: 10 m
- scan frequency: 5-12 Hz
- scan angle: 360 deg
- angular resolution: 0.43-0.86 deg
- measurement rate: 5000 measurements/s

Default simulation operating point:

- 10 Hz
- 500 horizontal samples per scan
- 360 / 500 = 0.72 deg per sample
- 500 samples x 10 Hz = 5000 samples/s

Therefore the selected scan density simultaneously matches the official measurement-rate value and remains inside the official angular-resolution interval.

The official minimum range was not supplied. `0.10 m` is only a simulation baseline and must be replaced if the real LiDAR model/manual gives another value.

## Depth camera

Official inputs:

- monocular structured-light depth technology
- working range: 0.6-8.0 m
- accuracy at 1 m: +/-3 mm
- maximum depth resolution: 1280x1024

Default simulation choices:

- 640x480 to reduce Gazebo load during navigation development
- 15 Hz update rate
- 60 deg horizontal FOV

The last two values are NOT official specifications. The official accuracy value is also not converted into a Gaussian-noise sigma because the source table does not define a stochastic noise model.

The ROS-facing depth plugin is configured to expose color image, depth image, camera info, and point cloud interfaces.

## IMU

Official inputs:

- 3-axis accelerometer + 3-axis gyroscope
- acceleration range: +/-16 g
- angular-rate range: +/-2000 deg/s
- sample-rate range: 0.1-200 Hz

Default simulation choice: 100 Hz. Simulated Gaussian noise is kept at zero until the real sensor noise interpretation is confirmed. The real PCB/chip axis rotation is still a hardware-verification item; the current logical `imu_link` is aligned with `base_link`.

## Ultrasonic

Disabled. The current robot configuration does not use the ultrasonic module.

## Linux / Gazebo validation required

The following remain UNVERIFIED until ROS Noetic + Gazebo Classic is available:

- plugin libraries load successfully
- exact ROS topic names after plugin namespace resolution
- `/scan` rate and frame ID
- depth image / point-cloud publication
- camera optical-axis orientation in rendered data
- `/imu/data` rate and sign convention
- runtime performance at the selected camera resolution
