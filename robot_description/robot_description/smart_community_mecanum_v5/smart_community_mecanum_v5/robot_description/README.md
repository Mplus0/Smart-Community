# robot_description v0.5

Robot geometry/TF package reconstructed from the official Mowen STL and real-robot measurements.

Main model:

```text
urdf/robotcar.urdf.xacro
```

Components:

```text
robotcar_base.xacro          chassis + 4 physical wheel joints
robotcar_sensors.xacro       LiDAR + camera + optical frame + IMU TF
robotcar_transmissions.xacro wheel transmissions
robotcar_gazebo.xacro        mecanum contact baseline + gazebo_ros_control
```

The high-detail official STL is used for visual appearance; simplified primitives are used for physics collision.

Sensor position evidence and unresolved items are documented in:

```text
docs/sensor_frame_derivation.md
config/sensor_frames.yaml
```

Current total empty mass baseline is 5.2 kg from the official hardware parameter table. Individual component masses/inertia remain approximations until measured.
