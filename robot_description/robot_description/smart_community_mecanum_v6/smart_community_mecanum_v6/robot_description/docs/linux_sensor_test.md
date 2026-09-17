# Linux sensor validation checklist (ROS Noetic + Gazebo Classic)

This checklist is intentionally deferred while the model is being prepared on Windows.

## 1. Build / model checks

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash

xacro $(rospack find robot_description)/urdf/robotcar.urdf.xacro > /tmp/robotcar_v6.urdf
check_urdf /tmp/robotcar_v6.urdf
```

## 2. Start sensor test world

```bash
roslaunch robot_description gazebo_sensor_test.launch
```

## 3. LiDAR

```bash
rostopic echo -n 1 /scan/header
rostopic hz /scan
rostopic echo -n 1 /scan/range_max
```

Expected baseline: `laser_link`, about 10 Hz, max range 10 m.

## 4. Depth camera

```bash
rostopic list | grep '^/camera/'
rostopic hz /camera/color/image_raw
rostopic hz /camera/depth/image_raw
rostopic echo -n 1 /camera/color/camera_info/header
rostopic echo -n 1 /camera/depth/points/header
```

Use `rqt_image_view` for color and depth images and RViz `PointCloud2` for `/camera/depth/points`.

## 5. IMU

```bash
rostopic echo -n 1 /imu/data/header
rostopic hz /imu/data
```

Expected baseline: `imu_link`, about 100 Hz.

## 6. Do not tune yet

Do not change TF locations merely to make visualization look convenient. If signs, orientations, or topics are wrong, first identify whether the issue is plugin convention, real sensor mounting, or frame definition.
