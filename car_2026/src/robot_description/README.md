# robot_description

智慧社区麦克纳姆机器人的模型、传感器坐标系与 Gazebo 接口配置。

## 模型组成

主入口为 `urdf/robotcar.urdf.xacro`，组合以下文件：

| 文件 | 内容 |
|---|---|
| `robotcar_base.xacro` | 车体网格、碰撞几何、惯性和四轮结构 |
| `robotcar_sensors.xacro` | 激光雷达、深度相机和 IMU 坐标系 |
| `robotcar_transmissions.xacro` | 四轮速度传动接口 |
| `robotcar_gazebo.xacro` | 接触参数、gazebo_ros_control 及平面运动插件 |
| `robotcar_sensor_plugins.xacro` | 激光、深度相机和 IMU 的 Gazebo ROS 插件 |

车体视觉网格为 `meshes/base_link.STL`。传感器规格、仿真参数和坐标配置位于 `config/`，参数说明位于 `docs/`；物理硬件规格与仿真参数分别记录。

## 启动

在 Docker 容器中编译并加载工作空间后，仅查看机器人模型：

```bash
roslaunch robot_description display.launch
```

比赛场景使用：

```bash
roslaunch robot_gazebo simulation.launch rviz:=true
```

仿真入口负责加载 Xacro、生成机器人、发布 TF，并启动底盘控制与 EKF。完整环境加载与启动顺序见 [工作空间说明](../../README.md)。

## 传感器话题

| 类型 | 话题 |
|---|---|
| 激光扫描 | `/scan` |
| 彩色相机 | `/camera/color/image_raw`、`/camera/color/camera_info` |
| 深度相机 | `/camera/depth/image_raw`、`/camera/depth/camera_info` |
| 点云 | `/camera/depth/points` |
| IMU | `/imu/data` |

传感器话题由 Gazebo 插件发布，仅启动模型显示不会产生这些数据。底盘融合里程计由 `robot_mecanum_control` 提供，导航定位由 `robot_navigation` 提供。

## 参数资料

- [物理参数](docs/physical_parameter_derivation.md)
- [传感器坐标系](docs/sensor_frame_derivation.md)
- [传感器仿真参数](docs/sensor_simulation_baseline.md)
