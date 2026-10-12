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

运行入口使用 Xacro，`urdf/robotcar.urdf` 和 `docs/official_reference/` 为参考材料；修改参考 URDF 不会自动改变仿真。`config/sensor_simulation.yaml` 等文件记录参数基线，实际 Gazebo 传感器定义在 Xacro 中，修改基线 YAML 后应核对并同步实际模型。

## 底盘、里程计与 TF

Gazebo 使用 `librobotcar_planar_move.so` 对 `base_footprint` 施加带质心补偿的平面运动，订阅 `/cmd_vel`，命令超时为 0.5 秒。四轮仍由 `gazebo_ros_control` 的速度控制器驱动旋转；平面插件本身不发布里程计或 TF。插件由 `robot_mecanum_control` 编译，因此应构建整个工作空间再启动仿真。

控制节点将速度转换为四轮命令，轮式里程计发布 `/wheel/odom`；EKF 融合轮式 vx/vy 与 IMU yaw/yaw rate，发布 `/odometry/filtered` 和 `odom → base_footprint`。AMCL（导航）或 Cartographer（建图）提供 `map → odom`，robot_state_publisher 发布车体、轮子和传感器变换。不要同时启动两套全局定位 TF 发布者。

底盘几何和限速见 [mecanum.yaml](../robot_mecanum_control/config/mecanum.yaml)，质心补偿说明见 [平面运动插件文档](../robot_mecanum_control/docs/COG_COMPENSATED_PLANAR_MOVE.md)。仿真驱动方式不等同于完整滚子接触动力学。

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

当前传感器仿真基线为：雷达 10 Hz、500 采样、0.10–10 m；RGB-D 640×480、15 Hz、水平视场 60°、裁剪距离 0.6–8 m；IMU 100 Hz。超声波未启用。导航中雷达使用上限为 8 m，与传感器自身 10 m 量程不同。上述分辨率、频率和噪声等包含仿真选择，不应全部当作实机规格。

`simulation.launch` 可设置 `gui`、`paused`、`rviz`、`world_name` 和出生位姿 `robot_x/robot_y/robot_z/robot_yaw`。默认 world 出生位置为 `(1.696892, -1.680704, 0.023994)`、yaw `1.568056`；这些不是路线 YAML 中的 map 坐标。改变出生位置后，应在 RViz 重新确认定位。

## 参数资料

- [物理参数](docs/physical_parameter_derivation.md)
- [传感器坐标系](docs/sensor_frame_derivation.md)
- [传感器仿真参数](docs/sensor_simulation_baseline.md)
