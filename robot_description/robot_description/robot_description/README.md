# robot_description

智慧社区机器人描述包。该目录根据官方主控机提供的 `mowen` 模型包重新整理，目标是作为 `Smart-Community/car_2026/src/robot_description/` 的干净几何基线。

## 1. 当前原则

本版本只整理**官方已经提供且能够追溯**的机器人本体信息，不猜测四轮、LiDAR、Camera、IMU 的实际安装位置。

当前 TF 只有：

```text
base_footprint
    └── base_link
```

后续传感器和轮组位置应依据实车测量或新的官方资料补充。

## 2. 目录

```text
robot_description/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── display.launch
├── meshes/
│   ├── base_link.STL
│   └── base_link.STL1
├── urdf/
│   ├── robotcar.urdf.xacro
│   └── robotcar.urdf
├── preview/
│   └── robotcar_web.urdf
└── docs/
    └── official_reference/
        ├── mowen.urdf
        ├── mowen.urdf.xacro
        └── mowen.csv
```

## 3. 官方模型基准

正式运行基准采用官方 `gazebo.launch` 实际加载的 `mowen.urdf`，而不是随包提供但存在路径问题的 `mowen.urdf.xacro`。

官方运行 URDF 中：

- `base_footprint -> base_link` 为 fixed joint；
- joint origin 为 `0 0 0`；
- Visual 使用 `base_link.STL`；
- Collision 同样使用 `base_link.STL`；
- 质量、质心和惯性参数已原样保留在新的 `robotcar.urdf(.xacro)` 中。

注意：这些物理参数只是**官方文件中的导出值**，尚未通过实车称重或惯性测量重新验证。

## 4. 已发现的官方源文件差异

随包提供的 `mowen.urdf.xacro` 与官方实际 spawn 的 `mowen.urdf` 存在差异：

1. Xacro 引用 `meshes/mecanum/base_link.STL`，但压缩包中没有 `meshes/mecanum/`；
2. Xacro 的 `base_joint` Z 偏移为 `0.0815 m`，而实际被 `gazebo.launch` 使用的 URDF 为 `0 m`；
3. 两份文件中的质心定义不一致。

因此本整理版本以实际运行的 `mowen.urdf` 为第一阶段基准，并将所有原文件保存在 `docs/official_reference/` 供追溯。

## 5. Mesh 信息

当前运行 Mesh：`meshes/base_link.STL`。

从 STL 顶点范围计算出的包围尺寸约为：

```text
X: 0.311 m
Y: 0.282 m
Z: 0.2905 m
```

另一份官方 `base_link.STL1` 被保留，但当前官方 URDF 并未引用它，不应在未确认用途前切换为正式模型。

## 6. ROS 使用

把本目录放入：

```text
Smart-Community/car_2026/src/robot_description/
```

工作空间中运行：

```bash
cd /workspace/car_2026
catkin_make
source devel/setup.bash
roslaunch robot_description display.launch
```

然后在 RViz 中添加 `RobotModel`，Fixed Frame 使用 `base_footprint`。

## 7. Windows / Web 可视化

不支持 Xacro 的可视化工具可直接使用：

```text
urdf/robotcar.urdf
```

ROS `package://` 路径无法解析时，可以尝试上传：

```text
preview/robotcar_web.urdf
meshes/base_link.STL
```

其中 Web 预览文件使用相对 mesh 路径；不同网站对外部 mesh 的上传方式可能不同。

## 8. 当前暂不加入的内容

以下内容需要真实安装尺寸或新的官方资料，当前不猜测：

- 四个麦克纳姆轮的独立 link / joint；
- LiDAR link 与扫描参数；
- Camera link / optical frame；
- IMU link；
- Gazebo 运动插件；
- Gazebo LiDAR / Camera / IMU plugins；
- 简化碰撞体；
- 重新标定后的质量、质心和惯性。

Gazebo 控制和传感器插件后续建议放在 `robot_gazebo` 中管理，使 `robot_description` 保持为可复用的机器人几何描述。
