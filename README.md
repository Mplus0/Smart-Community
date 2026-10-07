# Smart-Community

## 项目简介

面向“智慧社区”机器人比赛场景的 ROS 1 / Gazebo 项目，主要开发工作位于 `car_2026/`。当前用于比赛场景仿真、麦克纳姆底盘控制、建图定位与自主导航，并为后续视觉识别和比赛任务整合提供基础。

## 当前状态

- 已完成 Gazebo 比赛场景搭建及机器人模型接入。
- 已实现麦克纳姆底盘仿真控制、轮式里程计（wheel odom）与 IMU 的 EKF 融合。
- 已接入 Cartographer 建图，保存了地图及建图状态文件。
- 已实现 `localization_map` / `planning_map` 双地图方案和 AMCL 定位。
- 已接入 `move_base` + TEB 导航，并调整了 footprint、速度和障碍物相关参数；窄转角通行效果仍需持续验证。
- 已建立 `robot_competition` 目录及占位文件，比赛任务逻辑待完善。

## 目录结构

```text
smart-community/
├── car_2026/                     # 主要 ROS catkin 工作空间
│   ├── src/
│   │   ├── robot_description/    # 机器人模型与传感器配置
│   │   ├── robot_gazebo/         # 比赛场景、仿真模型与启动入口
│   │   ├── robot_mecanum_control/# 麦轮控制、轮式里程计与 EKF
│   │   ├── robot_navigation/     # 建图、定位、地图与导航
│   │   └── robot_competition/    # 比赛任务框架（待完善）
│   ├── build/                   # 本地编译产物
│   └── devel/                   # 本地开发环境产物
├── docker/                      # ROS Noetic / Gazebo 开发环境
├── datasets/                    # 数据集预留，目前仅有 .gitkeep
├── docs/                        # 参考方案及相关文档
├── models/                      # 模型文件预留，目前仅有 .gitkeep
├── scripts/                     # 仓库级脚本预留，目前仅有 .gitkeep
└── 复赛资料/                     # 任务要求、场地图及识别素材
```

根目录另有机器人使用说明书、赛项介绍和项目实现流程参考文档。

### 主要功能包

- **robot_description**：存放 URDF/Xacro、车体网格、传感器与控制器配置，以及模型检查相关资料。
- **robot_gazebo**：提供比赛场地 world、场景模型、仿真插件和启动文件；包含随机车牌素材生成脚本。
- **robot_mecanum_control**：实现速度指令到轮速的转换、轮式里程计和键盘控制，配置 wheel odom + IMU 的 EKF 融合。
- **robot_navigation**：包含 Cartographer、AMCL、move_base、TEB 和 costmap 配置，以及地图、禁区配置和规划地图生成脚本。
- **robot_competition**：预留主控制、航点管理与记录、红绿灯/人员/车牌识别、拍照等文件。目前 `package.xml`、`CMakeLists.txt`、launch、脚本及航点配置均为空，尚不能作为完整功能包构建或运行。

## 主要功能

- **仿真与底盘控制**：在比赛场景中运行麦克纳姆机器人，提供传感器数据和融合里程计。
- **建图与双地图导航**：`localization_map` 用于 AMCL 定位；`planning_map` 在定位地图基础上加入配置的禁区，供路径规划使用。
- **自主导航**：使用全局规划与 TEB 局部规划。当前 TEB 禁止横移，主要通过 `linear.x` 和 `angular.z` 运动，底层麦轮横移能力仍保留。

## 基本运行

以下示例面向已配置依赖并完成编译的 ROS Noetic 环境。容器环境说明见 [docker/README.md](docker/README.md)，容器内工作空间为 `/workspace/car_2026`，宿主机路径为 `~/workspace/smart-community/car_2026`。

在容器终端中启动仿真：

```bash
cd /workspace/car_2026
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch robot_gazebo simulation.launch
```

另开一个容器终端启动导航：

```bash
cd /workspace/car_2026
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch robot_navigation navigation.launch
```

仿真入口已包含底盘控制和 EKF；导航入口已包含 AMCL 与双地图服务，无需重复启动。可通过 RViz 设置初始位姿和导航目标。

当前 `robot_competition` 仍是空文件框架，重新编译前需完善该包或将其排除出构建；上述命令不包含比赛任务程序。

## 后续工作

- 完善比赛主控制程序与任务状态机。
- 实现航点录入、保存和任务航点管理。
- 实现红绿灯、人员和车牌识别。
- 实现拍照，并与导航和比赛任务流程整合。
- 持续验证窄转角导航效果，开展完整比赛流程联调。
