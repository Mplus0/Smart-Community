# Smart-Community 项目完整实现流程

> 版本：v1.0  
> 状态：当前基础系统方案已完成确认；感知模型细节等待主办方正式物料后补充  
> 适用阶段：Gazebo 复赛开发 → 系统联调 → 后续 Sim2Real 决赛迁移  
> 当前原则：先完成完整闭环，再统一调参；不为展示 UI 预留接口；不增加不必要 ROS 节点。

---

# 1. 项目目标

本项目目标是在 ROS1 Noetic + Gazebo Classic 11 环境中实现“智慧社区”自主巡检机器人，完成：

- Gazebo 智慧社区场景搭建；
- 四轮麦克纳姆机器人仿真；
- SLAM 建图；
- EKF 状态估计；
- AMCL 定位；
- 多点自主导航与避障；
- 交通灯、人物、交通标识、垃圾桶、火灾、异常温度、仪表、电动车、车牌等任务流程；
- 最终泊车；
- 一键启动；
- 任务结果保存、日志和播报；
- 异常恢复与完整比赛流程联调。

当前所有依赖主办方正式外观物料的视觉识别算法，统一暂缓到官方参考图或模型发布之后再确定。

---

# 2. 已确定的基础环境

## 2.1 Host

```text
Ubuntu 24.04
Docker
NVIDIA Container Toolkit
VS Code
Git
```

Host 负责 Docker、GPU、开发工具和项目文件管理。

## 2.2 Docker

```text
Ubuntu 20.04
ROS1 Noetic
Gazebo Classic 11
catkin
Python 3
```

当前 Docker 基础镜像沿用：

```text
osrf/ros:noetic-desktop-full
```

主要依赖方向：

```text
navigation
map-server
amcl
robot_state_publisher
joint_state_publisher
xacro
ros-control
gazebo_ros
gazebo_plugins
cv_bridge
image_transport
robot_localization
cartographer_ros
teb_local_planner
```

---

# 3. 总体软件架构

```text
                    ┌──────────────────────┐
                    │     Gazebo World     │
                    └──────────┬───────────┘
                               │
                    ┌──────────▼───────────┐
                    │ Robot / Sensors / TF │
                    └──────────┬───────────┘
                               │
                    ┌──────────▼───────────┐
                    │ robot_localization   │
                    │        EKF           │
                    └──────────┬───────────┘
                               │
               ┌───────────────┴───────────────┐
               │                               │
        Mapping Mode                    Competition Mode
               │                               │
        Cartographer                     map_server
                                               │
                                              AMCL
                                               │
                                         move_base
                                               │
                              GlobalPlanner + TEB Holonomic
                                               │
                                          /cmd_vel
                                               │
                                               ▼
                                         Robot Base

Camera ───────────────────► perception_node.py
                                      │
                                      ▼
                              stable task results
                                      │
                                      ▼
                               mission_manager.py
                                      │
                         move_base Action / results
                                      │
                                      ▼
                             route.yaml task flow
```

核心原则：

- 感知不直接控制导航；
- Mission 不实现视觉算法；
- Navigation 只负责运动；
- `route.yaml` 负责完整任务顺序和任务元数据；
- Gazebo Ground Truth 只允许用于调试和评估；
- 正式任务逻辑不得依赖 Gazebo API。

---

# 4. 项目目录与功能包职责

以下为后续开发统一采用的目标目录和包名。当前 `car_2026/src/` 中仅有 `CMakeLists.txt`，下列业务功能包及其文件尚待创建；目录树不代表功能已经实现。

```text
Smart-Community/
├── car_2026/
│   └── src/
│       ├── robot_description/
│       │   ├── urdf/
│       │   ├── meshes/
│       │   └── launch/
│       ├── robot_gazebo/
│       │   ├── launch/
│       │   ├── worlds/
│       │   ├── models/
│       │   ├── config/
│       │   └── rviz/
│       ├── robot_navigation/
│       │   ├── launch/
│       │   ├── config/
│       │   ├── maps/
│       │   └── rviz/
│       ├── robot_perception/
│       │   ├── scripts/
│       │   │   └── perception_node.py
│       │   ├── config/
│       │   ├── models/
│       │   └── launch/
│       ├── smart_community/
│       │   ├── scripts/
│       │   │   ├── mission_manager.py
│       │   │   ├── waypoint_loader.py
│       │   │   └── speech.py
│       │   ├── config/
│       │   │   └── route.yaml
│       │   └── launch/
│       └── robot_bringup/
│           └── launch/
│               ├── simulation.launch
│               ├── mapping.launch
│               └── competition.launch
├── datasets/
├── models/
├── docker/
├── docs/
├── scripts/
└── results/                  # 建议加入 .gitignore
```

正文中的包内路径均相对于 `Smart-Community/car_2026/src/`；容器中的工作空间为 `/workspace/car_2026`。

| 功能包 | 职责与主要文件 |
| --- | --- |
| `robot_description` | 机器人 URDF/Xacro、网格资源及模型加载 launch；几何基线放在 `urdf/robotcar.urdf.xacro` |
| `robot_gazebo` | Gazebo 场景、SDF 模型、机器人生成及仿真配置 |
| `robot_navigation` | EKF、Cartographer、AMCL、move_base、GlobalPlanner、TEB 与 costmap 配置；地图放在 `maps/` |
| `robot_perception` | `scripts/perception_node.py`、感知配置和运行所需模型 |
| `smart_community` | `scripts/mission_manager.py`、`scripts/waypoint_loader.py`、`scripts/speech.py` 及 `config/route.yaml`，负责比赛任务调度和结果管理 |
| `robot_bringup` | `launch/simulation.launch`、`launch/mapping.launch`、`launch/competition.launch`，负责组合启动各模块 |

创建功能包时，`package.xml` 的包名、`CMakeLists.txt` 的 `project(...)`、launch 中的 `pkg` 和 `$(find ...)` 引用均使用上述统一命名。

项目根目录的 `datasets/` 用于数据集，`models/` 用于训练产物和模型归档；`robot_perception/models/` 用于感知运行所需模型。项目根目录的 `scripts/` 用于开发辅助脚本，ROS 节点脚本放在对应功能包的 `scripts/` 中。

当前不创建 UI、系统管理器、导航管理器、感知管理器、独立日志节点或数据库。

---

# 5. 功能 1：ROS / Docker 基础环境

状态：✅ 已确认

目标：

```text
Host Ubuntu 24.04
    ↓
Docker
    ↓
Ubuntu 20.04 + ROS1 Noetic
    ↓
Gazebo 11
```

Sim2Real 原则：算法层只依赖 ROS Topic / Service / Action / TF，不直接依赖 Gazebo API。

---

# 6. 功能 2：机器人 URDF / Gazebo

状态：✅ 已确认

机器人几何基线导入后的目标路径（相对于项目根目录）：

```text
car_2026/src/robot_description/urdf/robotcar.urdf.xacro
```

当前工作区可用的模型源文件位于 `../src(robotcar_ws)/src/robotcar_gazebo/urdf/robotcar.urdf.xacro`（相对于 `Smart-Community/`）。后续导入时一并核对网格和插件引用；该来源路径不作为正式运行路径。

底盘：四轮麦克纳姆。

第一阶段保留 `libgazebo_ros_planar_move.so`，先验证：

```text
spawn
/cmd_vel
/odom
/scan
Camera
TF
```

只有实际发现与实车差异过大、导航不合理或 Sim2Real 必须轮速控制时，才替换底盘控制实现。

---

# 7. 功能 3：Gazebo 智慧社区场景

状态：✅ 已确认

场景按比赛示意图自行搭建，优先级：

```text
道路拓扑 > 建筑位置 > 任务区域 > 视觉细节 > 装饰
```

建议模型类别：

```text
arena
road
building
traffic_light
person_board
vehicle
license_plate
traffic_sign
trash_bin
fire_target
temperature_target
meter
ebike
```

采用独立 SDF 模型 + world include，分别放在 `robot_gazebo/models/` 和 `robot_gazebo/worlds/`，场景启动文件放在 `robot_gazebo/launch/`。正式物料发布后再替换视觉外观。

---

# 8. 功能 4：SLAM / 状态估计 / 导航核心

状态：✅ 已确认

正式组合：

```text
Cartographer
+
robot_localization EKF
+
AMCL
+
move_base
+
GlobalPlanner (Dijkstra)
+
TEB Local Planner (Holonomic)
```

Mapping Mode：

```text
Sensors → EKF → Cartographer → Map
```

Competition Mode：

```text
Map Server → AMCL
Sensors → EKF
AMCL + EKF → move_base
```

Cartographer 与 AMCL 不同时承担 `map -> odom`。

上述算法的项目配置统一放在 `robot_navigation/config/`，模块启动文件放在 `robot_navigation/launch/`，保存的地图放在 `robot_navigation/maps/`。`robot_bringup` 负责按建图或比赛模式组合启动。

---

# 9. 功能 5：AMCL / 静态地图 / TF

状态：✅ 已确认

TF 主链：

```text
map → odom → base_footprint → base_link → sensors
```

职责：

```text
map -> odom
Mapping：Cartographer
Navigation：AMCL

odom -> base_footprint
robot_localization EKF
```

AMCL 运动模型使用 `omni`。

---

# 10. 功能 6：GlobalPlanner + TEB + Costmap

状态：✅ 已确认

导航组件：

```text
move_base
GlobalPlanner
TEB Local Planner
global_costmap
local_costmap
```

TEB 采用 Holonomic：

```text
vx + vy + wz
```

麦克纳姆必要适配：

```text
max_vel_y > 0
min_turning_radius = 0
```

当前只追求能规划、能避障、能到点，完整流程跑通后再统一调参。

---

# 11. 功能 7：Waypoint / Task Point

状态：✅ 已确认

两种点：

```text
waypoint
task
```

基础字段：

```yaml
id:
type:
task:
x:
y:
yaw:
```

按需增加 `area`、`building`、`mode`、`parking_id`。路线文件统一放在 `smart_community/config/route.yaml`，由 `smart_community/scripts/waypoint_loader.py` 加载。导航使用 move_base Action。

---

# 12. 功能 8：感知层总体结构

状态：✅ 总体架构确认

第一版仅保留主要感知节点：

```text
robot_perception/scripts/perception_node.py
```

原则：

- 不为每个识别任务单独创建 ROS Node；
- 不强制创建自定义消息包；
- 使用当前 Gazebo 真实 Camera Topic；
- 模型启动时加载一次并全程常驻；
- Mission 读取稳定结果；
- 视觉算法不写入 `mission_manager.py`。

具体模型等待官方物料。

---

# 13. 功能 9：Mission Manager

状态：✅ 已确认

状态：

```text
INIT
NAVIGATE
TASK
NEXT_POINT
FINISH
ERROR
```

主程序放在 `smart_community/scripts/mission_manager.py`。第一版使用普通 Python 状态机，不强制 SMACH 或 Behavior Tree。

---

# 14. 功能 10：交通灯

状态：✅ 任务逻辑确认 / ⏳ 感知识别待官方物料

流程：

```text
停止线前 Task Point
→ 停车
→ 等待稳定灯色
→ GREEN 继续
→ RED / YELLOW / UNKNOWN 等待
```

具体 YOLO / HSV / ROI 方案待定。

---

# 15. 功能 11：卡通人形立牌

状态：✅ 任务逻辑确认 / ⏳ 类别和训练待官方物料

输出：

```text
A 街人数
B 街人数
社区总人数
外来人员数量
外来人员框选图片
```

街区归属由 `route.yaml` / Mission 决定，不训练 A/B 街位置类别。

---

# 16. 功能 12：车牌识别

状态：⏳ 待官方参考图

已知：三辆车、三个停车位结果。

暂不确定：

```text
YOLO + OCR
直接 OCR
OCR 引擎
自训练字符识别
车牌格式规则
```

---

# 17. 功能 13：交通标识

状态：✅ 任务逻辑确认 / ⏳ 视觉类别待官方物料

流程：

```text
路口前 Task Point
→ 稳定 sign_type
→ Mission 查询 route 配置
→ 选择合法下一路线
```

视觉只输出 `sign_type`，不直接发送导航 Goal。UNKNOWN 不默认通行。

---

# 18. 功能 14：垃圾桶

状态：✅ 任务框架确认 / ⏳ 感知识别待官方物料

最终输出：

```text
垃圾桶类别
开 / 闭
桶内垃圾
投放正确 / 错误
```

四类逻辑垃圾桶：可回收、其他、有害、厨余。

投放正确性优先采用规则映射：

```text
垃圾类别 → garbage_mapping.yaml → 正确垃圾桶 → 与实际垃圾桶比较
```

---

# 19. 功能 15：最终泊车

状态：✅ 已确认

第一版：

```text
parking_approach
→ final_parking
→ move_base
→ TEB Holonomic
```

最终 Goal 为完整 `x + y + yaw`。当前不实现独立 parking_controller 或视觉泊车。

---

# 20. 功能 16：一键启动 / 总 Launch

状态：✅ 已确认

```text
robot_bringup/launch/
├── simulation.launch
├── mapping.launch
└── competition.launch
```

用途：

```text
simulation.launch
→ Gazebo / Robot / Sensors

mapping.launch
→ Gazebo + Robot + EKF + Cartographer

competition.launch
→ Gazebo
→ Robot / TF
→ EKF
→ map_server
→ AMCL
→ move_base
→ perception
→ mission_manager
→ 自动执行完整流程
```

Mission 在 INIT 阶段自行等待核心依赖。当前方案完全不涉及 UI，也不预留 UI 接口。

---

# 21. 功能 17：结果、日志、图片与播报

状态：✅ 已确认

运行目录（相对于 `Smart-Community/` 项目根目录）：

```text
results/<timestamp>/
├── images/
├── mission_results.json
└── mission.log
```

职责：

```text
perception_node → 保存视觉证据图片
mission_manager → 结果、日志、控制台输出、播报调用
```

播报调用由 `smart_community/scripts/speech.py` 提供。每完成一个任务立即落盘。TTS 失败不能影响 Mission。当前不使用数据库。

结果目录应通过运行参数明确指定，不依赖节点启动时的工作目录。当前 Docker Compose 仅挂载 `car_2026/`，尚未挂载项目根目录的 `results/`；后续实现结果持久化时需配置对应挂载，并让感知与 Mission 使用同一次运行的输出目录。

---

# 22. 功能 18：楼宇火灾

状态：✅ 任务逻辑确认 / ⏳ 感知待定

区域：Building A、B、C。

流程：

```text
导航 → 停稳 → fire_count → 保存图片 → 记录 / 播报
```

`fire_count = 0` 是合法结果，不等于感知失败。

---

# 23. 功能 19：楼宇异常温度

状态：✅ 任务逻辑确认 / ⏳ 感知待定

区域：Building D。

核心结果：

```text
abnormal_floor
```

Mission 只记录最终楼层，不参与图像中的楼层判断。

---

# 24. 功能 20：站房仪表

状态：✅ 任务逻辑确认 / ⏳ 感知待定

当前按评分规则预期 2 个仪表。

第一版优先使用一个主要观察 Task Point；如官方场景无法同时观察，再拆成两个点。

Mission 不负责 OCR、指针检测、角度计算或刻度换算。

---

# 25. 功能 21：电动车

状态：✅ 任务逻辑确认 / ⏳ 感知待定

```text
A 街 → illegal_count
B 街 → illegal_count
停车区 → normal_count + fallen_count
```

区域身份由 `route.yaml` 提供。`illegal_count = 0` 是合法结果。

---

# 26. 功能 22：异常恢复与最终联调

状态：✅ 已确认

## 启动异常

```text
Map / TF / move_base / Camera 等核心依赖失败
→ ERROR
→ 禁止运动
```

## 导航异常

```text
move_base failed
→ move_base recovery
→ 当前 Goal retry 1 次
→ 仍失败
→ ERROR
```

## 感知任务异常

```text
Task timeout
→ retry 1 次
→ 仍失败
→ 记录 FAILED
→ 继续下一任务
```

单个视觉任务失败不能直接结束整场比赛。

ERROR 状态取消当前 move_base Goal，并停止发送新 Goal。

---

# 27. 正式开发实现顺序

## Phase 1：Docker / ROS 基础验证

完成 Docker、ROS Noetic、Gazebo 11、catkin workspace。

验收：`roscore`、`gazebo`、`catkin_make` 正常。

## Phase 2：机器人模型

实现 `robot_description`，导入 `urdf/robotcar.urdf.xacro`，验证 URDF/Xacro、TF、传感器和四轮模型。

完成 Gazebo spawn、`/cmd_vel`、`/odom`、`/scan`、Camera。

验收：机器人能够在空白场景稳定运动。

## Phase 3：基础 Gazebo 场景

创建 `robot_gazebo`。先完成地面、道路、墙体、建筑和基础障碍物，不急于制作全部视觉物料。

验收：机器人可以在完整道路拓扑中运动。

## Phase 4：EKF

使用 `robot_localization`，输入 odom + IMU，输出 `odom -> base_footprint`。

验收：TF 连续、无明显跳变。

## Phase 5：Cartographer 建图

启动 Gazebo + Robot + EKF + Cartographer + RViz，人工完成一次全场建图并保存地图。

验收：地图拓扑可以支撑导航。

## Phase 6：AMCL 定位

启动 map_server + AMCL + EKF，AMCL 使用 omni。

验收：机器人移动时定位稳定。

## Phase 7：move_base 基础导航

加入 GlobalPlanner + TEB + Costmap。

先测 RViz 单点 Goal，再测多次单点 Goal。

验收：能规划、能避障、能到点、`vy` 可用、无明显 TF / costmap 错误。

## Phase 8：Waypoint Mission

创建 `smart_community`，实现 `scripts/waypoint_loader.py`、`scripts/mission_manager.py`、`config/route.yaml`。

第一版只放普通 waypoint，验证：

```text
WP1 → WP2 → WP3 → parking_approach → final_parking
```

验收：Mission 可以自动运行到最终泊车。

## Phase 9：Task Point 机制

在 route 中加入 `type: task`。先使用测试结果验证：

```text
NAVIGATE → TASK → NEXT_POINT
```

验收：waypoint 和 task 可以混合执行。

## Phase 10：Perception 基础框架

创建 `robot_perception`，实现 `scripts/perception_node.py`。

先完成订阅 Camera、保存最新帧、模型加载入口和测试结果输出。

当前目标不是准确识别，而是先验证 Mission 与 Perception 的软件流程。

## Phase 11：结果与日志

实现：

```text
results/<timestamp>/
mission_results.json
mission.log
images/
```

每个 Task 完成后立即写文件。

## Phase 12：完整场景任务物料

官方资料发布后补充交通灯、人形立牌、车牌、交通标识、垃圾桶、火灾、异常温度、仪表和电动车。

## Phase 13：正式感知实现

逐项执行：

```text
官方物料分析
→ 确定标签 / 算法
→ 生成 / 收集数据集
→ 训练
→ 单任务 Gazebo 测试
→ 接入 perception_node
→ 接入 Mission
```

不要等待所有视觉任务都完成后再第一次联调。

---

# 28. 各感知任务当前状态

```text
交通灯       任务逻辑确认 / 视觉算法待定
人形立牌     任务逻辑确认 / 类别及训练待定
车牌         整体识别方案待官方参考图
交通标识     任务逻辑确认 / 类别及训练待定
垃圾桶       任务逻辑确认 / 视觉方案待定
火灾         任务逻辑确认 / 视觉方案待定
异常温度     任务逻辑确认 / 视觉方案待定
仪表         任务逻辑确认 / OCR/指针方案待定
电动车       任务逻辑确认 / 视觉方案待定
```

当前禁止为了提前写代码而自行猜测官方物料类别。

---

# 29. 参数调试顺序

完整流程第一次跑通前，只调必须调整才能运行的参数：

```text
TF
Topic 名称
footprint
麦克纳姆运动模型
基本速度限制
传感器配置
```

以下参数放到完整闭环后统一调：

```text
AMCL
EKF covariance
GlobalPlanner
TEB weights
max velocity
acceleration
costmap inflation
goal tolerance
task timeout
perception confidence
multi-frame window
parking tolerance
```

原则：

```text
先完成 → 再稳定 → 最后优化
```

---

# 30. 最终联调顺序

```text
Robot
→ TF / Sensors
→ EKF
→ Cartographer
→ Map
→ AMCL
→ move_base
→ Waypoint Mission
→ Task Point
→ Perception
→ 完整 Route
→ Final Parking
```

不要跳过前一层直接调后一层。

---

# 31. 正式比赛运行流程

最终目标：

```bash
roslaunch robot_bringup competition.launch
```

之后系统自动完成：

```text
启动 Gazebo / 实车 Driver
→ Robot / Sensors / TF
→ EKF
→ Map Server
→ AMCL
→ move_base
→ Perception
→ Mission INIT
→ 依赖检查
→ 自动执行 route.yaml
→ 各 Task
→ 最终停车
→ 结果汇总
→ FINISH
```

整个过程不得要求手动点击 RViz Goal、手动启动模型、手动切换任务、手动保存图片或手动输入下一目标。

---

# 32. 正式验收标准

## 启动

`competition.launch` 一条命令完成启动。

## 导航

完整多点路线可完成，无严重定位漂移，无频繁规划失败。

## Mission

Waypoint / Task 正确切换；单个感知失败不终止整场；导航严重失败进入 ERROR。

## 感知

官方物料发布后逐项测试准确率、稳定性、不同距离、不同角度和不同光照。

## 最终泊车

进入停车位且车头方向正确。

## 结果

必须生成：

```text
mission_results.json
mission.log
关键识别图片
```

## 时间

记录 `mission_duration_sec`。完整流程稳定后再优化总耗时。

---

# 33. 后续 Sim2Real 原则

Gazebo 与实车之间尽量只替换底层：

```text
Gazebo Sensors / Base
        ↓
Real Camera / LiDAR / IMU / Chassis
```

上层尽量保持：

```text
EKF
AMCL
move_base
Perception
Mission
route
```

Topic 不同时优先通过 launch remap 或参数处理，而不是大规模修改业务代码。

---

# 34. 当前明确不做的内容

基础比赛系统稳定前，不做：

```text
Web Dashboard
GUI
UI 控制接口
复杂 Behavior Tree
复杂 Recovery Framework
独立 Perception Manager
独立 Navigation Manager
数据库
过度 ROS 自定义消息
复杂视觉伺服
自研路径规划器
提前优化全部参数
```

---

# 35. 当前项目状态

```text
系统总体架构             ✅
Gazebo 技术路线           ✅
机器人模型方案            ✅
SLAM / 定位方案           ✅
导航方案                  ✅
Mission 方案              ✅
Waypoint / Task 机制      ✅
最终泊车                  ✅
一键启动                  ✅
结果 / 日志 / 播报        ✅
异常恢复                  ✅

交通灯任务逻辑            ✅ / 感知待定
人物任务逻辑              ✅ / 感知待定
车牌任务                  ⏳ 官方参考图
交通标识任务逻辑          ✅ / 感知待定
垃圾桶任务逻辑            ✅ / 感知待定
火灾任务逻辑              ✅ / 感知待定
异常温度任务逻辑          ✅ / 感知待定
仪表任务逻辑              ✅ / 感知待定
电动车任务逻辑            ✅ / 感知待定
```

---

# 36. 下一阶段实际工作

从现在开始不再继续扩展系统架构，按以下顺序进入代码实现：

```text
1. 按第 4 节命名整理 car_2026/src 下的 catkin package 目录
2. 导入模型至 robot_description/urdf/robotcar.urdf.xacro
3. 在 robot_gazebo/worlds 中建立最小 Gazebo baseline world
4. 验证 Robot / TF / Sensors / cmd_vel
5. 配置 robot_localization EKF
6. 配置 Cartographer
7. 保存地图
8. 配置 AMCL
9. 配置 move_base + GlobalPlanner + TEB
10. 验证单点导航
11. 在 smart_community 中创建 Mission 基础框架
12. 验证多点 waypoint
13. 加入 Task Point
14. 加入结果与日志
15. 完成 robot_bringup/launch/competition.launch
16. 跑通无真实感知的完整比赛流程
17. 官方物料发布后逐项补充感知
18. 最终统一调参、联调和压力测试
```

这份流程文件作为当前项目的主实现依据。后续如果方案发生正式变更，应同步更新本文件，再修改代码。
