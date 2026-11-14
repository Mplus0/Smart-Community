# PROJECT_MAP — AIC Smart Community

审阅日期：2026-10-14。分支：ai/sim-optimization。基线提交：9e0bfd2747ca2f21b59e948dcda74ca4af9afac8。

本文件是首次系统性静态代码审阅。审阅开始时 Git 工作区干净；本轮唯一新增文件为 PROJECT_MAP.md。未运行 ROS、Gazebo、业务脚本、构建、训练或依赖安装，未调整参数、未修复、未提交。文中的启动命令用于说明现有流程，不代表本轮已执行或验收成功。

证据等级：
- **已确认**：当前源码、配置、Git 或文件内容直接支持。
- **风险推导**：触发条件和代码路径明确，但未作动态复现。
- **待确认**：依赖外部包内部行为、实际运行环境、比赛现场或实车测量。

路径约定：下文 P = car_2026/src；例如 P/robot_navigation 对应仓库内 car_2026/src/robot_navigation。所有表中缩写路径均按此展开。函数名引用的是仓库实现；外部 ROS 算法只说明配置选用，不宣称已审阅其内部源码。

## 1. Project Overview

项目是 ROS 1 的麦克纳姆全向移动机器人比赛系统。当前可见架构面向 PC/Gazebo 仿真：已包含车体模型、激光/相机/IMU 仿真、轮式里程计与 EKF、Cartographer 建图、AMCL 定位、双地图导航、人物/车牌/交通灯视觉、航点任务状态机、结果留证和 PyQt5 控制台。没有找到完整实车驱动与实车启动入口。

比赛目标依据：比赛资料/复赛资料/任务要求.txt。包括沿指定路径巡检、整街区人员统计、车牌字符识别、红灯停车且车身不越停止线、不得压越车道线，以及终端与图像结果一一对应。图示道路的几何合规性及 PDF 中其他要求：待确认。

核心认识：
1. 系统的运动指令是 Twist(vx, vy, wz)，不是汽车转向角/油门；不能套用 Ackermann 横纵向控制架构。
2. 主任务控制器只下发 move_base 目标和停车零速度，不自行计算路径跟踪控制。
3. 仿真车体由自定义平面速度插件直接驱动；轮关节由另一条逆运动学链路驱动。两者并行，不能把当前仿真等同于真实麦轮接触动力学。
4. 视觉主要服务到点任务；导航障碍物来自 LaserScan，不来自人物 YOLO 或深度点云。
5. 交通灯是三分类器，不能证明画面中有交通灯，但当前任务层确实据其输出自动放行。详见 8.1、15/R01。
6. FINISH 只表示路线执行完毕，人物/车牌失败后仍可继续并最终 FINISH。

## 2. Tech Stack

| 类别 | 当前证据与用途 |
|---|---|
| 主要语言 | Python 3：24 个脚本；C++：2 个 Gazebo 插件；Bash 启停；Lua 配置 Cartographer；XML/Xacro/SDF/URDF；YAML 参数 |
| 中间件 | ROS 1 Noetic，rospy/roscpp、topic、TF、actionlib；未发现 ROS 2/rclcpp/rclpy |
| 仿真 | Gazebo Classic 11，ODE，gazebo_ros、gazebo_ros_control、传感器插件 |
| 构建 | Catkin/CMake；主工作空间 catkin_make；Cartographer 独立 catkin_make_isolated --install --use-ninja |
| 导航定位 | move_base、global_planner/GlobalPlanner、teb_local_planner/TebLocalPlannerROS、costmap_2d、AMCL、robot_localization EKF、Cartographer |
| 视觉 | Ultralytics YOLO、PyTorch、HyperLPR3、ONNX Runtime、OpenCV、NumPy、Pillow |
| GUI | PyQt5 + rospy，Linux PTY 管理比赛脚本 |
| 包管理 | apt/rosdep 管理系统与 ROS；uv 创建独立 Python 环境并安装 pip 锁定依赖；package.xml 声明 ROS 依赖 |
| 环境 | Docker Compose；ROS/任务为系统 Python 3.8，视觉为 Python 3.10.18 venv；未见 Conda |
| 硬件 SDK | 未发现底盘、CAN、串口、相机或激光实车 SDK 实现；硬件规格 YAML 不是驱动 |
| CI/测试框架 | 未发现 CI workflow、pytest/unittest/rostest/gtest 配置或自动比赛评分器；存在视觉评估脚本和镜像构建期 import 检查 |

依赖版本来源：docker/Dockerfile 与 P/robot_perception/docs/requirements-cpu.lock.txt。声明值包括 uv 0.8.22、torch 2.14.0+cpu、torchvision 0.29.0+cpu、ultralytics 8.4.159、hyperlpr3 0.1.3、onnxruntime 1.23.2、numpy 2.2.6、opencv-python 5.0.0.93、PyYAML 6.0.3、Pillow 12.3.0。这里只核对文件声明；实际可安装性、镜像内版本及 ABI 兼容性均待确认，没有联网安装验证。

## 3. Repository Structure

本轮扫描到 578 个文件，与 Git 跟踪文件数量一致。重点完整阅读 24 个 Python 源文件、2 个 C++ 源文件、13 个 launch、六包构建清单、核心 YAML/Lua/Xacro、启停脚本及主要说明文档。模型/图片/STL 大文件以用途、引用和必要哈希核对为主；没有反序列化权重或遍历第三方实现。

| 目录 | 职责 / 关键内容 |
|---|---|
| car_2026/ | ROS Catkin 工作空间；README、src、运行时 results 约定 |
| P/robot_description/ | 当前 Xacro 装配入口、车体/轮子/惯性、传感器 TF 与 Gazebo 插件；meshes；官方参考导出及推导文档 |
| P/robot_gazebo/ | competition_field.world、场景 SDF/纹理、TrafficLightPlugin、随机车牌纹理生成 |
| P/robot_mecanum_control/ | X 型麦轮 IK/FK、轮速命令、轮里程计、EKF launch、自定义 RobotcarPlanarMove、键盘遥控 |
| P/robot_navigation/ | AMCL/Cartographer/move_base 启动与参数，定位/规划地图，禁区和参考图生成 |
| P/robot_perception/ | 三路视觉节点、模型校验/缓存工具、模型资产、消息接口说明、隔离环境依赖 |
| P/robot_competition/ | Controller 状态机、路线校验/录入、感知窗口/融合、红绿灯等待、结果与证据保存 |
| P/tools/ | start.sh、start_competition.sh、stop.sh、gui.py；不是独立 Catkin 包 |
| docker/ | Dockerfile、Compose、构建上下文忽略规则、部署文档 |
| models/traffic_light/ | 交通灯部署源 best.pt、best.onnx、预训练 yolo11n-cls.pt |
| scripts/traffic_light/ | 数据划分、训练、分类评估、ONNX 导出 |
| yolo_train/ | 已有 raw/train/val/test 图像、best/last 权重；没有整场比赛评测结果 |
| 比赛资料/ | 任务要求、图示、人物/车牌/灯具素材、使用手册 PDF、赛项介绍 PDF、报告模板 DOCX |
| .gitignore | 排除构建、缓存、bag、日志、比赛结果与本地交通灯部署文件 |

二进制比赛 PDF/DOCX 仅清点，未展开逐页审阅；不能以此文件代替官方规则或硬件手册。资料中的驱动安装/协议细节需下一轮确认。未发现 AGENTS.md。

## 4. Build / Environment

### 4.1 已有环境流程

docker/Dockerfile：
- 基础镜像 osrf/ros:noetic-desktop-full，安装 ROS navigation/TEB/AMCL/EKF、Gazebo、Qt 等。
- Cartographer 固定提交 877157a0d91788a7700221d87232d412cb3c1ef4。
- Cartographer ROS 固定提交 c138034db0c47fe0ea5a2abe516acae02190dbf5。
- 系统 ROS Python 与视觉 venv 分离。视觉 launch-prefix 指定解释器，PYTHONPATH 加入 ROS 消息模块；不使用视觉 venv 编译工作空间。
- 镜像只做依赖与导入检查，不自动编译挂载的业务工作空间，也不自动准备 HyperLPR 用户缓存。

docker/docker-compose.yml：
- 服务 ros，容器 smart-community-dev，host 网络。
- 只挂载 ../car_2026 到 /workspace/car_2026；顶层 models、scripts、yolo_train 不在挂载内。
- 使用 Linux X11、/dev/dri、NVIDIA GPU 设备；CPU 推理锁文件不会因此自动切换 CUDA。
- 没有独立 Windows/无 GPU 配置。当前 Windows checkout 的实际 Docker/WSL 运行条件待确认。

文档所述工作空间构建顺序（本轮未执行）：

~~~bash
cd /workspace/car_2026
source /opt/ros/noetic/setup.bash
source /opt/cartographer_ws/install_isolated/setup.bash
catkin_make -j2
source devel/setup.bash --extend
~~~

### 4.2 构建和部署边界

- P/CMakeLists.txt 在 Git 中 mode=120000，目标为 /opt/ros/noetic/share/catkin/cmake/toplevel.cmake；当前 Windows 文件系统中却是普通文本文件，内容为该目标路径。若原样以普通文件挂载到 Linux，CMake 入口不可直接使用。Linux checkout 是否正确恢复链接待确认。
- P/robot_competition/CMakeLists.txt:16 安装 README.md 和 P2B.md；后者在当前仓库不存在。会影响 install 阶段；没有据此断言普通 catkin_make 一定失败。
- P/robot_perception/models/person_best.pt 和四个 HyperLPR ONNX 存在。四个 ONNX 的大小及 SHA-256 与 models_manifest.json 一致（本轮只读核验）。
- 默认需要的 P/robot_perception/models/traffic_light_best.pt 当前不存在。源 models/traffic_light/best.pt 存在，但 Compose 不挂载顶层 models；Docker 也不复制它。README 已明确要求人工部署，因此属于当前 checkout 的部署前置缺项。
- HyperLPR prepare_hyperlpr_models.py -> verify_cache 在导入 HyperLPR 前核验当前 HOME 下缓存，避免缺失时隐式下载。容器缓存是否已准备待确认。
- 依赖 apt 包与基础镜像未按 digest/全量版本锁定；视觉锁文件不覆盖整个运行环境。

## 5. Program Entry Points

| 入口 | 经代码确认的调用关系 | 配置入口 / 前置条件 |
|---|---|---|
| 比赛主入口 | P/robot_competition/launch/competition.launch -> scripts/main_controller.py -> main() -> Controller.run() -> initialize()/navigate()/task() | launch 在节点私有 task_config 命名空间载入 task_config.yaml；Controller 读取 ~waypoints_file 后 load_route |
| 集中基础环境 | P/tools/start.sh -> generate_random_plates.py -> roslaunch robot_competition environment.launch | source Noetic、Cartographer、devel；随机车牌脚本有网络和写文件副作用 |
| 基础环境 launch | P/robot_competition/launch/environment.launch -> simulation.launch + navigation.launch + perception.launch | 开启 RViz 与交通灯；固定视觉解释器/字体 |
| 人工确认后比赛 | P/tools/start_competition.sh -> 读取 yes -> competition.launch navigation_test_mode:=false | 不重新启动底盘/导航/视觉；人工确认定位 |
| GUI | P/tools/gui.py -> main() -> CompetitionGUI -> start_environment()/start_competition()/stop_all() | 调用上述 shell；用户弹窗确认后 PTY 自动向脚本输入 yes |
| 仿真 | P/robot_gazebo/launch/simulation.launch -> gazebo_ros/empty_world.launch、spawn_model、robot_state_publisher、mecanum_control、EKF | competition_field.world + robotcar.urdf.xacro |
| 仿真辅助脚本 | P/robot_gazebo/scripts/start_simulation.sh -> 随机车牌 -> pkill -9 旧 Gazebo -> simulation.launch | 遗留 melodic 条件 source；不作为推荐日常入口，本轮未运行 |
| 导航 | P/robot_navigation/launch/navigation.launch -> localization.launch + planning_map_server + move_base | move_base/costmap/TEB 多份 YAML；默认 use_sim_time=true |
| 全局定位 | P/robot_navigation/launch/localization.launch -> map_server + amcl | localization_map.yaml + amcl.yaml |
| 局部融合 | P/robot_mecanum_control/launch/localization.launch -> ekf_localization_node | ekf.yaml；与上一个同名 launch 功能不同 |
| 建图 | P/robot_navigation/launch/mapping.launch -> cartographer_node + occupancy_grid_node + RViz | cartographer_2d.lua；与 AMCL 互斥使用同一 map->odom TF |
| 底盘 | P/robot_mecanum_control/launch/mecanum_control.launch -> controller_spawner、MecanumCmdVel、WheelOdometry | mecanum_controllers.yaml + mecanum.yaml |
| 视觉 | P/robot_perception/launch/perception.launch -> 三个子 launch -> 各 Python main()/spin() | 默认人物/车牌开、灯关；集中入口把灯打开 |
| 模型显示 | P/robot_description/launch/display.launch -> xacro、joint_state_publisher_gui、robot_state_publisher、RViz | 无比赛/底盘运动闭环 |
| 辅助离线入口 | waypoint_manager.main 校验；waypoint_recorder.main 录 TF；camera_capture.main 采图；地图生成与四个训练脚本 main | 录点/采图/生成/训练会写文件，不属于只读审阅命令 |
| 实车入口 | 未发现 | 驱动、硬件 launch、使能/急停协议均待确认 |

## 6. Simulation Startup

### 6.1 当前默认顺序

1. 先在合适的 Linux/Docker 环境完成依赖、Catkin 编译、交通灯模型部署、HyperLPR 缓存和字体准备。
2. 加载工作空间；执行 bash src/tools/start.sh。脚本先生成两张车牌纹理，然后 exec environment.launch。
3. environment.launch 并行启动仿真、导航、视觉；XML include 的排列不能理解为所有节点已就绪的串行屏障。
4. simulation.launch 启动 Gazebo world，展开 Xacro，通过 spawn_model 放入机器人，并启动关节控制、IK、轮里程计、EKF、可选 RViz。
5. navigation.launch 启动定位地图、AMCL、规划地图、move_base；视觉各节点加载模型后开始消费相机。
6. 操作者在 RViz 2D Pose Estimate 初始化/核对定位。
7. bash src/tools/start_competition.sh，确认 yes 后启动 Controller；Controller 等待 /clock 和 move_base action server，再按路线执行。
8. stop.sh 先向当前用户的 competition.launch 发送 SIGINT，最多等待 20 秒；再停止 environment.launch，最多等待 20 秒。它不是硬件急停，不负责所有分终端独立 launch。

保留现有纹理可直接 roslaunch robot_competition environment.launch。分终端等价路径：
simulation.launch rviz:=true -> navigation.launch -> perception.launch enable_traffic_light:=true（含正确字体/解释器）-> competition.launch。前面箭头表示操作依赖，仍需实际检查节点就绪。

### 6.2 README 与代码一致性

已确认一致：六包结构、集中入口、12 点路线、3 帧绿灯、35 秒等待、task 端源时效 3 秒、FINISH 语义、双 Python、模型手工部署、TEB 小幅横移。

存在差异：
- 根 README 目录示例写“复赛资料/”，实际顶层为“比赛资料/”；所谓根目录 PDF 实际在比赛资料中。
- P/robot_perception/docs/interface.md 末尾仍称任务层“不消费交通灯”；当前 Controller 已订阅并放行，属于过时描述。
- teb_local_planner.yaml 注释写 lateral motion disabled，但 max_vel_y=0.05，不能按注释认为禁用横移。
- sensor_frames.yaml 相机姿态为零，实际 Xacro 为 pitch=-0.0872665 rad；sensor_simulation.yaml near=0.6，实际插件 near=0.05。
- WINDOWS_STAGE.md 声称有纯 Python IK/FK tests，当前仓库未找到相应测试文件；其摩擦/fdir1描述也落后于当前等向低摩擦方案。
- 辅助 start_simulation.sh 仍检查 melodic；主 start.sh 正确加载 Noetic。

启动是否成功、Gazebo 插件解析/渲染是否正确、ROS topic 实际频率：待确认。

## 7. Runtime Data Flow

~~~mermaid
flowchart TD
  A["start.sh / GUI"] --> E["environment.launch"]
  E --> G["Gazebo world + robot Xacro"]
  E --> N["AMCL + maps + move_base"]
  E --> V["三路视觉节点"]
  G --> S["scan / camera / imu / joint_states"]
  S --> O["WheelOdometry -> wheel/odom"]
  O --> K["EKF"]
  S --> K
  K --> F["odometry/filtered + odom->base_footprint"]
  S --> N
  F --> N
  S --> V
  V --> J["JSON + 标注图"]
  B["start_competition.sh"] --> C["Controller"]
  J --> T["PerceptionClient / TaskProcessor / TrafficWait"]
  C --> T
  T --> C
  C --> N
  N --> CMD["cmd_vel"]
  C --> CMD
  CMD --> PM["RobotcarPlanarMove -> 车体速度"]
  CMD --> IK["MecanumCmdVel -> 轮速控制器"]
  IK --> G
  PM --> G
  T --> R["ResultManager -> JSON / 日志 / 图像"]
  R --> UI["GUI"]
~~~

| 环节 | 文件 -> 类/函数/配置 | 输入 / 上游 | 输出 / 下游 |
|---|---|---|---|
| 配置 | competition.launch；Controller.initialize；waypoint_manager.load_route/validate_route | launch args、ROS 参数、路线 YAML | 校验后的 route.points、任务参数、action client |
| 环境地图 | simulation.launch -> competition_field.world；navigation.launch -> map_server | SDF world、PGM/YAML | Gazebo world；/map、/planning_map |
| 传感器 | robotcar_sensor_plugins.xacro | Gazebo 渲染/物理状态 | /scan、RGB/depth/CameraInfo/PointCloud2、/imu/data |
| 轮反馈 | gazebo_ros_control + joint_state_controller | 四轮关节状态 | /joint_states -> WheelOdometry._on_joint_state |
| 原始里程计 | mecanum_wheel_odometry.py -> forward_kinematics | 轮速 rad/s、时间戳 | /wheel/odom；不发 TF |
| 局部定位 | ekf.yaml -> 外部 ekf_localization_node | wheel vx/vy、IMU yaw/yaw rate | /odometry/filtered；odom->base_footprint |
| 全局定位 | amcl.yaml + localization.launch -> 外部 amcl | /scan、/map、odom TF、初始位姿 | map->odom、定位估计；源码不在仓库 |
| 建图替代路径 | mapping.launch/cartographer_2d.lua | scan、filtered odom、IMU | map->odom 与栅格地图；不与 AMCL 同时占用同条 TF |
| 视觉感知 | person._infer；plate.main；traffic._infer/classify | /camera/color/image_raw | 三路 JSON、三路标注图 |
| 任务决策 | Controller.run/navigate/task | ordered points、action 状态、任务结果 | map 坐标 MoveBaseGoal；交通灯停车/继续路线 |
| 全局规划 | move_base.yaml -> global_planner/GlobalPlanner | 目标、全局 costmap | global plan -> TEB；具体搜索默认项待确认 |
| 局部轨迹/控制 | teb_local_planner.yaml -> TebLocalPlannerROS | global plan、costmap、odom | 优化局部时序轨迹与 Twist；内部实现外部依赖 |
| 仿真命令 A | RobotcarPlanarMove.CmdVelCallback/Update | /cmd_vel | CoG 补偿后 SetLinearVel/SetAngularVel |
| 仿真命令 B | MecanumCmdVel._on_cmd/_on_timer | /cmd_vel | Float64MultiArray [FL,FR,RL,RR] -> wheel_velocity_controller |
| 实车命令 | 未实现完整路径 | 预期复用 Twist 或 wheel command 的边界 | CAN/Serial/SDK/电机闭环待确认 |

TF 预期树：map -> odom -> base_footprint -> base_link -> laser_link / camera_link -> camera_optical_frame / imu_link，以及四轮关节。map->odom 由 AMCL 或 Cartographer 提供；odom->base_footprint 由 EKF 提供；robot_state_publisher 提供机器人内部 TF。没有发现固定 world->map 标定转换，不能直接比较 Gazebo world 坐标与路线 map 坐标。

## 8. Core Modules

| 模块 | 识别结果 |
|---|---|
| A Perception | 人物 C/NC YOLO、HyperLPR 车牌、交通灯 ROI 三分类；激光障碍信息由 costmap 消费。无视觉障碍融合到导航证据 |
| B Localization | WheelOdometry + robot_localization EKF + AMCL；Cartographer 用于建图。无 GNSS/视觉定位实现 |
| C Map | localization_map 给 AMCL；planning_map 给两级 costmap；forbidden_zones 离线烧入规划地图 |
| D Mission/Decision | Controller INIT/NAVIGATE/TASK/WAIT_TRAFFIC/NEXT_POINT/FINISH/ERROR；顺序执行航点，有限重试 |
| E Planning | 外部 GlobalPlanner + TEB；无仓库自研全局路径搜索 |
| F Trajectory | TEB 时序轨迹；无独立自研平滑器/速度曲线模块 |
| G/H 横纵向控制 | TEB 同时给 vx/vy/wz，麦轮运动学合成四轮速度；没有独立转向舵机或油门控制器 |
| I Vehicle Interface | 仿真 VelocityJointInterface 和平面插件；实车接口缺失 |
| J Communication | ROS topics/action/TF、Gazebo Transport ~/visual；随机车牌 HTTP；GUI XML-RPC master probe |
| K Simulation | Gazebo 11/ODE + 场景 SDF + 传感器插件 + 两个自定义 C++ 插件 |
| L Visualization | RViz、Gazebo、image_view、PyQt5 GUI、采图 OpenCV 窗口 |
| M Logging | rospy/ROS 日志；ResultManager mission.log、mission_results.json、images；Gazebo gzmsg |
| N Safety | 输入校验、超时/取消、交通灯零速度、命令 watchdog；没有集中速度仲裁、安全监督节点或硬件急停闭环 |

### 8.1 交通灯专项：位置、消息与放行

实现位置：
- P/robot_perception/scripts/traffic_light_classification_node.py -> TrafficLightClassificationNode、crop_roi、classify、freshness_reason、make_payload。
- P/robot_perception/config/traffic_light_classification.yaml -> roi=[0,0,1,0.5]、imgsz=224。
- P/robot_competition/scripts/traffic_wait.py -> parse_frame、TrafficWait.receive/execute。
- P/robot_competition/scripts/main_controller.py -> Controller.initialize/run/stop_robot。

消息由 std_msgs/String 承载 JSON，话题 /perception/traffic_light_json：

~~~json
{
  "schema_version": 1,
  "engine": "yolo11n_traffic_light_classifier",
  "header": {
    "seq": 123,
    "stamp": {"secs": 100, "nsecs": 200000000},
    "frame_id": "camera_optical_frame"
  },
  "frame_valid": true,
  "reason": "ok",
  "roi_xyxy": [0, 0, 640, 240],
  "label": "GREEN",
  "confidence": 0.95,
  "processing_ms": 80.0,
  "score_kind": "conditional_class_probability",
  "presence_verified": false
}
~~~

上例为结构示例，不是本轮采集的运行数据。header 来自源图；ROI 是人工区域的像素边界，不是检测到的灯框。异常时 header/ROI 可为 null；label 变 UNKNOWN，reason 表示低置信度、断流、时间戳/解码/推理错误等。

“未验证灯存在”的定义与读取：
1. traffic_light_classification_node.py:89 -> make_payload 无条件写 presence_verified=False（:98）和 score_kind=conditional_class_probability。
2. frame_valid 只由 reason=="ok" 且 label 属于三色决定；新鲜度在 _infer 前后检查。它不表示存在性已验证。
3. classify 只对 red/yellow/green 三类归一化概率取最大项；低于 confidence 才变 UNKNOWN，没有“无灯”类或灯体检测器。
4. traffic_wait.py:31 -> parse_frame 在 :41-43 **要求 score_kind 正确且 presence_verified is False**；这是协议验证，不是存在性安全门槛。若将字段改为 true，当前消费者反而拒绝消息。
5. TrafficWait.execute 的 green 判据（:164）仅检查 frame_valid、label=="GREEN"、confidence>=阈值；通过连续帧和新鲜度条件后 :189 写 released=True。

完整放行链：

~~~text
相机 -> TrafficLightClassificationNode._receive
     -> step -> _infer（源时间/墙钟新鲜度）
     -> crop_roi -> classify -> make_payload(presence_verified=False)
     -> _publish -> /perception/traffic_light_json
     -> TrafficWait.receive（仅在窗口开启时入队）
     -> execute -> parse_frame（接受未验证存在的协议）
     -> 连续 GREEN 计数 / 队列清空 / 时效 / deadline 检查
     -> released=True -> finally 停车并 record_task
     -> Controller.run 从 WAIT_TRAFFIC 转 NEXT_POINT
     -> index++ -> NAVIGATE -> navigate -> send_goal
     -> move_base/TEB -> /cmd_vel -> 仿真底盘
~~~

已有保守行为：稳定期丢弃旧窗口帧；校验 schema/engine/有限概率/非空 frame/ROI；重复、倒序、未来、过期、来源变化、队列溢出、解码/推理故障进入 ERROR；低置信度 UNKNOWN 等待并清零 streak；无消息最终超时；墙钟限定 35 秒，等待时约每 0.05 秒发零速度。这些措施缓解过期/异常输入，但不弥补灯存在性缺口。

### 8.2 类似“感知不确定但继续执行”的逻辑

| 场景 | 当前处理 | 判定 |
|---|---|---|
| 无灯背景却高概率 GREEN | 满足三帧等条件可放行 | 已确认安全门槛缺口；R01 |
| 普通人物/车牌任务 FAILED_* / AMBIGUOUS | 有限重试后返回结果；Controller.task 不因失败停止，run 进入 NEXT_POINT | 明确的继续巡检策略，可能丢分；不等价于识别成功 |
| 有效空 detection/plate 数组 | fuse 返回 EMPTY_VALID，ResultManager 和 GUI 计为有效完成 | 能区分断流，却不能证明目标确实不存在或观察覆盖完整；R07 |
| 同一人物 C/NC 分类持续切换 | 不同类别不关联，可能建立多个稳定 track，全部计数 | 风险推导；R06 |
| non_overlapping_partitions_confirmed=true | 验证 ID 分配结构后允许观察数相加 | 人工声明，不做跨视角 ReID 或覆盖验证；R07 |
| plate_text_annotation_confirmed=true | 允许任务通过图像标注确认门槛 | 不验证实际图中文字；集中入口固定字体可降低风险，但现场变更后仍需核对 |
| stop_before_line=true / action SUCCEEDED | 认为已到合规停车点并开始灯等待 | 不测停止线、车身速度或停车几何；R04 |
| traffic UNKNOWN/异常/缺帧 | 不会直接放行 | 已有阻断，不能笼统报告“所有不确定输入都被放行” |

## 9. Control Architecture

### 9.1 算法识别

| 算法 | 仓库发现 |
|---|---|
| PID | 未发现自研 PID、PID 增益或 gazebo_ros_control pid_gains 配置；不能据此断言外部控制器/实车固件内部没有 PID |
| Pure Pursuit / Stanley / MPC / LQR | 未发现实现或调用 |
| Feedforward | 未发现明确的跟踪前馈控制；CoG 速度补偿是刚体运动学换算，不标作前馈控制器 |
| TEB | 配置明确选用，负责局部时序轨迹优化和速度输出；不是本仓库实现的 MPC |
| 自定义算法 | X-layout mecanum IK/FK、统一轮速缩放、欧拉里程计积分、CoG 补偿平面运动 |
| 定位滤波 | 外部 EKF、AMCL；不是车轮速度滤波或 PID |

### 9.2 麦轮 IK/FK 和限幅

P/robot_mecanum_control/scripts/mecanum_math.py：
- r=0.037230282 m；k=half_wheelbase+half_track=0.231000002 m。
- 顺序固定 [FL, FR, RL, RR]。
- ω=[vx-vy-k*wz, vx+vy+k*wz, vx+vy-k*wz, vx-vy+k*wz]/r。
- FK：vx=r*(FL+FR+RL+RR)/4；vy=r*(-FL+FR+RL-RR)/4；wz=r*(-FL+FR-RL+RR)/(4*k)。
- scale_to_limit 对四轮统一乘 min(1,limit/peak)，保持合成运动比例；max_abs_speed<=0 时不限制。

MecanumCmdVel._on_cmd -> IK -> scale_to_limit -> wheel_signs -> 缓存。
_on_timer 以 50 Hz 发布；ROS 时间超过 cmd_timeout=0.5 s 则四轮全零。命令访问有 threading.Lock。没有加速度斜率限制、低通滤波或有限值检查；wheel_signs 在限幅之后相乘，仅检查长度而不检查是否 ±1。当前四个值均为 +1，无证据表明当前符号配置已触发超限。

### 9.3 车体平面插件

RobotcarPlanarMove.Load -> 注册 /cmd_vel、自有 ROS callback queue 线程、Gazebo WorldUpdateBegin。
CmdVelCallback 在 mutex 下缓存 vx/vy/wz/ROS 时间；Update 读取快照，超时归零：
- 从 Gazebo 最终 base_footprint inertial 获取 CoG 偏移。
- v_cog_body=(vx-wz*r_y, vy+wz*r_x)。
- 使用当前世界 yaw 做二维旋转，然后只对 base_footprint 调用 SetLinearVel 和 SetAngularVel。
- 不发 odom/TF，不直接写轮 link 速度。
- 每个物理更新执行；world 配置 0.001 s，名义 1000 Hz 仿真时间，实际墙钟频率待确认。
- 没有与 IK 共用轮速限幅，也没有最大体速、加速度、死区、执行器延迟或非有限数拒绝。

### 9.4 控制频率与坐标单位

- move_base controller_frequency=10 Hz；global planner=2 Hz。
- IK 发布、joint_state 发布、EKF=50 Hz。
- 轮里程计由 /joint_states 回调触发；只对 0<dt<0.5 s 积分，yaw 未归一化，但生成四元数。
- 交通灯等待墙钟 sleep=0.05 s；人物/车牌轮询=0.02 s；视觉最大 5 Hz；键盘 rospy.Rate(20) 且输入等待会影响实际频率。
- 机器人右手系：+X 前、+Y 左、+Z 上；正 wz 为绕 +Z 正向；遥控 Q 为正、E 为负。
- vx/vy 为 m/s，wz/轮角速度为 rad/s，yaw 为 rad；路线 x/y 为 m。
- 相机 optical：+Z 前、+X 右、+Y 下。没有 steering angle、steering gain、km/h 转换。
- 纵向、横向、转动统一进入麦轮混合；当前 TEB 允许小横移，但 weight_kinematics_nh=100 强惩罚非完整运动。性能结果需仿真测量。

## 10. Important Parameters

“共享”含义：S=当前仿真使用；C=算法/任务配置可复用，但实车没有入口验证，不能认定已共享；D=说明/参考，未找到运行时加载。★表示可能直接影响比赛表现。没有修改任何值。

### 10.1 控制、规划与定位

| 参数 | 文件位置/配置项 | 当前值 | 用途/影响模块 | 比赛 | 仿真/实车 |
|---|---|---|---|---|---|
| wheel_radius | P/robot_mecanum_control/config/mecanum.yaml | 0.037230282 m | IK/FK 比例 | ★ | S/C待标定 |
| half_wheelbase / half_track | 同上 | 0.106000002 / 0.125 m | k、转动轮速；全轴距约0.212、轮距0.25 m | ★ | S/C待标定 |
| wheel_signs / roller_layout | 同上 | [1,1,1,1] / X | signs 被读取；roller_layout 未读取，算法固定 X | ★ | S/C待确认 |
| max_body_speed | 同上 | 0.6 m/s | 未被 MecanumCmdVel/平面插件读取，不是有效体速保护 | ★ | D |
| max_wheel_speed | 同上 | 16.115913385 rad/s | 四轮统一缩放 | ★ | S/C待确认 |
| cmd_timeout / publish_rate | 同上 | 0.5 s / 50 Hz | ROS 时间 watchdog、轮命令发布 | ★ | S/C |
| cmdTimeout | robot_description/urdf/robotcar_gazebo.xacro | 0.5 s | 平面插件独立 watchdog | ★ | S |
| wheel_max_velocity / effort | robot_description/urdf/robotcar_base.xacro | 16.115913385 rad/s / 1.0 N·m | URDF wheel joint 上限；真实能力未验证 | ★ | S |
| wheel_damping / friction | 同上 | 0.01 / 0.0 | 轮关节动力学 | ★ | S |
| wheel_mu / contact_kp / contact_kd | robot_description/urdf/robotcar_gazebo.xacro | 0.05(两方向) / 1000000 / 10 | 接触模型，未模拟滚子方向性 | ★ | S |
| total mass / base CoG | robotcar_base.xacro + derivation.md | 总5.2 kg；base CoG(-0.081965593727,0.008421364598,0.103109011698) m | 惯性及CoG补偿；分配为近似 | ★ | S/实测待确认 |
| planner/controller_frequency | robot_navigation/config/move_base.yaml | 2 / 10 Hz | 规划与速度控制 | ★ | S/C |
| planner/controller_patience | 同上 | 5 / 5 s | 失败恢复时机 | ★ | S/C |
| oscillation_timeout/distance | 同上 | 10 s / 0.10 m | 振荡判据 | ★ | S/C |
| recovery_behavior_enabled / clearing_rotation_allowed | 同上 | true / false | 恢复行为；外部实际行为待确认 | ★ | S/C |
| max_vel_x / backwards / y / theta | robot_navigation/config/teb_local_planner.yaml | 0.25 / 0.08 / 0.05 m/s / 0.45 rad/s | 正退横转速度 | ★ | S/C |
| acc_lim_x / y / theta | 同上 | 0.25 / 0.10 m/s² / 0.55 rad/s² | 轨迹加速度约束；不是底盘硬保护 | ★ | S/C |
| dt_ref / dt_hysteresis | 同上 | 0.25 / 0.10 s | TEB 时间离散 | ★ | S/C |
| max_global_plan_lookahead_dist | 同上 | 1.0 m | TEB 局部截取距离；非 Pure Pursuit lookahead | ★ | S/C |
| feasibility_check_no_poses | 同上 | 5 | 轨迹可行性检查范围 | ★ | S/C |
| min_turning_radius | 同上 | 0.0 m | 允许原地转动，非汽车轴距模型 | ★ | S/C |
| allow_init_with_backwards_motion | 同上 | true | 倒车初始化 | ★ | S/C |
| xy_goal/yaw_goal_tolerance | 同上 | 0.08 m / 0.05 rad | 到点、相机朝向、停车余量 | ★ | S/C |
| free_goal_vel / complete_global_plan | 同上 | false / true | 目标终端速度与路径处理 | ★ | S/C |
| footprint vertices | teb YAML + costmap_common.yaml | (±0.155,±0.141) m矩形 | 碰撞包络；需覆盖实际全车 | ★ | S/C |
| footprint_padding | costmap_common.yaml | 0.005 m | costmap 额外包络 | ★ | S/C |
| min_obstacle_dist / inflation_dist | teb YAML | 0.04 / 0.12 m | 避障间距 | ★ | S/C |
| iterations inner/outer | teb YAML | 5 / 4 | 精度/计算耗时 | ★ | S/C |
| weight_kinematics_nh / forward_drive | teb YAML | 100 / 10 | 横移/倒车倾向 | ★ | S/C |
| weight_max_vel_x/y/theta | teb YAML | 2 / 2 / 1 | 速度约束权重 | ★ | S/C |
| weight_acc_lim_x/y/theta | teb YAML | 1 / 1 / 1 | 加速度权重 | ★ | S/C |
| weight_optimaltime / obstacle / inflation | teb YAML | 1 / 50 / 0.20 | 时间和障碍物代价 | ★ | S/C |
| enable_homotopy_class_planning | teb YAML | false | 关闭多拓扑候选 | ★ | S/C |
| costmap frames | global_costmap.yaml / local_costmap.yaml | 两者均 map | AMCL修正会作用于局部窗口；效果待确认 | ★ | S/C |
| costmap update/publish | 同上 | global 5/2 Hz；local 10/5 Hz | 障碍更新/显示 | ★ | S/C |
| local width/height/resolution | local_costmap.yaml | 2/2/0.02 m | 局部窗口范围和精度 | ★ | S/C |
| obstacle/raytrace_range | global/local_costmap.yaml | global 3/4 m；local 2.5/3 m | 标记/清除障碍 | ★ | S/C |
| inflation_radius/cost_scaling_factor | 两级 costmap | 0.15 m / 4.0 | 通道可达与离障距离 | ★ | S/C |
| transform_tolerance | costmap、AMCL | 0.5 s | TF时效容忍 | ★ | S/C |
| EKF frequency/sensor_timeout | robot_mecanum_control/config/ekf.yaml | 50 Hz / 0.1 s | 状态融合；不是急停 | ★ | S/C |
| EKF odom0 / imu0_config | 同上 | wheel仅vx/vy；IMU仅yaw、vyaw | 融合变量选择 | ★ | S/C |
| imu0_relative / two_d_mode | 同上 | true / true | 首帧IMU为本地参考；平面融合 | ★ | S/C |
| EKF queues / transform_timeout | 同上 | odom10、IMU20 / 0 s | 缓冲与TF等待 | ★ | S/C |
| AMCL min/max particles | robot_navigation/config/amcl.yaml | 300 / 2000 | 精度/CPU | ★ | S/C |
| AMCL update_min_d/a | 同上 | 0.02 m / 0.03 rad | 更新阈值 | ★ | S/C |
| AMCL odom_model / alpha1..5 | 同上 | omni-corrected / 各0.10 | 全向里程计误差模型 | ★ | S/C待标定 |
| AMCL laser range/beams/sigma | 同上 | 0.10..8 m / 60 / 0.20 | 激光似然模型 | ★ | S/C |
| AMCL recovery_alpha_slow/fast | 同上 | 0 / 0 | 自适应恢复参数 | ★ | S/C |
| Cartographer range/submaps | cartographer_2d.lua | 0.1..8 m / 35 scans | 建图距离与子图 | ★ | S/C |
| Cartographer motion filter / optimize | 同上 | math.rad(0.2) / 每35 nodes | 建图采样/优化 | ★ | S/C |
| Cartographer min_score/global score | 同上 | 0.60 / 0.70 | 约束匹配门槛 | ★ | S/C |
| keyboard max speed | keyboard_teleop.py -> __init__/run | 1.0 m/s、2.0 rad/s；默认0.25、0.60 | 可绕过TEB上限；不是死手控制 | ★ | S/实车待确认 |

PID 增益、steering gain、最大转向角、MPC/LQR 权重：未找到。实际电机 PID/减速比/编码器分辨率/死区/执行器延迟：待确认。

### 10.2 传感器和仿真

| 参数 | 文件位置 | 当前实际值 | 用途 | 比赛 | 共享 |
|---|---|---|---|---|---|
| LiDAR rate/samples/range | robot_description/urdf/robotcar_sensor_plugins.xacro | 10 Hz、500、0.10..10 m；±π扫描 | 定位/避障 | ★ | S |
| LiDAR noise | 同上 + sensor_simulation.yaml | 插件未配置噪声；说明值0 | 理想化基线，真实噪声待标定 | ★ | S/D |
| camera image/rate/HFOV | 同上 | 640×480、15 Hz、1.047197551 rad(60°) | 识别目标像素大小与视野 | ★ | S |
| camera near/far | 同上 | 0.05 / 8.0 m | 渲染/深度截止；说明YAML near=0.6 | ★ | S |
| camera pitch | robotcar_sensors.xacro + sensor_plugins | -0.0872665 rad(约-5°) | ROI视野；说明YAML为0 | ★ | S |
| laser/camera/imu xyz | robotcar_sensors.xacro，相对base_link | laser(0.09199999,0,0.122011877)；camera(0.13499999,0,0.060287877)；imu(-0.043,0,0.107799877) m | TF外参；Z为估计 | ★ | S/实车待测 |
| IMU rate/noise/reference | sensor_plugins | 100 Hz / 0 / initialOrientationAsReference=false | IMU输入；EKF另设relative=true | ★ | S |
| world step/update/RTF目标 | robot_gazebo/worlds/competition_field.world | 0.001 s / 1000 Hz / 1.0 | 物理时间推进 | ★ | S |
| spawn x/y/z/yaw | robot_gazebo/launch/simulation.launch | 1.696892 / -1.680704 / 0.023994 m / 1.568056 rad | world初始姿态，非map路线坐标 | ★ | S |
| 红/黄/绿灯时长 | robot_gazebo/models/traffic_light/model.sdf | 10 / 5 / 15 s（仿真时间） | RED->GREEN->YELLOW->RED | ★ | S |
| map resolution/origin | robot_navigation/maps/localization_map.yaml、planning_map.yaml | 0.02 m；[-0.547105,-0.51134,0] | 定位/规划坐标 | ★ | S/C重建 |
| reference map origin | maps/gazebo_reference.yaml | [-0.512972,-0.530262,0] | 可视参考，不是正式定位地图 | 可 | S |
| 禁区多边形 | robot_navigation/config/forbidden_zones.yaml | parking_area、area_1、area_2 | 离线生成规划障碍，非运行时动态读取 | ★ | S/C重制 |

### 10.3 任务与视觉

| 参数 | 文件位置 | 当前值 | 用途 | 比赛 | 共享 |
|---|---|---|---|---|---|
| navigation_timeout/server/cancel/clock | competition.launch | 90/15/3/5 s | 导航、连接、取消确认、时钟等待 | ★ | S/C |
| retries / navigation_test_mode | competition.launch | 1 / false | 最多2次导航；测试模式跳过视觉和灯 | ★ | S/C |
| task_timeout/settle/observation | task_config.yaml -> defaults | 12/0.5/2 s | 人员车牌窗口；12s从settle后窗口起算 | ★ | S/C |
| min/max_valid_frames / support | 同上 | 4/24 / 3 | 多帧稳定性 | ★ | S/C |
| task_retry_count / confidence | 同上 | 1 / 0.5 | 重试、融合门槛 | ★ | S/C |
| source_age / image_match_timeout | 同上 | 2 / 1.5 s | 新鲜度和证据匹配等待 | ★ | S/C |
| match_iou / center_distance / gap | 同上 | 0.3 / 0.08(归一化) / 2帧 | 局部人物/车牌轨迹关联 | ★ | S/C |
| vote_fraction / evidence_images | 同上 | 0.67 / 3 | 车牌字符串投票/证据上限 | ★ | S/C |
| traffic timeout/green_frames/confidence | task_config.yaml -> traffic_light | 35 s / 3 / 0.8 | 放行判据 | ★ | S/C |
| traffic max_source_age/settle | 同上 | 3.0 / 0.5 s | 当前正式任务值；代码DEFAULTS为1.5/0.5 | ★ | S/C |
| person confidence/imgsz/iou/max_det | perception.launch + person node | 0.25/640/0.45/100 | 检出门槛；任务另按0.5筛选 | ★ | S/C |
| plate min_confidence | perception.launch | 0.5 | HyperLPR结果过滤 | ★ | S/C |
| 三路 max_rate | perception.launch | 各5 Hz，墙钟上限 | CPU吞吐与多帧等待 | ★ | S/C |
| stale_seconds 人/牌/灯 | perception.launch | 1.5/2.0/3.0 s | 数据陈旧判定；灯单独launch默认1.5 | ★ | S/C |
| traffic ROI/imgsz | traffic_light_classification.yaml | [0,0,1,0.5] / 224 | 全宽上半图分类 | ★ | S/C待重标 |
| plate_text_annotation_confirmed | task_config.yaml -> plate | true | 人工确认中文图像，不是OCR核验 | ★ | S/C须重验 |
| non_overlapping_partitions_confirmed | task_config.yaml -> counting | true | 人工确认分区互不重叠 | ★ | S/C须重验 |
| max_cache_frames/age/bytes | competition.launch | 24 / 10 s / 67108864 bytes | 感知缓存边界 | 可 | S/C |
| max_image_bytes | PerceptionClient 默认 | 8388608 bytes | 图像接收/校验上限 | 可 | S/C |
| traffic queue / loop | TrafficWait.receive/execute | 128条 / 0.05 s | 溢出拒绝、停止发布与判定 | ★ | S/C |
| 训练 ratios/seed | scripts/traffic_light/config.yaml | 0.70/0.15/0.15；57 | 随机单图片划分 | ★ | 离线 |
| epochs/imgsz/batch/patience | 同上 | 100/224/32/20 | 模型训练 | ★ | 离线 |

路线顺序为 light_01 -> wp_01 -> person_01..person_05 -> light_02 -> plate_01..plate_03 -> wp_02。精确坐标唯一来源为 P/robot_competition/config/waypoints.yaml -> route.points；12点均以 map/m/rad 表示。当前两个灯点分别约(1.160676,-0.021705,-0.002900)、(1.818009,1.865670,1.628513)，均 stop_before_line=true；其余点位置和朝向决定道路合规及目标视野。没有逐点覆盖 navigation_timeout/retries，因此使用 launch 默认。

## 11. Simulation Interface

- 场景：P/robot_gazebo/worlds/competition_field.world -> 4.2×4.2 m 场地框架、地面纹理、三车牌、两灯、五组人物板。
- 模型资产：P/robot_gazebo/models/*/model.sdf。人物是图像立牌；不是动态行人行为模型。
- 机器人实际装配：robotcar.urdf.xacro include base、sensors、transmissions、gazebo、sensor_plugins 五个文件。旧 robotcar.urdf / official_reference 不是 simulation.launch 的默认入口。
- TrafficLightPlugin.Load/OnUpdate/ApplyState/SetMaterial 使用 Gazebo Transport ~/visual 更新材质；没有 ROS 灯真值话题给任务层。初始红灯由 SDF 材质定义；两个实例采用相同周期，具体同步程度待运行确认。
- /cmd_vel 同时被平面插件与 MecanumCmdVel 订阅。轮命令 Float64MultiArray -> JointGroupVelocityController -> gazebo_ros_control -> VelocityJointInterface -> 4 joints。
- 地面/轮子是低等向摩擦，未使用真实滚子接触/驱动动力学；实际车体速度直接设置，不能验收电机电流、力矩余量、死区、制动距离。
- depth topic/point cloud 有生产端，但在当前导航/任务代码中未找到下游订阅；不能声称已完成3D避障。
- generate_random_plates.py 调用外部 HTTP，默认每张 timeout=6 s、retries=3，支持 --seed；成功覆写两张纹理并保存 generated_plates.json，失败保留旧图并记录 null。集中启动未传 seed，场景非完全固定。

## 12. Real Vehicle Interface

当前仓库没有完整实车数据链，不能给出可执行实车部署命令。

| 接口 | 实际发现 |
|---|---|
| CAN / SocketCAN | 未找到驱动、报文 ID/布局、设备配置或收发实现 |
| Serial / UART / RS232 | official_sensor_specs.yaml 声明 IMU TTL_serial/RS232；未找到串口打开、波特率、协议解析 |
| TCP/UDP底盘协议 | 未找到自定义车辆 socket；ROS通信不能当作底盘SDK证据 |
| GPIO/使能/急停 | 未找到 |
| ROS topic/action | 已有 /cmd_vel、wheel command、/joint_states、/wheel/odom、/imu/data、/scan、camera、/move_base action；可作为对接候选边界 |
| 车辆 SDK / ros_control硬件层 | 只有 Gazebo 仿真插件与 transmission 声明；没有自定义 RobotHW 实车 read/write |
| 真实传感器驱动 | 没有对应 launch/SDK；品牌、型号、时间同步方式待确认 |

明确分界：
- **当前仿真输出**：move_base/TEB -> /cmd_vel -> Gazebo 平面插件；同时 /cmd_vel -> IK -> wheel_velocity_controller -> 仿真关节。
- **实车输出**：在 /cmd_vel 或四轮命令接口之后缺失硬件传输/反馈/使能/急停实现。到底由厂商底盘接受 Twist，还是宿主下发四轮转速，待确认。
- 同理，实车需要用实际传感器/编码器生产端替换 Gazebo 来源，并保持时间戳、frame、单位和校验策略一致。
- 现有导航 launch 默认 use_sim_time=true；实车必须核对时间源，当前没有统一 real/sim 切换入口。

## 13. Sim-to-Real Boundary

| 项目 | 当前证据 | 迁移需确认 |
|---|---|---|
| 坐标/左右手系 | robotcar_sensors.xacro 规定右手系；IK正号按X布局 | 编码器顺序、四轮转向、IMU板轴、雷达0°与安装外参 |
| yaw | route用sin/cos构四元数；IMU仿真世界参考，EKF relative=true；AMCL map->odom | 开机yaw基准、磁航向/ENU/NED、重启一致性；未发现NED适配代码 |
| steering | 无舵角，wz为rad/s | 不应将wz直接写入转向角字段；底盘协议语义待确认 |
| 单位 | m、s、rad；轮命令rad/s | rpm、编码器ticks、减速比、厂商单位换算未实现 |
| 地图坐标 | map与world不同；参考图脚本有约180°图像对应假设 | 实测地图原点、道路/停止线坐标、固定外参，不把参考图当TF |
| 控制频率 | 10Hz TEB、50Hz轮命令/EKF、1000Hz名义平面更新 | 实际线程调度、网络延迟、丢包、CPU竞争 |
| 时间戳 | Twist无header，底盘以接收ROS时间计时；视觉双时钟校验 | 网络上延迟但持续到达的命令会被当作新接收；硬件时钟同步 |
| 断流 | 相机任务有时效/超时；底盘有0.5s指令watchdog | 进程退出、串口断开、持续陈旧定位、低层固件watchdog |
| 动态响应 | 车体瞬时SetVel，无执行器响应模型 | actuator delay、加速/制动、速度跟踪、转动耦合、负载 |
| 饱和 | IK四轮缩放；平面分支无同等限幅 | 所有命令来源统一限幅、有限值拒绝、实际电机能力 |
| 几何 | 轮径/轮距来自模型；总质量5.2kg，惯性/CoG近似 | 实测有效轮径、滚子磨损、载荷、重心、全车包络 |
| 噪声 | IMU gaussianNoise=0；雷达未显式加噪；相机为渲染图 | 轮滑、IMU bias、雷达反射/丢点、曝光/模糊/遮挡 |
| 视觉ROI | 固定上半图，无灯体存在验证 | 实车灯具视角/光照/背景、无灯/错误灯负样本 |
| 定位精度 | AMCL/EKF配置与人工初始化 | 实际协方差、TF年龄、定位跳变、地图错配与停止距离 |
| 安全 | 无硬件急停或集中仲裁 | 在接入实车前确认独立急停/使能/故障锁存策略 |

wheel odom 不设置 pose/twist covariance，ROS消息默认零值；EKF如何处理零协方差依赖外部实现，不能声称这些数据具有零误差。真实测量噪声及融合权重待确认。

## 14. Logging / Testing

### 14.1 已有能力

- ResultManager -> atomic_bytes 使用临时文件、flush/fsync、os.replace 保存 JSON/图像；mission.log 逐事件写入并 fsync。
- 结果路径 /workspace/car_2026/results/<UTC+uuid>/：任务尝试、导航状态、时长、原始识别、匹配证据、traffic released、计数政策等。
- 人物/车牌图像按源 stamp+frame_id 匹配（不要求 ROS 输出 seq 不变）；缓存有帧数、年龄、字节限制。
- GUI 只订阅原始相机；任务证据从已保存 JSON 加载，并限制到该 run/images 路径。
- ROS日志、Gazebo输出及 RViz 可人工诊断。未发现自动 rosbag 录制/回放脚本或已跟踪 bag。
- waypoint_manager.py 为可独立路线校验入口，不是单元测试套件。
- Dockerfile 有 ROS/Python/Qt/图像/依赖导入检查；不是导航功能测试。
- scripts/traffic_light/test.py 对测试集逐图分类，输出 accuracy 和 confusion_matrix.csv；不是 ROS/仿真 integration test。

本轮只读核验：HyperLPR 四文件清单一致；交通灯数据 train每类35、val每类7、test每类8，总105/21/24。跨 train/val/test 未发现 SHA-256 完全相同图像。由于单帧随机划分，时间相邻/同场景近似图像相关性仍待确认，不能将“无相同哈希”理解为无数据泄漏。没有加载模型或复算准确率。

### 14.2 能否“改代码 -> 自动仿真 -> 量化结果”

**当前未形成完整闭环。** 有启动入口和任务结果，但缺少：
1. 固定场景/随机种子、自动重置与初始定位基线。
2. 无人工交互的受控测试编排和节点就绪检查。
3. 场地/目标真值与自动匹配；交通灯、车牌生成记录未连接统一评分。
4. 车身越线/碰撞/路径合规、轨迹误差、速度/加速度、误放行等指标采集。
5. 多轮重复、失败分类、阈值验收、版本/参数/权重哈希绑定。
6. 测试套件、CI任务或可回放消息 fixtures。

mission_duration_sec、task_duration_sec、各任务 status 可作为未来指标输入，不能替代比赛总评分。FINISH 不保证全任务成功；EMPTY_VALID 也不保证比赛目标完成。

## 15. Risks

严重度按用户定义：P0=可能造成失控、崩溃或严重错误；P1=明显影响比赛表现；P2=一般工程问题/技术债。下面均是静态审阅；没有动态复现或实车验证。未对外部 ROS 包内部行为作无证据断言。

### R01 — P0：未验证灯体存在的分类结果可授权通行

- 文件/函数：P/robot_perception/scripts/traffic_light_classification_node.py:89 make_payload、:58 classify；P/robot_competition/scripts/traffic_wait.py:31 parse_frame、:164/:189 execute；Controller.run。
- 证据：producer 恒发 presence_verified=false；consumer 必须读到 false 才认可协议；green 判据及最终 released 条件不要求存在性证明。无灯背景也必选三类之一，只有低概率才UNKNOWN。
- 影响：若固定ROI拍不到真实灯却连续高概率GREEN，任务层仍可能下发下一导航目标，造成红灯/错误对象下放行。逻辑缺口已确认，实际误判率待确认。
- 后续验证：隔离仿真中回放无灯、遮挡、绿背景、错灯、偏转相机图像，记录完整JSON/streak/released/下一目标；测试须无实车运动。

### R02 — P0：取消失败或竞争发布者存在时，没有可确认的持续停车通道

- 文件/函数：P/robot_competition/scripts/main_controller.py -> initialize/stop_robot/run/shutdown/cancel；P/robot_mecanum_control/scripts/keyboard_teleop.py -> run；两个 /cmd_vel 消费端。
- 证据：错误分支仅先发一次零Twist，再尝试cancel；cancel超时只记录错误并退出。stop_pub仅在正式路线含交通灯时创建，纯导航/无灯路线异常时 stop_robot 是空操作。move_base、TrafficWait、遥控共用 /cmd_vel，没有 mux/优先级/故障锁存。
- 影响：若 action 取消链路故障而 move_base仍持续发速度，后续非零命令可覆盖一次零命令；命令watchdog因仍收消息而不触发。TrafficWait反复发零也不能保证压过另一个持续发布者。不能把GUI“停止全部”认作急停。
- 后续验证：在仿真隔离 action cancel/结果确认但保持 cmd_vel，或加入第二发布源；检查收到错误后车体实际停止时间和非零命令。实车需另查低层急停。

### R03 — P1：平面车体分支绕过轮速限幅和动态约束

- 文件/函数：RobotcarPlanarMove.CmdVelCallback/Update；MecanumCmdVel._on_cmd；mecanum_math.scale_to_limit。
- 证据：插件直接设置原始vx/vy/wz；另一分支统一限制四轮16.115913385。max_body_speed=0.6未读取，插件无加速度/有限值检查。键盘允许1.0m/s、2.0rad/s。
- 影响：超限输入或平移旋转组合时，车体运动与限幅后轮速/轮里程计不一致，掩盖真实饱和、影响EKF/导航评价；NaN/Inf输入没有防护，其Gazebo后果待确认。当前TEB默认最大组合不等于已发生饱和。
- 后续验证：仿真扫描单轴/横移/组合阶跃，比较原cmd、限幅轮速FK、真实link twist和滤波里程计；同时测延迟与加速度。

### R04 — P1：停车线安全依赖人工布尔声明，放行后不持续监视信号

- 文件/函数：waypoints.yaml -> stop_before_line；Controller.initialize/run；TrafficWait.execute；teb_local_planner.yaml -> goal tolerance。
- 证据：仅校验stop_before_line is True；action SUCCEEDED后视为已停线前，没有停止线检测或实测速度确认。到点容差0.08m；成功释放后accepting=false，执行下一导航目标期间不再监视该灯。
- 影响：定位/车身包络/到点误差可使车身越线；允许3s年龄的绿灯结果及放行后导航延迟可能使实际越线发生在变灯后。现场停止线距离和相位余量待确认。
- 后续验证：测量停止线、全车包络、最坏定位误差/停止距离；在绿灯末端注入延迟并记录车身穿线时刻与灯真值，不能只检查中心点。

### R05 — P1：未建立独立的传感器/定位健康到制动的安全闭环

- 文件/函数：Controller.check_running；ekf.yaml；costmap YAML；底盘watchdog。
- 证据：Controller仅检查关闭和仿真时钟，不订阅定位协方差、scan/odom心跳或TF质量。底盘只看新命令接收；costmap未显式配置expected_update_rate，EKF sensor_timeout不是停车接口。
- 影响：定位仍持续发布但错误、激光持续陈旧或传感器部分掉线时，无法从本仓库证明一定停止。外部move_base/EKF的具体故障行为待确认，不宣称完全没有外部保护。
- 后续验证：分别冻结/断开scan、IMU、joint_states、注入定位跳变和高协方差，记录TF、costmap freshness、cmd_vel和停止距离。

### R06 — P1：人物分类切换可能把同一目标统计为多个人

- 文件/函数：P/robot_competition/scripts/task_processor.py:53 associate、:68类别匹配、:94 fuse；result_manager.counts_for。
- 证据：C与NC类别不同则拒绝关联；新建track；每条达到3帧支持的track独立输出，未做跨类别合并。长缺帧后重新出现也可能新增track。
- 影响：同一个位置/人物连续3帧C再连续3帧NC，可形成两条稳定轨迹并分别计数；整街区统计偏大。静态可推导，当前数据发生率待确认。
- 后续验证：内存消息序列输入同bbox的C/C/C/NC/NC/NC、遮挡重现、邻近多人物，比较目标真值、track和计数。

### R07 — P1：空检测和人工分区声明不能证明完整识别，却可计为有效完成

- 文件/函数：task_processor.fuse/execute；result_manager.summarize_people/validate_counting；task_config.yaml；Controller.task/run。
- 证据：足够有效空帧 -> EMPTY_VALID，包含在COMPLETE；counting确认已为true，只做结构验证，无跨视角去重/覆盖证明。普通失败重试完仍继续路线。
- 影响：相机朝向错误、目标漏检或区域重叠可产生零计数/重复总数；路线FINISH并不表示得分成功。继续巡检是明确策略，本项不是指所有失败被伪装SUCCESS。
- 后续验证：对已知每区人数/车牌的固定场景比较EMPTY_VALID、失败与真值；验证视野覆盖、非重叠声明、中文图像和终端结果一致性。

### R08 — P1：当前 checkout 不满足默认交通灯模型部署条件

- 文件/函数：environment.launch -> enable_traffic_light=true；perception.launch 默认模型路径；TrafficLightClassificationNode.__init__；traffic_light_classification.launch required=true。
- 证据：默认包内traffic_light_best.pt不存在，顶层best.pt存在但不在Compose挂载；构造函数缺文件直接抛异常。
- 影响：直接按默认集中入口运行当前文件集会在灯节点加载处失败；required节点失败影响整个环境launch。README已说明人工准备，属于部署缺项，非隐蔽算法错误。
- 后续验证：只读检查实际容器路径、权重哈希/类别/任务类型、缓存/字体就绪；本轮不复制、不修复。

### R09 — P1：传感器说明配置与实际模型不一致，且实车外参仍未测定

- 文件/函数：sensor_frames.yaml vs robotcar_sensors.xacro；sensor_simulation.yaml vs robotcar_sensor_plugins.xacro。
- 证据：相机pitch说明0、实际-0.0872665；near说明0.6m、实际0.05m。相关说明YAML没有被launch/xacro加载，实际值写在Xacro。IMU物理板轴和传感器Z高度文档明确待验证。
- 影响：照说明标定/部署会使用不同视野和坐标；近距深度仿真可用范围与硬件工作范围不一致。当前深度未进入控制，但视觉ROI与Sim-to-Real仍受影响。
- 后续验证：读取运行robot_description并与CameraInfo/TF/实测安装比较；确认近距离目标和真实深度盲区。

### R10 — P1：交通灯离线评估与部署预处理不同，不能直接推定放行可靠性

- 文件/函数：camera_capture.main；scripts/traffic_light/test.py/train.py/prepare_dataset.py；traffic_light_classification_node.crop_roi/classify。
- 证据：采集保存完整原图，test.py直接预测整图，部署先裁上半图再分类；评估仅三类top1 accuracy，没有无灯背景误GREEN、时序放行或存在性指标。按单图片随机划分而非场景/序列分组。
- 影响：离线准确率不能代表实际ROI/错位/无灯场景可靠性；邻近帧相关性可能让泛化指标偏乐观（近似泄漏待确认，未发现完全相同跨集图像）。
- 后续验证：相同样本对齐部署ROI预处理，按场景/采集序列检查划分，单列无灯/红灯误放行与延迟指标；本轮未训练或评估。

### R11 — P1（条件触发）：Windows 普通文件形式的 Catkin 链接不能原样作为 Linux 构建入口

- 文件/配置：P/CMakeLists.txt。
- 证据：Git索引120000；当前fs.lstat不是符号链接，文件只有/opt/ros/noetic/...目标路径。
- 影响：若把当前文件原样挂载进Linux容器，CMake会把目标路径文本当脚本，构建受阻。另一个Linux原生checkout若正确恢复链接，则不受该条件影响。
- 后续验证：目标容器只读ls/readlink及git core.symlinks/实际挂载来源；本轮没有改变链接。

### R12 — P2：安装规则引用不存在文件

- 文件/函数：P/robot_competition/CMakeLists.txt:16 install(FILES README.md P2B.md...)。
- 证据：当前清单无P2B.md。
- 影响：install阶段可能报缺文件；devel工作空间运行不因此自动判失败。
- 后续验证：独立构建环境检查安装清单/安装阶段；本轮未执行构建。

### R13 — P2：缺少可重复的端到端自动测试与评分

- 文件/入口：scripts/traffic_light/test.py；P/tools/start.sh；ResultManager。
- 证据：唯一test.py是分类评估；无unit/integration/rostest/CI、无整场真值评分；默认车牌随机且在线生成。
- 影响：修改后难以区分路线/控制/识别回归，缺少多轮量化比较。
- 后续验证：先盘点外部未入库测试资产；定义固定场景与指标，确认日志足够后再单独建设测试流程。

### R14 — P2：运行文档和参数说明存在过时内容

- 文件：perception/docs/interface.md末尾、mecanum/docs/WINDOWS_STAGE.md、teb YAML注释、根README；mecanum.yaml -> max_body_speed/roller_layout。
- 证据：接口文档称未消费交通灯，代码已放行；测试文件声明找不到；横移“disabled”与数值不符；部分参数没有读取端。
- 影响：后续开发者误判安全边界或修改无效参数。
- 后续验证：按本地图逐条追踪读取端，区分运行参数与说明参数；本轮不改文档以外文件。

### R15 — P2：任务控制线程执行同步持久化，阻塞时间未限定

- 文件/函数：result_manager.event/atomic_bytes/save；TrafficWait.execute调用event/record_task。
- 证据：每事件fsync；交通灯循环逐帧日志；同线程也负责周期零速度和deadline检查。IO没有独立超时。
- 影响：慢磁盘/挂载停顿延长任务、延迟控制状态更新；现有底盘watchdog在无其他命令时可归零，但不能等同于IO安全隔离。
- 后续验证：受控仿真测fsync延迟、待写队列与实际零指令周期；磁盘满/只读故障是否保留清晰结果。

### R16 — P2：轮状态输入没有完整数组/有限值校验，协方差未建模

- 文件/函数：mecanum_wheel_odometry.py:37 _on_joint_state，:43索引、:64发布。
- 证据：只检查velocity非空和四个name存在，直接按name索引velocity；wheel_signs未检长度，速度未检finite，covariance未赋值。
- 影响：截短/损坏JointState可导致回调异常或不可信里程计；实际Gazebo正常消息不代表实车驱动也满足条件。零协方差的外部EKF处理待确认。
- 后续验证：隔离输入短数组/缺轮/NaN/时钟跳变，检查回调、发布与EKF诊断；用真实轮滑标定协方差。

并发审阅结果：MecanumCmdVel命令缓存、平面插件命令缓存、感知缓存、TrafficWait队列均有锁；平面线程有alive原子变量和析构join。没有足够证据报告确定性死锁/数据竞争。未找到业务FIXME；TODO命中主要是package许可证模板。没有为凑数量将模板TODO升格成安全问题。

## 16. Competition-Critical Components

| 优先级 | 组件 | 直接影响 / 下一步关注 |
|---|---|---|
| 高 | TrafficWait + 分类器 + 停止线航点 | 误放行、停线合规、等待耗时；先验证R01/R04 |
| 高 | /cmd_vel仲裁/取消/底盘watchdog | 故障下是否真实停止；先验证R02/R05 |
| 高 | map/forbidden_zones/route/footprint | 道路合规、可达性、到点视野与车身余量 |
| 高 | AMCL/EKF/轮里程计/TF | 全局误差、局部速度反馈、定位跳变 |
| 高 | TEB + 平面插件/IK边界 | 跟踪误差、速度、转弯、仿真可信度 |
| 高 | 人物/车牌任务融合与证据 | 识别分数、重复计数、完整号码及图片匹配 |
| 中 | 模型/ROI/推理吞吐/时间戳 | 多帧有效率、CPU负载与任务时延 |
| 中 | 场景固定种子/自动指标/回放 | 优化结果能否量化复现 |
| 中 | 传感器外参/噪声/实车接口 | 下一阶段迁移与标定 |
| 中 | 部署检查/构建/缓存 | 比赛环境可启动与重建可靠性 |
| 低 | GUI布局、日志展示、非运行参考资源 | 可操作性，通常不直接改变轨迹/识别 |
| 低 | ONNX导出路径 | 当前ROS交通灯仍加载.pt；除非后续切换推理后端 |

以上仅标识调查/优化入口，本轮没有优化，也未建议直接提高速度或放宽安全门槛。

## 17. Unknowns / TODO

1. 真实机器人厂商底盘接口、报文、SDK、motor使能/急停/断线停机策略；PDF硬件手册的具体内容未展开，待确认。
2. 实车轮序/正负、X布局、有效轮径/轴距/轮距、编码器和减速比、速度单位。
3. 电机闭环/PID、最大加速度/制动、死区、延迟、负载变化、真实CoG/惯性。
4. 激光/相机/IMU型号、时间戳来源和同步、板轴、相机内参/畸变与俯仰、传感器实际Z。
5. ROS/Gazebo实际安装版本、插件加载、topic解析、消息实际频率、仿真实时率和CPU占用。
6. 目标容器是否另有已部署交通灯模型和HyperLPR缓存、正确符号链接；当前本地文件缺项不等于所有外部容器均缺项。
7. Gazebo world与当前map的实际对应、路线/禁区/停止线是否满足比赛示意图及所有官方规则。
8. AMCL失锁/高协方差、scan掉线、EKF零协方差的外部依赖行为和实际停车响应。
9. 交通灯无灯/错灯/遮挡误GREEN概率、3s时效与绿灯末端通过余量。
10. 人物各区真值/视野非重叠、C/NC切换与再出现重复计数；车牌字体确认是否对实际图像成立。
11. 训练数据采集序列相关性、整图与ROI分布差异、是否存在外部未入库测试/评测脚本。
12. 旧URDF、说明YAML、参考图与当前Xacro的维护关系；哪些文件后续应作为单一参数来源需另行决策。

## 18. Recommended Next Investigation

下一轮最值得深入的五个问题，均应先形成可复现实验证据：

1. **交通灯放行是否能拒绝“无灯但高概率GREEN”？** 沿producer JSON -> parse_frame -> released -> 下一目标全链路回放，包含绿灯末端和延迟场景。
2. **任务异常、取消失败、竞争速度源、传感器/定位失效时，车辆能否在确定时间内停住？** 先在隔离仿真中测故障响应，确认需要的实车独立安全边界。
3. **仿真车体速度与四轮可实现速度一致到什么程度？** 比较原cmd、轮速FK、Gazebo link状态、EKF，特别是横移、转弯、饱和与CoG补偿。
4. **地图/路线/停车点/相机外参与目标真值是否一致？** 验证全车不越线、实际视野和人物/车牌结果，检查C/NC重复计数与有效空结果。
5. **能否建立一套可重复的部署与整场量化基线？** 先核对模型/链接/缓存，固定场景与参数，明确成功率、用时、越线、误放行、定位/跟踪误差和识别真值指标。

本轮到此结束：只新增 PROJECT_MAP.md；业务代码、配置、依赖、既有文档、模型与Git提交均未修改。风险列表全部收录于第15节。
