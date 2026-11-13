# Smart-Community

智慧社区机器人自主巡检系统，基于 ROS Noetic，包含实车与 Gazebo 仿真两个独立工作空间，提供导航巡检、视觉识别、任务调度和结果记录。

## 目录结构

```text
Smart-Community/
├── robot_ws/          # 实车：底盘、传感器、导航、感知和巡检任务
├── simulation_ws/     # 仿真：Gazebo 场景、机器人、导航、感知和任务控制台
├── docker/            # 仿真镜像、Compose 配置和部署说明
├── models/            # 模型权重与导出模型
├── yolo_train/        # 训练脚本、数据集及已有训练结果
├── opencv/            # 指针仪表离线标定、读数与调试图片
├── .gitignore
├── LICENSE
└── README.md
```

技术文档目前位于 `robot_ws/docs/` 和仿真功能包内部；仓库根目录尚无独立 `docs/`。

## 主要功能

- **实车**：串口底盘控制、里程计、雷达/IMU/相机接入、定位导航、六类 YOLO 检测、OCR、倒伏统计、路线调度、语音和录像。
- **仿真**：麦克纳姆底盘与传感器仿真、Cartographer 建图、AMCL 定位、双地图导航和 TEB 规划；人物、车牌及交通灯识别，多航点任务与绿灯等待。
- **运行控制**：仿真支持集中启动、人工确认后执行比赛和 PyQt5 控制台，保存任务状态、统计及图像证据。任务完成不代表每项识别均成功，应查看结果状态。

## 仪表离线读数

`opencv/` 提供主表盘的手动标定、指针检测和整数读数，不依赖 ROS、YOLO 或 GPU。当前要求拍摄构图与标定时基本一致，尚未接入实车巡检任务。

先按 [OpenCV 模块说明](opencv/README.md) 建立独立 Python 环境并安装依赖。在仓库根目录、使用该环境执行：

```bash
python -m opencv read opencv/photo/clear3.png --config opencv/configs/competition.json
```

结果位于 `opencv/outputs/read/clear3/`。更换拍摄构图时先运行标定，完整 Windows/Linux 命令见模块说明。

## 实车环境与启动

使用已有 Ubuntu 20.04 / ROS Noetic / Python 3 环境，依赖和设备说明见 [实车 README](robot_ws/README.md)。以下假定仓库部署在 `/home/mowen/workspace/Smart-Community`：

```bash
source /opt/ros/noetic/setup.bash
cd /home/mowen/workspace/Smart-Community/robot_ws
# 先生成厂商雷达消息，避免首次编译时消息头文件尚未生成
catkin_make lslidar_msgs_generate_messages -j2 -l2
catkin_make -j2 -l2
source devel/setup.bash
# 启动系统，默认不自动巡检
roslaunch community_bringup community.launch base_protocol:=aa_bb
```

比赛时使用 `roslaunch community_bringup finals.launch base_protocol:=aa_bb` 作为替代入口，不要同时运行两个总入口。`base_protocol`、串口设备、相机、地图及路线须按实际硬件确认。实车自研包为 `community_*`，雷达与 IMU 包保留厂商实现。

## 仿真环境与启动

使用 Docker 提供 Ubuntu 20.04、ROS Noetic、Gazebo 11 和 Cartographer。系统 Python 3.8 用于 ROS，独立 Python 3.10 用于视觉推理；不要全局激活视觉环境编译 ROS。默认 Compose 保留 NVIDIA GPU、host 网络及 X11 配置，适用于具备相应支持的 Linux 图形宿主机，视觉推理依赖为 CPU 版本。

在仓库根目录的宿主机终端执行：

```bash
cd docker
# 图形宿主机需按 Docker 文档配置 X11 授权
xhost +local:docker
docker compose build ros
docker compose up -d ros
docker compose exec --user developer ros bash
```

在容器中编译：

```bash
cd /workspace/simulation_ws
source /opt/ros/noetic/setup.bash
source /opt/cartographer_ws/install_isolated/setup.bash
catkin_make -j2
source devel/setup.bash --extend
```

按 [视觉部署说明](simulation_ws/src/robot_perception/README.md) 准备模型和 HyperLPR 缓存后，在容器中执行 `bash /workspace/simulation_ws/src/tools/start.sh`。在 RViz 校准定位，再在另一容器终端执行 `bash /workspace/simulation_ws/src/tools/start_competition.sh`，输入 `yes` 开始任务。停止命令为 `bash /workspace/simulation_ws/src/tools/stop.sh`。

GUI 入口：`python3 /workspace/simulation_ws/src/tools/gui.py`；不要与终端重复启动同一套环境。详细步骤见 [Docker 说明](docker/README.md) 和 [仿真 README](simulation_ws/README.md)。

## 模型与训练

人员检测训练入口为 `python yolo_train/scripts/person/train.py`，默认使用 YOLO26n，读取已按 9:1 划分的训练集和验证集，结果保存至 `yolo_train/results/person/`。依赖安装、参数调整与权重位置见 [人员训练说明](yolo_train/scripts/person/README.md)。

训练工具位于 `yolo_train/scripts/traffic_light/`，数据在 `yolo_train/datasets/traffic_light/`，部署权重在 `models/traffic_light/`。脚本的相对配置路径以 `yolo_train/` 为基准；已有数据已划分，重新划分时需将 `data.source` 指向 `datasets/traffic_light/raw`。训练前使用独立输出配置，避免覆盖已有权重。默认 Compose 挂载 `simulation_ws/` 和 `robot_ws/`，不挂载顶层模型和训练目录；交通灯权重的复制步骤见视觉部署说明。

## 注意事项

- 两个工作空间包含同名功能包，应在不同终端或环境中分别加载，不要叠加 `source`。
- 迁移前的 `build/`、`devel/` 等产物可能保存绝对路径，迁移后应重新生成；本次未编译、构建镜像或启动节点，以上迁移后的命令均待目标环境验证。
- 仿真结果写入 `simulation_ws/results/<run_id>/`，实车默认输出至 `~/.ros/community`；编译产物、缓存和新增运行结果由 `.gitignore` 忽略，必要模型、地图和配置继续保留。
- 原实车示例 `robot_ws/src/robot_description/world/test.world` 含历史 `/home/mowen/test_ws` 资源路径，默认实车入口不加载它；本次按限制保留模型文件。旧仿真辅助脚本 `start_simulation.sh` 仍有 Melodic 环境探测，推荐使用上述 Noetic 集中入口，该既有问题未在目录适配中修改。
- 当前实车任务尚缺异常温度、仪表识读、动态指示牌判断、独立泊车结果校验和全程 300 秒预算控制。

项目许可证见 [LICENSE](LICENSE)，第三方模型及依赖授权见 [视觉包声明](simulation_ws/src/robot_perception/NOTICE.md) 和原文件说明。
