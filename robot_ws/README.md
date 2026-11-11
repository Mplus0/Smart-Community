# 智慧社区实车 ROS 工作空间

`robot_ws` 是基于 ROS Noetic / Python 3 的实车工作空间，包含硬件接入、建图导航、视觉识别和巡检任务调度。仿真程序位于仓库同级的 `simulation_ws/`，两套工作空间分别编译和加载环境。

## 功能与结构

| 功能包 | 主要功能 |
|---|---|
| `community_bringup` | 底盘串口与里程计、相机、硬件组合启动、EKF、建图和导航 |
| `community_perception` | 人员、交通灯、火焰、垃圾桶、垃圾、电动车检测，车牌与文字 OCR，视频倒伏统计 |
| `community_mission` | 按路线执行巡检、汇总识别结果、语音播报、拍照和录像 |
| `lslidar` / `lslidar_driver` / `lslidar_msgs` | 雷达驱动及消息 |
| `wit_ros_imu` | IMU 驱动 |
| `robot_description` | 机器人 URDF 和相关资源 |

```text
robot_ws/
├── src/
│   ├── community_bringup/
│   │   ├── launch/         # 分层和组合启动入口
│   │   ├── config/         # 硬件、相机及导航参数
│   │   ├── maps/           # 导航地图
│   │   └── scripts/        # 底盘通信、里程计、键盘控制等
│   ├── community_perception/
│   │   ├── config/         # 模型路径、类别与检测阈值
│   │   ├── models/         # 实车 ONNX 模型
│   │   ├── scripts/        # 检测、OCR 和视频统计服务
│   │   └── srv/            # ROS 服务定义
│   ├── community_mission/
│   │   ├── config/         # 巡检路线、识别规则与运行参数
│   │   └── scripts/        # 巡检任务及辅助节点
│   └── ...                # 雷达、IMU 驱动和机器人描述
├── tools/                 # 图片采集、浏览器模型测试及离线检查
├── docs/                  # 架构、验收及驱动说明
└── requirements.txt       # Python 依赖
```

ROS 相关依赖通过目标机系统环境安装，Python 依赖见 `requirements.txt`。ROS 的 `cv_bridge` 使用系统 OpenCV/NumPy，安装依赖时保持 `numpy<2`。

## 终端环境

连接小车：

```bash
ssh mowen
```

除“Ubuntu 本地 RViz 调试”外，运行命令均在小车的 `robot_ws` 目录执行。每个小车终端先加载环境：

```bash
source /opt/ros/noetic/setup.bash
cd /home/mowen/workspace/Smart-Community/robot_ws
# 已编译的工作空间加载环境
source devel/setup.bash
```

首次部署时若尚无 `devel/setup.bash`，先跳过该行。首次部署或修改 ROS 功能包后编译：

```bash
catkin_make
source devel/setup.bash
```

## 键盘建图与导航

各命令分别在已加载环境的终端执行：

```bash
# 终端 1：已有硬件运行时跳过
roslaunch community_bringup hardware.launch base_protocol:=aa_bb
# 终端 2：建图；有图形桌面时可追加 rviz:=true
roslaunch community_bringup mapping.launch
# 或选择 Cartographer（默认使用 GMapping）
# roslaunch community_bringup mapping.launch algorithm:=cartographer
# 终端 3：键盘控制，SSH 使用 ssh -t mowen
rosrun community_bringup teleop_twist_keyboard.py
```

键盘终端保持焦点，`i` 前进、`,` 后退、`j/l` 左右转、`J/L` 左右横移，空格或 `k` 停车，Ctrl+C 退出。键盘控制期间停止自动巡检和其他速度发布节点。

保持建图运行，在另一终端保存地图：

```bash
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/community_test
```

停止建图和键盘控制，保留硬件，再启动新地图导航：

```bash
roslaunch community_bringup navigation.launch map:=$HOME/maps/community_test.yaml
```

本地 RViz 启动方式见下一节。RViz 的 Fixed Frame 设为 `map`，通过 `2D Pose Estimate` 初始化定位，再使用 `2D Nav Goal` 测试导航。建图与导航不同时运行。

## Ubuntu 本地 RViz 调试

小车运行硬件及建图或导航，本地电脑只运行 RViz。两种本地启动方式共用以下网络配置，选择其中一种即可；无需编译本地实车工作空间。

### 多机通信准备

以下示例使用小车 IP `192.168.31.59`、本机局域网 IP `192.168.31.83`。在本机执行 `ip route get 192.168.31.59`，查看 `src` 后的本机地址；地址变化时同步修改命令。小车与本机需能双向通信。

每个小车 ROS 终端先按“终端环境”加载工作空间，再在启动节点前设置：

```bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://192.168.31.59:11311
export ROS_IP=192.168.31.59
```

小车按上文启动硬件及 `mapping.launch rviz:=false`，或硬件及 `navigation.launch`。建图与导航不同时运行，已有硬件节点时不重复启动。

### 方式一：本机已安装 ROS Noetic

在本地 Ubuntu 图形桌面终端执行，仓库路径按实际位置修改：

```bash
source /opt/ros/noetic/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://192.168.31.59:11311
export ROS_IP=192.168.31.83

cd /home/mplus0/workspace/smart-community/robot_ws
# 建图显示
roslaunch src/community_bringup/launch/rviz/slam_rviz.launch
# 或导航显示
# roslaunch src/community_bringup/launch/rviz/navigation_rviz.launch
```

### 方式二：Ubuntu 22.04 使用 Docker

复用 `docker/docker-compose.yml` 的 Noetic 容器。Compose 已配置两套工作空间挂载、host 网络、X11 显示和 NVIDIA GPU；宿主机需具备现有 Compose 要求的 NVIDIA 驱动及容器 GPU 支持。无需重新构建已有镜像，首次应用新增挂载的方法见 [更新已有容器的挂载](../docker/README.md#更新已有容器的挂载)。

在本地 Ubuntu 图形桌面终端运行脚本，两个 IP 参数分别是小车地址和本机局域网地址：

```bash
cd /home/mplus0/workspace/smart-community/robot_ws
# 建图显示（省略 slam 时也是建图）
./tools/rviz.sh slam 192.168.31.59 192.168.31.83
# 或导航显示
./tools/rviz.sh navigation 192.168.31.59 192.168.31.83
```

脚本自动定位仓库、启动已有 Compose 容器、授权 X11 并运行 RViz；IP 仅传给本次 RViz 进程，不写入 Compose。支持 `--help`。首次使用或更新挂载需先按 Docker 文档创建/更新容器，脚本不会重新构建镜像或重新创建容器。

无需启动 Gazebo、容器内 ROS Master 或编译工作空间。退出 RViz 不会删除容器。所有使用该显示授权的容器图形程序退出后，可在宿主机撤销授权：

```bash
xhost -si:localuser:$(id -un)
```

### 显示内容

两个入口的 Fixed Frame 均为 `map`，默认不显示相机图像或三维模型：

| 入口 | 显示内容 |
|---|---|
| `slam_rviz.launch` | 地图、雷达、TF、里程计，仅用于查看 |
| `navigation_rviz.launch` | 增加全局/局部代价地图、规划路径、机器人轮廓，以及 `2D Pose Estimate`、`2D Nav Goal` 工具 |

配置位于 `src/community_bringup/rviz/`，可用 `config:=/绝对路径/custom.rviz` 覆盖；Docker 内需使用容器可访问的路径。两个 launch 文件及 `rviz/` 配置目录也可按原相对目录结构复制到其他 Ubuntu 电脑运行。实际多机通信与显示效果需连接小车验证。

## 巡检启动

确认地图、定位和路线后，选择分层或组合方式启动。

### 分层启动

各命令分别在已加载环境的终端执行，本车底盘协议使用 `aa_bb`：

```bash
# 终端 1：底盘、雷达、IMU、相机
roslaunch community_bringup hardware.launch base_protocol:=aa_bb
# 终端 2：定位与导航
roslaunch community_bringup navigation.launch
# 终端 3：识别、语音、录像服务
roslaunch community_bringup services.launch
# 终端 4：确认地图、定位和路线后启动自动任务
roslaunch community_mission mission.launch
```

### 组合启动

与分层启动二选一，不重复启动硬件或相机节点：

```bash
# 启动硬件、导航和服务，暂不运行任务
roslaunch community_bringup community.launch base_protocol:=aa_bb
# 或直接启动自动巡检，服务就绪后自动执行任务
roslaunch community_bringup finals.launch base_protocol:=aa_bb
```

## 相机与视觉工具

### 单独启动相机

终端 1 启动 ROS Master 并保持运行；已有 `roscore` 或 `roslaunch` 提供 Master 时跳过：

```bash
roscore
```

终端 2 启动相机；已有 `hardware.launch` 或其他相机节点运行时跳过：

```bash
rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _pixel_format:=mjpeg _image_width:=640 _image_height:=480 _framerate:=30
```

### 浏览器相机预览

加载小车终端环境并保持相机运行，默认通过 ROS 订阅 `/usb_cam/image_raw`：

```bash
python3 tools/camera_web_preview.py
```

通过环境变量 `CAMERA` 选择来源：

```bash
CAMERA=ros python3 tools/camera_web_preview.py       # 默认，需先启动相机话题
CAMERA=dji python3 tools/camera_web_preview.py       # DJI Pocket 3，3840×2160，30 fps
CAMERA=orbbec python3 tools/camera_web_preview.py    # Orbbec，1920×1080，30 fps
```

也可用 `--camera ros|dji|orbbec` 覆盖环境变量。直连模式使用 `/dev/v4l/by-id/` 的设备路径，避免插拔后设备编号变化；不依赖 ROS，且不要与其他程序同时占用同一相机。可用 `--width 1920 --height 1080 --fps 30` 调整直连采集参数，启动日志会显示实际协商的分辨率。DJI 默认使用 MJPEG 4K。

服务默认监听 `0.0.0.0:8765`，同一局域网的浏览器可直接访问 `http://小车IP:8765/`。用 `--host 127.0.0.1` 可限制为仅小车本机访问，此时需转发 8765 端口并打开 `http://localhost:8765/`。用 `--topic` 更改 ROS 话题，`--port` 更改端口；Ctrl+C 停止预览。

### 模型检测浏览器预览

`tools/test_detector_model.py` 通过 `--detector` 选择 `src/community_perception/config/detectors.yaml` 中的模型，自动读取模型路径、类别名、置信度和 IoU 阈值。脚本独立推理，无需启动巡检或检测服务，支持实车六个 ONNX 检测模型，显示检测框、类别、置信度、全画面目标数量和处理耗时。

保持相机运行，另开终端执行：

```bash
# 人员模型（默认）
python3 tools/test_detector_model.py --detector people_detector --port 8080
# 或选择垃圾桶模型
python3 tools/test_detector_model.py --detector trash_bin_detector --port 8080
# 覆盖配置阈值及最大推理频率
python3 tools/test_detector_model.py --detector trash_bin_detector --conf 0.5 --iou 0.4 --fps 10 --port 8080
```

浏览器访问 `http://小车IP:8080/`，Ctrl+C 退出。切换模型时先停止当前脚本，再使用新的 `--detector` 启动；不同模型同时预览需使用不同端口。

| `--detector` | 检测目标 |
|---|---|
| `people_detector` | 人员；按实车配置显示 `good`（社区人员）、`bad`（外来人员） |
| `traffic_light_detector` | 红、黄、绿交通灯 |
| `fire_detector` | 火焰 |
| `trash_bin_detector` | 垃圾桶颜色及开闭状态 |
| `waste_detector` | 垃圾类别 |
| `bicycle_detector` | 电动车站立、倒伏 |

自定义配置或单张图片测试：

```bash
python3 tools/test_detector_model.py --config /绝对路径/detectors.yaml --detector trash_bin_detector
# 临时覆盖模型文件，类别和阈值仍来自选定配置
python3 tools/test_detector_model.py --detector people_detector --model /绝对路径/people.onnx
# 无需 ROS 或相机的图片预览
python3 tools/test_detector_model.py --detector people_detector --image ../yolo_train/datasets/person/val/images/0011.jpg
```

配置沿用 `detectors.yaml` 的 `detectors` 结构；`model_path` 支持绝对路径、相对配置文件的路径及 `$(find community_perception)`。`class_names` 必须按模型类别 ID 顺序填写。支持静态 FP32 输入 `[1,3,H,W]`、原始检测输出 `[1,4+类别数,N]`；YOLO26 应以 `end2end=False, nms=False` 导出。旧入口 `tools/test_person_model.py` 转发到同一通用脚本。

### 图片采集

`tools/collect_dataset.py` 保存相机原图，支持浏览器拍照和终端采集；不会自动生成目标框标注或划分数据集。

保持相机运行，另开终端启动浏览器预览和拍照：

```bash
python3 tools/collect_dataset.py trashbin --datasets-dir ../yolo_train/datasets --web-port 8081 --web-host 0.0.0.0 --web-only
```

浏览器访问 `http://小车IP:8081/`，点击“拍照保存”。原图保存至 `yolo_train/datasets/trashbin/images/`；将 `trashbin` 替换为需要的数据集名称。默认订阅 `/usb_cam/image_raw`，可通过 `--topic` 修改。

终端拍照模式：

```bash
python3 tools/collect_dataset.py trashbin --datasets-dir ../yolo_train/datasets --no-preview
```

回车保存，输入 `q` 后回车退出。各网页工具使用不同端口；Ctrl+C 关闭。

## 常用配置与结果

默认导航地图 `src/community_bringup/maps/map.yaml` 已替换为 2026-11-11 实车保存的 `community_test` 地图，分辨率为 0.05 m/像素。更换地图后，巡检前需核对 `route.yaml` 中的航点是否与新地图坐标匹配。

| 位置 | 用途 |
|---|---|
| `src/community_bringup/config/hardware.yaml` | 硬件节点参数 |
| `src/community_bringup/config/camera.yaml` | 相机控制参数，启动时通过 `configure_camera:=true` 启用配置 |
| `src/community_bringup/config/navigation/` | EKF、定位、规划和代价地图参数 |
| `src/community_mission/config/route.yaml` | 巡检航点及每个点的任务 |
| `src/community_mission/config/runtime.yaml` | 等待时间、重试、录像和语音参数 |
| `src/community_mission/config/recognition_rules.yaml` | 人员 ROI、车辆去重等识别业务规则 |
| `src/community_perception/config/detectors.yaml` | 检测模型路径、类别、阈值和保存设置 |

默认设备为底盘 `/dev/carserial`、雷达 `/dev/lslidar`、IMU `/dev/imu`、相机 `/dev/video0`。组合启动支持通过 `base_port`、`imu_port`、`camera_device`、`map`、`route` 和 `data_dir` 覆盖对应参数。

航点按列表顺序执行，每点只配置一个 `task`：普通导航点用 `task: navigation`，人物点用 `task: person` 并指定 `area: A` 或 `B`，红绿灯点用 `task: traffic_light`。到点后直接执行该任务。绿灯等待默认上限 30 秒，需连续 3 帧有效绿灯。

巡检结果默认位于 `~/.ros/community/<run_id>/`，包含结果 JSON、日志、照片和视频；检测服务调试图位于 `~/.ros/community/detections/`，OCR 调试图位于 `~/.ros/community/ocr/`。图片采集目录由 `--datasets-dir` 和数据集名称决定。

## 开发检查

在工作空间目录执行离线检查：

```bash
python3 tools/validate_workspace.py
python3 tools/test_python_migration.py
```

识别效果、定位导航和完整巡检流程需要在目标硬件与现场验证。详细说明见 [架构与接口](docs/ARCHITECTURE.md)、[实车验收](docs/ACCEPTANCE.md) 和 [底盘协议说明](docs/BASE_PROTOCOL_REVIEW.md)。
