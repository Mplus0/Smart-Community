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
# 小车已安装的 Cartographer 环境，需先于 robot_ws 加载
source /home/mowen/cartographer_ws/install_isolated/setup.bash
cd /home/mowen/workspace/Smart-Community/robot_ws
# 保留 Cartographer 环境，叠加已编译的实车工作空间
source devel/setup.bash --extend
```

首次部署时若尚无 `devel/setup.bash`，先跳过该行。首次部署或修改 ROS 功能包后编译：

```bash
catkin_make
source devel/setup.bash --extend
```

实车 Cartographer 安装于 `/home/mowen/cartographer_ws/install_isolated`。可用 `rospack find cartographer_ros` 检查环境；若启动建图时提示找不到该包，按上述顺序重新加载，并保留 `--extend`。

## 键盘建图与导航

各命令分别在已加载环境的终端执行：

```bash
# 终端 1：已有硬件运行时跳过
roslaunch community_bringup hardware.launch base_protocol:=aa_bb
# 终端 2：默认 Cartographer 建图，栅格和子地图分辨率均为 0.02 m；有桌面时可追加 rviz:=true
roslaunch community_bringup mapping.launch
# 或选择 GMapping（默认同样为 0.02 m 栅格）
# roslaunch community_bringup mapping.launch algorithm:=gmapping
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
roslaunch community_bringup navigation.launch map:=$HOME/maps/community_test.yaml navigation_map:=$HOME/maps/community_test.yaml
```

本地 RViz 启动方式见下一节。RViz 的 Fixed Frame 设为 `map`，通过 `2D Pose Estimate` 初始化定位，再使用 `2D Nav Goal` 测试导航。建图与导航不同时运行。

### TEB 局部规划（可选）

导航默认保留 DWA，可通过 `local_planner:=teb` 切换为 TEB。先将代码同步到实车，在 Ubuntu 20.04 / ROS Noetic 上安装并编译：

```bash
sudo apt update
sudo apt install ros-noetic-teb-local-planner
cd ~/workspace/Smart-Community/robot_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
rospack find teb_local_planner
```

保持硬件运行，停止已有导航及建图后启动（无需重启硬件）：

```bash
roslaunch community_bringup navigation.launch local_planner:=teb
# 使用新地图时同时指定 map:=... 和 navigation_map:=...
# 恢复 DWA：停止当前导航，再使用 local_planner:=dwa 启动
```

在另一终端加载 ROS 环境后确认：

```bash
rosparam get /move_base/base_local_planner
# 应为 teb_local_planner/TebLocalPlannerROS
rostopic info /move_base/local_plan
```

TEB 参数位于 `src/community_bringup/config/navigation/teb_local_planner_params.yaml`，使用 `/odom`、现有 `base_link` 车体轮廓及全向运动约束；前进/后退/横移速度上限与 DWA 对齐为 0.27/0.27/0.13 m/s，合成平移速度上限仍为 0.18 m/s，最大角速度为 0.5 rad/s。平移加速度上限为 0.5 m/s²，角加速度按实车 DWA 当前生效的 `acc_lim_theta` 对齐为 3.2 rad/s²（DWA 文件中的旧键 `acc_lim_th: 2` 不代表该生效值）。到点容差沿用 0.08 m / 0.15 rad。初始配置关闭多拓扑候选搜索，保留代价地图障碍检测及整条局部轨迹可行性检查。参数数值不是实际避障距离或速度的保证，需低速实测绕障、横移、倒车和停车。

Infra 可在启动导航前选择 DWA / TEB，切换前先停止导航。两种规划器的局部路径统一发布到 `/move_base/local_plan`，供 Infra 和 RViz 显示。升级后需重建 Infra 前端并重启后端。本次配置尚未在实车加载 TEB 插件或验证运动效果。

### 定位地图与导航地图

默认使用 `src/community_bringup/maps/map.yaml`（`map.pgm`）进行 AMCL 定位，使用 `map_nav.yaml`（`map_nav.pgm`）进行导航。两张图均为 245 × 249 像素，分辨率为 2 cm/格。定位图发布到 `/map`，导航图发布到 `/navigation_map`，供全局和局部代价地图的静态层使用。

导航图以原始墙面为基准，左侧内收 70 cm、右侧 12 cm、上下各 10 cm，左下角保留 70 × 70 cm 通行区。右上矩形距内收后的上边界 70 cm、右边界 75 cm，宽 96 cm、高 157 cm（按栅格取 158 cm）。中间“└”形区域的左下角距原始左墙 130 cm、底墙 70 cm；竖条宽 58 cm、高 275 cm（取 276 cm），底部横条长 213 cm（取 214 cm）、厚 55 cm（取 56 cm）。

设置固定禁行区域时，只在 `map_nav.pgm` 上用纯黑色实心填充，保留原图；不要改变图片尺寸、分辨率、原点或方向。修改后重启导航，在 RViz 中检查全局、局部代价地图及路径，并在实车低速条件下验证绕行效果。

更换地图时需同时指定两份处于相同坐标系、具有相同尺寸、分辨率和原点的地图，避免定位与规划错位。未绘制禁行区域时，两参数可指向同一份 YAML（如上例）；已有独立导航地图时使用：

```bash
roslaunch community_bringup navigation.launch map:=$HOME/maps/community_test.yaml navigation_map:=$HOME/maps/community_test_nav.yaml
```

测试地图 `src/community_bringup/maps/square_4p2m.yaml` 为无内部障碍物的正方形：内部净空 4.2 × 4.2 m，分辨率 0.02 m/格，内部 210 × 210 格，外围增加一格厚的占用边框，总尺寸 212 × 212 像素。内部左下角为地图坐标 `(0, 0)`，右上角为 `(4.2, 4.2)`，YAML 原点为 `(-0.02, -0.02)`。默认地图未切换，测试时在已加载工作空间环境的终端执行：

```bash
TEST_MAP="$(rospack find community_bringup)/maps/square_4p2m.yaml"
roslaunch community_bringup navigation.launch map:="$TEST_MAP" navigation_map:="$TEST_MAP"
```

配套导航图为 `square_4p2m_nav.yaml`，沿场地四边内收 8 cm，并填充以下禁行区。尺寸以厘米计，矩形尺寸按“宽 × 高”解释；70 cm 间距均以原始 4.2 m 场地边界为基准。左上矩形紧贴内收后的左、上边界。2 cm 栅格下，99、283、59 cm 分别向上取整为 100、284、60 cm。

| 禁行区 | 实际栅格尺寸 | 地图坐标范围（米） |
|---|---|---|
| 左上矩形 | 70 × 340 cm | x: 0.08–0.78，y: 0.72–4.12 |
| 右上矩形 | 160 × 100 cm；右、上间距均为 70 cm | x: 1.90–3.50，y: 2.50–3.50 |
| L 型竖条 | 60 × 284 cm；顶部距场地上边界 70 cm | x: 1.30–1.90，y: 0.66–3.50 |
| L 型底部横条 | 220 × 60 cm；右端距场地右边界 70 cm | x: 1.30–3.50，y: 0.66–1.26 |

L 型两条区域重叠组成“└”；按上述尺寸，竖条右侧与右上矩形左侧相接，没有通道。使用配套导航图时运行：

```bash
TEST_MAP_DIR="$(rospack find community_bringup)/maps"
roslaunch community_bringup navigation.launch map:="$TEST_MAP_DIR/square_4p2m.yaml" navigation_map:="$TEST_MAP_DIR/square_4p2m_nav.yaml"
```

实车定位需设置正确的初始位姿；原地图的航点不能直接沿用。该地图尚未进行实车定位与导航验证。

`community.launch` 和 `finals.launch` 同样支持 `map`、`navigation_map` 参数。RViz 添加 Map 显示并选择 `/navigation_map` 可查看导航原图。

保持建图或定位运行，将小车停在目标点后记录航点和任务（保存为新文件）：

```bash
python3 tools/record_waypoints.py --output ~/maps/route.yaml
```

回车记录 `navigation`，也可输入 `person A`、`plate 1` 等任务；`u` 撤销上一点，`q` 退出。每点自动保存，坐标为 `map` 下的 `x/y/yaw`。使用记录的路线启动巡检：

```bash
roslaunch community_mission mission.launch route:=$HOME/maps/route.yaml
```

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

当前阶段使用仅导航模式：启动硬件、定位和导航后，选择以下任一命令，无需启动识别、语音和录像服务。所有航点均只导航，跳过识别、红绿灯等待、播报和录像：

```bash
# 有障碍物（默认 blocked）
roslaunch community_mission mission.launch navigation_only:=true obstacle_mode:=blocked
# 无障碍物
roslaunch community_mission mission.launch navigation_only:=true obstacle_mode:=clear
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

人员、火焰、垃圾桶及障碍物使用 `yolo_train/model/` 导出的 YOLO26n 模型，工作空间文件名分别为 `person.onnx`、`fire.onnx`、`trashbin.onnx`、`obstacle.onnx`。垃圾桶类别使用 `grey_close/grey_open`。检测总入口默认启动障碍物服务；可单独运行 `roslaunch community_perception obstacle_detector.launch`，或用 `--detector obstacle_detector` 进行浏览器预览。障碍物服务尚未接入自动路线选择，路线仍由 `obstacle_mode` 指定。

`tools/test_detector_model.py` 通过 `--detector` 选择 `src/community_perception/config/detectors.yaml` 中的模型，自动读取模型路径、类别名、置信度和 IoU 阈值。脚本独立推理，无需启动巡检或检测服务，支持实车七个 ONNX 检测模型，显示检测框、类别、置信度、全画面目标数量和处理耗时。

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
| `people_detector` | 人员；按实车配置显示 `community_person`（社区人员）、`outsider`（外来人员） |
| `traffic_light_detector` | 红、黄、绿交通灯 |
| `fire_detector` | 火焰 |
| `obstacle_detector` | 障碍物 |
| `trash_bin_detector` | 垃圾桶颜色及开闭状态 |
| `waste_detector` | 垃圾类别 |
| `bicycle_detector` | 电动车站立、倒伏 |

自定义配置或单张图片测试：

```bash
python3 tools/test_detector_model.py --config /绝对路径/detectors.yaml --detector trash_bin_detector
# 临时覆盖模型文件，类别和阈值仍来自选定配置
python3 tools/test_detector_model.py --detector people_detector --model /绝对路径/person.onnx
# 无需 ROS 或相机的图片预览
python3 tools/test_detector_model.py --detector people_detector --image ../yolo_train/datasets/person/val/images/0011.jpg
```

配置沿用 `detectors.yaml` 的 `detectors` 结构；`model_path` 支持绝对路径、相对配置文件的路径及 `$(find community_perception)`。`class_names` 必须按模型类别 ID 顺序填写。支持静态 FP32 输入 `[1,3,H,W]`、原始检测输出 `[1,4+类别数,N]`；YOLO26 应以 `end2end=False, nms=False` 导出。旧入口 `tools/test_person_model.py` 转发到同一通用脚本。

### 图片采集

`tools/collect_dataset.py` 每个进程固定使用 `--camera` 指定的相机，网页显示相机名称并提供实时画面和拍照按钮。两台 USB 相机在两个终端分别启动，可同时打开各自网页。

在 `robot_ws` 目录中执行：

```bash
# 终端 1：DJI，默认端口 8081
python3 tools/collect_dataset.py trashbin_dji --camera dji --web-only --web-host 0.0.0.0
# 终端 2：Orbbec 彩色，默认端口 8082
python3 tools/collect_dataset.py trashbin_orbbec --camera orbbec --web-only --web-host 0.0.0.0
```

分别访问 `http://小车IP:8081/` 和 `http://小车IP:8082/`。使用 VS Code SSH 端口转发时，转发 8081、8082，再打开 `http://localhost:8081/` 和 `http://localhost:8082/`。可用 `--web-port` 覆盖默认端口，两个进程必须使用不同端口。

原图分别保存至 `yolo_train/datasets/trashbin_dji/images/` 和 `yolo_train/datasets/trashbin_orbbec/images/`；可替换数据集名称或用 `--datasets-dir` 指定根目录。DJI 需选择 USB 网络摄像头模式。启动前停止占用对应相机的旧采集程序或相机节点；两个终端各自按 Ctrl+C 退出。

已有 ROS 相机话题时使用 `--camera ros --topic /usb_cam/image_raw --web-only`，默认端口 8080。终端回车拍照模式：

```bash
python3 tools/collect_dataset.py trashbin --camera ros --no-preview
```

回车保存，输入 `q` 后回车退出。此工具不生成目标框标注或划分数据集。

## 常用配置与结果

默认定位地图 `src/community_bringup/maps/map.yaml` 来自实车 `community_test`，同目录下的 `map_nav.yaml` 来自配套的 `community_test_nav`。更换地图后，巡检前需核对 `route.yaml` 中的航点是否与新地图坐标匹配，且不落在禁行区域内。

| 位置 | 用途 |
|---|---|
| `src/community_bringup/config/hardware.yaml` | 硬件节点参数 |
| `src/community_bringup/config/camera.yaml` | 相机控制参数，启动时通过 `configure_camera:=true` 启用配置 |
| `src/community_bringup/config/navigation/` | EKF、定位、规划和代价地图参数 |
| `src/community_mission/config/route.yaml` | 巡检航点及每个点的任务 |
| `src/community_mission/config/runtime.yaml` | 等待时间、重试、录像和语音参数 |
| `src/community_mission/config/recognition_rules.yaml` | 人员 ROI、车辆去重等识别业务规则 |
| `src/community_perception/config/detectors.yaml` | 检测模型路径、类别、阈值和保存设置 |

默认设备为底盘 `/dev/carserial`、雷达 `/dev/lslidar`、IMU `/dev/imu`、相机 `/dev/video0`。组合启动支持通过 `base_port`、`imu_port`、`camera_device`、`map`、`navigation_map`、`route` 和 `data_dir` 覆盖对应参数。

默认 `route.yaml` 包含 21 个航点，保留原路线坐标和朝向；补录的火焰检测点为 `wp_09`，B 区人员检测点为 `wp_13`。仅导航模式下，到达 `wp_04` 后按 `obstacle_mode` 手动选择后续顺序，不调用障碍物模型：

- `blocked`（默认）：01 → 02 → 03 → 04 → 05 → 06 → 07 → 08 → 09 → 10 → 13 → 11 → 12 → 14 → 15 → 16 → 17 → 18 → 19 → 20 → 21。
- `clear`：01 → 02 → 03 → 04 → 13 → 12 → 11 → 08 → 06 → 05 → 07 → 09 → 10 → 14 → 15 → 16 → 17 → 18 → 19 → 20 → 21。

上述编号均为 `wp_XX`。两条路线共用 YAML 中的航点坐标，分支只引用 ID；从 `wp_14` 起沿公共路线执行，到达 `wp_21` 后正常结束并保存结果。`obstacle_mode` 仅支持 `blocked` 和 `clear`，仅导航模式下生效。

每点只配置一个 `task`：人物任务同时执行所属 A/B 区违停检测；`wp_04` 障碍物识别、`wp_11` 仪表盘识别、`wp_12` 温度检测尚未接入，使用 `task: navigation` 占位。`wp_19`、`wp_20` 配置录像开始、结束，`wp_21` 为最终停车点。仅导航模式会跳过所有任务；现有完整任务模式仍按 YAML 列表顺序执行，绿灯等待默认上限 30 秒，需连续 3 帧有效绿灯。

巡检结果默认位于 `~/.ros/community/<run_id>/`，包含结果 JSON、日志、照片和视频；检测服务调试图位于 `~/.ros/community/detections/`，OCR 调试图位于 `~/.ros/community/ocr/`。图片采集目录由 `--datasets-dir` 和数据集名称决定。

## 开发检查

在工作空间目录执行离线检查：

```bash
python3 tools/validate_workspace.py
python3 tools/test_python_migration.py
```

识别效果、定位导航和完整巡检流程需要在目标硬件与现场验证。详细说明见 [架构与接口](docs/ARCHITECTURE.md)、[实车验收](docs/ACCEPTANCE.md) 和 [底盘协议说明](docs/BASE_PROTOCOL_REVIEW.md)。
