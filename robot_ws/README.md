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
├── infra/                 # 实车 Web 工作台
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

## Web 工作台

在已加载实车 ROS 环境的终端运行 `roslaunch community_bringup infra.launch`。依赖安装、浏览器访问和当前功能范围见 [工作台说明](infra/README.md)。

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

### 局部规划器（默认 DWA）

DWA 使用实车低速导航配置。move_base 与 DWA 控制频率均为 10 Hz；前进/后退/横移速度上限为 0.25/0.12/0.08 m/s，合成平移速度上限为 0.25 m/s，角速度上限为 0.3 rad/s，平移加速度为 0.6 m/s²、角加速度为 1.0 rad/s²。到点容差为 0.05 m / 0.05236 rad。参数位于 `src/community_bringup/config/navigation/dwa_local_planner_params.yaml`。保留当前全局规划、代价地图、车体轮廓、激光避障和巡检路线执行方案。该组合尚未在本车验证。

导航默认使用 DWA，`navigation.launch`、`community.launch` 及调用后者的 `finals.launch` 均生效。默认启动无需指定规划器；可通过 `local_planner:=teb` 在前两个入口切换为 TEB。使用 TEB 前，先将代码同步到实车，在 Ubuntu 20.04 / ROS Noetic 上安装并编译：

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
roslaunch community_bringup navigation.launch
# 切换 TEB：停止当前导航，再运行 roslaunch community_bringup navigation.launch local_planner:=teb
# 使用新地图时同时指定 map:=... 和 navigation_map:=...
# 恢复 DWA：停止当前导航，再使用默认命令或 local_planner:=dwa 启动
```

在另一终端加载 ROS 环境后确认：

```bash
rosparam get /move_base/base_local_planner
# 默认应为 dwa_local_planner/DWAPlannerROS；选择 TEB 时为 teb_local_planner/TebLocalPlannerROS
rostopic info /move_base/local_plan
```

TEB 参数保留在 `src/community_bringup/config/navigation/teb_local_planner_params.yaml`，本次未随 DWA 更新；其速度、加速度和到点容差与当前 DWA 配置不同。

move_base 关闭仅按平移进展判断的振荡超时（`oscillation_timeout: 0`），避免同位置转向任务被误判；`oscillation_distance: 0.05` 在此超时关闭时不触发振荡恢复。关闭默认恢复行为中的自动旋转（`clearing_rotation_allowed: false`），保留清图恢复和正常导航转向。无有效控制等待上限仍为 3 秒，全局规划等待上限为 5 秒，巡检单目标超时仍为 120 秒。修改恢复行为后需重启导航生效。

两种规划器的局部路径统一发布到 `/move_base/local_plan`，供 RViz 显示。已在实车确认 TEB 插件加载并运行导航，定位、绕障及通行稳定性仍需现场验证。

### 定位地图与导航地图

定位地图和导航地图分别保存为 `src/community_bringup/maps/localization_map.yaml/.pgm` 与 `navigation_map.yaml/.pgm`。定位图主要包含外框围栏，规划图包含固定禁行区域；两图均为 431 × 431 像素，分辨率为 1 cm/格，原点为 `(-0.03, -0.03, 0)`，对应约 4.25 m 场地。两张地图已绕栅格中心旋转 180°，YAML 原点不变，停车起点位于左下侧。相对旋转前地图的坐标转换为 `x_new = 4.25 - x_old`、`y_new = 4.25 - y_old`，朝向增加 π 并归一化。AMCL 默认初始位姿为 `(0.411719, 0.409062, 0)`，仅适用于小车停在该停车起点且车头沿地图 +x 的情况；其他摆放位置通过 RViz 设置实际位姿。

两图分别发布到 `/map` 和 `/navigation_map`。保留当前全局和局部代价地图的静态层、激光障碍层与膨胀层，车体轮廓保留原值（包围尺寸约 34 × 28 cm），全局和局部 `footprint_padding` 均为 0，不额外扩张车体。规划底图包含固定禁行区，`navigation_map_node.py` 订阅 `/navigation_map_base` 并发布 `/navigation_map`。选路后只在对应入口叠加一格厚的占用封口（当前地图为 1 cm），定位图和磁盘底图不变。全局和局部静态层均接收更新，激光实时避障继续保留。

场地布局分析图仍保存在 `src/community_bringup/maps/gazebo_layout_analysis.png`，原始参考图为同目录的 `field_reference.png`。导航调参前的基线备份位于 `backups/navigation/20261117_210643_before_tuning/`。

停止已有建图或导航，保留硬件运行，在已加载 ROS 环境的终端启动：

```bash
roslaunch community_bringup navigation.launch
```

在 RViz 使用 `2D Pose Estimate` 设置实际初始位姿，确认激光与围栏对齐，再以 `2D Nav Goal` 试跑。该地图与 DWA 组合尚未在本车验证。默认路线已更新为旋转后地图下重新采集的 21 个原任务点，并补充 11 个转角过渡点；正式巡检前先使用仅导航模式确认路线。

更换地图时同时指定匹配的 `map` 和 `navigation_map`：

```bash
roslaunch community_bringup navigation.launch map:=$HOME/maps/community_test.yaml navigation_map:=$HOME/maps/community_test_nav.yaml
```

`community.launch` 和 `finals.launch` 同样支持 `map`、`navigation_map` 参数。RViz 添加 Map 显示并选择 `/navigation_map` 可查看导航原图。

保持建图或定位运行，将小车停在目标点后记录航点和任务（保存为新文件）：

```bash
python3 tools/record_waypoints.py --output ~/maps/route.yaml
```

回车记录 `navigation`，也可输入 `person A`、`plate 1` 等任务；`u` 撤销上一点，`q` 退出。每点自动保存，坐标为 `map` 下的 `x/y/yaw`。使用记录的路线启动巡检：

```bash
roslaunch community_mission mission.launch route:=$HOME/maps/route.yaml
```

### 按场地图生成新扫描地图的禁行区

`tools/generate_forbidden_map.py` 从随附的 `maps/field_reference.png` 白线计算三个禁行区，不读取仿真手点坐标。参考原图与 Gazebo 地面纹理一致，整图对应 4.2 × 4.2 米，内侧白框约为 4.0 米。方向按停车区在左、起始停车位在左下解释。

先保存新的扫描地图，再确定场地左下、右下、右上、左上四角在扫描图 `map` 坐标系中的位置。角点应对应实际 4.2 米场地边界，而非参考图内侧白框；以起始停车位确认四角方向。四角仅用于一次场地标定，禁行区顶点由图片自动计算。将下面的 `X0 Y0 ... X3 Y3` 替换为实际角点坐标（米）：

```bash
python3 tools/generate_forbidden_map.py \
  --map "$HOME/maps/community_new.yaml" \
  --output "$HOME/maps/community_new_nav" \
  --corners X0 Y0 X1 Y1 X2 Y2 X3 Y3
```

脚本只拟合旋转和平移，不缩放或旋转扫描图；默认角点最大标定误差超过 8 cm 时拒绝生成。输出新的导航图 `.pgm/.yaml`、禁行区及标定记录 `_zones.yaml`、角点和区域轮廓预览 `_preview.png`，不覆盖已有输出，不修改航点或自动启动导航。核对预览及实测禁行边界后显式加载：

```bash
roslaunch community_bringup navigation.launch \
  map:="$HOME/maps/community_new.yaml" \
  navigation_map:="$HOME/maps/community_new_nav.yaml"
```

场地布局或尺寸与参考图片不同，需先核对实际区域；脚本不会自动识别道具或推断正方形朝向。离线几何检查：`python3 tools/test_generate_forbidden_map.py -v`。实际定位、避障和通行效果仍需实车验证。

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
# 前方被障碍物阻挡，向左绕行（默认 blocked）
roslaunch community_mission mission.launch navigation_only:=true obstacle_mode:=blocked
# 障碍物偏左，前方可直行
roslaunch community_mission mission.launch navigation_only:=true obstacle_mode:=clear
```

### 单航点运行

先启动硬件、定位和导航，确认当前定位；通过 `single_waypoint` 指定 `route.yaml` 中的航点 ID，直接导航到该点，坐标、朝向和任务沿用路线配置：

```bash
# 只导航到 wp_05，到达后结束；无需识别服务
roslaunch community_mission mission.launch single_waypoint:=wp_05 navigation_only:=true
# 导航到 wp_05，执行人员及违停识别后结束；须先启动相机和相关服务
roslaunch community_mission mission.launch single_waypoint:=wp_05 navigation_only:=false
# 导航到 wp_04，识别障碍物并记录选路结果，随后结束，不执行后续路线
roslaunch community_mission mission.launch single_waypoint:=wp_04 navigation_only:=false
```

不指定 `single_waypoint` 时仍执行完整路线。单点红绿灯任务沿用连续确认及超时规则；人员统计仅代表该点。录像开始/结束任务依赖配对，单点执行会在导航前拒绝，但可使用 `navigation_only:=true` 导航到录像点。结果仍记录全部航点，仅执行的航点更新状态，其余保留 `NOT_EXECUTED`。不要与正在运行的巡检或其他导航目标同时使用。

### 组合启动

与分层启动二选一，不重复启动硬件或相机节点：

```bash
# 启动硬件、导航和服务，暂不运行任务
roslaunch community_bringup community.launch base_protocol:=aa_bb
# 或直接启动自动巡检，服务就绪后自动执行任务
roslaunch community_bringup finals.launch base_protocol:=aa_bb
```

## 相机与视觉工具

### UGREEN 仪表相机

`hardware.launch` 同时启动两台可见光相机。Orbbec 彩色相机发布 `/usb_cam/image_raw`，用于常规识别和录像。UGREEN 按设备 ID 以 3840×2160 MJPEG 采集，发布 `/ugreen_4k/image_raw`，工作台相机页选择该 ROS 来源即可预览。同一设备应由一个采集节点持有，部署后启动正式节点前需退出此前手动启动的 UGREEN 预览进程。

`wp_11` 到点后从 UGREEN 获取新帧，使用随工作空间编译安装的 4K 标定读取仪表温度与湿度。结果写入 `mission_results.json`，原图和指针叠加图保存在本轮 `photo/` 下。读取超时会按航点任务失败收尾。单独启动 `mission.launch` 时，可通过 `meter_image_topic` 和 `meter_profile` 指定图像话题和标定文件。

在 Web 工作台的相机页选择 **仪表盘调试（实时过程图）**，自动切到 UGREEN 4K 视频源。四格画面依次显示原图与指针、原图灰度、模板与当前帧的匹配点、透视校正后的读针灰度图，右侧显示读数和匹配内点数。

在 `robot_ws/` 下可用单张照片离线检查表盘识别过程，例如：

```bash
python3 tools/meter_debug.py /path/to/frame.jpg --output-dir /tmp/meter_debug
```

输出目录须尚未创建。工具默认使用 4K 标定，可用 `--profile` 指定其他标定。输出包括原图、灰度图、RANSAC 内点匹配图、透视校正图、读针用灰度图、指针叠加图和 `result.json`。对齐失败时仍保存原图、灰度图和失败状态。

### Seek 红外热成像采集

`community_perception` 提供独立的 `thermal_camera` 节点。SDK 3.4 的必要文件位于
`src/community_perception/third_party/seekware/`，支持 Linux x86_64、aarch64 和 armv7l，
无需额外设置库路径。组合启动的 `services.launch` 会启动热像采集和楼宇 D 四分区检测；若已有红外采集节点，单独启动 `services.launch` 时可传 `start_thermal_camera:=false` 共享温度话题。
已在小车 x86_64 / ROS Noetic 上验证实际采集、温度输出和网页预览；ARM 运行库尚未实机验证。

首次使用，在接入红外相机的电脑上安装 USB 权限规则，然后重新插拔相机：

```bash
sudo cp src/community_perception/third_party/seekware/udev/10-seekthermal.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

系统需有 `libusb-1.0-0`，Python 需有 `numpy`、`rospkg` 和现有 ROS `cv_bridge`。
更新后执行 `catkin_make --pkg community_perception`，重新加载 `source devel/setup.bash`。
关闭厂家示例或其他占用红外相机的程序，再独立启动（不需要启动底盘或导航）：

```bash
roslaunch community_perception thermal_camera.launch
```

节点每两秒输出中心、最低和最高温度；默认发布上限 9 Hz，可用 `publish_rate:=15` 调整，实际频率受设备限制。
读取失败时退出并关闭设备，排除故障后重新启动。

| 话题 | 类型与内容 |
| --- | --- |
| `/thermal_camera/image_raw` | `sensor_msgs/Image`，`bgr8` 热成像伪彩色画面 |
| `/thermal_camera/temperature` | `sensor_msgs/Image`，`32FC1`，逐像素摄氏度，与画面同尺寸、同时间戳 |
| `/thermal_camera/temperature_stats` | `std_msgs/Float32MultiArray`，依次为中心 6×6 区域温度、最低温度、最高温度（℃） |

不支持测温的设备可能返回 NaN，不能将其作为有效温度。伪彩色亮度也不能代替温度值。
新开终端并加载环境，可查看统计或复用网页预览（默认端口 8765）：

```bash
rostopic echo /thermal_camera/temperature_stats
python3 tools/camera_web_preview.py --camera ros --topic /thermal_camera/image_raw --port 8765
```

浏览器访问 `http://小车IP:8765/`。`Ctrl+C` 停止节点或预览；本功能不控制小车运动、不自动保存图像。

### 楼宇 D 楼层温度检测（初版）

完整巡检到达 `wp_12` 时，通过 `/building_d_temperature/measure` 等待到点后的温度帧，默认 3 秒内取得各楼层已确认的结果。巡检结果记录各层最高温度与异常楼层。仅在存在异常时播报一句，如 `D大楼在 2、4 楼处发现高温情况`。测温无结果超时会记录任务失败并结束本轮。更新代码后需重新编译工作空间以生成测温服务类型。硬件、定位、导航和服务已运行时，可用 `roslaunch community_mission mission.launch single_waypoint:=wp_12` 验证单点任务。

保持红外采集运行，另开终端加载工作空间后启动：

```bash
roslaunch community_perception building_d_temperature.launch
```

若尚未启动红外采集，可用 `start_camera:=true` 一并启动，避免重复打开设备。
检测使用 `/thermal_camera/temperature` 的摄氏度矩阵，将完整画面从左到右四等分，
依次对应 **4、3、2、1 楼**。以每区有效像素的最高温度为准，严格大于 **40℃** 且连续
**3 帧**超温时输出 `楼宇D x楼温度异常`，多层异常分别输出，任一帧不再超温即解除该层异常。
可通过 `threshold:=40.0 confirm_frames:=3` 调整；`confirm_frames:=1` 为单帧判断。
分区边界按 `floor(i × 图像宽度 / 4)` 取整，206 像素宽时为 0、51、103、154、206。

结果发布为 `/building_d_temperature/result`（`std_msgs/String`），终端在状态变化时输出结果，
每两秒输出各层最高温度。确认期间显示“超温确认中”；无有效温度显示“数据无效”，
超过 3 秒未收到可解析的温度帧显示“数据超时”，恢复后重新累计确认帧。
NaN 和无穷值不参与判断。全画面参与检测，背景热源也可能触发对应分区，需将楼宇与分区对齐。
节点仅输出检测结果，不控制运动、不进行语音播报。

```bash
rostopic echo /building_d_temperature/result
```

网页可视化：检测节点同时发布 `/building_d_temperature/image`，按温度矩阵生成完整热图，
叠加楼层分区、最高温度和状态（绿色正常、黄色确认中、红色异常、灰色无有效数据）。
画面放大四倍显示，不改变检测分区或温度。断流超过 3 秒时显示 `DATA TIMEOUT`，避免继续显示旧结果。
保持检测运行，另开终端执行：

```bash
python3 tools/camera_web_preview.py --camera ros \
  --topic /building_d_temperature/image \
  --result-topic /building_d_temperature/result \
  --title '楼宇D楼层温度检测' --port 8766
```

浏览器访问 `http://小车IP:8766/`，画面下方显示中文检测结果。
此热图按当前帧有效温度范围映射颜色，异常判断仍以 40℃ 阈值为准。
原始热成像预览可继续使用 8765 端口。`Ctrl+C` 停止网页服务。

### 单独启动相机

终端 1 启动 ROS Master 并保持运行；已有 `roscore` 或 `roslaunch` 提供 Master 时跳过：

```bash
roscore
```

终端 2 启动相机；已有 `hardware.launch` 或其他相机节点运行时跳过：

```bash
rosrun usb_cam usb_cam_node _video_device:=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Orbbec_DaBai_DCW2_RGB_Camera_CH8X952020W-video-index0 _pixel_format:=mjpeg _image_width:=640 _image_height:=480 _framerate:=30
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

人员、火焰、垃圾桶及障碍物使用 `yolo_train/model/` 导出的 YOLO26n 模型，工作空间文件名分别为 `person.onnx`、`fire.onnx`、`trashbin.onnx`、`obstacle.onnx`。工作空间中的 `trashbin.onnx` 使用重新训练的垃圾桶模型，以 640×640 等比例缩放补边输入，检测四种颜色垃圾桶的开盖和关盖状态，灰色类别使用 `grey_close/grey_open`。检测总入口默认启动障碍物服务；可单独运行 `roslaunch community_perception obstacle_detector.launch`，或用 `--detector obstacle_detector` 进行浏览器预览。完整巡检在 `wp_04` 调用障碍物服务自动选路；仅导航模式由 `obstacle_mode` 手动指定。

`tools/detector_preview.py` 通过 `--detector` 选择 `src/community_perception/config/detectors.yaml` 中的模型，自动读取模型路径、类别名、置信度和 IoU 阈值。脚本独立推理，无需启动巡检或检测服务，支持实车七个 ONNX 检测模型，显示检测框、类别、置信度、全画面目标数量和处理耗时。

保持相机运行，另开终端执行：

```bash
# 人员模型（默认）
python3 tools/detector_preview.py --detector people_detector --port 8080
# 或选择垃圾桶模型
python3 tools/detector_preview.py --detector trash_bin_detector --port 8080
# 覆盖配置阈值及最大推理频率
python3 tools/detector_preview.py --detector trash_bin_detector --conf 0.5 --iou 0.4 --fps 10 --port 8080
```

浏览器访问 `http://小车IP:8080/`，Ctrl+C 退出。切换模型时先停止当前脚本，再使用新的 `--detector` 启动；不同模型同时预览需使用不同端口。

| `--detector` | 检测目标 |
|---|---|
| `people_detector` | 人员；按实车配置显示 `community_person`（社区人员）、`outsider`（外来人员） |
| `traffic_light_detector` | 红/黄灯 `stop`、绿灯 `go` |
| `fire_detector` | 火焰 |
| `obstacle_detector` | 障碍物 |
| `trash_bin_detector` | 垃圾桶颜色及开闭状态 |
| `waste_detector` | 有害、厨余、其他、可回收四类垃圾 |
| `bicycle_detector` | 电动车站立、倒伏 |

自定义配置或单张图片测试：

```bash
python3 tools/detector_preview.py --config /绝对路径/detectors.yaml --detector trash_bin_detector
# 临时覆盖模型文件，类别和阈值仍来自选定配置
python3 tools/detector_preview.py --detector people_detector --model /绝对路径/person.onnx
# 无需 ROS 或相机的图片预览
python3 tools/detector_preview.py --detector people_detector --image ../yolo_train/datasets/person/val/images/0011.jpg
```

配置沿用 `detectors.yaml` 的 `detectors` 结构；`model_path` 支持绝对路径、相对配置文件的路径及 `$(find community_perception)`。`class_names` 必须按模型类别 ID 顺序填写。支持静态 FP32 输入 `[1,3,H,W]`、原始检测输出 `[1,4+类别数,N]`；YOLO26 应以 `end2end=False, nms=False` 导出。

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

网页提供“开始录像 / 停止录像”按钮；终端模式输入 `r` 后回车，桌面预览窗口按 `R`，均可开始或停止录像。录像期间仍可拍照。视频保存到同一数据集的 `videos/` 目录，以时间戳命名，格式为 MJPEG AVI，保留相机实际输出分辨率，不含音频。默认按 10 FPS 采样最新帧，可用 `--record-fps 15` 调整（1～30）；帧率与网页预览独立，源帧率不足时会重复最新帧，写盘性能不足时实际时长可能偏短。

停止录像或按 Ctrl+C / Q 正常退出时会关闭视频文件；相机超过 2 秒无新帧或分辨率变化时自动停止录像并提示，恢复后需手动重新开始。停止后再使用播放器打开视频；关闭浏览器不会停止小车端录像。

## 常用配置与结果

默认定位图 `src/community_bringup/maps/localization_map.yaml` 与规划图 `navigation_map.yaml` 为配套场地先验地图。`route.yaml` 已按旋转后地图下的新采集顺序更新坐标和朝向，原任务与两条分支顺序保留；尚未完成整条路线的实车验证。

| 位置 | 用途 |
|---|---|
| `src/community_bringup/config/hardware.yaml` | 硬件节点参数 |
| `src/community_bringup/config/camera.yaml` | 相机控制参数，启动时通过 `configure_camera:=true` 启用配置 |
| `src/community_bringup/config/navigation/` | EKF、定位、规划和代价地图参数 |
| `src/community_mission/config/route.yaml` | 巡检航点及每个点的任务 |
| `src/community_mission/config/runtime.yaml` | 等待时间、重试、录像和语音参数 |
| `src/community_mission/config/recognition_rules.yaml` | 人员 ROI、车辆去重等识别业务规则 |
| `src/community_perception/config/detectors.yaml` | 检测模型路径、类别、阈值和保存设置 |

默认设备为底盘 `/dev/carserial`、雷达 `/dev/lslidar`、IMU `/dev/imu`。Orbbec 主相机按 `/dev/v4l/by-id/` 绑定。UGREEN 仪表相机默认使用本车的 `/dev/video1`，单独启动 `meter_camera.launch` 时可通过 `video_device` 指定其他设备。组合启动支持通过 `base_port`、`imu_port`、`camera_device`、`map`、`navigation_map`、`route` 和 `data_dir` 覆盖对应参数。

默认 `route.yaml` 包含 21 个原任务点和 11 个 `via_...` 转角过渡点，坐标和朝向来自 2026-11-17 新采集记录。航点按当前导航图的通道中心线校正：左右竖向通道 x 分别为 1.02、3.84 m，中间竖向通道 x 为 2.23 m；下、上横向通道 y 分别为 0.41、3.84 m，中间横向通道 y 为 1.62 m。各点保留原任务与朝向，录像起止点 `wp_19/20` 共用 x=1.02 m、yaw=3.1416 rad，保持横移目标排列。完整巡检到达 `wp_04` 后拍照，调用障碍物模型，按标志检测框中心横坐标选择后续顺序；仅导航模式仍按 `obstacle_mode` 手动选择，不调用模型：

- `blocked`（默认）：01 → 02 → 03 → 04 → 05 → 06 → 07 → 08 → 09 → 10 → 13 → 11 → 12 → 14 → 15 → 16 → 17 → 18 → 19 → 20 → 21。
- `clear`：01 → 02 → 03 → 04 → 13 → 12 → 11 → 08 → 06 → 05 → 07 → 09 → 10 → 14 → 15 → 16 → 17 → 18 → 19 → 20 → 21。

上述编号均为 `wp_XX`，仅列原任务点顺序。实际路线在直角处增加普通导航过渡点：公共前段 `wp_02 → via_02_03(3.84, 0.41) → wp_03`；右侧中部交叉口 `(3.84, 1.62)`、中部交叉口 `(2.23, 1.62)`、右上角 `(3.84, 3.84)`、中上交叉口 `(2.23, 3.84)` 和返回停车位前的 `(1.02, 0.41)` 按分支需要使用。过渡点朝向对齐后续行驶方向，两分支分别执行 29 / 27 个目标；录像 `wp_19 → wp_20` 之间不插点。原任务、任务点坐标及朝向不变，新增转角路线尚未进行实车验证。两条路线共用 YAML 中的航点坐标，分支只引用 ID；从 `wp_14` 起沿公共路线执行，到达 `wp_21` 后正常结束并保存结果。`obstacle_mode` 仅支持 `blocked` 和 `clear`，作为手动参数仅在导航测试模式生效；完整巡检由检测结果决定。

封口参数位于 `route.yaml` 的 `navigation_closures`：`blocked` 封右侧竖道的 y=1.93–1.94 m，`clear` 封横向支路的 x=3.53–3.54 m；封口向禁行支路内侧放置，两端接入静态禁行区，不扩张到相邻转角。每轮开始先恢复底图；仅导航模式在起步前按 `obstacle_mode` 封口，完整巡检在 `wp_04` 识别完成后封口并锁定本轮选择。更新后在停车点等待一个全局代价地图更新周期，再下发目标。`blocked` 的 `wp_10 → wp_13` 改为从中上交叉口、右上转角绕行，原任务点和任务顺序不变。此封口方案尚未进行实车整圈验证。

调试时停止巡检和其他导航目标后，可手动切换封口（服务仅更新地图，不发送运动目标）：

```bash
rosservice call /navigation_map/set_blocked "data: true"   # blocked
rosservice call /navigation_map/set_blocked "data: false"  # clear
rosservice call /navigation_map/reset "{}"                # 恢复底图
```

更换地图或场地后需同步核对 `navigation_closures` 的坐标；自定义路线可不配置该字段，此时巡检不自动设置封口。`navigation.launch` 支持 `route:=...` 指定封口配置来源，应与巡检使用同一份路线文件。

每点只配置一个 `task`：人物任务同时执行所属 A/B 区违停检测；`wp_04` 使用 `task: obstacle` 自动选路；`wp_11` 使用 `task: meter` 读取温湿度表，`wp_12` 使用 `task: temperature` 检查楼宇 D 温度。`wp_19`、`wp_20` 配置录像开始、结束，`wp_21` 为最终停车点。仅导航模式会跳过所有任务；完整任务模式按检测选择的分支执行各航点任务，绿灯等待默认上限 30 秒，需连续 3 帧有效 `go`。

巡检结果默认位于 `~/.ros/community/<run_id>/`，包含结果 JSON、日志、照片和视频；检测服务调试图位于 `~/.ros/community/detections/`，OCR 调试图位于 `~/.ros/community/ocr/`。图片采集目录由 `--datasets-dir` 和数据集名称决定。

### 垃圾桶与垃圾分类

`wp_16` 先用 `trashbin.onnx` 检测整图中的桶，再合并桶框，向左右、上方、下方分别扩展最大桶框宽/高的 `0.5 / 0.1 / 0.5`，裁剪原图生成一个整体 ROI。无桶时不调用垃圾检测。`waste_detector` 使用新训练导出的 `trash_sorting.onnx`，输入为 1024×1024、等比例缩放补边，检测四类垃圾；其他检测器保持原预处理。模型副本位于 `yolo_train/model/trash_sorting.onnx`。

垃圾框还原到原图坐标后，按横向最近的桶匹配；距离不超过该桶宽度的 `0.6`，并要求垃圾中心位于桶高度 45% 以下至桶底下方半个桶高的范围。每个垃圾最多归属一个桶；一个桶只对应一个垃圾时比较类别。红/绿/灰/蓝桶分别对应有害/厨余/其他/可回收。漏检或多个目标对应同一桶时记录 `UNKNOWN`；类别不一致记录 `INCORRECT`；只有四种桶均对应正确且无未匹配目标时才输出全部 `CORRECT`。桶盖开关单独记录。

ROI 与匹配参数位于 `community_mission/config/recognition_rules.yaml`；置信度位于 `community_perception/config/detectors.yaml`，任务模式下桶为 `0.75`、垃圾为 `0.5`。任务正常返回后，原图、`*_roi.jpg`、带框和匹配连线的 `*_trash.jpg` 保存到本次运行的 `photo/`，JSON 记录 ROI、桶/垃圾框、置信度和匹配结果。橙色表示无法确认，绿色表示对应正确，红色表示投放错误；不触发语音。

更新后重新编译工作空间并重启识别服务。保持硬件、定位、导航和服务运行后，可使用 `roslaunch community_mission mission.launch single_waypoint:=wp_16 navigation_only:=false` 验证单点完整流程。`tools/detector_preview.py --detector waste_detector` 只预览垃圾模型，不执行桶检测与 ROI 匹配。

停车查看完整垃圾识别流程时，在 `robot_ws` 目录运行以下工具。它直接加载本地两个 ONNX 模型，复用巡检的 ROI 与匹配规则，无需启动巡检、导航或检测服务：

```bash
# 相机实时测试：先启动 ROS 相机并加载 ROS 环境
python3 tools/detector_preview.py --trash --topic /usb_cam/image_raw --fps 2 --port 8080
# 单张图片测试：不需要 ROS
python3 tools/detector_preview.py --trash --image ../yolo_train/datasets/trash_sorting/val/images/0268.jpg --port 8080
```

浏览器访问 `http://小车IP:8080/`。黄色框为整体 ROI，绿/红/橙框分别表示正确/错误/无法确认，连线表示对应关系；网页显示中文判断，`/result.json` 提供当前结构化结果。默认阈值与巡检一致，`--conf` 可覆盖垃圾模型阈值，桶阈值读取 `mission_confidence.trash_bin_detector`。Ctrl+C 退出，工具不控制小车运动、不播报、不持续保存图片。

## 实车验收

识别效果、定位导航和完整巡检流程需要在目标硬件与现场验证。详细说明见 [架构与接口](docs/ARCHITECTURE.md)、[实车验收](docs/ACCEPTANCE.md) 和 [底盘协议说明](docs/BASE_PROTOCOL_REVIEW.md)。

实车巡检流程当前不触发任何语音播报，也不在结束时等待播报队列；语音服务、队列和播报函数保留，供后续接入。

红绿灯航点使用 `community_perception/models/traffic_light.onnx` 两分类模型（0=`stop`、1=`go`）。默认连续 3 帧有效且仅含 `go` 的检测结果才放行，继续下一航点导航；`stop`、空结果、混合结果或无效检测打断连续计数，重复时间戳同样重置计数。连续确认帧数仍由 `green_confirm_frames` 配置，默认 30 秒超时后终止任务。

障碍物选路规则：检测框中心横坐标除以图像宽度，小于 `mission.obstacle_clear_x_ratio`（默认 `0.35`）选 `clear`，否则选 `blocked`。该阈值依据固定视角下偏左与居中两个摆放位置设置，需在 wp_04 实车视角复核。无有效障碍物、检测到多个障碍物、无效框或服务异常时终止任务；照片、检测框、置信度、坐标比例和所选分支记录到本次运行结果。
