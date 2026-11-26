# 智慧社区实车 ROS 工作空间

现场命令见 [单点导航、单任务与裁判系统调试](docs/单点导航与裁判系统调试.md)，按 Windows 本机与 SSH 小车终端区分。

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
roslaunch community_bringup navigation.launch map:=$HOME/maps/community_test.yaml
```

本地 RViz 启动方式见下一节。RViz 的 Fixed Frame 设为 `map`，通过 `2D Pose Estimate` 初始化定位，再使用 `2D Nav Goal` 测试导航。建图与导航不同时运行。

### 局部规划器（默认 DWA）

DWA 使用实车全向导航配置。move_base 与 DWA 控制频率均为 10 Hz；前进/后退/横移速度上限为 0.50/0.12/0.08 m/s，普通导航保留低速横移修正，完整巡检的 `normal_lateral_speed` 同为 0.08 m/s，录像段 `recording_lateral_speed` 保持 0.15 m/s；合成平移速度上限为 0.50 m/s，角速度下限和上限为 0.523599/0.872665 rad/s（30/50°/s），平移加速度为 0.6 m/s²、角加速度为 2.0 rad/s²。到点容差为 0.10 m / 0.087266 rad（5°）。参数位于 `src/community_bringup/config/navigation/dwa_local_planner_params.yaml`。保留当前全局规划、代价地图、车体轮廓、激光避障和巡检路线执行方案。该组合尚未在本车验证。

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
# 使用新地图时指定 map:=...
# 恢复 DWA：停止当前导航，再使用默认命令或 local_planner:=dwa 启动
```

在另一终端加载 ROS 环境后确认：

```bash
rosparam get /move_base/base_local_planner
# 默认应为 dwa_local_planner/DWAPlannerROS；选择 TEB 时为 teb_local_planner/TebLocalPlannerROS
rostopic info /move_base/local_plan
```

TEB 参数位于 `src/community_bringup/config/navigation/teb_local_planner_params.yaml`。当前为全向提速试验配置：前进/倒车/横移上限为 0.70/0.25/0.70 m/s，最大转速为 1.2 rad/s（约 68.8°/s），允许接近目标时自然减速；前后/横向加速度为 1.0 m/s²，角加速度为 3.0 rad/s²，到点容差为 10 cm / 5°。`weight_kinematics_nh` 和 `weight_kinematics_forward_drive` 均为 0，不添加横移惩罚或前进方向偏好。当前安装的 TEB 不支持 `max_vel_trans`，前进和横移同时达到上限时合成速度约 0.99 m/s。保留车体轮廓和到点停车配置；全局/局部代价地图膨胀半径均为 0.18 m、衰减系数为 10.0，TEB 障碍间距为 0.10 m、障碍膨胀距离为 0.18 m；这组值不是已测得的底盘极限，尚需实车验证。

move_base 关闭仅按平移进展判断的振荡超时（`oscillation_timeout: 0`），避免同位置转向任务被误判；`oscillation_distance: 0.05` 在此超时关闭时不触发振荡恢复。关闭默认恢复行为中的自动旋转（`clearing_rotation_allowed: false`），保留清图恢复和正常导航转向。无有效控制等待上限仍为 3 秒，全局规划等待上限为 5 秒，巡检单目标超时仍为 120 秒。修改恢复行为后需重启导航生效。

两种规划器的局部路径统一发布到 `/move_base/local_plan`，供 RViz 显示。已在实车确认 TEB 插件加载并运行导航，定位、绕障及通行稳定性仍需现场验证。

### 单地图导航

AMCL、全局和局部代价地图统一使用 `/map`，由一个 `map_server` 发布。默认加载 `src/community_bringup/maps/localization_map.yaml`，分辨率 0.01 m，原点为 `(-0.03, -0.03, 0)`。全局和局部代价地图保留静态层、激光障碍层与膨胀层，实时激光避障继续生效。

默认地图为规则正方形围栏，内侧边界为 `x/y = 0.00～4.20 m`，内部净尺寸 `4.20 × 4.20 m`；围栏向外厚 0.02 m，外围留白 0.01 m。地图为 `426 × 426` 格，画布范围为 `x/y = -0.03～4.23 m`。现有航点坐标保持不变，需在现场核对定位与路线。

AMCL 默认初始位姿为 `(0.411719, 0.409062, 0)`，其他摆放位置通过 RViz 的 `2D Pose Estimate` 设置实际位姿，确认激光与地图对齐后再运行。

停止已有建图或导航，保留硬件，在已加载 ROS 环境的终端启动：

```bash
roslaunch community_bringup navigation.launch local_planner:=teb
# 使用 DWA：local_planner:=dwa；不指定时默认 DWA。
# 使用其他地图：
roslaunch community_bringup navigation.launch map:=$HOME/maps/community_test.yaml
```

组合启动同样仅使用 `map` 指定地图。RViz 地图话题选择 `/map`。分支选择只切换航点顺序；激光和所加载地图中的障碍决定实际通行范围。默认场地图主要包含外围围栏，使用前核对现场障碍和地图一致性。单地图配置尚未完成实车验证，重新采集的航点需在对应地图下验收。

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

当前阶段使用主路线的 clear/blocked 两个分支验收。启动硬件、定位和导航后运行，无需启动识别、语音和录像服务；所有航点只导航，跳过全部任务：

```bash
roslaunch community_mission mission.launch navigation_only:=true obstacle_mode:=clear
```

常规巡检与仅导航模式中的所有执行点（包括纯导航点和转角过渡点）均等待 `move_base` 返回到点成功，再处理任务并切换到下一点。

### 仅任务实测（不导航）

`task_only:=true` 跳过导航，直接使用当前相机画面执行真实任务；不创建 `move_base` 客户端、不等待导航服务、不发送或取消导航目标，也不修改规划器速度。与 `navigation_only:=true` 互斥。相机和该任务依赖的识别、语音、录像等服务仍须正常运行。

各命令分别在已加载 ROS 和本工作空间环境的终端执行：

```bash
# 终端 1：仅启动相机和任务服务，无需底盘、定位或导航
# 已有相机和服务运行时不要重复启动。
roslaunch community_bringup debug_perception.launch
# 终端 2：先将测试目标放入相机视野，再运行一个航点任务
roslaunch community_mission mission.launch task_only:=true single_waypoint:=wp_01
# 单点人员识别 / 火情 / 车牌 / 垃圾分类（每次结束后再执行下一条）
roslaunch community_mission mission.launch task_only:=true single_waypoint:=wp_06
roslaunch community_mission mission.launch task_only:=true single_waypoint:=wp_19
roslaunch community_mission mission.launch task_only:=true single_waypoint:=wp_27
roslaunch community_mission mission.launch task_only:=true single_waypoint:=wp_29
# 查看状态（另开终端）；完成后状态应为 FINISH，失败为 ERROR
rostopic echo /mission/state
```

不指定 `single_waypoint` 时，按完整路线连续执行任务，`wp_05` 的真实障碍识别仍决定 clear/blocked 分支，纯导航点直接跳过。每个任务之间不等待人工确认，建议先逐点实测。红绿灯仍要求连续有效绿灯帧，超时将终止；录像开始/停止仍须配对，单独选择录像航点会被拒绝。完整路线的录像段不再有行驶耗时，可能很短；录像实测需准备包含合适任务时长的配对路线。

工作台「巡检任务」可勾选「仅任务（不导航）」并选择「单航点」，无需启动底盘和导航。`/mission/state` 和每次运行的 `mission_results.json` 均记录 `task_only`；跳过的导航不会写为导航成功，任务结果及照片、视频仍写入 `~/.ros/community/<run_id>/`。纯导航点的航点成功仅表示已跳过该点，不能作为到点证据。此模式不会停止其他节点已发出的运动命令。

### 实测终端与输出

| 终端 | 启动命令 | 输出用途 |
| --- | --- | --- |
| 1：相机与服务 | `roslaunch community_bringup debug_perception.launch` | 模型加载、识别服务、语音、录像及硬件诊断；不作为本轮巡检的最终结论 |
| 2：主状态机 | `roslaunch community_mission mission.launch task_only:=true single_waypoint:=wp_06` | 本轮开始、任务准备/开始、结果、证据路径、航点完成与最终汇总；主要看此终端 |
| 3：状态观察 | `rostopic echo /mission/state` | JSON 格式状态消息，包含 `state`、`current_waypoint`、`task_only`、`last_operation`、`run_id`；不显示完整识别结果 |

先在终端 1 等待服务就绪，再提前运行终端 3 的观察命令，最后在终端 2 启动任务。状态机退出后，发布节点的 latch 不会继续保留，事后查询以结果文件为准。使用工作台启动时，同样的主状态机输出在 `mission` 的启动日志中查看。

终端 2 的人员单点测试示例（下面的数量、耗时和 run_id 仅为示例，实际以识别结果为准；省略 ROS 时间与级别前缀）：

```text
[巡检开始] 模式=仅任务 范围=单航点=wp_06 run_id=<run_id>
[输出目录] /home/<user>/.ros/community/<run_id>
[导航跳过] 航点=wp_06 原因=仅任务模式
[任务准备] 航点=wp_06 类型=person 等待新的相机图像
[任务开始] 航点=wp_06 类型=person
[任务结果] 航点=wp_06 类型=person 状态=SUCCESS 耗时=0.20s 结果=区域=A 人数=3 外来人员=1 违停=2 说明=识别完成
[任务证据] 航点=wp_06 文件=/home/<user>/.ros/community/<run_id>/photo/005.jpg
[航点完成] 航点=wp_06 类型=person
[巡检结束] 状态=FINISH 模式=仅任务 完成航点=1 任务成功=1（其中空结果=0） 任务失败=0 耗时=0.3s
[结果文件] /home/<user>/.ros/community/<run_id>/mission_results.json
[事件日志] /home/<user>/.ros/community/<run_id>/mission.log
```

服务识别成功但没有目标时，任务状态为 `EMPTY_VALID`，说明为“未检测到有效目标”，归入任务成功并单列空结果数量。服务异常时为 `FAILED`，主状态机进入 `ERROR`，打印 `[结束原因]`。若在取图等准备阶段就失败，可能没有任务结果记录，应同时查看 `[状态] ... ERROR` 和结束原因。`[任务证据]` 只列出当时实际存在的文件，缺失路径列为 `[证据未生成]`；完整检测数据仍保存在结果 JSON 中。

人员、车牌、火情、障碍物和车辆拍照任务在本轮 `photo/` 下同时保存原图与 `<序号>_<任务类型>_annotated.jpg` 标注图，例如 `005.jpg` 和 `005_person_annotated.jpg`。图中包含航点、状态、结果摘要；检测任务叠加原始检测框、类别和置信度，人员黄框是人员参考 ROI，不能将所有车辆框视为违停。车牌图显示中文车牌文字及 OCR 置信度；当前 OCR 服务不返回框坐标，因此不绘制车牌定位框。标注直接复用本次推理结果，不重复识别；完整框数据写入任务结果的 `detections`。

有效空结果也会保存标注图。取得原图后发生识别异常时，尽可能保存失败说明图；若相机取图失败则无法生成图片。原图及已生成标注图的路径统一写入该任务的 `operations[].paths`，工作台结果页可直接浏览。字体或磁盘写入失败会打印 `[标注保存失败]` 并记录 `evidence_errors`，不改变识别结果或障碍分支；此时只有原图，不能认为标注证据已齐全。

标注使用 Pillow 和系统中文字体，优先自动查找 Noto CJK、文泉驿等字体；找不到时可通过 `roslaunch community_mission mission.launch task_only:=true single_waypoint:=wp_27 evidence_font:=/实际路径/中文字体.ttc` 指定，工作台启动配置也支持 `evidence_font`。新增辅助模块后需在机器人上重新运行 `catkin_make` 并加载 `devel/setup.bash`。

终端 3 的正常最终状态为 `FINISH`，失败为 `ERROR`；没有目标并不意味着状态机失败。单点或完整路线执行结束后，终端 2 的 `mission.launch` 正常退出属于预期，终端 1 的独立服务继续运行。结果保存异常会输出 `[结果写入失败]`，此时结果文件可能不完整，不能只看 `FINISH`。

### 单航点运行

常规单点模式先启动硬件、定位和导航，确认当前定位；通过 `single_waypoint` 指定默认 `route.yaml` 中的航点 ID，直接导航到该点，坐标、朝向和任务沿用路线配置：

```bash
# 只导航到 wp_05，到达后结束；无需识别服务
roslaunch community_mission mission.launch single_waypoint:=wp_05 navigation_only:=true
# 导航到 wp_06，执行人员及违停识别后结束；须先启动相机和相关服务
roslaunch community_mission mission.launch single_waypoint:=wp_06 navigation_only:=false
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

`hardware.launch` 同时启动两台可见光相机。Orbbec 彩色相机以 1920×1080 MJPEG 采集，发布 `/usb_cam/image_raw`，用于常规识别和录像。UGREEN 按设备 ID 以 1920×1080 MJPEG 采集，发布 `/ugreen_4k/image_raw`，工作台相机页选择该 ROS 来源即可预览。同一设备应由一个采集节点持有，部署后启动正式节点前需退出此前手动启动的 UGREEN 预览进程。

配置 `task: meter` 的航点到点后从 UGREEN 获取新帧，使用随工作空间编译安装的 4K 标定读取仪表温度与湿度。结果写入 `mission_results.json`，原图和指针叠加图保存在本轮 `photo/` 下。读取超时会按航点任务失败收尾。单独启动 `mission.launch` 时，可通过 `meter_image_topic` 和 `meter_profile` 指定图像话题和标定文件。

在 Web 工作台的相机页选择 **仪表盘调试（实时过程图）**，自动切到 UGREEN 1080p 视频源。四格画面依次显示原图与指针、原图灰度、模板与当前帧的匹配点、透视校正后的读针灰度图，右侧显示读数和匹配内点数。

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

配置 `task: temperature` 的航点通过 `/building_d_temperature/measure` 等待到点后的温度帧，默认 3 秒内取得各楼层已确认的结果。巡检结果记录各层最高温度与异常楼层，仅在存在异常时播报。测温无结果超时会记录任务失败并结束本轮。当前默认新路线未配置测温任务；需先为目标航点设置该任务，再进行单点验收。更新代码后需重新编译工作空间以生成测温服务类型。

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
rosrun usb_cam usb_cam_node _video_device:=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Orbbec_DaBai_DCW2_RGB_Camera_CH8X952020W-video-index0 _pixel_format:=mjpeg _image_width:=1920 _image_height:=1080 _framerate:=30
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

人员、火焰、垃圾桶及障碍物使用 `yolo_train/model/` 导出的 YOLO26n 模型，工作空间文件名分别为 `person.onnx`、`fire.onnx`、`trashbin.onnx`、`obstacle.onnx`。工作空间中的 `trashbin.onnx` 使用重新训练的垃圾桶模型，以 1280×1280 等比例缩放补边输入，检测四种颜色垃圾桶的开盖和关盖状态，灰色类别使用 `grey_close/grey_open`。检测总入口默认启动障碍物服务；可单独运行 `roslaunch community_perception obstacle_detector.launch`，或用 `--detector obstacle_detector` 进行浏览器预览。默认主路线在 `wp_05` 调用障碍物服务自动选择后续 clear/blocked 分支，独立路线文件用于单独调试。

`fire.onnx` 已更新为 2026-11-25 光照增强训练的最佳模型（训练集 4737 张、验证集 79 张，第 80 轮最佳，第 130 轮早停），使用静态 FP32 640×640 输入及外部 NMS，与 `yolo_train/model/fire.onnx` 一致。火焰检测启用 `letterbox: true`，通过等比例缩放补边保持与训练一致的预处理，避免直接拉伸造成漏检。替换后需重启正在运行的火焰检测服务或浏览器预览以重新加载模型。

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
| `waste_detector` | 24 类具体垃圾物品 |
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

默认使用 `src/community_bringup/maps/localization_map.yaml` 作为定位与规划共用地图。默认主路线为 `route.yaml`，clear/blocked 航点基于 2026-11-25 采集结果，按 4.2 m 场地图及 0.60 m 主道路宽度调整至道路中央；`route_clear.yaml`、`route_blocked.yaml` 的对应坐标同步更新，任务、朝向与执行顺序不变。道路中心参考线为竖路 `x=1.00/2.20/3.80 m`、横路 `y=0.40/1.60/3.80 m`，各线仅对应实际道路所在区段。截图比例配准尚需现场校准，整条路线尚未完成实车验证。

| 位置 | 用途 |
|---|---|
| `src/community_bringup/config/hardware.yaml` | 硬件节点参数 |
| `src/community_bringup/config/camera.yaml` | 相机控制参数，启动时通过 `configure_camera:=true` 启用配置 |
| `src/community_bringup/config/navigation/` | EKF、定位、规划和代价地图参数 |
| `src/community_mission/config/route.yaml` | 公共前段、分支与公共收尾段 |
| `src/community_mission/config/route_clear.yaml` / `route_blocked.yaml` | 独立路线调试文件 |
| `src/community_mission/config/runtime.yaml` | 等待时间、重试、录像和语音参数 |
| `src/community_mission/config/recognition_rules.yaml` | 人员 ROI、车辆去重等识别业务规则 |
| `src/community_perception/config/detectors.yaml` | 检测模型路径、类别、阈值和保存设置 |

默认设备为底盘 `/dev/carserial`、雷达 `/dev/lslidar`、IMU `/dev/imu`。Orbbec 主相机和 UGREEN 仪表相机按 `/dev/v4l/by-id/` 绑定。单独启动 `meter_camera.launch` 时可通过 `video_device` 指定其他设备。组合启动支持通过 `base_port`、`imu_port`、`camera_device`、`map`、`route` 和 `data_dir` 覆盖对应参数。

独立 `route_clear.yaml` 按 `wp_01` 至 `wp_39` 顺序执行，任务点及最终停车位保留实测坐标和朝向；普通导航点经过直线投影和转角对齐，单点位置调整不超过 8.5 cm，不额外插入中间点。相邻转向点共用位置，车牌 3 后的回退点已对齐到任务位置。每点仍等待导航成功后再继续，最终停车点为 `wp_39`。任务对应如下：

| 航点 | 任务 |
|---|---|
| `wp_01`、`wp_24` | 红绿灯 |
| `wp_02`、`wp_06`、`wp_18` | A 区人员及违停 |
| `wp_07`、`wp_14`、`wp_17` | B 区人员及违停 |
| `wp_19`、`wp_22`、`wp_34` | 火灾 |
| `wp_27`、`wp_28`、`wp_31` | 车位 1、2、3 车牌 |
| `wp_29` | 垃圾分类 |
| `wp_35`、`wp_36` | 录像开始、结束，朝西横移 |
| 其余点 | 普通导航 |

主状态机默认使用 `route.yaml`：公共前段采用 clear 的 `wp_01`～`wp_05`，包含首个红绿灯和 A 区人员任务，`wp_05` 的任务恢复为障碍识别。完整巡检在该点检测障碍物，中心横坐标比例小于 0.35 选择 clear，否则选择 blocked；识别异常会终止任务。进入分支后不重复公共前段，plate1 起引用同一组 `wp_27`～`wp_39` 公共收尾点。

clear 执行 39 个目标，blocked 执行 38 个目标。blocked 的分支任务及取景朝向沿用独立 blocked 文件，允许两分支任务不同；公共前段统一执行首个红绿灯。仅导航模式跳过全部任务，在 `wp_05` 后按 `obstacle_mode` 手动选择，不调用障碍物模型：

```bash
roslaunch community_mission mission.launch navigation_only:=true obstacle_mode:=clear
roslaunch community_mission mission.launch navigation_only:=true obstacle_mode:=blocked
# 完整巡检：先启动硬件、导航和 services，确认定位后运行
roslaunch community_mission mission.launch
```

不指定 `obstacle_mode` 时，仅导航默认 clear。完整巡检忽略手动选路参数，由检测结果决定。独立文件仍可通过 `route:=.../route_clear.yaml` 或 `route:=.../route_blocked.yaml` 显式加载，此时不自动选路。

当前默认路线未启用仪表和楼宇测温任务。红绿灯等待默认上限 30 秒，需连续 3 帧有效 `go`。

### blocked 路线

`src/community_mission/config/route_blocked.yaml` 使用 2026-11-25 03:41 采集的绕行前段，共 38 点。前 25 点使用 `blocked_wp_01`～`blocked_wp_25` 编号，任务以采集文件为准，任务点坐标和朝向不变，普通导航点做转角对齐和直线投影，最大位置调整不超过 10 cm。plate1 起使用 clear 中的 `wp_27`～`wp_39` 公共收尾段，点位、任务、朝向和顺序完全一致。

前段人员任务为 `blocked_wp_02/06/08/09/14/21`，分别对应 A/A/B/A/B/B；`blocked_wp_10` 为火灾。采集文件中的普通导航点保持原任务，包括 `blocked_wp_01/13/15`，该独立 blocked 路线不包含红绿灯等待，也不调用障碍选路。任务点朝向保留新采集值，不按旧相机朝向修正；当前仍使用现有图像来源，未增加按航点切换相机的配置。

默认主路线已接入 blocked 分支。若需跳过公共选路逻辑单独验收完整 blocked 文件，停止其他巡检和键盘控制、确认定位后运行：

```bash
roslaunch community_mission mission.launch \
  route:="$(rospack find community_mission)/config/route_blocked.yaml" \
  navigation_only:=true
```

路线结构检查通过，实际取景、通行和整圈运行仍需实车验证。

巡检结果默认位于 `~/.ros/community/<run_id>/`，包含结果 JSON、日志、照片和视频；检测服务调试图位于 `~/.ros/community/detections/`，OCR 调试图位于 `~/.ros/community/ocr/`。图片采集目录由 `--datasets-dir` 和数据集名称决定。

### 垃圾桶与垃圾分类

`wp_29` 先用 `trashbin.onnx` 检测整图中的桶，再合并桶框，向左右、上方、下方分别扩展最大桶框宽/高的 `0.5 / 0.1 / 0.5`，裁剪原图生成一个整体 ROI。无桶时不调用垃圾检测。`waste_detector` 使用 `trash_sorting_aug_autodl.pt` 导出的 `trash_sorting.onnx`，以 1920×1920 等比例缩放补边输入识别 24 类物品。投放判断通过 `waste_types` 将具体物品归入有害、厨余、其他和可回收四大类。

垃圾框还原到原图坐标后，按横向最近的桶匹配；距离不超过该桶宽度的 `0.6`，并要求垃圾中心位于桶高度 45% 以下至桶底下方半个桶高的范围。每个垃圾最多归属一个桶；一个桶只对应一个垃圾时比较类别。红/绿/灰/蓝桶分别对应有害/厨余/其他/可回收。漏检或多个目标对应同一桶时记录 `UNKNOWN`；类别不一致记录 `INCORRECT`；只有四种桶均对应正确且无未匹配目标时才输出全部 `CORRECT`。桶盖开关单独记录。

ROI 与匹配参数位于 `community_mission/config/recognition_rules.yaml`；置信度位于 `community_perception/config/detectors.yaml`。单独启动与主任务启动的默认阈值统一为：人员 `0.5`、垃圾桶 `0.7`、红绿灯 `0.75`；垃圾类别仍为 `0.5`。显式启动参数可覆盖默认值，修改配置后须重启识别服务。任务正常返回后，原图、`*_roi.jpg`、带框和匹配连线的 `*_trash.jpg` 保存到本次运行的 `photo/`，JSON 记录 ROI、桶/垃圾框、置信度和匹配结果。橙色表示无法确认，绿色表示对应正确，红色表示投放错误；不触发语音。

更新后重新编译工作空间并重启识别服务。保持硬件、定位、导航和服务运行后，可使用 `roslaunch community_mission mission.launch single_waypoint:=wp_29 navigation_only:=false` 验证单点完整流程。`tools/detector_preview.py --detector waste_detector` 只预览垃圾模型，不执行桶检测与 ROI 匹配。

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

### 电表读数验收

配置 `task: meter` 的仪表任务从 `/ugreen_4k/image_raw` 获取到点后的新帧，同时读取温度、湿度和电量。电表通过 `/electricity_meter_ocr` 复用 PaddleOCR，在整帧中定位 `kWh`，读取其上方相邻数字行，连续两帧读数一致后确认。UGREEN 画面需同时包含温湿度表、电表数字和单位。

任务默认等待 60 秒，可通过 `mission.launch meter_timeout:=90` 调整。三项全部有效才完成任务，超时记录失败及现场图片。结果中的 `electricity_kwh` 为保留小数点及前导零的字符串，`electricity_unit` 为 `kWh`，并记录 `electricity_confidence`。原图和叠加图与温湿度读数一起保存。

现场参考图片已通过 OCR 读取 `123.4 kWh`，实际停车视角和不同读数仍需现场验收。


楼宇火灾预览与火焰航点共用 ROS 服务 `/building_name_ocr_preview` 及任务模块中的楼名提取规则。识别到火焰后才请求同一原始图像的楼名；楼名缺失或 OCR 失败时仍记录火情，使用默认楼名“大厦”。航点仅依赖 ROS 识别服务，可在工作台关闭时独立运行。

## 裁判系统通信

主状态机可将已保存的任务结果通过后台队列发送给选手笔记本上的裁判系统。启用示例（小车终端）：

```bash
roslaunch community_mission mission.launch judge_enabled:=true \
  judge_url:=http://192.168.31.83:8080 judge_team:=麦当劳薯条三重奏
```

`community_bringup` 的 `community.launch`、`field.launch`、`finals.launch` 同样接受这三个参数，可追加到原有启动命令。默认 `judge_enabled:=false`；只导航模式不连接裁判系统。地址需填写笔记本的局域网 IP，当前为 `192.168.31.83:8080`，换网络后应重新确认；笔记本需放行 TCP 8080。

启用后自动调用 `/api/hello`；车牌、人群、火情、垃圾桶、仪表、温度和电动车任务完成后调用 `/api/report`。优先发送已保存的标注图片；人群按 A/B 街区累积，电动车在视频统计完成后汇总街区违停与正常/倒伏数量，并附之前保存的电动车图片。垃圾桶按当前画面从左到右编号。指示牌仅在主状态机实际产生 announcement 结果时上报，不会新增识别或恢复已暂停的播报任务。红绿灯、导航和避障分支结果不上传，失败/跳过任务不作为成功结果上传。

网络请求使用单独线程，超时为 3 秒。上报失败不会改变任务结果或中断巡检；上报结果及失败原因保存在本轮输出目录的 `judge_reports.jsonl`，原始任务结果仍在 `mission_results.json`。不自动重试，避免请求已被接收时重复提交；结束时最多等待通信线程 3 秒，未完成项目应结合任务结果文件检查。
