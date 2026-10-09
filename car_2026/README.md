# car_2026 ROS 工作空间

本工作空间使用 ROS Noetic / catkin，容器路径为 `/workspace/car_2026`。比赛按“仿真、导航、视觉、任务控制器”分终端启动，任务入口为 `robot_competition/launch/competition.launch`。

## 功能包

| 功能包 | 职责 |
|---|---|
| `robot_description` | URDF/Xacro、网格、传感器、物理参数 |
| `robot_gazebo` | 比赛 world、人物/车辆/灯具素材、交通灯仿真插件 |
| `robot_mecanum_control` | 麦轮控制、轮式里程计、平面运动插件、EKF、键盘调试 |
| `robot_navigation` | Cartographer、AMCL、双地图、move_base / TEB |
| `robot_perception` | 人物、车牌、交通灯分类节点，JSON 与标注图输出 |
| `robot_competition` | 航点录入与校验、任务状态机、任务结果及证据保存 |

`build/`、`devel/`、`install/` 为生成目录。正式默认路线为 `src/robot_competition/config/waypoints.yaml`。

## 环境与编译

宿主机只用于进入容器；每个新终端执行：

```bash
docker exec -it --user developer smart-community-dev bash
```

以下所有命令均在容器内执行。容器为 Ubuntu 20.04 / ROS Noetic / Gazebo 11；创建容器及 X11 配置见 [Docker 说明](../docker/README.md)。每个 ROS 终端加载：

```bash
cd /workspace/car_2026
source /opt/ros/noetic/setup.bash
source /opt/cartographer_ws/install_isolated/setup.bash
source devel/setup.bash --extend
```

首次或修改源码后的编译（首次编译前无需加载 `devel`）：

```bash
cd /workspace/car_2026
source /opt/ros/noetic/setup.bash
source /opt/cartographer_ws/install_isolated/setup.bash
catkin_make -j2
source devel/setup.bash --extend
```

Cartographer 单独安装在 `/opt/cartographer_ws/install_isolated`；`--extend` 保留该环境，避免主工作空间覆盖其包搜索路径。catkin 和任务主程序使用系统 Python 3.8；视觉节点使用 `/home/developer/.venvs/robot-perception/bin/python`（Python 3.10）。不要激活视觉 venv 后编译 ROS。模型和缓存部署细节见 [视觉包 README](src/robot_perception/README.md)。

## 比赛分终端启动

### 终端 1：Gazebo

```bash
roslaunch robot_gazebo simulation.launch rviz:=true
```

该入口已经包含机器人、麦轮控制与 EKF，不必重复启动。默认 `rviz:=false`；图形显示需 X11。已有 Gazebo 时复用现有实例。本命令不重生成车牌纹理；`start_simulation.sh` 是会替换随机车牌并终止旧 Gazebo 进程的辅助工具，不作为日常默认入口。

### 终端 2：导航

```bash
roslaunch robot_navigation navigation.launch
```

该入口包含 AMCL、定位地图 `/map`、规划地图 `/planning_map` 和 `move_base`，无需额外启动 `localization.launch`。在 RViz 使用 **2D Pose Estimate** 设置初始位姿并确认激光与地图对齐，再启动比赛。默认 TEB 配置禁止横移；底层麦轮支持横移。

### 终端 3：视觉

首次使用或容器重建后，先准备并检查当前用户的 HyperLPR 缓存：

```bash
"$HOME/.venvs/robot-perception/bin/python" src/robot_perception/scripts/prepare_hyperlpr_models.py \
  --source /workspace/car_2026/src/robot_perception/models/hyperlpr3
"$HOME/.venvs/robot-perception/bin/python" src/robot_perception/scripts/prepare_hyperlpr_models.py \
  --source /workspace/car_2026/src/robot_perception/models/hyperlpr3 --verify-only
```

确认包内 `models/person_best.pt`、HyperLPR ONNX 以及 `models/traffic_light_best.pt` 已部署。交通灯权重的部署来源为仓库 `models/traffic_light/best.pt`；默认容器只挂载工作空间，准备方法见 [视觉包说明](src/robot_perception/README.md)。不重新训练或覆盖已有权重。

```bash
roslaunch robot_perception perception.launch \
  python:="$HOME/.venvs/robot-perception/bin/python" \
  enable_traffic_light:=true show_images:=false
```

必须显式启用交通灯，`perception.launch` 默认仅启用人物和车牌。仅调试单路时可选以下一个入口，不与同名节点重复启动：

```bash
roslaunch robot_perception person_detection.launch
roslaunch robot_perception plate_recognition.launch
roslaunch robot_perception traffic_light_classification.launch
```

### 终端 4：比赛主程序

确认定位、`/move_base` 与三路视觉输出正常后：

```bash
roslaunch robot_competition competition.launch
```

默认读取比赛包内的 `config/waypoints.yaml` 与 `config/task_config.yaml`，导航和视觉节点需独立启动。默认 `navigation_test_mode=false`。更换路线时通过 `waypoints_file` 指向自备文件，先校验并确认坐标适合当前地图。`navigation_test_mode:=true` 会跳过视觉与红绿灯控制，不可用于正式比赛。

交通灯点必须有 `stop_before_line: true`，到点后进入 `WAIT_TRAFFIC`；默认连续 3 帧有效 GREEN、置信度 ≥ 0.8 才放行，等待上限 35 秒。人物/车牌观察与融合参数独立配置。车牌中文证据需要可用字体，并在实际确认标注完整后设置 `plate_text_annotation_confirmed`；默认 false 时有识别号码也可能记为 `FAILED_IMAGE_ANNOTATION`，详见 [P2-B](src/robot_competition/P2B.md)。

## 建图（与导航模式分开）

保留终端 1 的仿真，停止 AMCL/导航及比赛任务后，在另一终端启动：

```bash
roslaunch robot_navigation mapping.launch
```

Cartographer 使用 `/scan`、`/odometry/filtered` 与 `/imu/data`，自带建图 RViz。不要同时运行 AMCL 与 Cartographer 来发布同一条定位 TF。手动探索可单独运行已有工具：

```bash
python3 src/robot_mecanum_control/scripts/keyboard_teleop.py
```

保存当前栅格地图到本地结果目录，不覆盖包内地图：

```bash
mkdir -p /workspace/car_2026/results/maps
rosrun map_server map_saver -f /workspace/car_2026/results/maps/session map:=/map
```

导航使用包内 `localization_map` 和 `planning_map`；重新建图后需配置匹配的规划地图。上述命令仅保存栅格地图，不保存 Cartographer 状态。

## 调试与结果

```bash
rosnode list
rostopic hz /scan
rostopic hz /camera/color/image_raw
rostopic echo -n 1 /odometry/filtered
rosrun tf tf_echo map base_footprint
rostopic echo -n 1 /move_base/status
rostopic echo -n 1 /perception/person_detections_json
rostopic echo -n 1 /perception/plates_json
rostopic echo -n 1 /perception/traffic_light_json
rosrun image_view image_view image:=/perception/person_image
rosparam get /main_controller/task_config
rosrun robot_competition waypoint_manager.py src/robot_competition/config/waypoints.yaml
rosrun robot_competition waypoint_recorder.py --help
```

航点录入方法见 [比赛包 README](src/robot_competition/README.md)。采集工具 `scripts/camera_capture.py` 位于比赛包内，可在容器中直接用系统 Python 运行；默认数据写入 `/workspace/car_2026/datasets/traffic_light`，不参与比赛主程序。

任务结果默认保存为 `/workspace/car_2026/results/<run_id>/`：`mission_results.json`、`mission.log` 和 `images/`。红绿灯任务包含识别结果、等待时间、连续绿灯数及 `released`。ROS 日志默认在当前用户 `~/.ros/log/`。随机车牌生成记录在 `src/robot_gazebo/results/generated_plates.json`，与生成的场景纹理对应。

## 文件维护

配置、模型、地图及训练资源作为部署输入维护。缓存、编译产物、日志、航点 `.yaml.lock` 和运行结果不提交；锁文件会自动生成，不要在录点时删除。删除 `build/` 或 `devel/` 后，需要重新编译工作空间再启动程序。
