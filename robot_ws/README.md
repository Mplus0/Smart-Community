# 智慧社区实车 ROS 工作空间

ROS Noetic / Python 3 自主巡检工程。2026-11-07 已在 mowen 完成启动审查、硬件数据检查及架空低速运动测试；导航和完整路线尚未验收。

## 功能与目录

| 功能包 | 已实现功能 |
|---|---|
| community_bringup | 底盘串口、速度反馈、轮式里程计、相机配置；组合雷达/IMU/相机；EKF、地图、AMCL、move_base 导航 |
| community_perception | 六模型 YOLO：人员、交通灯、火焰、垃圾桶、垃圾、电动车；车牌/文字 OCR；SORT 视频倒伏统计 |
| community_mission | 26 个具名航点调度、到点巡检、连续绿灯确认、导航超时/有限重试、语音队列、录像、独立巡检结果与日志 |
| lslidar / lslidar_driver / lslidar_msgs | 雷达元包、厂商驱动与消息 |
| wit_ros_imu | 厂商 IMU，默认 normal 串口驱动 |
| robot_description | URDF 几何和固定关节 TF，默认 mowen.urdf |

自研节点与辅助模块在各包 scripts/，参数在 config/，模型在 community_perception/models/，地图在 community_bringup/maps/。

人员检测使用 `community_perception/models/people.onnx`，由 `yolo_train/model/person.pt`（YOLO26n）导出。输入为 FP32 `[1, 3, 640, 640]`，输出为 `[1, 6, 8400]`，ONNX opset 12；导出参数为 `end2end=False, nms=False, dynamic=False, simplify=False`，由现有检测服务执行后处理。模型类别 0 `community_person`、1 `outsider` 在 `detectors.yaml` 中分别映射为业务标签 `good`、`bad`。
[完整架构及接口](docs/ARCHITECTURE.md) · [架构图](docs/workspace_architecture.svg) · [逐步验收](docs/ACCEPTANCE.md)。

## 编译产物与检查状态

- 用户已完成编译；devel 中生成的六种 ROS 服务均可导入，rospy/cv_bridge/OpenCV/NumPy/pyserial/ONNX Runtime/PaddleOCR 可导入。
- community.launch / finals.launch 的 roslaunch --nodes 解析成功，分别 21 / 22 个节点。这不代表所有节点运行成功。
- 离线回归 38/38 通过。静态完整性检查仍有历史换行符、缺失 export.log 及本次三个启动参数变更造成的基线差异，详见 docs/VALIDATION.md。
- 原厂非默认 CAN/Modbus Python 脚本及未启用 Xacro 有历史语法问题；主入口不使用这些文件。
- 雷达/IMU 约10Hz，底盘反馈与里程计约20Hz；架空前进0.10m/s持续1秒后回零，用户确认车轮转动并停止。相机、录像及服务调用已实测；识别效果按用户要求暂不验收。
- 中文语音使用临时 cmn 与音频输出配置，现场确认正常；默认 mb-cn1 的依赖及重启自动恢复尚未验证。

## 启动方法

每个新终端先执行：

```bash
source /opt/ros/noetic/setup.bash
cd /home/mowen/workspace/Smart-Community/robot_ws
source devel/setup.bash
```

base_protocol 是必填参数，按下位机固件确认后选择：
aa_bb 为三轴速度（x/y/yaw），legacy_11 仅发送 x/yaw，不支持横移。
两种发送协议当前均使用 AA BB 反馈解析，旧固件的反馈兼容性须实测，不能只看端口名推断。

首次验收按层启动，先不要运行自动任务：

```bash
# 终端 1：仅硬件；本车已确认使用 aa_bb
roslaunch community_bringup hardware.launch base_protocol:=aa_bb
# 硬件合格后，终端 2：定位与导航
roslaunch community_bringup navigation.launch
# 终端 3：识别、语音和录像服务
roslaunch community_bringup services.launch
# 定位、服务和停车检查合格后，终端 4：自动执行配置路线
roslaunch community_mission mission.launch
```

日常组合启动（与上面的分层节点二选一，避免重复占用设备）：

```bash
# 启动硬件、导航、服务；默认 run_mission:=false
roslaunch community_bringup community.launch base_protocol:=aa_bb
# 自动巡检入口：相当于 community.launch run_mission:=true
roslaunch community_bringup finals.launch base_protocol:=aa_bb
```

finals.launch 不包含人工验收或定位确认界面；服务就绪后任务会推进。required 任务节点退出可能关闭同一 roslaunch 下的其他节点。
首次推荐分层方式，便于定位故障。

默认设备：底盘 /dev/carserial、雷达 /dev/lslidar、IMU /dev/imu、相机 /dev/video0。
总入口可覆盖 base_port、imu_port、camera_device、configure_camera、map、route、data_dir。
雷达端口由厂商 lslidar_serial.launch 指定，总入口没有雷达端口参数。
相机调参默认关闭，确认需要时使用 configure_camera:=true。

## 配置与结果

- route.yaml：26 个具名航点、任务关联及人员街区/停车场编号；启动前确认地图、坐标和现场一致。
- runtime.yaml：绿灯连续 3 帧、交通灯等待 15 秒、单目标等待 120 秒、导航默认不重试、取消确认 2 秒。
- recognition_rules.yaml：人员 ROI、车辆去重、垃圾分类及匹配规则。
- detectors.yaml / ocr_server.yaml / fall_counter.yaml：模型、阈值、OCR 与视频统计配置。
- hardware.yaml 和 navigation/：设备参数、EKF、AMCL、DWA 和代价地图。此次审查未调整运动参数。
- 显式 launch 覆盖优先于 YAML；留空的可选覆盖使用 YAML 默认值。

任务状态：INIT → NAVIGATE →（交通灯点 WAIT_TRAFFIC）→ TASK → NEXT_POINT → FINISH；失败进入 ERROR。
同步识别服务可能阻塞，等待期限不能强制打断推理；必须实测延迟。
结果默认保存到 ~/.ros/community/<run_id>/：mission_results.json、mission.log、photo/、videos/。
服务调试图另在 detections/、ocr/，不属于某次任务证据。

## 键盘建图与导航测试

每个终端先加载上文 ROS 和 devel 环境。小车需安装 `ros-noetic-gmapping`，
图形显示需 `ros-noetic-rviz`。新增脚本后执行一次 `catkin_make` 并重新加载 devel。

```bash
# 终端 1：硬件（已有硬件节点运行时不要重复启动）
roslaunch community_bringup hardware.launch base_protocol:=aa_bb
# 终端 2：建图、机器人 TF 和 EKF；有桌面时可追加 rviz:=true
roslaunch community_bringup mapping.launch
# 终端 3：在交互终端进行键盘控制，SSH 使用 ssh -t mowen
rosrun community_bringup teleop_twist_keyboard.py
```

默认线速度 0.1 m/s、角速度 0.3 rad/s；10 Hz 发送，停止按键输入 0.6 秒后回零。
`i` 前进、`,` 后退、`j/l` 左右转；大写 `J/L` 左右横移（Shift+j/l）；
空格或 `k` 停车，Ctrl+C 退出。`q/z` 调整速度，q 不是退出键。
终端必须保持焦点；其他速度发布节点应停止，尤其不要同时运行 move_base 或自动巡检。
参数可覆盖，例如 `_speed:=0.08 _turn:=0.2`。
脚本来自 [ros-teleop/teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard)，
版本与改动见 [来源说明](src/community_bringup/docs/TELEOP_SOURCE.md)。

RViz 将 Fixed Frame 设为 `map`，添加 Map（`/map`）、LaserScan（`/scan`）、TF、RobotModel。
确认激光与障碍物重合后再缓慢遥控建图。建图时观察相机中目标的尺寸、角度与遮挡，
作为采集位置的依据；导航使用的最终位置应在同一地图坐标系中记录。

```bash
# 保持建图运行，另开终端保存新地图（不要覆盖现有地图）
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/community_test
# 保存后停止键盘和 mapping.launch，保留硬件，启动新地图定位导航
roslaunch community_bringup navigation.launch map:=$HOME/maps/community_test.yaml
rosrun rviz rviz
```

建图和 navigation.launch 二选一，避免重复的 map→odom 与 EKF/机器人 TF。
导航时先用 RViz 的 2D Pose Estimate 初始化定位，确认地图与扫描对齐后，
再在空旷处设置近距离的 2D Nav Goal。首次测试不启动 mission.launch 或 finals.launch。
以上入口具备建图/导航链路，地面运动、地图质量与导航到点效果需现场验收。

## 图片采集

用于采集人物、车辆、红绿灯、垃圾等各类目标及背景图片。先启动相机并加载 ROS 工作空间环境，在 `robot_ws` 目录执行：

```bash
python3 tools/collect_dataset.py community_images
# SSH 无图形界面：回车保存一张，输入 q 后回车退出
python3 tools/collect_dataset.py community_images --no-preview
```

默认订阅 `/usb_cam/image_raw`，可用 `--topic` 指定其他 `sensor_msgs/Image` 话题。
预览窗口按空格或 S 保存原图，按 Q 或 Esc 退出。
数据集名称由命令的位置参数指定，例如 `community_images`、`traffic_light` 或 `bicycle`，可按采集任务自行命名。
图像按时间命名，保存至 `robot_ws/datasets/<数据集名称>/images/`，重复运行可继续采集。
可用 `--datasets-dir /目标目录` 更改数据集根目录。
采集图像不含检测框或标签，训练前需标注并划分训练集、验证集；本地数据集不纳入 Git。

## 人物模型浏览器测试

工具 `tools/test_person_model.py` 直接加载实车的 `people.onnx`，默认订阅 `/usb_cam/image_raw`，无需启动检测服务或巡检任务，也不直接占用 USB 相机。使用与实车检测服务相同的缩放、阈值和 NMS 方式，绿框表示社区人员，红框表示外来人员；画面上方显示两类人数和单帧处理耗时。人数统计为全画面检测结果，不应用巡检任务的 ROI 规则。

先在小车终端加载环境并启动相机，已有相机话题发布时跳过相机启动命令：

```bash
source /opt/ros/noetic/setup.bash
cd /home/mowen/workspace/Smart-Community/robot_ws
source devel/setup.bash
# 仅启动相机；已经运行 hardware.launch 时不要重复启动
rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _pixel_format:=mjpeg _image_width:=640 _image_height:=480 _framerate:=30
```

另开终端，同样加载环境后执行：

```bash
cd /home/mowen/workspace/Smart-Community/robot_ws
python3 tools/test_person_model.py --port 8080
```

在同一局域网的电脑浏览器访问 `http://小车IP:8080/`，小车本机访问 `http://127.0.0.1:8080/`。默认监听 `0.0.0.0`，使用 MJPEG 实时预览，Ctrl+C 关闭。相机停止发布时会保留最后一帧。端口被占用时通过 `--port 8081` 更换端口；无法连接时检查小车 IP、网络及端口放行。

可覆盖相机话题、置信度、IoU 阈值和最大推理频率：

```bash
python3 tools/test_person_model.py --topic /usb_cam/image_raw --conf 0.5 --iou 0.4 --fps 10 --port 8080
# 指定其他兼容的两类 ONNX 模型
python3 tools/test_person_model.py --model /绝对路径/people.onnx --port 8080
# 单张图片预览，无需 ROS 或相机；依赖 OpenCV、NumPy、ONNX Runtime
python3 tools/test_person_model.py --image ../yolo_train/datasets/person/val/images/0011.jpg --port 8080
```

实时模式使用 ROS Noetic 的 `rospy`、`cv_bridge`、`sensor_msgs` 和现有 ONNX Runtime/OpenCV/NumPy 依赖，不需要 Flask 或图形桌面，也不需要重新编译工作空间。HTTP 部分使用 Python 标准库 [ThreadingHTTPServer](https://docs.python.org/3.8/library/http.server.html#http.server.ThreadingHTTPServer)。

## 开发检查及边界

```bash
python3 tools/validate_workspace.py
python3 tools/test_python_migration.py
# 仅在改动后需要重新构建时
catkin_make
source devel/setup.bash
```

底盘有 0.5 秒命令超时发送零速度的逻辑；实际停车效果仍依赖固件和现场测量。
识别效果、实时性和比赛要求是否满足均以实车验收为准。详细审查记录见 docs/ACCEPTANCE.md。


## USB 连接与稳定别名

本车应通过包含底盘、IMU、雷达串口的板载 USB 集线器连接工控机。
调试中直连另一接口时只有单个 CH340 且无反馈；恢复集线器连接并校正绑定后反馈恢复。
不能只凭 CH340 名称或 ttyUSB 编号判断设备用途。

当前主机映射（物理 USB 插口改变后须重新核对）：

| 用途 | 稳定别名 | 本次设备节点 | 本次 USB 父设备路径 |
|---|---|---|---|
| 底盘 | /dev/carserial | ttyUSB1 | 1-1.2 |
| IMU | /dev/imu | ttyUSB0 | 1-1.1 |
| 雷达 | /dev/lslidar | ttyACM0 | 1-1.4 |

三个硬件组合入口均以 /dev/imu 为默认 IMU 端口，避免将 ttyUSB1 误作 IMU。
udev 规则位于目标机 /etc/udev/rules.d/，属于主机配置，不随 Git 提交部署。
换机时先通过拔插和数据帧核对设备身份，再建立别名。不要同时运行新旧底盘节点占用串口。

## 图片采集

用于采集人物、车辆、红绿灯、垃圾等各类目标及背景图片。相机启动并发布 `/usb_cam/image_raw` 后，在已加载 ROS 环境的终端执行：

```bash
cd /home/mowen/workspace/Smart-Community/robot_ws
python3 tools/collect_dataset.py community_images --datasets-dir ../yolo_train/datasets --no-preview
```

`community_images` 是自定义数据集名称，可替换为 `bicycle`、`traffic_light` 等。上述命令将原图保存至 `yolo_train/datasets/community_images/images/`；不指定 `--datasets-dir` 时，默认保存至 `robot_ws/datasets/<数据集名称>/images/`。图像按时间命名，重复运行可继续采集。可通过 `--topic` 指定其他 `sensor_msgs/Image` 相机话题。

当前用户环境的 OpenCV 不支持 GUI 时，脚本会自动转为终端采集。若需实时预览，请在小车图形桌面终端使用系统 OpenCV：

```bash
python3 -s tools/collect_dataset.py community_images --datasets-dir ../yolo_train/datasets
```

`-s` 禁用用户级 Python 包，使用已安装且支持 GTK 的系统 OpenCV；终端仍需先加载 ROS 环境。纯 SSH 会话使用 `--no-preview`。

SSH 无窗口模式下，按回车保存一张最新图像，输入 `q` 后回车退出。去掉 `--no-preview` 使用图形预览，S/空格保存，Q/Esc 退出。保存的是原图，不包含预览提示或检测框。

所有数据集（包括 `person`）统一保存至 `<数据集根目录>/<数据集名称>/images/`，不自动分类，也不创建类别或训练集划分目录。分类与标注由用户后续手动完成。

采集用于 YOLO 训练的图片后，仍需逐个目标画框标注并划分训练集、验证集；分类文件夹不能代替检测标签。

### 浏览器预览与拍照

在已加载 ROS 环境、相机已启动的 robot_ws 终端执行：

```bash
python3 tools/collect_dataset.py trashbin --datasets-dir ../yolo_train/datasets --web-port 8080 --web-host 0.0.0.0 --web-only
```

同一局域网访问 http://小车IP:8080/，可实时预览并点击“拍照保存”。
VS Code Remote SSH 可在“端口”面板转发 8080，再打开转发地址。
预览约 10 帧/秒，保存原始分辨率 JPEG（质量 95），目录是 yolo_train/datasets/trashbin/images/。
相机断流超过 2 秒时拒绝保存旧画面；本次保存数量从每次启动时的 0 开始统计。
--web-port 默认 0（关闭），--web-host 默认 127.0.0.1（仅本机）。
--web-only 适合后台运行；需要同时保留终端回车拍照时，将它替换为 --no-preview。
启动前先退出旧采集进程，ROS 主节点与相机节点保持运行。
