# 智慧社区 ROS 工作空间

ROS 1 智慧社区巡检工程，自研节点使用 Python 3。自研包的运行代码统一放在 scripts/，去掉包内 src/、Python 安装配置和重复包装入口；仅完成静态检查与离线回归，未进行 ROS 编译和实车测试。

## 目录及功能包

```text
robot_ws/
├── src/
│   ├── community_bringup/     # 系统启动、底盘串口、里程计、相机、导航配置与地图
│   ├── community_perception/  # YOLO、OCR、倒伏统计、models、srv
│   ├── community_mission/     # 路线、任务调度、语音、录像、srv
│   ├── lslidar/               # 厂商元包
│   ├── lslidar_driver/        # 厂商雷达实现
│   ├── lslidar_msgs/          # 厂商消息
│   ├── wit_ros_imu/           # 厂商 IMU
│   ├── robot_description/     # 原车模型，仅修改包与资源引用
│   └── CMakeLists.txt
├── docs/                     # 架构说明、完整性基线、检查报告
├── tools/                    # 静态检查与离线回归
├── requirements.txt
└── README.md
```

三个 community_* 包由项目维护；雷达与 IMU 驱动实现未修改。robot_description 保留模型几何与参数。community_perception 内 SORT 是第三方代码，保留文件头声明；ONNX 权重是原项目资源。

## 自研 Python 文件分布

工作空间根目录 `src/` 保留。以下文件均直接位于对应包的 `scripts/`，每个节点文件包含实现及启动入口，无需 `setup.py` 或 `__init__.py`。

```text
community_bringup/scripts/
├── configure_camera.py      # 相机参数设置
├── serial_bridge.py          # 底盘串口收发与速度反馈
└── odometry_node.py          # 平面里程计积分及 ROS 发布节点
community_perception/scripts/
├── yolo_detector_service.py  # YOLO 推理及检测服务
├── license_plate_server.py   # 车牌 OCR 识别服务
├── fall_counter_server.py   # 视频倒伏统计服务
└── sort.py                  # SORT 跟踪辅助模块，保留第三方声明
community_mission/scripts/
├── mission_node.py          # 路线调度、导航与任务启动入口
├── inspection.py            # 到点巡检及视频识别任务
├── rules.py                 # 识别结果处理与路线校验规则
├── services.py              # 服务调用与语音队列
├── result_manager.py        # 每次巡检的结果、耗时及日志记录
├── speech_server.py         # 语音播报服务
└── video_recorder_node.py   # 相机录像服务
```

各包 `CMakeLists.txt` 负责节点及辅助模块部署，`package.xml` 声明 ROS 依赖；`launch/` 保留现有启动入口，`config/` 保存配置，`srv/` 保存服务定义，`models/` 保存原模型权重。辅助模块在 Catkin devel 和 install 空间均放在节点旁，沿用普通同目录导入。原有 `roslaunch`、`rosrun` 命令及 Topic、Service、Action 名称不变。

`tools/validate_workspace.py` 检查语法、引用、安装声明和文件完整性；`tools/test_python_migration.py` 运行离线规则、里程计和任务回归。`docs/structure_integrity.json` 保留历史基线，记录移动/删除路径并校验当前文件哈希。

## 主要功能

保留六种 YOLO 模型（人员、交通灯、火情、垃圾桶、垃圾、电动车）、OCR、倒伏视频统计、路线导航、播报和录像。原路线全部航点及任务关联保留在 community_mission/config/route.yaml。模型统一在 community_perception/models，地图为 community_bringup/maps/map.yaml、map.pgm。

任务规则、推理算法、底盘协议及导航参数未调整。本工程仍缺异常温度、仪表识读、动态指示牌判断、独立泊车结果校验和全程 300 秒预算控制；不能将结构整理视为已经完整通过总决赛要求。

## 环境与依赖

目标为已有 ROS 1/Python 3 的 Linux 工控机，以 Noetic 为参考。需要 ROS 导航栈、robot_localization、usb_cam、cv_bridge、actionlib、dynamic_reconfigure 及厂商包所声明依赖。Python 依赖见 requirements.txt；OpenCV/NumPy 须匹配 cv_bridge，PaddleOCR 使用 2.x API。语音需要 espeak-ng 与 mb-cn1 语音资源，相机调参需要 v4l2-ctl。OCR 模型缓存须在目标机准备。

## 编译与启动（全部待工控机验证）

以下为目标机使用说明，本次未执行。路径以仓库部署在 /home/mowen/workspace/Smart-Community 为例：

```bash
source /opt/ros/noetic/setup.bash
cd /home/mowen/workspace/Smart-Community/robot_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
# 调试：启动硬件、导航和服务，默认不自动巡检
roslaunch community_bringup community.launch base_protocol:=aa_bb
# 比赛：一键执行现有路线；不要与上一命令同时运行
roslaunch community_bringup finals.launch base_protocol:=aa_bb
# 若已独立启动硬件/导航/服务，可单独执行任务
roslaunch community_mission mission.launch
```

base_protocol 必须按下位机确认：aa_bb 对应原新版三轴协议，legacy_11 对应原旧版两轴协议。默认底盘 /dev/carserial、雷达 /dev/lslidar、IMU /dev/ttyUSB1、相机 /dev/video0；udev 与串口权限需在工控机核对。

总入口支持 base_port、imu_port、camera_device、configure_camera、map、route、data_dir。输出默认 ~/.ros/community/<run_id>/，每次巡检独立保存图片、视频和结果。navigation.launch 统一启动模型 TF、EKF、地图、AMCL 和 move_base；hardware.launch 提供硬件及原始轮式里程计。

## 调试命令（待工控机验证）

```bash
rosnode list
rostopic hz /scan
rostopic hz /wit/imu
rostopic hz /usb_cam/image_raw
rostopic echo -n 1 /odom
rosservice list
rosservice type /people_detector/detect
rosservice type /speech_say
rosrun tf tf_echo map base_footprint
```

Topic、Service、Action 名称保留；自定义服务类型改为 community_perception/* 和 community_mission/*，外部客户端需要重新生成并更新导入。详见 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)。

## 开发机检查

```bash
python tools/validate_workspace.py
python tools/test_python_migration.py
```

不导入 ROS、不启动节点。结果见 [docs/VALIDATION.md](docs/VALIDATION.md) 和 [docs/PYTHON_TESTS.md](docs/PYTHON_TESTS.md)。原 IMU 非默认脚本语法问题及未启用的模型 Xacro 文件问题保持原状；默认使用完整的 mowen.urdf。

## 任务状态机

`mission_node.py` 使用 `INIT → NAVIGATE → TASK → NEXT_POINT` 推进原有 26 个航点；红绿灯点在到达后先进入 `WAIT_TRAFFIC`，确认绿灯再进入 `TASK`。同点任务按巡检、播报、开始录像、停止录像及视频统计的原有顺序执行。最后等待语音队列结束后进入 `FINISH`；异常进入 `ERROR` 并交由主入口退出清理，不重试。`transition()` 统一记录状态、具名航点 ID 和原因，日志使用 `[状态]`、`[导航]`、`[任务]`、`[红绿灯]` 前缀。

## 具名航点配置

`community_mission/config/route.yaml` 的 `points` 按顺序执行。每点包含唯一 `id`、有限数值 `x/y/yaw`、`type` 和 `tasks` 列表；纯导航点使用 `type: waypoint`、`tasks: []`，有任务的点使用 `type: task`。任务名称为 `person`、`plate`、`traffic_light`、`fire`、`trash`、`vehicle_photo`、`announcement`、`record_start`、`record_stop`。同点可关联多项任务，仍按既定业务顺序执行，绿灯确认在巡检之前。

人员点用 `area: A/B` 指定统计街区，车牌点用 `parking_lot` 指定播报编号。Launch 仍通过 rosparam 加载配置，`rules.validate_route()` 校验坐标、ID、任务及录像启停顺序；任务不再由数字索引触发。照片文件仍按路线顺序编号保存。原 26 个航点坐标、朝向、任务对应关系，以及人员街区和停车场编号均保持一致。

## 巡检结果

沿用 `data_dir` 启动参数作为结果根目录（默认 `~/.ros/community`），每次创建时间戳与 UUID 组成的独立 `run_id`。目录包含 `mission_results.json`、`mission.log`、`photo/` 和 `videos/`。图片和录像复用原保存流程，无新增图像处理。

JSON 记录当前航点、各航点完成状态、导航和任务执行状态、识别结果、UTC 开始/结束时间、单调时钟耗时、异常原因及媒体路径；未执行的配置任务标为 `NOT_EXECUTED`。正常空识别结果标记为 `EMPTY_VALID` 并计入成功数，播报记录入队文本；录像停止后单独记录 `vehicle_video` 视频统计任务。成功/失败任务数不包含导航，完成航点数仅计入已完成该点全部任务的航点。

终端显示必要进度、识别结果及结束汇总；完整状态转换写入文本日志。JSON 在每次更新时通过同目录临时文件替换保存。运行中的结果写入失败会提示并记录，不改变导航或识别决定；结果根目录须可创建。异常结束保留已有记录，未执行任务不会计作成功。未验证断电恢复或实车运行。

## 识别任务接口

`mission_node.py` 按原任务顺序调用 `Inspection.execute(point, task_type, source)`，其中 `source` 是相机图像，视频统计时为视频路径。`inspection.py` 只负责识别、统计及原有播报，不依赖结果管理器；主程序将返回值交给 `ResultManager.record_task()` 保存。

统一返回 `waypoint_id`、`task_type`、`status`、`reason`、`result`、`duration_sec`，并附开始和结束时间。`SUCCESS` 表示获得有效识别结果，`EMPTY_VALID` 表示服务正常但没有有效目标，`FAILED` 表示服务调用或处理异常。主程序保存失败结果后终止，不重试；正常空结果继续巡检。人员按原 ROI 统计数、车牌按原首个文本、火情按火焰数、垃圾按桶和垃圾检测列表、电动车按 stand/fall 或视频计数判定空结果。

火情中的楼名 OCR 失败仍使用默认楼名播报，不抑制已发现的火情；异常说明保留在 `reason` 和结果的 `ocr_error` 中。ROS 服务、模型推理和原有统计算法保持不变。

## 红绿灯等待与导航异常

`mission.launch`、`community.launch`、`finals.launch` 均可传入以下参数：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `green_confirm_frames` | 3 | 连续有效绿灯帧数，正整数 |
| `traffic_light_timeout` | 15.0 | 红绿灯等待期限，秒 |
| `goal_timeout` | 120.0 | 每次导航目标等待期限，秒 |
| `navigation_retries` | 0 | 初次导航失败后的最多重试次数，非负整数 |
| `navigation_cancel_timeout` | 2.0 | 重试前等待取消终态确认的期限，秒 |

例如：`roslaunch community_bringup finals.launch base_protocol:=aa_bb green_confirm_frames:=3 traffic_light_timeout:=15 navigation_retries:=0`。

进入 `WAIT_TRAFFIC` 后只读取新图像；图像时间戳必须递增。空结果、红/黄灯、冲突标签、无效置信度或检测框、重复/倒序帧均不能累计绿灯，且会清零连续计数。保留检测服务原有阈值，不增加模型置信度阈值参数。达到确认帧数且未超时、未退出，才允许继续本航点任务。图像或服务异常立即失败；等待超时记录 `TRAFFIC_TIMEOUT` 并进入 `ERROR`。

导航记录成功及实际尝试次数；异常原因区分 `NAV_TIMEOUT`、`NAV_CANCELLED`、`NAV_FAILED`、`NAV_LOST` 和 `CANCEL_TIMEOUT`。仅 ABORTED/REJECTED 或等待超时可在配置预算内重试；发送下一目标前必须确认旧目标已结束。外部取消、状态丢失、取消确认超时不重试。异常退出设置停止标志并取消目标，之后不再发出新目标；底盘超时停车逻辑保持原样。

### 实车验收（尚未执行）

- 检查相机时间戳正常递增，红/黄灯、空帧、绿灯闪烁和重复图像均不放行，连续 3 帧有效绿灯才继续。
- 验证默认 15 秒等待期限、断流和识别服务故障。现有同步 ROS 服务不能由此期限强制中断；须测量服务阻塞情况下的实际退出延迟，迟到绿灯返回后不得放行。
- 分别制造导航失败、超时、外部取消和目标丢失，核对取消消息、`ERROR` 状态、结果记录及后续无新目标。
- 单独授权开启 1 次重试，验证只有收到旧目标终态后才重发，取消无应答时停止，重试不超预算。
- 验证 Ctrl-C/节点退出时取消导航、停止录像；现场测量车辆停止时间与距离，并确认 `/cmd_vel` 失联后的底盘超时停车和急停有效。离线测试仅验证程序调用，不能证明车辆已停止。

## 配置文件与覆盖顺序

- `community_perception/config/detectors.yaml`：六种 YOLO 的模型包路径、类别顺序、保存类别和独立启动默认值；`mission_confidence` 单独保留总入口阈值。
- `community_perception/config/ocr_server.yaml`：OCR 阈值、水平裁剪比例、引擎及初始化重试参数。
- `community_perception/config/fall_counter.yaml`：视频检测、ROI、抽帧及 SORT 调用参数。
- `community_mission/config/runtime.yaml`：任务等待、有限重试、语音、录像及原有运行速度参数。速度是原值迁移，不是导航调参。
- `community_mission/config/recognition_rules.yaml`：ROI、去重/匹配阈值及垃圾标签中文映射；保留模型标签 `orther`。
- `community_bringup/config/hardware.yaml`：底盘、里程计、IMU 和相机节点参数；协议及端口仍使用启动参数。
- `community_bringup/config/navigation/amcl.yaml`：AMCL 原参数原值迁移；DWA、EKF 和代价地图 YAML 保持原样。

已有航点配置和相机控制配置继续使用。显式 Launch 参数覆盖 YAML；未指定的覆盖参数以空值表示，读取 YAML 默认值。YOLO 总入口每模型置信度覆盖优先于通用置信度，通用 IoU 现在传入所有启用模型。独立启动与总入口原有阈值差异保留，不静默统一。模型路径在 YAML 中使用包查找，Launch 通过 `subst_value="true"` 展开；直接 rosrun YOLO 前须加载配置并指定 model_name。

`data_dir/<run_id>/` 保存任务证据及结果；长期运行的 YOLO/OCR 服务调试图像分别保存到 `data_dir/detections`、`data_dir/ocr`，不冒充某次巡检证据。独立服务默认输出也统一在 `~/.ros/community/` 下。OCR 临时图像使用系统临时目录。内部图片/录像文件名保留，录像路径在任务对象初始化时只生成一次。

厂商雷达双机示例的缺失 RViz 配置及未启用 Xacro 的历史问题仍保留；未修改厂商驱动。所有新 YAML 随功能包安装。配置加载及运动行为还需 ROS 工控机验证。
