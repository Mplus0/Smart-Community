# robot_competition

负责路线管理、TF 航点录入、move_base 多航点任务执行，以及到点后的人员/车牌结果采集、融合和保存；在交通灯点停车等待连续有效绿灯后放行。视觉模型由 `robot_perception` 独立运行，任务主程序使用系统 ROS Python 3.8。完整分终端启动命令见 [工作空间说明](../../README.md)。

下文包含人物/车牌参数、结果格式、统计口径和字体配置；视觉消息字段见 [视觉包说明](../robot_perception/README.md)。

## 文件职责与运行逻辑

- `package.xml`、`CMakeLists.txt`：Noetic Python 3 依赖和节点/资源安装。
- `scripts/waypoint_manager.py`：无 ROS 依赖的 YAML 加载、严格校验和命令行检查。
- `scripts/waypoint_recorder.py`：交互录入 `map -> base_footprint` TF 位姿；显式追加、重复 ID 拒绝、文件锁与同目录原子替换。
- `scripts/main_controller.py`：Action 通信、任务调用、状态转换和安全终止。
- `scripts/perception_client.py`：持续订阅 JSON/标注图，严格校验、时间窗口与有界缓存。
- `scripts/task_processor.py`：有限重试、人员短时匹配、车牌全文投票、同源证据匹配。
- `scripts/traffic_wait.py`：红绿灯任务窗口、连续绿灯判断、墙钟超时及异常停止。
- `scripts/result_manager.py`：按点/区域/全局保守统计，原子 JSON、日志及有限标注图片。
- `config/task_config.yaml`：默认采集、融合规则和计数分区约定。
- `config/waypoints.yaml`：默认路线，包含交通灯点 `light_01`、`light_02`。
- `launch/competition.launch`：仅启动主程序。
- `launch/environment.launch`：集中启动 Gazebo/RViz、导航和三路视觉，不启动主程序；固定使用 developer 的视觉解释器及文泉驿中文字体。
- `scripts/camera_capture.py`：交通灯分类图片采集工具，不参与正式任务。

`../tools/start.sh`、`start_competition.sh`、`stop.sh` 和 `gui.py` 提供终端与图形运行入口，使用方法见 [工作空间说明](../../README.md)。默认路线顺序为 `light_01 → wp_01 → person_01…person_05 → light_02 → plate_01…plate_03 → wp_02`，共 12 点。路线中的 map 坐标不是 Gazebo world 坐标。

状态：`INIT → NAVIGATE → TASK（人物/车牌）或 WAIT_TRAFFIC（交通灯）→ NEXT_POINT → NAVIGATE / FINISH`；普通航点直接进入 NEXT_POINT。错误进入 `ERROR` 并取消本节点目标、停止后续发送。空路线直接 `FINISH`。只有 Action 的 `SUCCEEDED` 才算到点。失败和超时可有限重试；收到外部取消或 LOST 则终止。取消确认超时不会重发目标。服务器、导航、取消及仿真时钟 watchdog 均使用墙上时钟上限，不依赖 `/clock` 继续走动。

人员/车牌点在导航成功并等待稳定时间后开启新帧采集窗口；每次识别结果立即保存，普通识别失败有限重试后继续路线。时钟失效、关闭、存储异常仍进入 ERROR。正式模式在 INIT 检查所有交通灯点的 `stop_before_line: true`；任一点未确认就拒绝整条路线。到达交通灯点后进入 `WAIT_TRAFFIC`，持续发布零速度；放行后才进入 `NEXT_POINT`。`navigation_test_mode` 默认 false；显式 true 时跳过视觉及红绿灯等待，不可用于正式比赛。

## 红绿灯等待

交通灯点由 `waypoints.yaml` 配置。比赛视觉终端需启用交通灯分类节点，再启动 `competition.launch`；后者只启动任务控制器。默认订阅 `/perception/traffic_light_json`，可通过 launch 的 `traffic_light_json_topic` 修改；零速度话题默认 `/cmd_vel`，可用 `cmd_vel_topic` 匹配底盘配置。

`config/task_config.yaml` 的独立 `traffic_light` 段使用 `traffic_timeout_sec=35`、`green_confirm_frames=3`、`min_confidence=0.8`、`max_source_age_sec=3.0`、`settle_sec=0.5`，不继承人物/车牌参数。超时采用单调墙钟，包含稳定期。稳定期结束后清空接收队列，只接受源时间严格晚于当前窗口起点的数据；延迟到达的窗口前数据丢弃并重置绿灯计数。有效 RED、YELLOW、低置信度 GREEN 和分类器的低置信度 UNKNOWN 均等待并重置计数。必须连续 3 帧有效且置信度至少 0.8 的 GREEN 才放行，源或接收时间间隔超过 3.0 秒也打断连续性。

当前窗口内的重复、倒序、过期、未来时间戳、损坏 JSON、摄像头/推理故障、源或 ROI 改变、缓冲溢出均进入 ERROR，禁止后续导航；无数据或始终未确认绿灯在 35 秒后 ERROR。分类节点不验证灯体存在，任务依赖航点位置和相机 ROI 对应实际交通灯。

`mission_results.json` 的 `tasks` 中记录 `last_recognition`、`last_label`、`green_streak`、`wait_duration_sec`、`released`、`rejected_frames`、最终状态和最近 128 条观测；`mission.log` 写入所有 `TRAFFIC_OBSERVATION`、拒绝原因及同一份 `TASK_RESULT`。

## YAML 格式

根对象必须是 `route`，其中 `frame_id: map`、`points` 为有序列表。每点必填 `id`、`type`、`task`、`x`、`y`、`yaw`。`type: waypoint` 对应 `task: none`；`type: task` 对应 `person / plate / traffic_light`。坐标单位为米、yaw 为弧度，必须为有限数值（拒绝布尔、字符串、null、NaN、Infinity）。ID 必须唯一且非空；拒绝重复 YAML 键及未知字段。

可选字段：`area`、`description`（字符串），`navigation_timeout`（正数秒）、`retries`（非负整数，表示首次之外的重试次数）；交通灯点额外允许布尔 `stop_before_line`。点级超时/重试覆盖 launch 的默认值。

下面仅是结构示例，**null 会被拒绝，不可直接导航**：

```yaml
route:
  frame_id: map
  points:
    - id: light_01
      type: task
      task: traffic_light
      x: null
      y: null
      yaw: null
      stop_before_line: false
      description: 待实测确认整个车身位于停止线前
```

`stop_before_line: true` 是操作者的实测确认，程序不自动验证停止线。应考虑车体、轮子外缘、定位误差、move_base 到点容差及制动余量；既有导航位置容差为 0.08 m，不能只看机器人中心。航点间路径也必须人工验证沿比赛指定道路，不得假定仅端点安全就能保证全程不越线。

## 人员、车牌融合与统计

参数由 `task_config.yaml` 的 `defaults` 提供，再由 `person` / `plate` 覆盖；当前 `person` 为空对象。

| 参数 | 当前配置 | 含义 |
|---|---|---|
| `task_timeout_sec` / `task_retry_count` | 12 秒 / 1 | 单次任务时间上限、额外重试次数 |
| `settle_sec` / `observation_sec` | 0.5 / 2.0 秒 | 到点稳定期、观察窗口 |
| `min_valid_frames` / `max_valid_frames` | 4 / 24 | 采集有效帧上下限 |
| `min_support_frames` / `min_confidence` | 3 / 0.5 | 融合结果支持帧数和识别阈值 |
| `max_source_age_sec` | 2.0 秒 | 人员/车牌源帧时效上限 |
| `image_match_timeout_sec` / `max_evidence_images` | 1.5 秒 / 3 | 同源标注图等待上限、单次证据图数量限制 |
| `match_iou` / `match_center_distance` | 0.3 / 0.08 | 短时框匹配阈值；中心距离按图像尺寸归一化 |
| `max_track_gap_frames` | 2 | 短时匹配允许的帧间隔 |
| `vote_fraction` | 0.67 | 车牌完整字符串投票比例 |
| `plate.plate_text_annotation_confirmed` | true | 操作者声明已确认完整中文车牌标注 |

人员通过短时检测框匹配获得稳定的社区/非社区人员结果，不提供跨视角 ReID。车牌按完整字符串投票，空间上有多个候选或文字冲突时可返回 `AMBIGUOUS`，不会拼接各帧字符生成号码。JSON 与证据图按源时间戳、`frame_id` 匹配，不能只比较跨话题 `seq`。

当前 `counting.non_overlapping_partitions_confirmed: true`，`assignments` 将 `person_01` 至 `person_05` 分别映射到 `block_01` 至 `block_05`。这是人工分区声明，不是自动去重证明。更换路线时同步更新 `area`、`assignments` 与确认标志；确认模式要求每个人员点有唯一分区和非空 area，否则初始化失败。无完整覆盖或存在未解决重叠时，汇总 `counts` 为 null；`observed_counts_not_unique` 仅为观测相加，不能当作独立人数。

中文字体应通过视觉入口的 `plate_font_path` 配置，集中入口固定为 `/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc`。任务配置 true 不会检查图中文字；应目视确认完整号码。代码在缺省配置时回退为 false，未确认而识别成功的车牌任务会记为 `FAILED_IMAGE_ANNOTATION`。

## 启动参数与输出

| `competition.launch` 参数 | 默认值 |
|---|---|
| `waypoints_file` / `task_config_file` | 包内 `config/waypoints.yaml` / `config/task_config.yaml` |
| `move_base_action` | `/move_base` |
| `navigation_timeout` / `server_timeout` | 90 / 15 秒 |
| `cancel_timeout` / `clock_timeout` | 3 / 5 秒 |
| `retries` / `navigation_test_mode` | 1 / false |
| `results_root` | `/workspace/car_2026/results`，必须为绝对路径 |
| `max_cache_frames` / `max_cache_age_sec` / `max_cache_bytes` | 24 / 10 秒 / 67108864 字节 |

输出话题参数在 [launch 文件](launch/competition.launch) 中定义，可重映射人物、车牌 JSON/图像以及交通灯 JSON 和速度话题。自定义路线示例：

```bash
roslaunch robot_competition competition.launch \
  waypoints_file:=/workspace/car_2026/route_measured.yaml \
  task_config_file:=/workspace/car_2026/task_measured.yaml
```

两个文件须事先准备，人员统计分区应与自定义路线对应。

每次运行创建 `results/<run_id>/`，run_id 含 UTC 时间与随机后缀：

| 文件 / 字段 | 内容 |
|---|---|
| `mission_results.json` | 原子更新的结构化结果，`schema_version=1` |
| `status` / `error` / `mission_duration_sec` | 运行状态、错误与耗时 |
| `waypoint_order` / `waypoint_executions` | 路线顺序、各点导航执行记录 |
| `pending_task` / `tasks` | 当前任务与每次已完成尝试；重试记录保留 |
| `person_summary` | 按点、区域和全局统计及覆盖/重复标志 |
| `plates` | 每个车牌点最后一次尝试的号码与状态 |
| `tasks[].evidence` / `images/` | 同源标注图元数据与 JPEG 文件 |
| `mission.log` | 逐行 JSON 事件，包含任务结果与交通灯观测 |

`SUCCESS` 和 `EMPTY_VALID` 为有效完成；有效空结果与断流不同。常见失败包括 `FAILED_TIMEOUT`、`FAILED_INVALID`、`FAILED_UNSTABLE`、`AMBIGUOUS`、`FAILED_IMAGE_MISSING`、`FAILED_IMAGE_ANNOTATION`、`FAILED_INTERRUPTED`。纯导航测试记为 `SKIPPED_NAVIGATION_TEST`。总状态 `FINISH` 仅表示路线走完，识别失败仍可能存在，应结合各点最新任务状态判断。GUI 从这里读取结果和证据，不会启动额外推理模型。

## Docker 构建与检查

宿主机只使用 Docker 入口；所有以下 ROS/Python/catkin 命令均在容器终端运行。进入现有容器（每个终端各执行一次）：

```bash
docker exec -it --user developer smart-community-dev bash
```

容器内：

```bash
source /opt/ros/noetic/setup.bash
source /opt/cartographer_ws/install_isolated/setup.bash
cd /workspace/car_2026
catkin_make -j2
source devel/setup.bash --extend
rosrun robot_competition waypoint_manager.py src/robot_competition/config/waypoints.yaml
```


## 航点录入的三终端启动与定位

三个容器终端均按 [工作空间说明](../../README.md) 加载 ROS、Cartographer 和工作空间环境。

终端 1：

```bash
roslaunch robot_gazebo simulation.launch rviz:=true
```

终端 2：

```bash
roslaunch robot_navigation navigation.launch
```

在 RViz 设置 Fixed Frame 为 `map`，显示 `/map`、LaserScan `/scan`、TF、RobotModel。用 **2D Pose Estimate** 设置 AMCL 初始位置和朝向，检查激光与墙体/地图对齐，移动后仍稳定。用 **2D Nav Goal** 将机器人逐段移动到合规位置，等待到点并检查朝向；也可使用既有遥控工具移动，但不要与主控制器同时发送命令。录入时主程序必须尚未运行。

终端 3 录入新文件：

```bash
rosrun robot_competition waypoint_recorder.py --output /workspace/car_2026/route_measured.yaml
```

交互示例（每条 `add` 之前在 RViz 移动到对应位置、等停稳并检查定位）：

```text
add wp_01 waypoint none description="实测普通航点"
add person_01 task person area=block_01
add plate_01 task plate area=parking
add light_01 task traffic_light stop_before_line=true description="已确认全车停止线前且留足余量"
quit
```

最后一条只在确实检查了全车停车安全后使用。未确认时录入 `stop_before_line=false`，正式运行将拒绝该路线。每次成功录入立即保存；输入错误、TF 缺失/陈旧、重复 ID、写入失败均提示且不加入该点。TF 可用不等于 AMCL 定位准确，仍须目视检查。使用 `quit`、Ctrl-D 或 Ctrl-C 退出。再次添加必须显式：

```bash
rosrun robot_competition waypoint_recorder.py --output /workspace/car_2026/route_measured.yaml --append
rosrun robot_competition waypoint_manager.py /workspace/car_2026/route_measured.yaml
```

不提供隐式覆盖或替换点操作；需改点时先备份 YAML，人工修改后重新验证。`.lock` 是协作录入锁文件，可保留；其他编辑器不遵守此锁，勿同时修改同一文件。

完成录入后，准备与新路线对应的 `task_measured.yaml`（同步更新 counting 分区配置），主程序入口如下；需先启动独立视觉终端，完整步骤见 [工作空间说明](../../README.md)：

```bash
roslaunch robot_competition competition.launch \
  waypoints_file:=/workspace/car_2026/route_measured.yaml \
  task_config_file:=/workspace/car_2026/task_measured.yaml
```

仅在已检查环境安全的纯导航测试中，才显式启用：

```bash
roslaunch robot_competition competition.launch \
  waypoints_file:=/workspace/car_2026/route_measured.yaml \
  task_config_file:=/workspace/car_2026/task_measured.yaml \
  navigation_test_mode:=true navigation_timeout:=90 retries:=1
```

参数：`move_base_action` 默认 `/move_base`；`server_timeout` 15 秒，`navigation_timeout` 90 秒，`cancel_timeout` 3 秒，`clock_timeout` 5 秒，`retries` 1。暂停 Gazebo 超过 watchdog 时限会终止任务；重启任务从第一个航点开始，须先检查机器人位置与路线。任何时刻只运行一个主控制器；不要同时发送 RViz Goal。

## 故障定位（容器内）

```bash
rosnode list
rostopic echo -n 1 /clock
rosrun tf tf_echo map base_footprint
rostopic echo -n 1 /amcl_pose
rostopic echo -n 1 /move_base/status
rosparam get /move_base/global_costmap/robot_base_frame
rosparam get /main_controller
```

若 TF 不可用，先检查 AMCL 初始位姿、激光、EKF 与 `/clock`；若目标超时，检查地图、costmap、TEB 和实际通道宽度。主程序取消自己发送的导航目标，并在红绿灯等待及错误处理时向配置的速度话题发布零速度。Action 通信丢失时取消不能保证已被导航端接收，应先检查底盘已停止再恢复任务。
