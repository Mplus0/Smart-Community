# P2-B：任务感知集成与结果保存

主程序在系统 ROS Noetic Python 3.8 中运行，只订阅既有 `robot_perception` 的 JSON 与标注图，使用系统 NumPy/OpenCV 保存 JPEG；不导入 torch、ultralytics、hyperlpr3，不加载权重，不改变 Python 3.10 推理隔离环境。本文描述 P2-B 人物/车牌任务，保留当时的测试记录；当前已接入 P2-C 红绿灯等待与放行，见 [README](README.md)。

## 文件与控制流程

| 文件 | 职责 |
|---|---|
| `scripts/main_controller.py` | 保留 P1 状态机及 Action 处理，在 TASK 调用处理器；FINISH/ERROR 保存汇总 |
| `scripts/perception_client.py` | 两路订阅、JSON/图像校验、时间戳与接收时间检查、有界缓存 |
| `scripts/task_processor.py` | 稳定等待、新帧窗口、有限重试、人员匹配、车牌全文投票、同源图片选择 |
| `scripts/result_manager.py` | 每轮唯一目录、原子结果写入、任务日志、有限 JPEG、分区统计 |
| `config/task_config.yaml` | 默认任务参数、分类覆盖参数及非重叠计数约定 |
| `launch/competition.launch` | 只启动主控制器，加载配置与话题/缓存/输出路径参数 |
| `CMakeLists.txt`、`package.xml` | 安装新增脚本/文档，补充 sensor_msgs、std_msgs、系统 NumPy/OpenCV 依赖 |
| `tests/test_p2b.py`、`tests/run_docker_checks.sh` | 合成 JSON/Image、独立 ROS Master 与假 Action Server；不加载模型 |
| `tests/test_p1.py` | 仅增加临时结果目录和短任务超时；原 21 项测试断言保留 |
| 项目 `.gitignore` | 忽略 `car_2026/results/` |

普通 waypoint 只导航。person/plate 点必须先收到 `move_base SUCCEEDED`，等待 `settle_sec`，然后创建独立窗口。到点稳定仅依赖 move_base 成功及等待时间，没有新增轮速静止检测；运行时不要同时发送遥控/RViz Goal。

每次尝试的窗口开始时清空该路已有缓存，但保留已接收时间戳水位，阻止旧消息重播。仅接纳窗口开始后接收到、源时间戳不早于窗口起点的帧；起点和来源使用整数纳秒比较。来源时间戳必须递增、非零、不在 ROS 未来；源帧年龄和接收后墙钟年龄均不超过 `max_source_age_sec`。同一窗口来源 frame_id 或分辨率改变也拒绝。

达到 `min_valid_frames` 且观察至少 `observation_sec` 后融合；达到 `max_valid_frames` 可提前结束。到 `task_timeout_sec` 仍不足最少帧则失败。图片匹配另有有限等待。每次尝试耗时上限约为 `settle_sec + task_timeout_sec + image_match_timeout_sec` 加文件 I/O/调度开销；最大尝试次数为 `task_retry_count + 1`。全部等待使用 monotonic 墙钟。模型断流只令本任务失败；ROS 关闭、仿真时钟暂停/倒退、结果存储异常会令整轮 ERROR，不继续导航。

## 输入、校验和输出接口

| 私有参数（同名 launch arg） | 默认话题 | 类型 |
|---|---|---|
| `~person_json_topic` | `/perception/person_detections_json` | `std_msgs/String` |
| `~person_image_topic` | `/perception/person_image` | `sensor_msgs/Image` |
| `~plate_json_topic` | `/perception/plates_json` | `std_msgs/String` |
| `~plate_image_topic` | `/perception/plates_image` | `sensor_msgs/Image` |

JSON 必须为对象，`schema_version` 为整数 1；`header` 包含整数 seq、stamp.secs/nsecs 和非空 frame_id。width/height 是 1–8192 的整数，processing_ms 是有限非负数。人物 engine 为 `yolo26_person_detector`；只有布尔 `frame_valid: true` 且 `reason: ok` 的帧有效，`detections` 为数组。映射固定 `0/community_person/C`、`1/non_community_person/NC`。车牌 engine 为 `hyperlpr3`、`plates` 为数组，没有 frame_valid/reason 要求；项含完整 Unicode text、整数 plate_type_id（兼容 HyperLPR 的 -1/UNKNOWN）、confidence、bbox_xyxy。

每帧最多 100 个对象；置信度必须有限且在 [0,1]，bbox 为四个有限坐标，`0 <= x1 < x2 <= width`、`0 <= y1 < y2 <= height`。拒绝重复 JSON 键、NaN/Infinity、类型错误（包括冒充数字的 bool）及超 256 KiB JSON。无效帧只记警告和诊断计数，不算有效空帧。完整契约沿用 [P2-A interface](../robot_perception/docs/interface.md)。

图像必须为 bgr8，正确处理每行 step 填充；单帧限 8 MiB。只按**同一任务类型、完全相同 stamp + frame_id、相同尺寸**匹配；不使用 seq，人物与车牌不能相互配图。没有匹配图时记录缺失，不能拿相机原图或上一帧顶替。本包不新增业务输出话题；结果输出至 ROS 终端日志及文件。原有 `/move_base` Action 与 P1 相同。

## 参数

`~task_config` 由 `task_config_file` 指向的 YAML 加载；不是顶层 `~task_timeout_sec`。YAML 有 `defaults`、`person`、`plate`、`counting` 四部分，person/plate 覆盖 defaults。配置启动时读取，运行中修改不即时生效。

| defaults 字段 | 默认值 | 含义 |
|---|---:|---|
| task_timeout_sec | 12.0 | 每次采集墙钟上限 |
| settle_sec | 0.5 | 到点后/重试前稳定等待 |
| observation_sec | 2.0 | 正常结束前最短观察时间 |
| min_valid_frames | 4 | 最少有效、唯一源帧 |
| max_valid_frames | 24 | 单次采集帧数上限，硬上限 128 |
| min_support_frames | 3 | 稳定目标/车牌全文的最少支持帧 |
| max_source_age_sec | 2.0 | ROS 源帧年龄和墙钟接收年龄上限 |
| image_match_timeout_sec | 1.5 | 融合后等同源标注图的最长时间 |
| task_retry_count | 1 | 首次失败后允许额外尝试次数 |
| min_confidence | 0.5 | 参与关联/融合的单帧置信度阈值 |
| match_iou | 0.3 | 相邻目标框匹配 IoU 下限 |
| match_center_distance | 0.08 | 匹配中心距离上限；dx/width、dy/height 的欧氏距离 |
| max_track_gap_frames | 2 | 同一短轨迹允许缺失的有效帧数 |
| vote_fraction | 0.67 | 车牌全文赢家占该轨迹有效样本比例 |
| max_evidence_images | 3 | 每次尝试最多保存图片数，硬上限 8 |
| plate_text_annotation_confirmed | false | 仅对车牌非空成功生效：操作者确认完整中文图像标注 |

要求 `1 <= min_support_frames <= min_valid_frames <= max_valid_frames <= 128`；观察时间不超过超时；拒绝未知参数、非法范围。CPU 较慢时在 person/plate 中分别增大采集超时和合理的源帧年龄，不能通过重复旧帧补足数量。

其他 launch args 同时作为节点私有参数：

| 参数 | 默认值 | 含义 |
|---|---|---|
| task_config_file | 包内 `config/task_config.yaml` | 任务配置文件 |
| results_root | `/workspace/car_2026/results` | 必须是绝对路径 |
| max_cache_frames | 24 | 每路 JSON/图像各自的条数上限 |
| max_cache_age_sec | 10.0 | 缓存墙钟保留时间 |
| max_cache_bytes | 67108864 | 两路缓存图像总字节上限（64 MiB），不得小于单图 8 MiB 限额 |

活动任务另外保留最多 max_valid_frames 条 JSON 和最多 max_evidence_images 张候选图；结果中保存的历史任务数据随任务数增长。缓存不足/图像太大只会明确失败，不会偷偷替换图片。

P1 参数：`waypoints_file` 默认指向包内现有 `config/waypoints.yaml`、`move_base_action=/move_base`、`navigation_timeout=90`、`server_timeout=15`、`cancel_timeout=3`、`clock_timeout=5`、`retries=1`、`navigation_test_mode=false`。点级 navigation_timeout/retries 仍优先。

## 融合与统计口径

人员：按相同类别，通过 IoU 或中心距离做窗口内一对一贪心短时匹配；达到 min_support_frames 的轨迹计一个目标，不把每帧人数相加。融合 confidence 是支持帧均值。单帧误检被过滤；所有原始数组确实为空才是 EMPTY_VALID，低置信度或不稳定目标不冒充零人。此方法面向固定观察点，不是永久 ID；遮挡、交叉、运动、长时间消失再出现仍可能造成误关联/轨迹碎裂，需实际场景验证。

`person_summary.by_waypoint` 保留每点结果；`by_area` 按航点 area 归组（不是模型判定区域）；`total` 汇总整个已配置路线。仅 SUCCESS/EMPTY_VALID 可进入 counts，失败为 null。默认多个点的实体关系未知：`potential_duplicate=true`、`counts=null`，只有 `observed_counts_not_unique` 记录可用观察次数之和，**它不是街区唯一人数**。`coverage_complete` 仅表示配置中各观察点任务均成功，不证明物理街区已被完整覆盖。单一观察点也只代表该观察范围。

若已人工确认每个人只归一个观察点，可在自己的任务配置中填写（ID 必须对应实测路线，仅示意）：

```yaml
counting:
  non_overlapping_partitions_confirmed: true
  assignments:
    person_01: block01_west
    person_02: block01_east
```

每个人物点需有非空 area 和全局唯一分区名，同一分区只能有一个指定人物计数点；多个分区可属同一 area。额外不计数的途经点用普通 waypoint。配置确认后才允许相加，scope 为 `declared_non_overlapping_partitions`；这是操作者对视野/实体归属的保证，程序不会验证物理非重叠，也没有全局 ReID。存在失败点时总 counts 仍为 null，不把缺失值当零。

车牌：NFKC 标准化、去空白、英文字母大写，保留中文；按整串号码投票，不拼接字符。空间上只能有一个可关联的候选轨迹；同帧多个过阈值候选或窗口内多个空间轨迹均 AMBIGUOUS，不擅自挑最高置信度车辆。赢家需至少 min_support_frames 且占比 >= vote_fraction，无并列。confidence 为支持赢家原始结果的均值，支持数及全部原始结果保留。

保存的是 P2-A 原始标注话题图，不重绘“平均框”。目标的代表框/代表置信度与实际保存帧对应；若最佳帧图像缺失，可改选同一轨迹的其他支持帧，同时更新代表字段。图上数字是该帧置信度，融合均值另存 confidence；原图可能包含被融合过滤的单帧候选，不能把图片框数直接当最终统计。`support_observations` 和 `raw_frames` 可逐项追溯这一差异。

## 状态与重试

| TASK status | 意义 / 行为 |
|---|---|
| SUCCESS | 有稳定结果且同源标注图齐全；车牌还需中文标注确认 |
| EMPTY_VALID | 足够有效帧全部为空且有同源图；人员可计 0 |
| FAILED_TIMEOUT | 超时仍无足够有效新帧，包括模型退出、断流、持续重复/陈旧帧 |
| FAILED_INVALID | 帧数不足且窗口内收到无效 JSON/无效人物状态 |
| FAILED_UNSTABLE | 非空但低置信度或支持不足 |
| AMBIGUOUS | 多个空间车牌候选，或全文投票无法唯一确认 |
| FAILED_IMAGE_MISSING | 原判定保留为 decision_status；匹配图缺失或图片限额无法覆盖目标 |
| FAILED_IMAGE_ANNOTATION | 车牌文本已融合，但完整中文图片标注尚未确认；保留文本和图片供检查 |
| FAILED_INTERRUPTED | 关闭、时钟失效、图片编码/写入等异常；持久化后由主程序进入 ERROR |
| SKIPPED_NAVIGATION_TEST | 显式纯导航模式；未执行识别，不计零 |

SUCCESS/EMPTY_VALID 不重试；FAILED_IMAGE_ANNOTATION 需修正字体/确认，不在同一次运行内重试；其他普通失败有限重试后继续下一点。每次尝试有不同 execution ID 和独立记录，汇总只取该点最后一次尝试，绝不把重试结果累加。整轮 FINISH 表示路线流程结束，不等于每次识别成功。异常退出尽量 ERROR；强制杀进程/断电时最后文件可能仍为 RUNNING、pending_task 非空，不自动恢复执行，已完成结果仍可读取。

## 结果 schema（版本 1）与持久化

默认宿主机目录为仓库 `car_2026/results/<UTC时间_随机ID>/`，容器为 `/workspace/car_2026/results/<同ID>/`，每轮独立目录，不覆盖旧结果；挂载保持时容器重建不会删除它。自定义 results_root 时需自行确保对应持久挂载。生成结果已被 Git 忽略，不自动清理历史轮次，应按需归档。

- `mission_results.json`：schema_version、run_id、status、started_at_utc、waypoint_order、waypoint_executions（顺序/坐标/导航状态/耗时）、tasks（按 recognition_order 的尝试列表）、pending_task、counting_policy、person_summary、plates、mission_duration_sec、error。
- 每个 tasks 项固定含 task_execution_id、recognition_order、run_id、waypoint_id、task_type、area、attempt、status、reason、valid_frames、detections、plates、raw_frames、evidence、image_missing、task_duration_sec、settings。窗口开启后有 window_start（整数 ros_ns、进程内 wall_monotonic）；正常采集后有 diagnostics。进行过融合时有 decision_status、uncovered_evidence_groups；有文字投票时有 votes。
- detections 项含 class_id/label/display_label、confidence（融合均值）、support_frames、bbox_xyxy、representative_frame_index、support_observations。plates 项含 text（标准化全文）、raw_text、confidence、support_frames、vote_fraction、plate_type_id、bbox_xyxy、representative_frame_index、support_observations。完成证据选择后有 evidence_image_path（缺失为 null）和已配图的 representative_confidence。
- raw_frames 保留来源 JSON；数组下标就是 frame_index。evidence 项含 source_header、frame_index、image_path（相对本轮目录）、annotated_topic_image=true、contains_all_raw_frame_detections=true；文件为 `images/<task_execution_id>_<task_type>_<序号>.jpg`。
- person_summary 含 by_waypoint、by_area、total、note；分组 counts 与 observed_counts_not_unique 都以 community_count/non_community_count 表示；counts 可为 null。顶层 plates 是按路线顺序的各车牌点最后状态，recognized_plates 可能保存失败候选，必须结合 status 判断，不能把候选当已验收成功。
- `mission.log`：逐行 UTF-8 JSON，event/elapsed_sec/data。TASK_RESULT 的 data 与任务 JSON 和 ROS 日志共用一个判决对象；记录状态转换和最终汇总。

如下为**示意节选，省略了其他固定字段，不是实测识别结果**：

```json
{
  "schema_version": 1,
  "status": "FINISH",
  "tasks": [{
    "waypoint_id": "person_01",
    "task_type": "person",
    "area": "block_01",
    "status": "FAILED_TIMEOUT",
    "valid_frames": 0,
    "detections": [],
    "raw_frames": [],
    "evidence": []
  }],
  "person_summary": {
    "total": {"coverage_complete": false, "counts": null}
  }
}
```

任务开始即保存 pending_task，结束立即原子保存整个 JSON；采用同目录临时文件、flush/fsync、os.replace，失败保留上一版。JPEG 同样原子写入，每次尝试最多 max_evidence_images 张，不逐帧落图。FINISH/ERROR 再保存汇总；文件系统故障只能尽力保存，不能承诺磁盘不可写或断电时最后一个事件一定落盘。

## Docker 构建和可重复自动检查

宿主机只执行 Docker 入口；以下 ROS/Python/catkin 都在容器内运行。本次不要求重建镜像，现有系统依赖和隔离环境可用。

```bash
docker exec -it --user developer smart-community-dev bash
```

进入后：

```bash
source /opt/ros/noetic/setup.bash
cd /workspace/car_2026
catkin_make -j2
source devel/setup.bash
bash src/robot_competition/tests/run_docker_checks.sh
PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages \
  /home/developer/.venvs/robot-perception/bin/python src/robot_perception/tests/test_adapters.py
```

脚本拒绝使用已经占用的 11329 端口，创建独立 ROS Master，假导航 `/p1_fake_move_base`，临时结果置于容器 /tmp。合成图片只用于接口测试，不代表实际模型的标注效果。测试不连接真实 `/move_base`，不更改真实 Master 参数，不需要模型或相机。

## 四终端正式联调（用户后续执行）

每个终端先各自进入容器并加载环境：

```bash
docker exec -it --user developer smart-community-dev bash
source /opt/ros/noetic/setup.bash
cd /workspace/car_2026
source devel/setup.bash
```

终端 1：`roslaunch robot_gazebo simulation.launch rviz:=true`

终端 2：`roslaunch robot_navigation navigation.launch`

先在 RViz 完成 AMCL 初始定位，确认激光对齐。按 [README 航点录入](README.md) 准备已有安全路线的复制文件，至少两个普通点、一个 person 点、一个 plate 点；相机在任务点应能看清目标。此阶段选择**无交通灯任务且路径不需灯控放行**的实测路线，不能把示例坐标用于自动行车。

终端 3（使用 P2-A 既有参数）：

```bash
roslaunch robot_perception perception.launch person_device:=cpu show_images:=false
```

中文标注：当前容器检查未发现 P2-A 默认候选 WQY/Noto CJK 字体。准备有使用授权的中文字体到挂载目录，例如 `/workspace/car_2026/fonts/Chinese.ttf`（这里是用户提供文件的示例位置，仓库不捆绑字体），将终端 3 命令改为：

```bash
roslaunch robot_perception perception.launch person_device:=cpu show_images:=false \
  plate_font_path:=/workspace/car_2026/fonts/Chinese.ttf
```

确认 `/perception/plates_image` 中实际绘出完整中文号码和置信度后，在自用 task_config YAML 的 plate 部分设置 `plate_text_annotation_confirmed: true`。这是持久配置确认，不是任务中人工触发推理；主程序不会 OCR 检查字体。未确认保持 false，会保存候选但返回 FAILED_IMAGE_ANNOTATION，不能把 plate# 当满足主办方要求。

例如先复制配置（容器内），再编辑自己的副本：

```bash
cp src/robot_competition/config/task_config.yaml /workspace/car_2026/task_config_local.yaml
```

终端 4（将路线文件替换为自己已实测的文件）：

```bash
rosrun robot_competition waypoint_manager.py /workspace/car_2026/route_measured.yaml
roslaunch robot_competition competition.launch \
  waypoints_file:=/workspace/car_2026/route_measured.yaml \
  task_config_file:=/workspace/car_2026/task_config_local.yaml \
  results_root:=/workspace/car_2026/results \
  navigation_test_mode:=false
```

`competition.launch` 不启动第二套仿真、导航或视觉节点。现有节点已运行时不要重复启动对应终端。任何时刻只运行一个主控制器；此时不发送 RViz Goal/遥控。无人为逐点触发识别步骤。

## 查看结果与手动故障验收

任务终端输出 TASK_RESULT 和 MISSION_FINISH/ERROR，并打印本轮结果目录。可在 RViz 添加两个 Image 显示，分别选 `/perception/person_image`、`/perception/plates_image`；或额外容器终端分别运行：

```bash
rosrun image_view image_view image:=/perception/person_image
rosrun image_view image_view image:=/perception/plates_image
```

第二条应在另一个终端运行。将日志打印的真实目录赋给 `RUN_DIR`，查看结果：

```bash
RUN_DIR=/workspace/car_2026/results/替换为本轮目录名
python3 -m json.tool "$RUN_DIR/mission_results.json"
tail -f "$RUN_DIR/mission.log"
```

人数查 person_summary.by_waypoint/by_area/total；车牌查顶层 plates 与 tasks。核对图片时从目标 evidence_image_path 找文件，确认 evidence.frame_index 对应 raw_frames 的 header.stamp/frame_id；代表框、代表置信度从该帧 support_observations 核对。JPEG 自身不存 ROS Header，以结果 JSON 作为关联凭据，seq 不参与匹配。

手动失联分支：在已确认安全的联调中停止终端 3，下一观察窗口应 FAILED_TIMEOUT（人物先前可能发无效通知则 FAILED_INVALID），有限重试后继续后续普通点，旧图不能计入。空场景测试需视觉节点继续运行并真实发布足量新空数组及对应图，才可 EMPTY_VALID；人物 frame_valid=false 不能计零。暂停仿真超过 clock_timeout 应 ERROR，不发后续目标。以上步骤是待用户执行的验收方法，不表示已进行真实 Gazebo 验收。

## 自动验证记录与下一阶段边界

2026-10-09，在既有 smart-community-dev 容器、developer 用户下完成：

| 检查 | 结果 |
|---|---|
| `catkin_make -j2` | 成功，退出码 0 |
| P1 `test_p1.py` | 21 项通过 |
| P2-B `test_p2b.py` | 32 项通过 |
| P2-A `test_adapters.py` | 5 项通过 |
| 六个主程序/辅助脚本语法、rosrun 入口、默认空路线、launch 解析 | 通过；默认 launch 仅列出 main_controller |
| 系统环境 | Python 3.8.10 / OpenCV 4.2.0 / NumPy 1.17.4 可用 |
| `git diff --check`、results 忽略规则 | 通过 |

构建仍有既有 Gazebo 导出变量 CMake 警告，未阻塞构建。容器测试日志为 `/tmp/p2b-build.log`、`/tmp/p2b-checks.log`、`/tmp/p2b-adapters.log`，不纳入仓库。

覆盖新帧纳秒边界、重复/倒序/未来/上一窗口、无效消息和图像、缓存边界、异步配图与 seq 差异、代表图片回退、稳定人数、区域重复风险、车牌 Unicode/冲突/多车、空帧/断流/低置信度、超时重试、编码失败保留旧结果、原子写入故障、TASK 时钟重置/暂停/关闭、识别失败继续导航、导航失败停止、P1 红绿灯正式拦截及显式测试模式。原有 P1 21 项、P2-A 适配器 5 项均需通过。

**未进行真实 Gazebo 验收**，未运行真实模型识别效果测试、未测完整实景街区覆盖或全局唯一人数，也未确认实际中文车牌图片效果。没有把合成结果标成实测结果。

上述记录属于 P2-B 阶段。当前 P2-C 在交通灯点导航成功后进入 WAIT_TRAFFIC，消费现有分类 JSON 并执行连续绿灯确认；stop_before_line 检查保留。navigation_test_mode 默认 false，只能用于隔离纯导航测试。最新启动流程见 [工作空间说明](../../README.md)。
