# 网页工作台（Workbench）方案

> 文档位置：`robot_ws/docs/WORKBENCH.md`。本文档只描述设计与分工，尚未实施。实施前请再次 `pull` 远程最新代码。

## 1. 背景与目标

当前 `robot_ws/tools/` 已经有一组 Python 网页脚本：

| 脚本 | 用途 | 端口 |
|---|---|---|
| `camera_web_preview.py` | 相机原始画面预览（ROS/DJI/Orbbec/红外） | 8765/8766 |
| `detector_preview.py` | 单模型检测预览，支持 `--trash` 完整垃圾分类流程 | 8080 |
| `collect_dataset.py` | 图片/录像采集 | 8081/8082 |

实际使用时的痛点：

- 每个工具独占一个端口，页面间不联动，无法同时看相机 + 任务状态；
- `roslaunch hardware / navigation / services / mission` 还是要多开 SSH 终端手打，参数（`base_protocol`、`navigation_only`、`obstacle_mode`、`single_waypoint`、`map`、`navigation_map` …）容易打错；
- 识别结果（人员/垃圾分类/红绿灯/障碍物/火焰/温度/车牌/倒伏/选路）目前只落盘到 `~/.ros/community/<run_id>/`，调试和上场时看不到实时反馈；
- 定位、2D Pose Estimate、2D Nav Goal 必须回到本地 RViz；
- 工具以脚本形式启动，断开 SSH 后进程状态不好查。

**目标**：在小车端新增一个统一的 Web 工作台 `robot_ws/tools/workbench/`，浏览器访问 `http://小车IP:8000/`，一站式完成：

- **一键启停** 常用 `roslaunch` 与 `shell` 脚本（hardware / navigation / services / mission / 建图 / 保存地图 / 单模型预览 …）；
- **总览**：节点、硬件心跳、电量/CPU/温度、巡检状态；
- **遥控与地图**：虚拟摇杆发 `/cmd_vel`，canvas 看地图+AMCL 位姿+costmap+激光，鼠标点击 `2D Pose Estimate` / `2D Nav Goal`；
- **相机 + 检测**：任意相机源 × 任意检测器切换预览，**视觉识别结果可视化**：画框、置信度、类别、ROI、垃圾匹配连线、楼层温度热图、红绿灯状态、选路箭头等；
- **巡检任务**：启动完整/单点巡检，切换 `navigation_only / obstacle_mode`，实时显示当前航点、任务状态、`/mission/state`，直接浏览本次 `run_id` 的 `result.json`/照片/视频。

原有的 `camera_web_preview.py`、`detector_preview.py`、`collect_dataset.py` 作为离线调试备份保留，不删除、不改行为；工作台内部通过 `from workbench.camera_core import ...` 复用其核心类。

## 2. 技术栈

- 后端：**FastAPI + uvicorn**（Python 3），`async` 适合 WebSocket + MJPEG 并发；进程管理 `subprocess.Popen` + `preexec_fn=os.setsid`。
- ROS 桥：复用项目已依赖的 `rospy` + `cv_bridge`，不引入 `rosbridge_suite`，节点名 `workbench_node` 向 ROS Master 注册。
- 前端：**原生 HTML + ES module + Canvas**，不引入 npm/Vite/React；视频用 `<img src="/api/stream.mjpeg?...">`，状态用一条 WebSocket，画框等可视化叠加由后端直接画在 MJPEG 帧上。
- 依赖追加：`fastapi`、`uvicorn[standard]`、`psutil`（`requirements.txt`）；保持 `numpy<2`。

### 2.1 参考实现（U 盘 `F:/vla-infra/`）

| 参考文件 | 借鉴点 |
|---|---|
| `web/services/console/backend/ros_bridge.py` | 单一 ROS 桥对象集中管理订阅/发布/服务 |
| `web/services/console/backend/status_hub.py` | 5 Hz 快照聚合 + WebSocket 广播；我们用 2 Hz |
| `web/services/console/backend/stack.py` | 独立 session 启动脚本，`SIGINT → SIGTERM → SIGKILL` 级联停止；日志 `logs/<name>_<ts>.log` |
| `web/services/console/run.sh` | pidfile + "只杀本仓库自己的 pid" 端口归属规则 |
| `web/services/console/frontend/App.tsx` | WS 自动重连 + 多卡片布局；用原生 JS 复刻 |
| `methods/*/web_demo/templates/index.html` | Flask + 单文件 HTML + MJPEG `<img>` 自刷新，适合相机标签页 |

React/Vite 对本项目过重，不采用；只把其布局和交互模式对齐到我们原生 JS 版。

## 3. 目录结构

### 3.1 新增：工作台本体

```
robot_ws/tools/workbench/
├── server.py              # uvicorn 入口，组装 FastAPI app、启停信号
├── ros_bridge.py          # rospy 订阅/发布/服务；写入 state_bus；提供发布接口
├── state_bus.py           # 进程内 asyncio 事件总线（state 快照 + 日志行）
├── launch_manager.py      # roslaunch / shell 子进程登记、启停、日志、健康
├── overlay.py             # 画检测框、ROI、匹配连线、温度热图、选路箭头的工具
├── camera_core.py         # 从 tools/camera_web_preview.py 抽出的 Preview / 取帧循环
├── detector_core.py       # 从 tools/detector_preview.py 抽出的 YoloDetector / 垃圾分类流水线
├── config.py              # 端口、安全、模型默认值、launch 清单
├── routes/
│   ├── __init__.py
│   ├── dashboard.py       # GET /api/health、WS /api/state
│   ├── control.py         # POST /api/launch/{name}/{action}、WS /api/teleop、GET /api/logs/{name}
│   ├── mission.py         # POST /api/mission/*、GET /api/mission/runs、/api/mission/runs/{id}/file
│   ├── nav.py             # GET /api/nav/map.png、POST /api/nav/pose_estimate、/api/nav/goal、/api/nav/closure
│   └── camera.py          # GET /api/camera/sources、/api/detectors、/api/stream.mjpeg、POST /api/snapshot
├── web/
│   ├── index.html         # 顶部 Tab：总览 / 遥控+地图 / 相机检测 / 巡检任务
│   ├── app.js             # WS + fetch + canvas
│   └── style.css
├── tests/
│   ├── test_launch_manager.py
│   ├── test_state_bus.py
│   └── test_overlay.py
├── run.sh                 # 启停与状态；仿 vla-infra run.sh
└── README.md              # 仅本工具的启动与使用说明
```

### 3.2 新增：launch 入口与 shell 脚手架

| 新增文件 | 作用 |
|---|---|
| `src/community_bringup/launch/workbench.launch` | 启动 `workbench_node`，参数 `port:=8000`、`bind:=0.0.0.0`、`readonly:=false` |
| `src/community_bringup/launch/field.launch` | 组合入口，等价"开机即跑完整巡检"：依次 hardware + navigation + services + mission；参数全部透传，内部复用 `community.launch` + `mission.launch` |
| `src/community_bringup/launch/debug_perception.launch` | **仅识别调试**：启动相机 + 任选一个识别 launch，不启动底盘，支持 `detector:=people_detector\|traffic_light_detector\|fire_detector\|obstacle_detector\|trash_bin_detector\|waste_detector\|bicycle_detector`，内部 `include` 对应现有 `*_detector.launch` |
| `tools/start_workbench.sh` | 一行启动：`source` ROS 环境 → 拉起 `workbench.launch` → 打印访问地址 |
| `tools/start_field.sh` | 一行开场：等价 `field.launch base_protocol:=aa_bb`；允许 `--navigation-only` / `--clear` / `--blocked` |
| `tools/save_map.sh` | 封装 `map_server map_saver`，默认写到 `~/maps/community_<ts>.yaml` |
| `tools/kill_ros.sh` | 应急：`rosnode kill -a` + 杀残留 python/rviz（谨慎，仅供工作台"紧急停止"按钮调用） |

所有 shell 脚本：`set -euo pipefail`，脚本头部加帮助，统一加载 `source /opt/ros/noetic/setup.bash` 和 `/home/mowen/cartographer_ws/install_isolated/setup.bash`、`devel/setup.bash --extend`，与 `robot_ws/README.md` "终端环境" 一节一致。

### 3.3 最小增量修改

- `src/community_mission/scripts/mission_node.py`：在 `Mission` 类状态切换处额外 `publish` 一次 JSON 到话题 `/mission/state`（`std_msgs/String`）；字段 `state / index / current_waypoint / reason / navigation_only / obstacle_mode / run_id / last_detection`。仅加一个 publisher + 若干行 publish，不改控制逻辑。
- `src/community_perception/scripts/yolo_detector_service.py` 等：在响应 `ObjectDetection` 服务时**额外向工作台订阅的 `/perception/debug` 话题广播一次最近检测框**（`vision_msgs` 不可用则用 `std_msgs/String` 装 JSON），**仅当参数 `~publish_debug` 为真才启用**，默认关闭，巡检时不影响性能。工作台启动时通过 `rosparam set` 打开。
- `robot_ws/tools/camera_web_preview.py`、`detector_preview.py`：顶部改为 `from workbench.camera_core import Preview`、`from workbench.detector_core import YoloDetector`；函数体保留不变。两个脚本原有命令行行为完全不变。
- `robot_ws/requirements.txt`：追加 `fastapi`、`uvicorn[standard]`、`psutil`。

## 4. 页面模块

### 4.1 总览 Dashboard

单一 WebSocket `/api/state`，后端 2 Hz 推一次 JSON：

```json
{
  "ros": {"master": true, "nodes": ["/amcl", "/move_base", ...]},
  "topics": {
    "/odom": {"hz": 20.1, "age_s": 0.03},
    "/scan": {"hz": 10.0, "age_s": 0.05},
    "/imu":  {"hz": 100.0, "age_s": 0.01},
    "/usb_cam/image_raw": {"hz": 29.7, "age_s": 0.04},
    "/amcl_pose": {"hz": 2.0, "age_s": 0.2}
  },
  "pose": {"x": 0.41, "y": 0.41, "yaw": 0.0, "frame": "map"},
  "host":  {"cpu": 32.0, "mem": 48.1, "disk": 71.3, "cpu_temp_c": 58.3, "battery_v": null},
  "launches": [
    {"name": "hardware",  "status": "running",  "pid": 12345, "args": {"base_protocol": "aa_bb"}, "since_s": 325},
    {"name": "mission",   "status": "exited",   "code": 0,    "since_s": 10},
    ...
  ],
  "mission": {"state": "NAVIGATING", "index": 5, "waypoint": "wp_05",
              "navigation_only": false, "obstacle_mode": "blocked",
              "run_id": "20261117_210643",
              "last_detection": {"name": "person", "count": 2, "boxes": [...]}}
}
```

卡片分区：ROS 节点、心跳话题（红/黄/绿按 `age_s` 着色）、硬件指标（折线图用 canvas 画，保留最近 60 s）、巡检状态、Launch 列表。顶部有 **"紧急停止"**：按一下 → `POST /api/estop` → 停 mission、停 move_base、发零速 `/cmd_vel`、停相机录像；二次确认后才执行。

### 4.2 遥控 + 地图

- `GET /api/nav/map.png?layer=localization|navigation`：后端订阅 `/map`、`/navigation_map`，`OccupancyGrid → PNG`，返回时附带 `X-Map-Origin: x,y,yaw` + `X-Map-Resolution: 0.02` 头；前端 canvas 加载。
- 叠加（canvas 多层）：
  - 机器人位姿、车体轮廓（读取 `footprint`）；
  - `/move_base/global_plan`（蓝线）、`/move_base/local_plan`（绿线）；
  - `/move_base/*_costmap/costmap_updates`（半透明热力）；
  - `/scan` 投影点（红点，衰减）；
  - 航点（来自 `route.yaml`，带 ID 标签），已到达的打绿勾、未到的灰、当前点闪烁。
- 鼠标工具：
  - **Pose Estimate** 模式：点击一次定位，拖动设朝向，松开 → `POST /api/nav/pose_estimate {x,y,yaw}` → 发布 `/initialpose`；
  - **Nav Goal** 模式：同理 → `POST /api/nav/goal` → 发布 `/move_base_simple/goal`；
  - **选航点** 下拉：选 `wp_XX` → `POST /api/mission/single {waypoint, navigation_only}`。
- 遥控：WebSocket `/api/teleop`，前端屏幕虚拟摇杆 + 键盘（WASD/IJKL，与 `teleop_twist_keyboard.py` 一致），后端限幅发布 `/cmd_vel`：线速 ≤ 0.25 m/s、角速 ≤ 0.3 rad/s（与 DWA 配置一致，常量复用）；心跳 500 ms 无指令或 WS 断 → 发零速 + 停 1 s 不再发，避免冲突。

### 4.3 相机与检测（**视觉识别结果可视化**，重点）

- **相机源**：下拉可选 —— ROS 话题（`/usb_cam/image_raw`、`/thermal_camera/image_raw`、`/building_d_temperature/image` 以及运行时从 `rostopic list` 找到的 `sensor_msgs/Image`）、`dji`、`orbbec`。
- **检测器**：下拉从 `detectors.yaml` 加载所有条目，外加 `__none__`、`__trash_pipeline__`、`__traffic_light_state__`、`__thermal_building_d__`、`__fall_count__`、`__license_plate__` 六个"组合调试模式"。
- **可视化内容**（由 `overlay.py` 在后端画到 MJPEG 帧上，避免前端多次绘制）：
  | 模式 | 叠加 |
  |---|---|
  | 单模型 `people/fire/obstacle/bicycle/traffic_light/trash_bin/waste` | 绿框 + 类别 + 置信度，右上角计数 |
  | `__trash_pipeline__` | 黄色 ROI + 桶框 + 垃圾框 + 匹配连线；颜色：绿=对、红=错、橙=未确认；底部中文判断 |
  | `__traffic_light_state__` | 画框 + 大字"STOP / GO"、连续帧计数、超时倒计时 |
  | `__thermal_building_d__` | 四分区、每分区最高温度、超温确认帧计数、状态色（正常/确认中/异常/无数据/超时） |
  | `__fall_count__` | 跟踪 ID、站/倒状态、本地累计倒伏数 |
  | `__license_plate__` | 车牌框 + 识别出的字符（PaddleOCR）+ 置信度 |
  | `obstacle_detector` + `wp_04` 预览模式 | 画框 + 中心 x 比例 + 预测分支（blocked/clear）箭头 |
- **流端点**：`GET /api/stream.mjpeg?source=...&detector=...&fps=10&conf=0.5&iou=0.4`；后端维护一个 `(source, detector, params)` → `Preview` 的缓存，多浏览器共享同一帧缓冲；同配置的第二个连接直接 `fan-out`。
- **快照**：`POST /api/snapshot?source=...&detector=...` → 保存到 `~/.ros/community/workbench_snaps/<ts>.jpg`；返回路径供前端 `<a download>`。
- **"识别结果看板"**：相机 Tab 下方一个侧栏，以 WebSocket `/api/perception` 实时显示所有识别服务的最近调用历史（航点、模型、框数、是否成功），行为与 `services.py` 的 `log_detection` 一致；点击某条可复现画面（后端缓存最后 N 张）。

### 4.4 巡检任务

- **Launch 面板** —— 所有 launch 以卡片形式列出，参数可编辑，操作：启动 / 停止 / 看日志尾部 400 行 / 下载完整日志：
  - `hardware.launch`（`base_protocol`、`base_port`、`imu_port`、`camera_device`、`configure_camera`）
  - `mapping.launch`（`algorithm:=cartographer|gmapping`、`rviz`）
  - `navigation.launch`（`map`、`navigation_map`、`local_planner`、`route`）
  - `services.launch`
  - `mission.launch`（`navigation_only`、`obstacle_mode`、`single_waypoint`、`route`）
  - `community.launch` / `finals.launch` / **新 `field.launch`**（`base_protocol`、`map`、`navigation_map`、`route`）
  - `debug_perception.launch`（`detector`、`camera_device`）
  - `thermal_camera.launch` / `building_d_temperature.launch`
- **Shell 面板** —— 快捷按钮：`save_map.sh`（弹对话框填文件名）、`start_workbench.sh`（本身，不可点）、`kill_ros.sh`（红色按钮，两步确认）、`record_waypoints.py`（指南而非一键）。
- **巡检状态**：订阅 `/mission/state`（新增的 JSON 话题）——用时间线显示每个航点 `NAVIGATING → RECOGNIZING → DONE/FAILED`；当前检测结果即时显示在右侧。
- **结果浏览**：扫描 `~/.ros/community/`，按 `run_id` 列表；进入某个 run 显示 `result.json` 表格、`photo/` 缩略图、`videos/` 下载链接；支持下载整个目录（`tar.gz`，流式输出，注意 `AGENTS.md` "不提交无关临时文件"，此处只是下载到本地，不入 git）。
- **手动服务**：`/navigation_map/set_blocked`、`/navigation_map/reset` 按钮，用于不启动 mission 的现场调试。

## 5. launch_manager 行为契约

- 启动：`Popen([..., 'roslaunch', pkg, launch_file, *args], preexec_fn=os.setsid, stdout=log, stderr=log, env=ros_env())`；记录 `pid/pgid/args/start_ts/log_path`。
- 停止：`os.killpg(pgid, SIGINT)` → 等 10 s → 再 `SIGTERM` → 等 5 s → `SIGKILL`。记录每一步与最终 exitcode。
- 已在运行时拒绝重复启动，返回当前 pid 与 args。
- 工作台进程 `atexit` + `signal(SIGINT/SIGTERM)` 向所有子进程组发 `SIGINT`，保证 Ctrl+C 不留残留。
- 日志：`~/.ros/community/workbench/<launch>_<ts>.log`；网页 tail 读文件末尾 400 行；下载走 `GET /api/logs/{name}/download`。
- 页面刷新或断网不影响 launch —— 唯一会因 WS 断开的副作用是 teleop 自动发零速。

## 6. 安全与权限

- 默认 `bind=0.0.0.0`、无登录（仿现有工具风格，仅内网）；`--host 127.0.0.1` 或 `WORKBENCH_READONLY=1` 用于只读副本（写接口统一 409）。
- "紧急停止"按钮、停 hardware、`kill_ros.sh`：前端二次确认。
- 遥控：WS 心跳 500 ms，超时发零速并暂停 1 s。
- `cmd_vel` 严格限幅（线速 0.25 / 角速 0.3，与 DWA 一致）。
- 遥控/导航/launch 写接口要求 `X-Workbench-Token` header，token 从 `WORKBENCH_TOKEN` 读；为空时打印警告但放行，保持开箱即用。

## 7. 启动与使用（实施后）

```bash
# 小车 SSH：
ssh mowen
source /opt/ros/noetic/setup.bash
source /home/mowen/cartographer_ws/install_isolated/setup.bash
cd ~/workspace/Smart-Community/robot_ws
source devel/setup.bash --extend

# 启动工作台
roslaunch community_bringup workbench.launch     # 或 bash tools/start_workbench.sh

# 浏览器
http://<小车IP>:8000/
```

常用场景：

1. **纯调试识别**：`workbench` → Launch 面板点 `debug_perception.launch detector:=waste_detector` → 相机 Tab 选 `__trash_pipeline__` → 看匹配连线和中文判断。
2. **只建图**：`workbench` → `hardware.launch` → `mapping.launch` → 遥控 Tab 开 teleop 建图 → `save_map.sh`。
3. **上场完整巡检**：`workbench` → `field.launch base_protocol:=aa_bb` → 任务 Tab 看状态 → 结果 Tab 看 `result.json` 和照片。

## 8. 验证

### 8.1 单元

- `tests/test_launch_manager.py`：`fakeroot` 下用 `sleep 10` 模拟 launch，验证启动、SIGINT 停止、SIGKILL 升级、重复启动拒绝、pgid 清理。
- `tests/test_state_bus.py`：多订阅者 fan-out、慢订阅者 drop oldest（队列上限 8）、`close` 释放。
- `tests/test_overlay.py`：给定固定检测结果，叠加输出的 JPEG 校验尺寸 + 存在 `cv2.rectangle` 像素签名（非 0 像素数阈值）。

### 8.2 本地（无硬件）

```bash
roscore
rostopic pub /odom nav_msgs/Odometry ...  # 任意伪造心跳
python3 tools/workbench/server.py --port 8000
```

浏览器核对：状态推送、MJPEG（走 USB 相机）、launch 启停（用 `sleep` 做假 launch）、teleop 发 `/cmd_vel`。

### 8.3 实车

1. SSH mowen → 加载环境 → `roslaunch community_bringup workbench.launch`；
2. 本机打开 `http://192.168.31.59:8000/`；
3. 启 `hardware.launch base_protocol:=aa_bb`，看 `/odom`、`/scan`、`/imu` 全绿；
4. 启 `navigation.launch`；地图 Tab 画布显示地图+激光对齐，`2D Pose Estimate` 校准 → `2D Nav Goal` 走一段；
5. 相机 Tab 依次切 `people_detector`、`__trash_pipeline__`、`__thermal_building_d__` 看叠加是否正确；
6. 启 `mission.launch navigation_only:=true obstacle_mode:=clear`，任务 Tab 追踪 `/mission/state`；
7. 结束后结果 Tab 打开当前 `run_id`，核对 `result.json`、照片、视频。

实车识别准确率、定位稳定性、完整巡检效果**必须在现场验证**，本工作台不替代实车验收。

## 9. 不做 / 以后再做

- **用户认证 / HTTPS**：仅内网使用，暂不做；若要对外演示再加 nginx + basic auth。
- **移动端自适应**：桌面优先，手机可看但不保证操作体验。
- **多小车聚合**：只针对单台小车；不做多车汇总看板。
- **与仿真工作空间联动**：`simulation_ws` 仍用各自 launch，不接入本工作台（符合 `AGENTS.md` "优先开发实车程序，除非明确要求，否则不修改仿真工作空间"）。
- **历史数据分析**：只提供文件浏览与下载，不做离线数据统计。

## 10. 实施拆分（一旦方案最终确认）

| 批次 | 内容 | 预计产出 |
|---|---|---|
| 1 | 骨架：`workbench/` 目录、`server.py`、`state_bus`、`launch_manager`、`run.sh`、空的 tab 页、`workbench.launch`、`requirements.txt` | 工作台能启动、能启停任一 launch |
| 2 | `ros_bridge` 心跳订阅 + Dashboard + Pose/Goal + 地图图层 | 总览、地图两个 tab 可用 |
| 3 | `camera_core` + `detector_core` 抽取；相机 Tab 单模型可视化；`overlay.py` | 相机检测页可用 |
| 4 | `mission_node.py` 发 `/mission/state`；巡检 Tab、结果浏览；`field.launch`、`debug_perception.launch`、`save_map.sh`、`start_*.sh` | 全功能可用 |
| 5 | 组合识别模式（垃圾、红绿灯、温度、倒伏、车牌）可视化；紧急停止；只读模式；测试补齐 | 验收 |

每批次写完都可独立测试，不互相阻塞。
