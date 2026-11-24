# 实车 Web 工作台

浏览器中完成 launch 启停、状态与日志查看、地图遥控与导航、相机识别预览、巡检任务启停和结果浏览。后端为 FastAPI，前端为原生 HTML + ES module，没有前端构建步骤。

> 以下功能已在无 ROS 环境下做过接口和逻辑测试，**尚未在实车上联调**。首次上车请按文末「实车验证清单」逐项确认。

## 启动

按 [实车环境说明](../README.md#终端环境) 加载 ROS、Cartographer 和工作空间，首次部署先编译 ROS 包。在 `robot_ws/` 中安装 Python 依赖：

```bash
python3 -m pip install -r requirements.txt
```

推荐用脚本启动（内部执行 `roslaunch community_bringup infra.launch`，会自动拉起 roscore）：

```bash
tools/start_infra.sh                          # 0.0.0.0:8000
tools/start_infra.sh --port 9000 --bind 0.0.0.0 --token 口令 --readonly
```

也可以用 `bash infra/run.sh start|status|stop` 管理独立进程。这种方式需要先单独运行 `roscore`：工作台会等 master 出现后再连接 ROS，但不会重连到之后新启动的 master。

浏览器访问 `http://<小车IP>:8000/`。启用 Token 时，首次访问 `http://<小车IP>:8000/?token=口令`（会保存在浏览器中），或在提示框中输入。

| 环境变量 | 作用 |
|---|---|
| `INFRA_PORT` / `INFRA_BIND` | 监听端口和地址，默认 `8000` / `0.0.0.0` |
| `INFRA_READONLY` | `1` 启用只读模式：禁止启停、发布导航指令和遥控，急停除外 |
| `INFRA_TOKEN` | 写操作需携带的口令（HTTP 头 `X-Infra-Token`，WebSocket 使用 `?token=`） |
| `INFRA_RUNTIME_DIR` | 工作台日志和快照目录，默认 `~/.ros/community/infra` |
| `INFRA_MISSION_ROOT` | 巡检结果根目录，默认 `~/.ros/community` |

## 功能

顶部固定显示 ROS Master、机器人位姿、主机资源和连接状态。比赛就位后点击右上角「比赛一键启动」，会以本车的 `aa_bb` 协议启动 `finals.launch`，按默认比赛路线自动巡检。旁边的「紧急停止」可随时操作。

**总览**：话题频率（`/odom_raw`、`/odom`、`/wit/imu`、`/scan`、`/usb_cam/image_raw` 等）、launch 列表和实时日志。

点击「启动」会弹出参数表单。启动前会做以下检查，不满足时拒绝启动并提示原因：

- 必填参数：`hardware`、`community`、`finals`、`field` 必须选择 `base_protocol`（`aa_bb` 或 `legacy_11`）。
- 参数取值合法，地图和路线文件确实存在。
- 不会重复启动同一组件。例如 `hardware` 和 `field` 不能同时运行，`navigation` 和 `mapping` 不能同时运行。终端里手动启动的节点也会被检测到。
- 前置组件已经在运行。例如 `navigation` 需要先启动 `hardware`。

**紧急停止**：依次执行“发送零速度 → 取消全部 move_base 目标 → 停止 `/community_mission` 节点 → 再次发送零速度”。底盘驱动不会先被关闭。

需要注意，mission 节点在 launch 中设为 `required`。如果任务是由 `field` 或 `finals` 启动的，停止任务后整套 launch 会随之退出，底盘驱动退出时会再下发一次零速度。如果 ROS 未连接，急停会改为直接停止所有 launch。

**遥控 · 地图**：

- 地图：可显示 `/navigation_map` 或 `/map`，叠加全局和局部代价地图、激光点、全局路径（`/move_base/GlobalPlanner/plan`）、局部路径和机器人位姿。位姿优先取 TF `map→base_footprint`，取不到时用 `/amcl_pose`。
- 设置位姿和目标：选择「2D Pose Estimate」或「2D Nav Goal」后，在地图上按下确定位置，拖动确定朝向。
- 遥控：使用方向键或 W/A/S/D，限速为线速度 0.25 m/s、角速度 0.3 rad/s。松开按键、超过 0.5 s 没有指令或断开连接时，会自动发送零速度。导航运行时，遥控会与 move_base 争抢 `/cmd_vel`。

**相机 · 检测**：

- 选择「红外测温（楼宇 D）」会自动切到楼宇 D 分区热图，右侧显示高温、确认中及数据异常等状态。先启动「识别 / 语音 / 录像服务」以运行热像采集和测温节点。
- 选择「仪表盘识别（UGREEN 1080p）」会自动切到 UGREEN 图像，显示指针叠加、温度和湿度。先启动「底盘与传感器」以运行 UGREEN 采集节点。识别使用与 `wp_11` 相同的 4K 标定。
- 「车牌识别」通过车辆画面上的 OCR 显示车牌号码和置信度。将车牌放在画面中间，并启动「识别 / 语音 / 录像服务」。
- 「楼宇火灾识别」显示火焰框和数量，发现火焰后调用 OCR 读取同一画面中的楼名。楼名未识别或 OCR 不可用时仍显示火情。楼名和车牌的后台预览 OCR 结果会间隔更新，不自动保存照片。

- 处理流程：相机来源 → 帧缓存 → 检测器（ONNX） → 结果叠加 → MJPEG。
- 来源列表每 5 秒刷新，显示小车检测到的视频采集设备和已发布的 ROS 图像话题。选择来源或检测器后立即切换；置信度和 FPS 在确认输入后生效。
- 检测器来自 `community_perception/config/detectors.yaml`，另外还有一个「垃圾分类完整流程」。
- 「原始画面采集」可输入小车上的保存目录和文件名，拍摄 JPG 或录制 MJPEG AVI。留空使用默认 captures 目录和时间名，同名自动编号。录像会持续到点击停止或视频源异常；离开页面后可返回停止。空闲预览在离开页面后关闭。「保存快照」会保存叠加了识别结果的当前画面。

**巡检任务**：

- 启动和停止：可以启动完整巡检或单航点任务。前提是 `hardware` 和 `navigation` 已在运行，执行完整任务时还需要运行 `services`。
- 实时状态：显示 `/mission/state` 中的当前航点、任务类型、执行结果和错误信息。
- 巡检记录：只列出包含 `mission_results.json` 的目录，可以查看各航点和任务的结果、照片、视频，或打包下载。

「仅导航」对应 `navigation_only:=true`。勾选后会跳过**全部**航点任务，包括红绿灯等待、拍照识别和录像，小车只按顺序导航。wp_04 之后走哪条分支由 `obstacle_mode` 决定。

## 测试

在 `robot_ws/` 中运行：

```bash
python3 -m pytest infra/tests -q
```

测试内容包括状态总线（含跨线程日志）、launch 参数校验与组件冲突、启动管理器（需要 POSIX 环境）。

## 实车验证清单

1. 运行 `tools/start_infra.sh`，用 `aa_bb` 协议启动 `hardware`。总览页中 `/odom_raw`、`/wit/imu`、`/scan`、`/usb_cam/image_raw` 应显示为绿色。
2. 在 `hardware` 运行时再启动 `field`，应被拒绝。
3. 启动 `navigation`，确认地图、代价地图、激光点和位姿显示正确。用 2D Pose Estimate 定位后，发送一个近处的 2D Nav Goal，确认全局和局部路径正常显示。
4. 在导航过程中按「紧急停止」，确认小车立即停止，且 `hardware` 仍在运行。
5. 遥控：按住方向键时小车移动，松开后立即停止；关闭浏览器标签页后也应停止。
6. 在相机页切换到 `people_detector` 和「垃圾分类完整流程」，确认画面上叠加了识别框，并测试保存快照。
7. 以「仅导航」模式执行单航点任务，确认状态正常更新、任务结束后能看到巡检记录；再用「停止任务」测试中途停止。
