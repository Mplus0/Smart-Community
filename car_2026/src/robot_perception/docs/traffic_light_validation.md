# YOLO11n 交通灯接入与 P2-A/P2-B 回归准备

## 本轮执行状态（2026-10-09）

用户明确要求不要在宿主机运行代码或 Docker，因此后续只编辑文件。本轮未执行新增 Python、模型推理、语法/单元测试、catkin 构建、P2-A/P2-B 回归或 Gazebo 联合测试，不能宣称通过。

指令收窄前只读取了仓库、现有测试及启动文档，确认源模型 `models/traffic_light/best.pt` 存在（当时文件大小 3,201,364 字节），并触发 Docker Desktop 启动；未启动 ROS 容器、推理节点或 Gazebo。没有读取模型内部任务/类别，也没有完成模型二进制复制。以下均为**待执行命令和验收方法**。

人物/车牌节点与其模型、robot_competition 业务代码均未修改。历史 P2-A/P2-B 测试结果保留在原文档，但不能作为这次新增代码的验证证据。

## 启动前准备

1. 使用文件管理器将仓库 `models/traffic_light/best.pt` **复制**到 `car_2026/src/robot_perception/models/traffic_light_best.pt`，保留源文件。不使用 ONNX 或 ImageNet 预训练权重代替训练完成的 best.pt。此复制本轮尚未执行。
2. 现有 Compose 挂载 `car_2026` 到容器 `/workspace/car_2026`，因此复制后的包内文件可见；仓库顶层 `models/` 没有被该 Compose 挂载，不能直接把顶层宿主机路径传给节点。
3. 后续所有命令只在用户自行进入的既有 ROS 容器中执行，使用 developer 用户。不要在 Windows 宿主机执行 Python/测试，不新建推理环境，不替换系统 Python。
4. 若文件通过 Windows 编辑导致没有 Linux 执行位，回归脚本会对新增节点执行 `chmod +x`；这是 source-space roslaunch 所需。用户提交时需保留该可执行位。

```bash
source /opt/ros/noetic/setup.bash
cd /workspace/car_2026
test -s src/robot_perception/models/traffic_light_best.pt
chmod +x src/robot_perception/scripts/traffic_light_classification_node.py
sha256sum src/robot_perception/models/traffic_light_best.pt
/home/developer/.venvs/robot-perception/bin/python --version
```

仅复制到包内并不会提交模型到 Git。该权重不应默认加入普通 Git；如需版本管理，另行使用 Git LFS，勿无选择地 `git add .`。

## 自动回归（容器中手动执行）

```bash
cd /workspace/car_2026
set -o pipefail
bash src/robot_perception/tests/run_regression_checks.sh \
  2>&1 | tee /tmp/traffic-light-regression.log
```

脚本顺序：检查模型与脚本、catkin 构建、既有环境检查、原 P2-A 五项适配器测试、新交通灯测试、原 P1/P2-B 回归、默认双路及显式三路 launch 解析。P1/P2-B 脚本使用独立 Master 11329 与假 Action Server，不发送真实导航目标，已有占用会拒绝运行。

新增测试覆盖：精确上半图/奇数高度/ROI 参数/原图不变、全部类别顺序排列、错误任务与类别拒绝、224 CPU predict 参数、概率合法性、低置信度、Header 序列化不污染源图、输出原分辨率、JSON 字段、解码与推理异常、两种时钟过期、推理后过期、零/未来/重复/倒退时间戳、最新帧覆盖、限频及断流 UNKNOWN。

真实模型测试由 `TRAFFIC_LIGHT_MODEL` 显式启用，回归脚本自动传入包内权重；缺失会失败，不伪造测试通过。该测试只用合成图验证真实模型加载和 CPU 推理 API，**不是识别效果或分类准确率测试**。直接运行单元测试但未设此变量时，会明确 skip 真实模型测试。

```bash
source /opt/ros/noetic/setup.bash
cd /workspace/car_2026
TRAFFIC_LIGHT_MODEL="$PWD/src/robot_perception/models/traffic_light_best.pt" \
  YOLO_AUTOINSTALL=false YOLO_OFFLINE=true \
  /home/developer/.venvs/robot-perception/bin/python src/robot_perception/tests/test_traffic_light.py
```

## Gazebo 三路联合验收

复用已有 Gazebo；没有仿真时，由用户在容器另一终端启动 `roslaunch robot_gazebo simulation.launch`。此测试不需要比赛主程序或导航，不发布运动指令，不重复启动已有仿真/视觉节点。

已有相机、尚无视觉节点时，以下命令会启动三路、观察 30 秒、只结束本脚本启动的视觉 launch：

```bash
source /opt/ros/noetic/setup.bash
cd /workspace/car_2026
source devel/setup.bash
bash src/robot_perception/tests/run_live_check.sh \
  /home/developer/.venvs/robot-perception/bin/python true
```

报告为 `/tmp/p2a-live-traffic-true/report.json`，消息明细和各流末帧图片在同目录，launch 日志为 `/tmp/p2a-live-launch.log`。第二个参数省略或为 false 则仍是原双路测试，报告为 `/tmp/p2a-live-traffic-false/`；可先后运行两种模式对比资源和吞吐，不同时运行两套节点。

若三路已由用户启动，只运行观察器：

```bash
/home/developer/.venvs/robot-perception/bin/python \
  src/robot_perception/tests/observe_topics.py --traffic-light --seconds 60 \
  --output /tmp/traffic-light-observation
```

验收核对：

- 相机、三路 JSON 和三路标注图均有输出；默认 launch 仍只有人物与车牌。
- 记录各路实际 Hz、处理耗时均值/P95/最大值、源 ROS 时间年龄均值/P95、进程 CPU 均值/峰值及 RSS 峰值。CPU 按进程统计，可超过 100%；ROS 源年龄受仿真时间倍率影响，不应冒充墙钟端到端延迟。报告另有 `observer_camera_to_json_ms_mean/p95`：观察器收到同源相机帧至 JSON 的墙钟差，仅统计匹配且非负的记录，不包含相机发布前的采集延迟。
- 标注图分辨率等于源图，保留 stamp/frame_id。默认交通灯 ROI 框为 `[0,0,width,height//2]`，人物/车牌未裁图。检查时间戳匹配；观察开始前在途帧和订阅丢帧可能导致部分不匹配，不能要求每条都匹配。
- 连续新源帧下，交通灯处理频率不高于配置 `max_rate`（允许短窗口统计边界误差）；长耗时帧必须 UNKNOWN，而不是发布有效过期 GREEN。观察器收到全 UNKNOWN 仍不代表性能或识别验收通过，必须检查原因和处理耗时。
- 分别观察红/黄/绿真实目标、无灯、不同位置/距离/遮挡与光照。对无灯场景记录误分类，不能仅凭高分将它判为检测成功。必要时缩小人工 ROI 或后续另做存在性检测；本轮不新增第四训练类别。
- 在单独测试中停止相机输入：不晚于 `stale_seconds + 一个处理周期`（若正在推理还需等待其结束）发布 UNKNOWN；重启相机的新时间戳可恢复。暂停仿真/重放旧帧不得生成有效 GREEN。仿真时间回退后应重启交通灯节点。不要把模型进程被杀后“没有新消息”误认为绿灯，下游需自行超时。

当前没有上述实测频率、延迟、资源或识别结果。现有 P2-B 不消费交通灯话题；正式交通灯任务仍由原保护逻辑拦截，不能用于自动越线。
