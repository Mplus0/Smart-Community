# robot_perception：人物 / 车牌 / 可选交通灯分类

人物、车牌和交通灯分类节点均已接入。当前分终端启动和容器内回归入口见 [工作空间说明](../../README.md)；历史验证记录保留在 docs 中，不等同于完整 Gazebo 三路性能验收。

人物与车牌保留交接版识别逻辑，新增 YOLO11n 交通灯 ROI 分类。本包不控制运动、不启动导航或比赛主程序，不做任务统计，不移植 HSV 方案。默认仍只启动原两路；显式启用第三路后才加载交通灯模型。接口见 [docs/interface.md](docs/interface.md)，验证记录见 [docs/validation.md](docs/validation.md)，本轮准备与验收见 [docs/traffic_light_validation.md](docs/traffic_light_validation.md)，授权状态见 [NOTICE.md](NOTICE.md)。

## 选择性合并与差异

| 文件 | 来源及改动 |
|---|---|
| `scripts/person_detection_node.py` | 来自交接包；默认 device 从 `0` 改为 `cpu`，输出图复制 Header，防止 rospy 序列化修改源帧 seq。模型 names 检查、C/NC 语义、BGR 解码、预处理、predict 参数及 JSON 字段保持原逻辑。 |
| `scripts/plate_recognition_node.py` | 来自交接包；导入 HyperLPR 前校验本机缓存，缺失/冲突明确失败，删除 WSL Windows 字体候选路径，输出图复制 Header。检测等级、解码、推理、置信度过滤、JSON 字段和标注逻辑保持原样。 |
| `scripts/prepare_hyperlpr_models.py` | 来自交接包；提取可复用的只读校验函数，新增 `--verify-only`。保留四模型大小/SHA256 校验、当前用户缓存、冲突拒绝和独占写入。 |
| `models/person_best.pt`、四个 ONNX、`models_manifest.json` | 原样复制，不修改、不重新训练。 |
| `launch/person_detection.launch`、`plate_recognition.launch` | 基于交接接口重建：显式解释器、CPU 默认、缓存目录、stale 参数和可选预览。 |
| `launch/perception.launch` | 默认组合人物/车牌，可选 `enable_traffic_light:=true` 增加交通灯；默认无 GUI。关闭一路也关闭其预览；节点启动失败结束该视觉 launch。 |
| `scripts/traffic_light_classification_node.py`、`config/traffic_light_classification.yaml`、`launch/traffic_light_classification.launch` | 新增 YOLO11n 三分类、相对 ROI、源帧新鲜度检查、UNKNOWN 拒绝及单路启动。 |
| `package.xml`、`CMakeLists.txt`、README、NOTICE、docs、tests | 本项目新建，安装范围明确；测试仅辅助验证，不是任务结果保存功能。 |

没有复制旧 HSV 节点或原三路 `perception_all.launch`；新交通灯分类器使用本仓库训练权重。P1 状态机、航点、导航、地图和仿真场景均不修改。

## Docker 隔离环境

宿主机每个终端先进入同一个容器用户：

```bash
docker exec -it --user developer smart-community-dev bash
```

后续命令全部在容器内执行。构建使用系统 Python 3.8；推理使用独立 Python 3.10 环境，默认 `$HOME/.venvs/robot-perception/bin/python`。不激活推理环境来重建 ROS 工作空间，不全局升级 pip/NumPy/OpenCV/Torch。

本次环境方案用 Python 3.10 满足交接版 Torch 2.14 与 ONNX Runtime 1.23.2 的最低 Python 要求；Ultralytics 8.4.159 的元数据仍允许 Python >=3.8，不能因此推断整套交接依赖适配系统 Python 3.8。参考：[Ultralytics 安装文档](https://docs.ultralytics.com/quickstart/)。实际版本及导入结果以 validation.md 为准。

受控安装步骤（需要网络下载解释器与依赖，不下载模型；约需数百 MB 下载与额外磁盘空间）：

```bash
# 不改动系统 site-packages；uv 引导工具只放 /tmp。
python3 -m pip install --disable-pip-version-check --target /tmp/p2a-uv-bootstrap uv==0.8.22
/tmp/p2a-uv-bootstrap/bin/uv --native-tls venv --python 3.10.18 "$HOME/.venvs/robot-perception"
/tmp/p2a-uv-bootstrap/bin/uv --native-tls pip install \
  --python "$HOME/.venvs/robot-perception/bin/python" \
  --index-url https://download.pytorch.org/whl/cpu \
  torch==2.14.0+cpu torchvision==0.29.0+cpu
/tmp/p2a-uv-bootstrap/bin/uv --native-tls pip install \
  --python "$HOME/.venvs/robot-perception/bin/python" \
  -r /workspace/car_2026/src/robot_perception/docs/requirements-cpu.txt
```

已有同名环境时先检查，不覆盖其他用途的环境。现已将隔离环境安装加入仓库 Dockerfile，按新配置构建镜像后，新容器会自带 Python 3.10 和视觉依赖；旧镜像仍需按上文手工部署。重建步骤见 [Docker 说明](../../../docker/README.md)。用户目录没有持久挂载，HyperLPR 模型缓存仍需在新容器内离线准备。CPU 包不使用 CUDA；即使 `nvidia-smi` 可见显卡，本配置仍默认 CPU。GPU 环境若后续另建，通过 `python:=... person_device:=0` 显式选择，不能只修改 device 而继续使用 CPU Torch。

加载 ROS 后检查完整导入链：

```bash
source /opt/ros/noetic/setup.bash
cd /workspace/car_2026
"$HOME/.venvs/robot-perception/bin/python" src/robot_perception/tests/check_environment.py
```

检查同时覆盖 `rospy`、`sensor_msgs.msg`、`std_msgs.msg`，不仅是 torch。两节点手动解码 `sensor_msgs/Image`，不加载跨 Python ABI 的 `cv_bridge`。不要将系统 `/usr/lib/python3/dist-packages` 整体注入推理环境；launch 仅添加可配置 `ros_python_path`（默认 Noetic Python 路径）并继承已加载的工作空间 PYTHONPATH。

## 离线准备车牌模型

`docs/requirements-cpu.lock.txt` 记录本次实际安装的全部 62 个包版本。复现时先按上文安装 CPU Torch，再将第二步安装命令中的 `requirements-cpu.txt` 换成该 lock 文件，可固定传递依赖版本。

必须使用与节点相同 Linux 用户、实际 HOME 和 Python；HOME 继承容器登录用户，不伪装队友用户路径。可通过 launch 参数选择 Python 和模型源目录。

```bash
cd /workspace/car_2026
"$HOME/.venvs/robot-perception/bin/python" src/robot_perception/scripts/prepare_hyperlpr_models.py \
  --source /workspace/car_2026/src/robot_perception/models/hyperlpr3
"$HOME/.venvs/robot-perception/bin/python" src/robot_perception/scripts/prepare_hyperlpr_models.py \
  --source /workspace/car_2026/src/robot_perception/models/hyperlpr3 --verify-only
```

目标为当前用户 `~/.hyperlpr3/20230229/onnx/`。四个模型同哈希可复用；冲突不覆盖。若中断留下不完整文件，下次校验会失败，应人工确认后处理该缓存。节点导入库之前也会重新校验，缺缓存就退出而非联网补齐。人物 launch 设置 `YOLO_AUTOINSTALL=false`、`YOLO_OFFLINE=true`；只使用包内现有权重。

## 构建与独立启动

容器终端内：

```bash
source /opt/ros/noetic/setup.bash
cd /workspace/car_2026
catkin_make -j2
source devel/setup.bash
rospack find robot_perception
roslaunch --nodes robot_perception perception.launch
```

如果现有 Gazebo 已在运行，直接复用，不能重复启动。否则终端 1：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/car_2026/devel/setup.bash
roslaunch robot_gazebo simulation.launch
```

终端 2（独立视觉测试不要求启动 AMCL/move_base）：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/car_2026/devel/setup.bash
roslaunch robot_perception perception.launch \
  python:="$HOME/.venvs/robot-perception/bin/python" person_device:=cpu show_images:=false
```

也可只启用一路（不要与上述双路入口同时运行）：

```bash
roslaunch robot_perception perception.launch enable_plate:=false
roslaunch robot_perception perception.launch enable_person:=false
# 或分别使用单路入口，同样不要重复同名节点：
roslaunch robot_perception person_detection.launch device:=cpu confidence:=0.25
roslaunch robot_perception plate_recognition.launch min_confidence:=0.5
```

统一入口参数：`enable_person`、`enable_plate`、`enable_traffic_light`（默认 false）、`camera_topic`、`python`、`ros_python_path`、`show_images`；人物 `person_model/person_device/person_confidence/person_imgsz/person_max_rate/person_stale_seconds`；车牌 `plate_models_dir/plate_min_confidence/plate_max_rate/plate_stale_seconds/plate_font_path`；交通灯 `traffic_light_model/traffic_light_config/traffic_light_device/traffic_light_confidence/traffic_light_max_rate/traffic_light_stale_seconds`。单路输入参数名为 `input_topic`。

交通灯权重需先按 [准备步骤](docs/traffic_light_validation.md) 复制到包内。之后在已有容器中选择一种启动方式，不重复启动同名节点：

```bash
# 三路联合；人物和车牌仍使用完整原图
roslaunch robot_perception perception.launch enable_traffic_light:=true
# 或仅交通灯
roslaunch robot_perception traffic_light_classification.launch device:=cpu
```

交通灯输入为原图上半部分，默认相对 ROI `[0.0, 0.0, 1.0, 0.5]`，`imgsz=224`。输出颜色是分类结果，不能证明存在交通灯；本节点不控制运动，等待和放行由 `robot_competition` 的 P2-C 状态机处理。

统一/单路人物阈值均默认 0.25，采用交接单路默认值；交接原三路入口的 0.8 未沿用，可显式 `person_confidence:=0.8` 对比。高阈值可能漏检，不能保证消除灯具误检。默认相机 `/camera/color/image_raw`，默认每路上限 5 Hz。

终端 3 查看：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/car_2026/devel/setup.bash
rostopic echo -n 1 /perception/person_detections_json
rostopic echo -n 1 /perception/plates_json
rosrun image_view image_view image:=/perception/person_image
# 另一终端可看车牌标注图
rosrun image_view image_view image:=/perception/plates_image
```

`show_images:=true` 也可启动已启用节点的预览，需容器 X11 可用。中文字体缺失时图像只显示 `plate#编号`；JSON 保留完整号码。若已有 Noto CJK，可指定 `plate_font_path:=/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc`。本轮不捆绑字体、不修改系统字体。

## 复现检查与限制

```bash
cd /workspace/car_2026
python3 src/robot_perception/tests/check_static.py
"$HOME/.venvs/robot-perception/bin/python" src/robot_perception/tests/test_adapters.py
# 已有 Gazebo、尚未启动视觉节点时；启动两路并观察 30 秒，结束只清理该测试启动的视觉 launch。
bash src/robot_perception/tests/run_live_check.sh
```

运行证据放容器 `/tmp/p2a-*`，不纳入 Git。`observe_topics.py` 只读订阅并校验 JSON、记录源时间戳、实际接收频率、处理耗时和进程资源，不操控机器人；测试图像/报告只用于验收。

人物 C/NC 泛化及灯具误检、车牌不同距离/角度/光照、长期资源占用仍需场景验证。车牌没有 frame_valid，断流/异常会沉默；下游同时检查源时间戳与墙上时钟超时。P2-B 任务窗口和持久化见 [P2B.md](../robot_competition/P2B.md)，P2-C 交通灯订阅、等待与放行见 [比赛包说明](../robot_competition/README.md)。
