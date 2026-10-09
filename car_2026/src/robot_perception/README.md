# robot_perception

提供人物检测、车牌识别和交通灯 ROI 分类，订阅相机图像并发布 JSON 与标注图。运动控制、任务统计和结果保存由 `robot_competition` 执行。

## 节点与输出

| 节点 | 算法 | JSON / 标注图话题 |
|---|---|---|
| `person_detection` | YOLO 人物检测，C/NC 分类 | `/perception/person_detections_json`、`/perception/person_image` |
| `plate_recognition` | HyperLPR3 | `/perception/plates_json`、`/perception/plates_image` |
| `traffic_light_classification` | YOLO11n ROI 三分类 | `/perception/traffic_light_json`、`/perception/traffic_light_image` |

默认输入为 `/camera/color/image_raw`。JSON 与标注图保留源图像 Header，供任务窗口判断新鲜度和关联图像证据。字段定义见 [接口说明](docs/interface.md)，模型和依赖授权见 [NOTICE.md](NOTICE.md)。

## 环境

使用 [Docker 环境](../../../docker/README.md) 中的 Python 3.10 视觉解释器，默认路径为 `$HOME/.venvs/robot-perception/bin/python`。依赖版本由 [requirements-cpu.lock.txt](docs/requirements-cpu.lock.txt) 固定，默认安装 CPU 推理依赖。

catkin 和比赛主程序使用系统 Python 3.8，不要激活视觉 venv 后编译 ROS。每个容器终端加载 ROS 和工作空间：

```bash
cd /workspace/car_2026
source /opt/ros/noetic/setup.bash
source /opt/cartographer_ws/install_isolated/setup.bash
source devel/setup.bash --extend
```

完整构建步骤见 [工作空间说明](../../README.md)。

## 模型准备

| 用途 | 包内路径 |
|---|---|
| 人物 | `models/person_best.pt` |
| 车牌 | `models/hyperlpr3/20230229/onnx/` 及 `models/hyperlpr3/models_manifest.json` |
| 交通灯 | `models/traffic_light_best.pt` |

首次部署交通灯模型时，将仓库根目录的 `models/traffic_light/best.pt` 复制到 `car_2026/src/robot_perception/models/traffic_light_best.pt`；已有有效文件无需覆盖。源权重应保留，不能用 ONNX 或预训练权重替代此文件。默认 Compose 仅挂载 `car_2026`，仓库顶层 `models/` 不在容器内；文件复制可在宿主机文件管理器中完成。

HyperLPR 使用当前 Linux 用户的缓存。使用与节点相同的用户和解释器，在容器中执行：

```bash
"$HOME/.venvs/robot-perception/bin/python" src/robot_perception/scripts/prepare_hyperlpr_models.py \
  --source /workspace/car_2026/src/robot_perception/models/hyperlpr3
"$HOME/.venvs/robot-perception/bin/python" src/robot_perception/scripts/prepare_hyperlpr_models.py \
  --source /workspace/car_2026/src/robot_perception/models/hyperlpr3 --verify-only
```

缓存目标为 `~/.hyperlpr3/20230229/onnx/`。准备工具校验四个模型的大小和 SHA256，相同文件复用，冲突文件不覆盖。节点启动时检查缓存，缺失或不一致时退出；不会联网补齐模型。容器重建后需要重新准备用户缓存。

## 启动

相机或 Gazebo 已运行时，在独立视觉终端启动三路识别：

```bash
roslaunch robot_perception perception.launch \
  python:="$HOME/.venvs/robot-perception/bin/python" \
  enable_traffic_light:=true show_images:=false
```

`perception.launch` 默认启用人物和车牌，交通灯需要显式 `enable_traffic_light:=true`。节点启动失败会结束该视觉 launch。比赛主程序在另一终端启动，参见 [比赛任务说明](../robot_competition/README.md)。

需要单路运行时选择对应入口，不与组合入口重复启动同名节点：

```bash
roslaunch robot_perception person_detection.launch device:=cpu confidence:=0.25
roslaunch robot_perception plate_recognition.launch min_confidence:=0.5
roslaunch robot_perception traffic_light_classification.launch device:=cpu
```

## 主要参数

以下参数属于 `perception.launch`；单路入口的输入话题参数名为 `input_topic`。

| 参数 | 默认值 / 用途 |
|---|---|
| `enable_person`、`enable_plate` | `true` |
| `enable_traffic_light` | `false` |
| `camera_topic` | `/camera/color/image_raw` |
| `python` | `$HOME/.venvs/robot-perception/bin/python` |
| `ros_python_path` | `/opt/ros/noetic/lib/python3/dist-packages` |
| `show_images` | `false`；设为 true 时打开已启用节点的图像窗口 |
| `person_device`、`traffic_light_device` | `cpu` |
| `person_confidence`、`plate_min_confidence`、`traffic_light_confidence` | `0.25`、`0.5`、`0.8` |
| `person_max_rate`、`plate_max_rate`、`traffic_light_max_rate` | 每路 `5.0` Hz 上限 |
| `person_stale_seconds`、`plate_stale_seconds`、`traffic_light_stale_seconds` | `1.5`、`2.0`、`1.5` 秒 |
| `person_imgsz` | `640` |
| `person_model`、`plate_models_dir`、`traffic_light_model` | 上述包内模型路径 |
| `traffic_light_config` | `config/traffic_light_classification.yaml` |
| `plate_font_path` | 空字符串；可指定支持中文的字体文件 |

交通灯默认裁剪原图上半部分，相对 ROI 为 `[0.0, 0.0, 1.0, 0.5]`，分类尺寸为 224。输出 RED、YELLOW、GREEN 或 UNKNOWN；分类置信度不能证明灯体存在，停车和放行由任务控制器判断。

需要完整中文车牌标注时，配置容器中实际存在的字体，例如 `plate_font_path:=/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc`。字体不可用时图像可能显示 `plate#编号`，JSON 中仍保留完整号码。任务端的 `plate_text_annotation_confirmed` 应在确认标注完整后配置，见 [人物/车牌任务参数](../robot_competition/P2B.md)。

## 查看输出与故障处理

```bash
rostopic hz /camera/color/image_raw
rostopic echo -n 1 /perception/person_detections_json
rostopic echo -n 1 /perception/plates_json
rostopic echo -n 1 /perception/traffic_light_json
rosrun image_view image_view image:=/perception/person_image
```

图像窗口需要容器 X11 可用。无输出时依次检查相机话题、解释器路径、权重和 HyperLPR 缓存；节点日志会报告加载或推理错误。

车牌消息没有 `frame_valid` 字段，断流或异常可能不发布结果，使用方应同时检查源时间戳和等待超时。识别效果受目标距离、角度和光照影响；人物统计范围与车牌融合规则见 [任务结果说明](../robot_competition/P2B.md)。
