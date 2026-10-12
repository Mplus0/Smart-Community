# Smart-Community

基于 ROS 1 Noetic 和 Gazebo 11 的智慧社区机器人系统，已实现麦克纳姆底盘仿真、建图定位、多航点导航、人物与车牌识别、交通灯等待放行、任务结果保存和 PyQt5 比赛控制台。本文按当前仓库代码及配置整理；部署入口以 Docker 仿真环境为主，实车需要另行对接底盘和传感器。

## 功能组成

- **仿真与底盘**：机器人 Xacro、传感器和比赛场景，麦轮控制、轮式里程计与 IMU 的 EKF 融合。
- **建图与导航**：Cartographer 建图、AMCL 定位、定位/规划双地图、move_base 与 TEB 路径规划。
- **视觉识别**：人物 YOLO、HyperLPR3 车牌识别和 YOLO 交通灯 ROI 分类，输出 JSON 与标注图。
- **任务执行**：按航点顺序导航，到点采集人物/车牌结果；交通灯点停车等待连续有效绿灯；保存任务状态、统计和图像证据。
- **运行控制**：集中启动基础环境、人工确认后开始比赛、按顺序停止；GUI 显示实时相机、任务证据、结果和比赛日志。

## 运行环境

ROS 与 Python 程序在 Docker 容器中运行：Ubuntu 20.04、ROS Noetic、Gazebo 11。任务控制器使用系统 Python 3.8，视觉推理使用独立 Python 3.10 环境。宿主机需要 Docker Compose；默认容器配置使用 NVIDIA GPU 和 X11 图形显示，视觉推理默认使用 CPU。

## 目录结构

```text
car_2026/       ROS catkin 工作空间
  src/         六个 robot_* 功能包
  results/     任务结果目录，运行时创建
docker/        镜像、Compose 配置与部署说明
yolo_train/    已有交通灯 raw/train/val/test 数据集及训练权重
models/        模型权重与导出模型
scripts/       交通灯数据准备、训练、评估及导出工具
复赛资料/       比赛资料与素材
```

根目录 PDF 和实现流程文件供资料查阅；运行和部署以本 README 及下列使用说明为准。

## 部署与使用

1. 按 [Docker 部署说明](docker/README.md) 构建镜像、创建容器。
2. 按 [视觉包说明](car_2026/src/robot_perception/README.md) 准备模型与 HyperLPR 缓存。
3. 按 [工作空间说明](car_2026/README.md) 编译，选择集中启动、GUI 或分终端启动流程。

已有容器的终端入口（宿主机执行）：

```bash
docker exec -it --user developer smart-community-dev bash
```

比赛主入口为 `robot_competition/launch/competition.launch`。默认路线为 `car_2026/src/robot_competition/config/waypoints.yaml`；使用前应确认地图、定位、路线和停车位置与运行场地一致。

完成编译、模型缓存和中文字体准备后，在容器中启动基础环境：

```bash
cd /workspace/car_2026
bash src/tools/start.sh
```

在 RViz 使用 **2D Pose Estimate** 校准定位，然后在另一个容器终端执行：

```bash
bash /workspace/car_2026/src/tools/start_competition.sh
```

输入 `yes` 后开始比赛。停止时执行 `bash /workspace/car_2026/src/tools/stop.sh`。GUI 入口为 `python3 /workspace/car_2026/src/tools/gui.py`，环境加载及依赖见工作空间说明；不要同时用 GUI 和终端重复启动同一套环境。

默认路线共有 12 点：2 个交通灯点、5 个人员点、3 个车牌点和 2 个普通航点。当前配置采用 3 帧连续有效绿灯、置信度至少 0.8 放行，最长等待 35 秒；导航成功和识别成功分别记录。`FINISH` 表示路线执行完毕，仍须查看各任务状态，不能据此认定全部识别成功。

## 交通灯数据与训练

已有数据位于 `yolo_train/datasets/traffic_light/`，按 `red/yellow/green` 分类；训练权重位于 `yolo_train/results/traffic_light/yolo11n_baseline/weights/`，部署源文件为 `models/traffic_light/best.pt`，另有导出的 `best.onnx`。ROS 分类节点使用 `.pt` 模型。

训练工具位于 `scripts/traffic_light/`，包含数据划分、微调、评估和 ONNX 导出。**当前 `config.yaml` 仍指向根目录的 `datasets/traffic_light` 和 `results/traffic_light`，与已有数据位置不同**。使用已有数据时，先复制配置为自己的 YAML，将 `data.source` 改为 `yolo_train/datasets/traffic_light/raw`、`data.root` 改为 `yolo_train/datasets/traffic_light`，并设置独立的 `output.project`、`output.best` 与 `export.path`，避免覆盖部署权重。

在能访问完整仓库、已安装 Ultralytics、PyTorch、PyYAML 和 ONNX 的 Python 环境中，从仓库根目录运行（`my_config.yaml` 为上述自备配置）：

```bash
python scripts/traffic_light/prepare_dataset.py --config my_config.yaml
python scripts/traffic_light/train.py --config my_config.yaml
python scripts/traffic_light/test.py --config my_config.yaml
python scripts/traffic_light/export_onnx.py --config my_config.yaml
```

数据已划分时可跳过第一步。脚本按仓库根目录解析相对路径；默认划分比例为 70%/15%/15%，随机种子 57，训练 100 epochs、尺寸 224、batch 32、patience 20。训练/评估自动选择可用 CUDA 或 CPU；评估生成混淆矩阵，导出使用 opset 17 并校验 ONNX。默认 Compose 仅挂载 `car_2026`，不包含这些训练脚本和顶层数据；训练环境与比赛容器需分别准备。相机采集方法见工作空间 README。

## 使用文档

- [工作空间、启动流程、建图与结果查看](car_2026/README.md)
- [比赛任务、航点录入与故障处理](car_2026/src/robot_competition/README.md)
- [视觉模型、输入输出与配置](car_2026/src/robot_perception/README.md)
- [机器人模型与传感器](car_2026/src/robot_description/README.md)
- [导航参数、双地图与调试](car_2026/src/robot_navigation/config/README.md)
- [随机车牌生成](car_2026/src/robot_gazebo/README_RANDOM_PLATE.md)

任务结果保存在 `car_2026/results/<run_id>/`。缓存、编译产物、日志和运行结果不提交到版本控制。项目许可证见 [LICENSE](LICENSE)，视觉模型及依赖的授权说明见 [NOTICE.md](car_2026/src/robot_perception/NOTICE.md)。
