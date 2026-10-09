# Smart-Community

基于 ROS 1 和 Gazebo 的智慧社区机器人系统，提供麦克纳姆底盘仿真、建图定位、多航点导航、人物与车牌识别、交通灯等待放行及任务结果保存。

## 功能组成

- **仿真与底盘**：机器人 Xacro、传感器和比赛场景，麦轮控制、轮式里程计与 IMU 的 EKF 融合。
- **建图与导航**：Cartographer 建图、AMCL 定位、定位/规划双地图、move_base 与 TEB 路径规划。
- **视觉识别**：人物 YOLO、HyperLPR3 车牌识别和 YOLO 交通灯 ROI 分类，输出 JSON 与标注图。
- **任务执行**：按航点顺序导航，到点采集人物/车牌结果；交通灯点停车等待连续有效绿灯；保存任务状态、统计和图像证据。

## 运行环境

ROS 与 Python 程序在 Docker 容器中运行：Ubuntu 20.04、ROS Noetic、Gazebo 11。任务控制器使用系统 Python 3.8，视觉推理使用独立 Python 3.10 环境。宿主机需要 Docker Compose；默认容器配置使用 NVIDIA GPU 和 X11 图形显示，视觉推理默认使用 CPU。

## 目录结构

```text
car_2026/       ROS catkin 工作空间
  src/         六个 robot_* 功能包
  results/     任务结果目录，运行时创建
docker/        镜像、Compose 配置与部署说明
datasets/      交通灯数据集
models/        模型权重与导出模型
scripts/       交通灯数据准备、训练、评估及导出工具
results/       训练权重及训练输出位置
复赛资料/       比赛资料与素材
```

根目录 PDF 和实现流程文件供资料查阅；运行和部署以本 README 及下列使用说明为准。

## 部署与使用

1. 按 [Docker 部署说明](docker/README.md) 构建镜像、创建容器。
2. 按 [视觉包说明](car_2026/src/robot_perception/README.md) 准备模型与 HyperLPR 缓存。
3. 按 [工作空间说明](car_2026/README.md) 编译，在四个容器终端分别启动仿真、导航、视觉及比赛主程序。

已有容器的终端入口（宿主机执行）：

```bash
docker exec -it --user developer smart-community-dev bash
```

比赛主入口为 `robot_competition/launch/competition.launch`。默认路线为 `car_2026/src/robot_competition/config/waypoints.yaml`；使用前应确认地图、定位、路线和停车位置与运行场地一致。

## 使用文档

- [工作空间、启动流程、建图与结果查看](car_2026/README.md)
- [比赛任务、航点录入与故障处理](car_2026/src/robot_competition/README.md)
- [人物/车牌参数、统计口径与结果格式](car_2026/src/robot_competition/P2B.md)
- [视觉模型、输入输出与配置](car_2026/src/robot_perception/README.md)
- [机器人模型与传感器](car_2026/src/robot_description/README.md)
- [随机车牌生成](car_2026/src/robot_gazebo/README_RANDOM_PLATE.md)

任务结果保存在 `car_2026/results/<run_id>/`。缓存、编译产物、日志和运行结果不提交到版本控制。项目许可证见 [LICENSE](LICENSE)，视觉模型及依赖的授权说明见 [NOTICE.md](car_2026/src/robot_perception/NOTICE.md)。
