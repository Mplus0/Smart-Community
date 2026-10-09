# Smart-Community

面向智慧社区比赛的 ROS 1 / Gazebo 机器人项目。主要运行代码位于 `car_2026/`，开发环境为 Docker 中的 Ubuntu 20.04、ROS Noetic 和 Gazebo 11。

## 主要模块

- **仿真与底盘**：机器人 Xacro/传感器模型、比赛场景、麦克纳姆控制、轮式里程计与 IMU 的 EKF 融合。
- **建图与导航**：Cartographer 建图、AMCL 定位、定位/规划双地图、move_base 与 TEB 多航点导航。
- **视觉识别**：人物 YOLO、HyperLPR3 车牌和 YOLO 交通灯 ROI 分类，发布 JSON 与标注图；推理使用独立 Python 环境。
- **比赛任务**：P1 航点导航，P2 人物/车牌任务窗口与结果保存，以及红绿灯停车等待、连续绿灯确认和异常停止。任务控制器与视觉节点分别启动。

## 目录

```text
car_2026/       ROS catkin 工作空间、功能包及工作空间使用说明
  src/         六个 robot_* 功能包
  results/     本地任务结果（忽略提交）
docker/        Dockerfile、Compose 与容器环境说明
datasets/      交通灯采集数据及训练/验证/测试集
models/        训练权重与导出模型
scripts/       交通灯数据准备、训练、评估及导出工具
results/       保留训练权重；新增运行产物忽略提交
docs/          历史参考方案及技术资料
复赛资料/       比赛资料、交接代码和素材
```

根目录 PDF 和实现流程文档属于参考资料；当前启动方式以工作空间文档及实际 launch 为准。

## 使用入口

进入已有容器（宿主机执行）：

```bash
docker exec -it --user developer smart-community-dev bash
```

编译、仿真、导航、建图、三路视觉和最终比赛主程序的**分终端命令**见 [car_2026/README.md](car_2026/README.md)。容器创建与依赖部署见 [docker/README.md](docker/README.md)。

- [比赛任务、航点及结果说明](car_2026/src/robot_competition/README.md)
- [人物/车牌任务细节](car_2026/src/robot_competition/P2B.md)
- [视觉环境、模型与接口说明](car_2026/src/robot_perception/README.md)
- [机器人模型说明](car_2026/src/robot_description/README.md)

构建与运行检查在 Docker 中执行。正式模型、地图、场景、航点和训练资源保留；开发验收脚本、临时路线及验收产物不再保留，缓存、编译产物、日志及本地任务结果不提交。
