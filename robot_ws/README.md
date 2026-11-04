# 智慧社区 ROS 工作空间

ROS 1 智慧社区巡检工程，自研节点使用 Python 3。本次完成 14→9 包结构合并，只进行静态检查与离线规则回归；没有安装 ROS、编译或实车测试。

## 目录及功能包

```text
robot_ws/
├── src/
│   ├── community_bringup/     # launch、config/navigation、maps、相机配置
│   ├── community_base/        # 串口桥接、速度反馈、里程计
│   ├── community_perception/  # YOLO、OCR、倒伏统计、models、srv
│   ├── community_mission/     # 路线、任务调度、语音、录像、srv
│   ├── lslidar/               # 厂商元包
│   ├── lslidar_driver/        # 厂商雷达实现
│   ├── lslidar_msgs/          # 厂商消息
│   ├── wit_ros_imu/           # 厂商 IMU
│   ├── robot_description/     # 原车模型，仅修改包与资源引用
│   └── CMakeLists.txt
├── docs/                     # 架构说明、完整性基线、检查报告
├── tools/                    # 静态检查与离线回归
├── requirements.txt
└── README.md
```

四个 community_* 包由项目维护；雷达与 IMU 驱动实现未修改。robot_description 保留模型几何与参数。community_perception 内 SORT 是第三方代码，保留文件头声明；ONNX 权重是原项目资源。

## 主要功能

保留六种 YOLO 模型（人员、交通灯、火情、垃圾桶、垃圾、电动车）、OCR、倒伏视频统计、路线导航、播报和录像。原路线全部航点及任务索引保留在 community_mission/config/route.yaml。模型统一在 community_perception/models，地图为 community_bringup/maps/map.yaml、map.pgm。

任务规则、推理算法、底盘协议及导航参数未调整。本工程仍缺异常温度、仪表识读、动态指示牌判断、独立泊车结果校验和全程 300 秒预算控制；不能将结构整理视为已经完整通过总决赛要求。

## 环境与依赖

目标为已有 ROS 1/Python 3 的 Linux 工控机，以 Noetic 为参考。需要 ROS 导航栈、robot_localization、usb_cam、cv_bridge、actionlib、dynamic_reconfigure 及厂商包所声明依赖。Python 依赖见 requirements.txt；OpenCV/NumPy 须匹配 cv_bridge，PaddleOCR 使用 2.x API。语音需要 espeak-ng 与 mb-cn1 语音资源，相机调参需要 v4l2-ctl。OCR 模型缓存须在目标机准备。

## 编译与启动（全部待工控机验证）

以下为目标机使用说明，本次未执行。路径以仓库部署在 /home/mowen/workspace/Smart-Community 为例：

```bash
source /opt/ros/noetic/setup.bash
cd /home/mowen/workspace/Smart-Community/robot_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
# 调试：启动硬件、导航和服务，默认不自动巡检
roslaunch community_bringup community.launch base_protocol:=aa_bb
# 比赛：一键执行现有路线；不要与上一命令同时运行
roslaunch community_bringup finals.launch base_protocol:=aa_bb
# 若已独立启动硬件/导航/服务，可单独执行任务
roslaunch community_mission mission.launch
```

base_protocol 必须按下位机确认：aa_bb 对应原新版三轴协议，legacy_11 对应原旧版两轴协议。默认底盘 /dev/carserial、雷达 /dev/lslidar、IMU /dev/ttyUSB1、相机 /dev/video0；udev 与串口权限需在工控机核对。

总入口支持 base_port、imu_port、camera_device、configure_camera、map、route、data_dir。输出默认 ~/.ros/community，重复任务可能覆盖同名图片/视频。navigation.launch 统一启动模型 TF、EKF、地图、AMCL 和 move_base；hardware.launch 提供硬件及原始轮式里程计。

## 调试命令（待工控机验证）

```bash
rosnode list
rostopic hz /scan
rostopic hz /wit/imu
rostopic hz /usb_cam/image_raw
rostopic echo -n 1 /odom
rosservice list
rosservice type /people_detector/detect
rosservice type /speech_say
rosrun tf tf_echo map base_footprint
```

Topic、Service、Action 名称保留；自定义服务类型改为 community_perception/* 和 community_mission/*，外部客户端需要重新生成并更新导入。详见 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)。

## 开发机检查

```bash
python tools/validate_workspace.py
python tools/test_python_migration.py
```

不导入 ROS、不启动节点。结果见 [docs/VALIDATION.md](docs/VALIDATION.md) 和 [docs/PYTHON_TESTS.md](docs/PYTHON_TESTS.md)。原 IMU 非默认脚本语法问题及未启用的模型 Xacro 文件问题保持原状；默认使用完整的 mowen.urdf。
