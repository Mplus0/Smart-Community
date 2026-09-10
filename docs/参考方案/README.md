# 智慧社区巡逻机器人系统（ROS Noetic 版）

基于 ROS 的自主导航 + 视觉识别巡逻机器人。机器人在 Gazebo 虚拟社区中沿预设导航点自主巡航，在关键位置自动拍照，并通过 **YOLOv5**（目标检测）与 **EasyOCR**（车牌/文字识别）对图像做智能识别。

> 本目录是从原「ROS Melodic + Python2.7」项目迁移、整理出的**可直接 `catkin_make` 的干净工作空间**，目标平台为 **Ubuntu 20.04 + ROS Noetic（Python3）**。

---

## 目录结构

```
hzx_ws_noetic/
├── README.md                  # 本说明
├── docs/
│   └── 系统架构图.md           # 架构图 + 流程图（Mermaid + ASCII）
└── src/
    ├── mbot_description/      # 机器人模型（URDF/Xacro + 网格）
    ├── mbot_gazebo/           # Gazebo 仿真启动 + 社区世界 hzx.world
    ├── mbot_navigation/       # SLAM 建图 + 导航（gmapping/amcl/move_base）
    ├── photo_service/         # 拍照服务 + SMACH 状态机 + 识别脚本
    ├── yolov5_ros/            # YOLOv5 ROS 检测节点 + 权重
    └── yolov5_ros_msgs/       # YOLO 检测消息定义（BoundingBox/BoundingBoxes）
```

---

## 一、环境要求

| 项目 | 版本 |
|---|---|
| 系统 | Ubuntu 20.04 LTS |
| ROS | ROS Noetic（desktop-full） |
| Gazebo | Gazebo 11 |
| Python | Python 3.8（系统自带） |

---

## 二、安装依赖

### 1. ROS Noetic（如已安装可跳过）

```bash
# 配置软件源
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo apt update

# 安装桌面完整版
sudo apt install ros-noetic-desktop-full

# 初始化
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

### 2. ROS 功能包依赖

```bash
sudo apt install -y \
  ros-noetic-gmapping \
  ros-noetic-amcl \
  ros-noetic-map-server \
  ros-noetic-move-base \
  ros-noetic-smach \
  ros-noetic-smach-ros \
  ros-noetic-teleop-twist-keyboard \
  ros-noetic-joint-state-publisher \
  ros-noetic-robot-state-publisher \
  ros-noetic-xacro \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control \
  ros-noetic-cv-bridge \
  ros-noetic-vision-opencv
```

### 3. Python 依赖

```bash
# ROS 侧节点（photo_server / smach / yolo_v5）需要 torch 与 opencv
pip3 install --user torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip3 install --user numpy

# 车牌识别脚本（ceshi2.py）需要 easyocr（较慢，可只装一次）
pip3 install --user easyocr
```

> **说明**：`torch` 建议装 CPU 版（`--index-url .../cpu`）以满足本项目「CPU 环境实时推理」的定位。
> 若遇到 `cv2` 与 `cv_bridge` 的 OpenCV 版本冲突，请优先使用 apt 自带的系统 OpenCV，不要额外 `pip install opencv-python`。

---

## 三、创建工作空间并编译

```bash
# 1. 把本目录（hzx_ws_noetic）复制到主目录
cp -r ~/下载/hzx_ws_noetic ~/hzx_ws_noetic   # 路径按实际调整
cd ~/hzx_ws_noetic

# 2. 编译
catkin_make

# 3. 让脚本可执行（首次）
chmod +x src/photo_service/scripts/*.py
chmod +x src/mbot_navigation/scripts/*.py
chmod +x src/yolov5_ros/scripts/*.py

# 4. 加载工作空间环境
echo "source ~/hzx_ws_noetic/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 四、运行步骤

按顺序开多个终端（每个终端都先 `source ~/hzx_ws_noetic/devel/setup.bash`，或已写入 `.bashrc`）。

### 步骤 1：启动 Gazebo 仿真 + 机器人

```bash
roslaunch mbot_gazebo mbot_gazebo.launch
```

看到 Gazebo 打开，机器人（黄色圆柱体 + 激光雷达 + 相机）出现在社区世界中。

### 步骤 2：SLAM 建图（首次需要，或重新建图时）

**终端 A**（已有 Gazebo，见步骤 1）
**终端 B**：启动建图 + RViz

```bash
roslaunch mbot_navigation gmapping_demo.launch
```

**终端 C**：键盘遥控建图（绕场一周，让激光雷达扫全地图）

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

**终端 D**：保存地图

```bash
cd ~/hzx_ws_noetic/src/mbot_navigation/map
rosrun map_server map_saver -f map
```

> 仓库已附带一份建好的 `map.pgm / map.yaml`，直接跳到步骤 3 也可以。

### 步骤 3：启动导航 + 多点巡航

**终端 B**（关掉建图后）：

```bash
roslaunch mbot_navigation nav_demo.launch
```

这会启动 map_server + amcl + move_base + RViz，并自动运行 `waypoint_nav.py` 沿 `waypoints.yaml` 巡航。

> 初始位姿由 launch 自动发布到 `(1.71, -1.629)`，需与 Gazebo 出生点一致。若定位漂移，可在 RViz 用「2D Pose Estimate」手动重设。

### 步骤 4：巡逻 + 定点拍照（SMACH 状态机）

在步骤 3 基础上，另开终端：

```bash
roslaunch photo_service photo_nav.launch
```

机器人在 49 个导航点巡航，途经 `ro1`、`ro2`、`chepai` 三个拍照点时自动调用 `/take_photo`，照片保存到 `~/hzx_ws_noetic/src/photo_service/photo/photo1`、`photo2`。

### 步骤 5：YOLOv5 目标检测

```bash
roslaunch yolov5_ros yolo_v5.launch
```

订阅 `/kinect/rgb/image_raw`，输出检测框图（`/yolov5/detection_image`）。

### 步骤 6：离线识别（可选，无 Gazebo 时）

**YOLO 批量检测**（对已拍照片做亮度增强 + 检测）：

```bash
rosrun photo_service detect.py
```

**EasyOCR 车牌识别**（单张图片）：

```bash
rosrun photo_service ceshi2.py ~/hzx_ws_noetic/src/photo_service/photo/photo2/chepai.jpg
```

---

## 五、与原项目的主要差异（迁移说明）

| 项 | 原项目 | 本工作空间 |
|---|---|---|
| ROS/Python | Melodic + Python 2.7 | Noetic + Python 3 |
| 服务加载 | `imp.load_source` 手动加载 TakePhoto | 直接 `from photo_service.srv import TakePhoto` |
| 保存路径 | 硬编码 `photo_servicc`（拼写错误） | 用 `rospkg` 自动定位包路径 |
| 相机插件 | `libgazebo_ros_openni_kinect.so`（Noetic 已移除） | `libgazebo_ros_camera.so`（RGB） |
| 三维雷达 | VLP-16（依赖 velodyne C++ 插件） | 移除（二维导航用不到） |
| 地图路径 | 绝对路径 `/home/mw/...` | 相对路径 `map.pgm` |
| 消息包 | 缺失 `yolov5_ros_msgs` | 已补建 |
| 包名 | `demo01_gazebo`/`demo02_rviz` 及拼写错误 | 语义化命名 `mbot_gazebo`/`mbot_navigation` |

---

## 六、常见问题

1. **`ImportError: No module named photo_service.srv`** — 未 `source devel/setup.bash`，或未 `catkin_make` 生成服务。
2. **`cv2` / `cv_bridge` 冲突** — 卸载 `pip` 版 opencv：`pip3 uninstall opencv-python opencv-contrib-python`，改用系统 OpenCV。
3. **Gazebo 相机无图像** — 确认 `rostopic hz /kinect/rgb/image_raw` 有输出；必要时在 launch 里调大 `updateRate`。
4. **导航定位漂移** — 检查 Gazebo 出生点与 `nav_demo.launch` 初始位姿是否一致，必要时 RViz 手动 `2D Pose Estimate`。
5. **torch 加载慢** — 首次会下载/缓存模型；`shequ.pt` 为社区场景自训权重，`yolov5s.pt` 为通用权重，可按需在 launch 中切换。

---

## 七、成果说明

- **技术报告**：见原压缩包 `智慧社区技术报告（1).pdf`（省赛模板，9 页）。
- **演示视频关键画面**：机器人沿社区道路巡航，在 `ro1/ro2`（人群立牌）、`chepai`（车牌）三点停下拍照，YOLOv5 框出目标、EasyOCR 输出车牌号。
