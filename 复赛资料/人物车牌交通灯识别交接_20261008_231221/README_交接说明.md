# 人物 + 车牌 + 交通灯识别交接

生成时间：2026-10-08T23:12:21.368692+08:00。源于实际WSL运行工程快照；源文件、副本哈希和路径适配见manifest.json。旧两功能交接ZIP保留，本包增加当前HyperLPR3车牌识别。

## 包含的文件

| 文件（robot_perception内） | 用途 |
| --- | --- |
| scripts/person_detection_node.py | YOLO人物ROS节点，发布C/NC带框图和检测JSON |
| models/person_best.pt | YOLO26n人物训练权重，原best.pt字节不变 |
| scripts/plate_recognition_node.py | HyperLPR3车牌定位、号码识别、中文带框图和JSON |
| models/hyperlpr3/20230229/onnx/*.onnx | 4个车牌模型：320/640检测、号码识别、类型分类 |
| models/hyperlpr3/models_manifest.json | 4模型大小和SHA256，供离线准备脚本校验 |
| scripts/prepare_hyperlpr_models.py | 一次性把随包模型复制到当前用户缓存，校验且不覆盖冲突文件 |
| scripts/traffic_light_detector.py | HSV颜色、横排三灯定位、亮度与多帧确认算法 |
| scripts/traffic_light_node.py | 灯态ROS图像/JSON收发，预览仅显示颜色名 |
| config/traffic_light_hsv.yaml | 当前仿真灯态规则参数，green_confirm_frames=3 |
| launch/person_detection.launch | 单独人物识别 |
| launch/plate_recognition.launch | 单独车牌识别 |
| launch/traffic_light_recognition.launch | 单独交通灯识别 |
| launch/perception_all.launch | 三识别节点+三个image_view画面，默认全启用 |
| CMakeLists.txt、package.xml | ROS包构建/安装与ROS依赖 |
| docs/接口说明.md | 三路话题、字段和有效性说明 |

压缩包根目录还包含此说明、environment_reference.json（依赖版本参考）和manifest.json（完整来源/适配记录）。本包不包含训练数据集、采集工具、地图、场景、底盘或巡检主功能包；这些由原工程提供。已有robot_perception时先合并文件和安装项，避免整包覆盖现有工具。

人物是YOLO26n，类0=community_person/C，类1=non_community_person/NC。车牌采用HyperLPR3 0.1.3自带检测/识别/分类模型，不需要旧YOLO车牌检测权重或EasyOCR。交通灯是OpenCV规则算法，没有.pt权重。

## 1. 安装到接收方ROS工作空间

把robot_perception目录放到工作空间src中。以下工作空间名只是示例；已有同名包先合并。不要在Conda推理环境中重建整个ROS工程。

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
chmod +x "$(rospack find robot_perception)"/scripts/*_node.py
chmod +x "$(rospack find robot_perception)/scripts/prepare_hyperlpr_models.py"
```

Windows解压工具可能丢失执行位，所以上面显式补上。ROS依赖为rospy/sensor_msgs/std_msgs/image_view；推理环境为Conda y13/Python3.10。三个节点手动解码Image，不使用cv_bridge。相机、ROS命令在系统ROS环境中运行，识别节点由launch指定Conda解释器。

## 2. 确认推理依赖，离线准备车牌模型（首次部署）

另开已加载ROS工作空间的终端，激活装有YOLO26、Torch/torchvision、OpenCV、NumPy、PyYAML及ROS Python消息依赖的环境。当前原工程名称为y13。完整版本参考见environment_reference.json，不要盲目替换已有CUDA/Torch。

```bash
conda activate y13
python -m pip show ultralytics torch torchvision opencv-python numpy PyYAML hyperlpr3 onnxruntime Pillow
```

车牌部分额外需要hyperlpr3==0.1.3、onnxruntime和Pillow。当前原工程使用onnxruntime==1.23.2的CPU版本，车牌不会因为YOLO能用CUDA就自动使用GPU。接收方如缺依赖，先看pip拟安装内容：

```bash
python -m pip install --dry-run hyperlpr3==0.1.3 onnxruntime==1.23.2 Pillow
# 确认依赖方案适合当前环境后，去掉--dry-run安装。
```

该库在导入时会检查~/.hyperlpr3/20230229；仅把模型放在ROS包内仍可能触发下载。因此首次启动前执行：

```bash
python "$(rospack find robot_perception)/scripts/prepare_hyperlpr_models.py" \
  --source "$(rospack find robot_perception)/models/hyperlpr3"
```

预期输出Installed或Reused四个模型及“HyperLPR3 local models ready”。脚本无网络访问/推理；相同缓存可重复执行，冲突缓存报错且不覆盖。准备和运行应使用同一Linux用户/HOME。如果提示hash mismatch，先确认使用正确模型/环境，不要直接覆盖已有模型。字符表由安装的HyperLPR3库提供。

## 3. 从小车/Gazebo未启动状态开始

本包不提供相机。原比赛工程中，在ROS环境已加载的终端运行：

```bash
roslaunch robot_gazebo test_community_robot.launch
```

等待/camera/color/image_raw有图像。实车则先启动相机驱动；如果已有相机会话，复用该会话。

## 4. 启动三种识别与三张结果画面

在已加载工作空间、已激活y13、已完成步骤2的终端运行：

```bash
roslaunch robot_perception perception_all.launch \
  python:="$(command -v python)" person_device:=0
```

默认三识别及三image_view窗口。人物阈值0.8沿用当前原统一入口；单独person_detection.launch阈值0.25。可用person_confidence:=0.55等覆盖；高阈值可能漏掉小人，不能保证消除全部灯具误检。

常用参数：show_images:=false（无GUI）、camera_topic:=/自己的图像话题、person_device:=cpu、enable_person/enable_plate/enable_traffic_light:=false、三路*_max_rate、person_confidence、plate_min_confidence。关闭某路但保持show_images=true时，该路窗口可能空白；只要两个功能时可关闭总窗口开关，再手动打开需要的话题。

车牌图中文字需要中文字体。WSL原节点会尝试Windows微软雅黑、文泉驿和Noto CJK。普通Ubuntu可安装fonts-noto-cjk，或给统一入口传入实际字体：

```bash
roslaunch robot_perception perception_all.launch \
  python:="$(command -v python)" person_device:=0 \
  plate_font_path:=/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc
```

包内不分发Windows字体；无中文字体时预览显示plate#编号，JSON车牌号仍正常。

## 5. 单独启动或查看输出

```bash
roslaunch robot_perception person_detection.launch python:="$(command -v python)" device:=0
roslaunch robot_perception plate_recognition.launch python:="$(command -v python)"
roslaunch robot_perception traffic_light_recognition.launch python:="$(command -v python)"
```

以上三条是独立可选入口，勿在全功能launch已启动时重复启动同名节点。单独启动不弹图像窗口，可另开终端用image_view。

```bash
rosrun image_view image_view image:=/perception/person_image
rosrun image_view image_view image:=/perception/plates_image
rosrun image_view image_view image:=/perception/traffic_light_image

rostopic echo -n 1 /perception/person_detections_json
rostopic echo -n 1 /perception/plates_json
rostopic echo -n 1 /perception/traffic_light_json
```

launch默认python3，调用时传入Conda解释器可避免误用ROS Python3.8。launch将可配置ROS Python路径加到继承的PYTHONPATH前，保留工作空间环境。人员权重默认指向包内models/person_best.pt；源码副本的shebang改为env python3。实际WSL运行源代码/权重/缓存未修改。

## 验证证据与待办

本次只做交接文件静态检查：Python语法、XML/YAML、launch参数/资源引用、5个模型与源文件字节及SHA256、ZIP内容/CRC。没有启动ROS/Gazebo/模型推理，也没有执行缓存准备脚本；接收方构建、缓存部署、字体显示及三路实际运行待验证。

历史人物30图test指标mAP50=0.995、mAP50-95≈0.934，且当前仿真正负样本在线首验完成；不代表陌生人物或整场泛化。用户报告灯具误检成人，难负样本再训练尚未执行。车牌曾在用户640×480照片返回贵T37Z85、confidence≈0.9998，与照片相符；单张成功不代表全部角度/距离通过。交通灯有当前视角三色识别用户反馈，完整周期/多视角/背景误检仍待验。三路同时运行的频率、长期稳定性和GUI目前待完整验收。

三个节点输出独立JSON/图像，没有合成单图或同帧融合。主功能包需按docs/接口说明.md检查时间与有效性；现有车牌JSON没有frame_valid，断流靠下游接收超时处理。街区归属、去重、停车/放行和导航联动由M/C协调，节点不控制小车。
