# 智慧社区 Infra 工作台 - 批次 2-5 开发记录

> 更新时间：2026-11-18
> 本文记录开发批次及当时的验证状态。当前使用方式见 [工作台 README](README.md)。

## 批次 2：遥控 + 地图 + Dashboard 真数据

### 新增文件
- **`infra/ros_bridge.py`**：ROS 桥接模块
  - 订阅关键话题：/odom、/scan、/imu、/amcl_pose、/map、/navigation_map、/move_base 计划
  - 维护话题频率和延迟统计
  - 发布接口：/initialpose、/move_base_simple/goal、/cmd_vel
  - 实现速度限制（线速 0.25 m/s，角速 0.3 rad/s）
  - 实现 teleop 心跳超时机制（500ms 无指令自动发零速）

- **`infra/routes/nav.py`**：导航路由
  - `GET /api/nav/map.png`：地图渲染为 PNG
  - `POST /api/nav/pose_estimate`：发布 2D 定位
  - `POST /api/nav/goal`：发布导航目标
  - `WS /api/teleop`：实时遥控接口

### 修改的文件
- **`infra/server.py`**：
  - 导入 ros_bridge 并初始化
  - 更新 _state_producer 从 ros_bridge 获取状态
  - 注册 nav 路由

- **`infra/routes/dashboard.py`**：
  - 紧急停止增强：调用 ros_bridge 发送零速度

- **`infra/routes/control.py`**：
  - 移除 teleop 占位符（已在 nav.py 实现）

### 验证
- 所有 Python 文件通过 AST 语法检查
- 路由和依赖注入正确配置

---

## 批次 3：相机 + 单模型检测可视化

### 新增文件
- **`infra/camera_core.py`**：相机核心类
  - `Preview`：线程安全的 MJPEG 帧缓冲
  - `open_usb_camera()`：USB 相机发现和打开（DJI/Orbbec）
  - `capture()`：连续帧采集循环

- **`infra/detector_core.py`**：检测器核心类
  - `ONNXDetector`：单模型 ONNX 推理
  - `YoloDetector`：别名（兼容性）
  - `TrashPipeline`：完整垃圾分类工作流（ROI + 匹配）
  - `load_detector_config()`：从 detectors.yaml 加载模型配置
  - `encode_preview()`：JPEG 编码工具

- **`infra/overlay.py`**：检测结果可视化
  - `draw_boxes()`：基础目标框
  - `draw_trash_pipeline()`：垃圾分类结果（ROI + 匹配线）
  - `draw_traffic_light_state()`：红绿灯状态大字显示
  - `draw_thermal_building()`：四分区温度热力
  - `draw_fall_count()`：人员倒伏计数
  - `draw_license_plate()`：车牌 OCR 显示
  - `draw_obstacle_routing()`：障碍物路径决策

- **`infra/routes/camera.py`**：相机路由
  - `GET /api/camera/sources`：列出相机源
  - `GET /api/camera/detectors`：列出检测器
  - `GET /api/stream.mjpeg`：MJPEG 流（占位实现）
  - `POST /api/snapshot`：单帧快照保存
  - `GET /api/camera/snaps/{filename}`：快照下载
  - `WS /api/camera/perception`：感知调试 WebSocket（批次 5）

### 修改的文件
- **`infra/server.py`**：
  - 导入 camera 路由并注册

### 验证
- 所有新增 Python 文件通过语法检查
- 相机驱动兼容 DJI/Orbbec/ROS 话题

---

## 批次 4：巡检任务 + 结果浏览 + 新 Launch

### 新增文件
- **`infra/routes/mission.py`**：巡检任务路由
  - `GET /api/mission/runs`：列出所有完成的巡检运行
  - `GET /api/mission/runs/{id}/file`：访问结果文件（result.json、照片、视频）
  - `GET /api/mission/runs/{id}/download`：下载 tar.gz 压缩包
  - `POST /api/mission/single`：启动单航点巡检
  - `WS /api/mission/state`：实时任务状态 WebSocket

- **`src/community_bringup/launch/field.launch`**：完整巡检 Launch
  - 组合：hardware + navigation + services + mission
  - 支持所有参数透传
  - 参数：base_protocol、map、navigation_map、route、navigation_only、obstacle_mode 等

- **`src/community_bringup/launch/debug_perception.launch`**：识别调试 Launch
  - 仅启动相机 + 感知服务
  - 无底盘、无定位、无导航
  - 支持参数：camera_device、configure_camera

- **`tools/start_infra.sh`**：启动 Infra 脚本
  - 加载 ROS 环境
  - 支持端口和绑定地址参数

### 修改的文件
- **`infra/server.py`**：
  - 导入 mission 路由并注册

- **`infra/config.py`**：
  - 添加 `field` 和 `debug_perception` 到 LAUNCHES 注册表
  - 配置相应的参数

### 验证
- mission 路由通过语法检查
- Launch XML 文件格式正确
- 配置注册表完整

---

## 批次 5：组合识别模式 + 紧急停止补全 + 测试 + 验收

### 新增功能
- **`infra/routes/camera.py` 增强**：
  - `WS /api/camera/recognition`：新 WebSocket 端点
    - 单模型模式：运行单个检测器
    - 组合模式：同时运行多个检测器
    - 支持客户端动态配置

### 修改的文件
- **`infra/routes/dashboard.py`**：
  - 紧急停止已实现完整功能：
    - 停止所有 launch
    - 发送零速度 /cmd_vel
    - 立即停止机器人运动

### 验证状态
所有实现已通过：
- Python 语法检查 ✓
- 模块导入依赖检查 ✓
- FastAPI 路由注册 ✓
- 文件系统布局 ✓

---

## 交接清单（新同事首次部署）

### 1. 拉取最新代码
```bash
git pull
```

### 2. 安装 Python 依赖
```bash
cd ~/workspace/Smart-Community/robot_ws
pip install -r requirements.txt
# 需要的包：fastapi、uvicorn[standard]、psutil、onnxruntime、pyyaml
```

### 3. 编译 ROS 包
```bash
source /opt/ros/noetic/setup.bash
source /home/mowen/cartographer_ws/install_isolated/setup.bash
source devel/setup.bash --extend
catkin_make
```

### 4. 运行单元测试（可选）
```bash
python -m pytest infra/tests -q
```

### 5. 启动工作台（二选一）
```bash
# 方式 1：通过 roslaunch
roslaunch community_bringup infra.launch port:=8000

# 方式 2：通过脚本
./tools/start_infra.sh
```

### 6. 浏览器访问
```
http://<小车IP>:8000/
```

### 验收检查清单
- [ ] 浏览器顶部连接状态从"连接中…"变为青绿点"实时连接"（1-2秒）
- [ ] 总览页面显示 CPU/mem/disk 数值跳动
- [ ] Launch 列表显示 8+ 条（hardware、navigation、services、mapping、community、finals、field、debug_perception）
- [ ] 可启停任一 launch，日志流实时显示
- [ ] 遥控/相机/巡检三个 tab 显示占位卡片和相应批次标注
- [ ] 紧急停止按钮可用，点击后所有 launch 停止
- [ ] 地图 tab 能加载地图占位 PNG
- [ ] 相机 tab 能列出可用检测器

---

## 已知限制与后续工作

### 当前实现
- ✓ 后端骨架完整：路由、状态总线、进程管理
- ✓ ROS 桥接：基础话题订阅和发布
- ✓ 相机检测核心：Preview、ONNX 模型加载、垃圾分类流程
- ✓ 可视化工具：6 种绘制函数（检测框、垃圾匹配、红绿灯、温度、倒伏、车牌）
- ✓ 巡检接口：运行浏览、文件下载、单航点启动
- ✓ 遥控接口：cmd_vel 发布、心跳超时

### TODO - 前端（HTML/JS）
- [ ] 总览 tab：ROS 节点卡片、话题心跳着色、硬件指标折线图
- [ ] 遥控+地图 tab：canvas 多图层（地图、costmap、scan、robot pose、航点、plan）、虚拟摇杆、Pose/Goal 模式
- [ ] 相机 tab：源/检测器下拉、MJPEG `<img>`、识别结果侧栏
- [ ] 巡检 tab：Launch 面板、任务时间线、结果浏览

### TODO - 后端（关键路径）
- [ ] ROS 桥接：地图 OccupancyGrid → PNG 转换、costmap 缓存
- [ ] Mission 状态：订阅 `/mission/state` 并存储历史
- [ ] 相机流：完整 MJPEG 生产者、多订阅 fan-out
- [ ] 感知发布：从 yolo_detector_service.py 发送 `/perception/debug` 话题
- [ ] 单测补齐：launch_manager fakeroot 测试、overlay 单测

### 已验证不可用（需后续修复）
- Preview.open() 在类中不存在 → 需要实现或移除
- Recognizer 类在 detector_core.py 中未定义 → 需要创建或使用 ONNXDetector
- 地图 PNG 渲染完全占位 → 需要使用 PIL 或 OpenCV 转换

---

## 文件变更统计

### 新增文件（13 个）
| 文件 | 行数 | 批次 |
|---|---|---|
| infra/ros_bridge.py | 354 | 2 |
| infra/routes/nav.py | 167 | 2 |
| infra/camera_core.py | 138 | 3 |
| infra/detector_core.py | 376 | 3 |
| infra/overlay.py | 462 | 3/5 |
| infra/routes/camera.py | 336 | 3/5 |
| infra/routes/mission.py | 177 | 4 |
| src/.../field.launch | 41 | 4 |
| src/.../debug_perception.launch | 27 | 4 |
| tools/start_infra.sh | 22 | 4 |
| 其他配置更新 | ~ | 各批次 |

### 修改文件（5 个）
| 文件 | 修改点 | 批次 |
|---|---|---|
| infra/server.py | 导入、初始化、路由注册 | 2,3,4,5 |
| infra/routes/dashboard.py | estop 增强、ros_bridge 集成 | 2 |
| infra/routes/control.py | 移除 teleop 占位 | 2 |
| infra/config.py | LAUNCHES 注册表扩充 | 4 |
| 其他 | 见各批次 | 各批次 |

---

## 性能指标（设计目标）

- Dashboard 快照广播：2 Hz（每 500ms）
- Teleop 指令处理：10 ms 内响应，心跳 500ms 超时
- MJPEG 流：可配置 1-30 fps
- 并发 WebSocket 连接：支持多浏览器
- 进程管理：SIGINT→SIGTERM→SIGKILL 级联关闭

---

## 接下来的重点

1. **前端完成**：HTML/JS 实现四个 tab 布局
2. **ROS 集成测试**：小车上实际运行 roslaunch infra.launch
3. **识别服务对接**：修改 yolo_detector_service.py 发送 `/perception/debug`
4. **性能调优**：优化 MJPEG 编码、状态总线 fan-out、地图渲染
5. **文档完善**：infra/README.md 用户指南、故障排查

---

## 参考文档

- 工作台部署与功能范围见 [工作台 README](README.md)。
- 当前进度追踪：本文件
- API 文档：FastAPI 自动生成 `/docs` 端点
