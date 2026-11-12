# Smart Community Infra

基于实车 `robot_ws` 的 Vue 3 + FastAPI 调试工作台。所有新增内容位于本目录；复用现有 Launch、ONNX 前后处理、ROS 服务和巡检结果文件，不修改比赛程序及仿真工作空间。

## Ubuntu 安装与启动

目标环境：Ubuntu 20.04 / ROS Noetic、Python 3.8+、Node.js 20.19+。本机需要编译实车工作空间，以获得 `community_perception.srv` 等自定义消息；本机无需连接串口设备。实车仍按工作空间 README 配好硬件和 ROS 多机通信。

```bash
sudo apt install python3-venv python3-pip python3-opencv python3-yaml ros-noetic-cv-bridge ros-noetic-tf2-ros
cd /你的路径/Smart-Community/robot_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
cd infra
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
cd frontend
npm ci
npm run build
cd ..
# 仅在 Ubuntu 上启动；不使用 --reload 或多 worker，避免重复 ROS 节点
uvicorn backend.main:app --host 127.0.0.1 --port 8000
```

访问 `http://127.0.0.1:8000`。前端开发可另开终端在 `frontend` 执行 `npm run dev`，Vite 会代理 `/api` 与控制 WebSocket 到 8000。不需要启动其他浏览器预览或采集脚本。

## ROS 与 SSH 配置

启动前修改 `config.json`：

- `master_uri`：实车 Master，例如 `http://192.168.31.59:11311`。
- `ros_ip`：本机 Ubuntu 的局域网 IP，必须能从实车反向连接。
- `ssh_host`：已有 SSH 配置别名，默认 `mowen`；要求已确认 host key、配置免密登录。留空表示在本机启动 Launch。
- `remote_workspace`：执行 Launch 的机器上的绝对实车工作空间路径；该机器须已编译工作空间。
- `remote_results`：同一机器的绝对巡检结果目录。
- `media_root`：工作台本机默认媒体保存目录。页面可临时修改保存目录、数据集。

本机和实车应 `unset ROS_HOSTNAME`。实车启动节点前设置 `ROS_MASTER_URI` 为实车地址、`ROS_IP` 为实车地址，并加载 Noetic 和实车工作空间环境。本机桥接根据配置设置 ROS 环境；页面修改 IP 只保存配置，重启后生效。SSH 启动的实车节点使用 Master URI 的主机部分作为 ROS_IP，所以 Master URI 请填写实车数字 IP。检查 `ip route get 实车IP`、两端双向可达、11311 及 ROS 动态端口可达。无需在本机另启 roscore。

后端默认绑定回环地址，API 没有鉴权，只用于可信本机调试；从其他电脑访问时优先 SSH 转发端口。

## 页面使用

- **系统总览**：Master 可达性、ROS 注册节点、关键 Topic 新鲜度及 TF 自检，按组启动/停止硬件、建图、导航、视觉等服务，查看 `/rosout_agg`。节点列表表示 Master 注册信息；设备健康以最近 3 秒收到的数据为依据，相机窗口为 2 秒。
- **底盘**：先停止导航、巡检和其他 `/cmd_vel` 发布者，再选手动模式并解锁。按住方向按钮或 Q/W/E、A/D、Z/S/C 八方向移动，R/F 旋转，松开停止。输入框内不响应快捷键。速度上限 X/Y 各 0.5 m/s、Yaw 1 rad/s。显示 `/odom` 速度和 odom 坐标下的 ±5 m 轨迹。
- **视觉**：选择现有六个模型，独立 ONNX 测试支持临时 Confidence / IoU。ROS 服务测试和 OCR 使用既有服务配置，不修改外部节点参数。连续测试逐帧串行执行，显示本次调用耗时及其倒数计算的处理 FPS（含传输/首次加载，不代表相机 FPS）。可保存对应原图、绘制结果图和结果 JSON。
- **地图**：启动 GMapping 或 Cartographer，保存地图（填写实车绝对文件前缀，父目录须已存在）；停止建图后加载 YAML 启动导航。自动模式解锁后，选择初始位姿/目标工具，设定 yaw 弧度并点击地图。显示占据栅格、经 TF 转到 map 的雷达、机器人、全局和局部路径；不具备完整 RViz 功能。
- **巡检**：导航就绪后启动完整巡检或仅导航。可选实车路线文件；直接读取最近一次 `mission_results.json` 和 `mission.log`，显示航点状态和结果。所显示的最近结果可能属于此前任务，请核对 run_id。
- **媒体**：选择已有 ROS `sensor_msgs/Image` Topic，支持拍照、定时采集、10 FPS MJPEG AVI 录像、文件浏览/下载和所选文件 ZIP 导出。DJI/Orbbec 也应通过已有 ROS 相机节点发布图像；Infra 不打开设备。录像请下载后用支持 MJPEG AVI 的播放器播放。停止录像后再下载/导出文件。

媒体保存位置属于本机；地图、路线和任务 JSON 路径属于 Launch 所在机器。定时采集和录像依赖工作台后端；退出会结束本机采集。

## 控制与进程边界

- 初始控制锁定。手动指令每 100 ms 续租，350 ms 未续租、窗口失焦、按键/按钮释放或 WebSocket 断连会停车。ROS 离线也拒绝运动；后台每 50 ms 检查租约和其他速度发布者。
- 自动模式不持续发布零速。软件急停会取消导航、发布零速、锁定 Infra，并停止 Infra 自己的巡检进程；急停期间持续发零速和取消目标，解除前先停止巡检。无法禁止外部控制器重新发命令，因此仍须停止外部控制器并保留物理急停。现有底盘 `command_timeout` 机制保持不变。
- 建图/导航互斥；导航/巡检须自动模式解锁；有外部速度发布者或导航/巡检节点时不能进入手动模式。ROS1 没有全局速度仲裁保证，操作时不要从其他终端新增控制器；检测到竞争发布者后 Infra 停止自己的运动指令。
- 通过 SSH 或本机 detached session 启动 Launch；仅停止 `data/processes.json` 记录且唯一日志文件身份匹配的进程组，绝不调用 `rosnode kill` 清理外部节点。可跨工作台重启继续管理，勿在运行中删除记录或更改 SSH 目标。启动失败请查看返回的 `/tmp/community-infra-*.log`（在执行 Launch 的机器上）。
- 关闭浏览器只释放手动控制；关闭后端不会停止远端导航/巡检，也不会取消自动模式下的目标。手动控制时退出会发零速。需要停止自主任务时显式点击停止或急停。

## 已做检查与实车验收

当前 Windows 环境仅做代码编写和静态构建，**没有启动后端、ROS、SSH Launch 或实车**。

已验证：后端 Python 3.8 语法解析、JSON 配置解析、Vue/Vite 生产构建、Git 空白检查。依赖安装时 npm audit 返回 0 漏洞。

仍须在 Ubuntu 与实车验证：Python/ROS 自定义消息导入与多机通信、首次离线及重连；SSH 启停/重启后进程身份核对；八方向/旋转与里程计；按键释放/失焦/断网/后端退出及底盘超时停车；外部速度发布者拦截、急停与自主任务保留；六模型与 OCR 真实效果和耗时；地图方向/TF/路径、初始化定位、目标与取消、两种建图及地图保存加载；两种巡检模式与结果文件；相机来源、采集间隔、录像与导出。请先架空车轮、低速验证停车边界，再上场导航。
