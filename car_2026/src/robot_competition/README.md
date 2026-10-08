# robot_competition P1 / P2-B

已实现路线管理、TF 航点录入、move_base 多航点状态机，以及到点后的人员/车牌结果订阅、多帧融合和持久化。模型仍由 `robot_perception` 独立运行，主程序使用系统 ROS Python 3.8。灯色识别与放行未实现，旧视觉占位脚本不安装、不启动。

**P2-B 的四终端启动、全部新增参数、消息/结果 schema、统计边界、字体配置和自动测试记录见 [P2B.md](P2B.md)。** 本文保留 P1 航点录入及导航操作说明；下方 2026-10-08 记录仅表示当时的 P1 验证范围。

## 文件职责与运行逻辑

- `package.xml`、`CMakeLists.txt`：Noetic Python 3 依赖和节点/资源安装。
- `scripts/waypoint_manager.py`：无 ROS 依赖的 YAML 加载、严格校验和命令行检查。
- `scripts/waypoint_recorder.py`：交互录入 `map -> base_footprint` TF 位姿；显式追加、重复 ID 拒绝、文件锁与同目录原子替换。
- `scripts/main_controller.py`：Action 通信、任务调用、状态转换和安全终止。
- `scripts/perception_client.py`：持续订阅 JSON/标注图，严格校验、时间窗口与有界缓存。
- `scripts/task_processor.py`：有限重试、人员短时匹配、车牌全文投票、同源证据匹配。
- `scripts/result_manager.py`：按点/区域/全局保守统计，原子 JSON、日志及有限标注图片。
- `config/task_config.yaml`：默认采集、融合规则和计数分区约定。
- `config/waypoints.yaml`：默认空路线，未提供任何猜测的比赛坐标。
- `launch/competition.launch`：仅启动主程序。
- `tests/test_p1.py`、`tests/run_docker_checks.sh`：隔离 ROS Master 下的 schema、保存、TF 和假 Action Server 测试；不证明实际 Gazebo 导航可用。

状态：`INIT → NAVIGATE → TASK（仅任务点）→ NEXT_POINT → NAVIGATE / FINISH`，错误进入 `ERROR` 并取消本节点目标、停止后续发送。空路线直接 `FINISH`。只有 Action 的 `SUCCEEDED` 才算到点。失败和超时可有限重试；收到外部取消或 LOST 则终止。取消确认超时不会重发目标。服务器、导航、取消及仿真时钟 watchdog 均使用墙上时钟上限，不依赖 `/clock` 继续走动。

人员/车牌点在导航成功并等待稳定时间后开启新帧采集窗口；每次识别结果立即保存，普通识别失败有限重试后继续路线。时钟失效、关闭、存储异常仍进入 ERROR。正式模式在 INIT 检查所有交通灯点的 `stop_before_line: true`；任一点未确认就拒绝整条路线。到达已确认交通灯点后，以 `ERROR / TRAFFIC_LIGHT_NOT_IMPLEMENTED` 明确结束，不发送后续目标。这里预留未来 `WAIT_TRAFFIC` 接口。`navigation_test_mode` 默认 false；显式 true 时跳过视觉并记录 `SKIPPED_NAVIGATION_TEST`，也绕过交通灯占位，仅用于隔离的纯导航测试。

## YAML 格式

根对象必须是 `route`，其中 `frame_id: map`、`points` 为有序列表。每点必填 `id`、`type`、`task`、`x`、`y`、`yaw`。`type: waypoint` 对应 `task: none`；`type: task` 对应 `person / plate / traffic_light`。坐标单位为米、yaw 为弧度，必须为有限数值（拒绝布尔、字符串、null、NaN、Infinity）。ID 必须唯一且非空；拒绝重复 YAML 键及未知字段。

可选字段：`area`、`description`（字符串），`navigation_timeout`（正数秒）、`retries`（非负整数，表示首次之外的重试次数）；交通灯点额外允许布尔 `stop_before_line`。点级超时/重试覆盖 launch 的默认值。

下面仅是结构示例，**null 会被拒绝，不可直接导航**：

```yaml
route:
  frame_id: map
  points:
    - id: light_01
      type: task
      task: traffic_light
      x: null
      y: null
      yaw: null
      stop_before_line: false
      description: 待实测确认整个车身位于停止线前
```

`stop_before_line: true` 是操作者的实测确认，程序不自动验证停止线。应考虑车体、轮子外缘、定位误差、move_base 到点容差及制动余量；既有导航位置容差为 0.08 m，不能只看机器人中心。航点间路径也必须人工验证沿比赛指定道路，不得假定仅端点安全就能保证全程不越线。

## Docker 构建与检查

宿主机只使用 Docker 入口；所有以下 ROS/Python/catkin 命令均在容器终端运行。进入现有容器（每个终端各执行一次）：

```bash
docker exec -it --user developer smart-community-dev bash
```

容器内：

```bash
source /opt/ros/noetic/setup.bash
cd /workspace/car_2026
catkin_make
source devel/setup.bash
bash src/robot_competition/tests/run_docker_checks.sh
rosrun robot_competition waypoint_manager.py src/robot_competition/config/waypoints.yaml
```

测试脚本使用本机独立端口 11329，要求该端口未被占用；ROS 日志保存在容器 `/tmp/robot-competition-tests.*`。测试不发送到 `/move_base`。

## 航点录入的三终端启动与定位

三个容器终端先执行 `source /opt/ros/noetic/setup.bash`、`cd /workspace/car_2026`、`source devel/setup.bash`。

终端 1：

```bash
roslaunch robot_gazebo simulation.launch rviz:=true
```

终端 2：

```bash
roslaunch robot_navigation navigation.launch
```

在 RViz 设置 Fixed Frame 为 `map`，显示 `/map`、LaserScan `/scan`、TF、RobotModel。用 **2D Pose Estimate** 设置 AMCL 初始位置和朝向，检查激光与墙体/地图对齐，移动后仍稳定。用 **2D Nav Goal** 将机器人逐段移动到合规位置，等待到点并检查朝向；也可使用既有遥控工具移动，但不要与主控制器同时发送命令。录入时主程序必须尚未运行。

终端 3 录入新文件：

```bash
rosrun robot_competition waypoint_recorder.py --output /workspace/car_2026/route_measured.yaml
```

交互示例（每条 `add` 之前在 RViz 移动到对应位置、等停稳并检查定位）：

```text
add wp_01 waypoint none description="实测普通航点"
add person_01 task person area=block_01
add plate_01 task plate area=parking
add light_01 task traffic_light stop_before_line=true description="已确认全车停止线前且留足余量"
quit
```

最后一条只在确实检查了全车停车安全后使用。未确认时录入 `stop_before_line=false`，正式运行将拒绝该路线。每次成功录入立即保存；输入错误、TF 缺失/陈旧、重复 ID、写入失败均提示且不加入该点。TF 可用不等于 AMCL 定位准确，仍须目视检查。使用 `quit`、Ctrl-D 或 Ctrl-C 退出。再次添加必须显式：

```bash
rosrun robot_competition waypoint_recorder.py --output /workspace/car_2026/route_measured.yaml --append
rosrun robot_competition waypoint_manager.py /workspace/car_2026/route_measured.yaml
```

不提供隐式覆盖或替换点操作；需改点时先备份 YAML，人工修改后重新验证。`.lock` 是协作录入锁文件，可保留；其他编辑器不遵守此锁，勿同时修改同一文件。

完成录入后，主程序入口如下；人员/车牌正式联调还需先启动独立视觉终端，完整步骤见 [P2B.md](P2B.md)：

```bash
roslaunch robot_competition competition.launch waypoints_file:=/workspace/car_2026/route_measured.yaml
```

仅在已检查环境安全的纯导航测试中，才显式启用：

```bash
roslaunch robot_competition competition.launch waypoints_file:=/workspace/car_2026/route_measured.yaml navigation_test_mode:=true navigation_timeout:=90 retries:=1
```

参数：`move_base_action` 默认 `/move_base`；`server_timeout` 15 秒，`navigation_timeout` 90 秒，`cancel_timeout` 3 秒，`clock_timeout` 5 秒，`retries` 1。暂停 Gazebo 超过 watchdog 时限会终止任务；重启任务从第一个航点开始，须先检查机器人位置与路线。任何时刻只运行一个主控制器；不要同时发送 RViz Goal。

## 故障定位（容器内）

```bash
rosnode list
rostopic echo -n 1 /clock
rosrun tf2_ros tf2_echo map base_footprint
rostopic echo -n 1 /amcl_pose
rostopic echo -n 1 /move_base/status
rosparam get /move_base/global_costmap/robot_base_frame
rosparam get /main_controller
```

若 TF 不可用，先检查 AMCL 初始位姿、激光、EKF 与 `/clock`；若目标超时，检查地图、costmap、TEB 和实际通道宽度。主程序只取消自己发送的目标，不接管其他客户端或直接输出速度。Action 通信丢失时取消不能保证已被导航端接收，应先检查底盘已停止再恢复任务。

## 验收边界

自动测试覆盖配置异常、录入保存往返、写入故障保护、TF 位姿转换/陈旧拒绝、Action 成功/失败/超时/取消、重试、关闭回调、交通灯默认拦截与显式测试模式，以及墙上时钟 watchdog。假 Action Server 仅用于测试；其中合成坐标不作为正式路线。

真实 Gazebo 多航点、完整指定路线合规、AMCL 精度、窄转角与全车停止线前余量仍须使用实测路线验证。上述三终端启动命令是操作步骤，不能仅凭构建和自动测试成功声称已通过真实仿真验收。

## 本次实际验证记录（2026-10-08）

运行环境：已有 `smart-community-dev` 容器，`developer` 用户，ROS Noetic / Python 3.8。宿主机仅进行文件编辑、Git 检查和 Docker 操作，未运行项目 Python、ROS 或 catkin。

已执行：

```bash
# 以下是宿主机调用 Docker 的入口，实际命令全部在容器内执行。
docker exec --user developer smart-community-dev bash -lc 'source /opt/ros/noetic/setup.bash; cd /workspace/car_2026; catkin_make -j2'
docker exec --user developer smart-community-dev bash /workspace/car_2026/src/robot_competition/tests/run_docker_checks.sh
```

结果：

- `catkin_make -j2`：整工作空间 5 个包构建成功，退出码 0。
- `python3 -m py_compile`：三个实现脚本语法通过。
- `test_p1.py`：最终 21 项测试通过，退出码 0；真实 ROS Action 消息通信连接隔离假服务器，模拟时钟停滞使用固定时钟注入。
- `rosrun ... waypoint_manager.py`：默认空路线校验通过，明确输出 `EMPTY_ROUTE`。
- `rosrun ... waypoint_recorder.py --help`：入口及共享模块导入通过。
- `rosrun ... main_controller.py _waypoints_file:=...`：默认空路线 `INIT → FINISH`，不发送目标。
- `roslaunch --nodes robot_competition competition.launch`：解析通过，仅列出 `/main_controller`。
- `git diff --check`：通过。

最终容器测试日志：`/tmp/p1-checks.log`（容器临时文件，不纳入仓库）。初轮测试发现重复取消已完成目标及测试服务器退出清理问题，已修正后重跑；最终测试没有 Action 通信状态错误。ROS 自身可能输出 XML-RPC socket 的 ResourceWarning，不影响测试结论。

尚未运行：Gazebo + AMCL + 真实 `/move_base` 多航点巡检，以及真实 TF 交互录入。检查环境时没有运行 ROS Master，且本次未提供已实测路线，因此没有启动仿真自动行车。P1 的真实路线验收仍未完成。

构建中观察到既有 `robot_mecanum_control/CMakeLists.txt:10` 和 `robot_gazebo/CMakeLists.txt:23` 的 `catkin_package(DEPENDS gazebo)` 导出警告：对应 `gazebo_INCLUDE_DIRS / gazebo_LIBRARIES` 未定义。当前不阻塞构建，未修改这些包；后续可独立核对 Gazebo 的 CMake 变量及导出声明。既有 Gazebo Classic 依赖还会输出弃用提示，本次未变更基础环境。
