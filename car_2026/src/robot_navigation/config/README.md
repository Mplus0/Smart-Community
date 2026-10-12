# 导航参数调试指南

本文按 2026-10-12 仓库中的配置整理，表中的值是文件当前值，不是上游默认值；运行时以 `rosparam get` 和动态参数界面为准。距离单位为 m，时间为 s，角度为 rad，线速度为 m/s，角速度为 rad/s。部分 YAML 注释仍写着禁止横移或差速式导航，应以实际参数值为准。

## 1. 配置入口与调试顺序

[导航启动文件](../launch/navigation.launch) 启动 AMCL、两份地图和 move_base，使用全局规划器 `global_planner/GlobalPlanner`、局部规划器 `teb_local_planner/TebLocalPlannerROS`。

| 配置文件 | 作用与运行时位置 | 优先级 |
| --- | --- | --- |
| [teb_local_planner.yaml](teb_local_planner.yaml) | 速度、转弯、到点、局部避障；`/move_base/TebLocalPlannerROS` | 高 |
| [costmap_common.yaml](costmap_common.yaml) | 车体轮廓、通用传感器配置；分别加载至两个 costmap | 高 |
| [global_costmap.yaml](global_costmap.yaml) | 全局代价地图；`/move_base/global_costmap` | 高 |
| [local_costmap.yaml](local_costmap.yaml) | 局部代价地图；`/move_base/local_costmap` | 高 |
| [move_base.yaml](move_base.yaml) | 规划频率、超时、恢复；`/move_base` | 中 |
| [amcl.yaml](amcl.yaml) | 定位；`/amcl` | 定位异常时优先 |
| [forbidden_zones.yaml](forbidden_zones.yaml) | 规划地图中的禁区多边形；由离线脚本读取 | 路径被禁区影响时优先 |
| [cartographer_2d.lua](cartographer_2d.lua) | Cartographer 建图；仅 `mapping.launch` 加载 | 重新建图时使用 |

建议按“定位和 TF → 车体轮廓/地图 → 低速运动 → 避障 → 到点精度 → 速度和效率”调试。每次只修改一组相关参数，并用相同起点、目标、地图重复测试。

## 2. TEB：速度、转向与到点

以下参数均位于 `teb_local_planner.yaml` 的 `TebLocalPlannerROS` 下。

| 参数 | 当前值 | 作用与调试关注点 |
| --- | --- | --- |
| `max_vel_x` | `0.25` | 前进速度上限。先低速验证路径，再逐步增加；同时观察实际里程计速度和制动距离。 |
| `max_vel_x_backwards` | `0.08` | 倒车速度上限。倒车过慢时检查此项。 |
| `max_vel_y` / `acc_lim_y` | `0.05` / `0.10` | 当前允许小幅横移；同时检查侧向加速度、运动学约束和底层控制。 |
| `max_vel_theta` | `0.45` | 转向角速度上限，约 25.8°/s。 |
| `acc_lim_x` / `acc_lim_theta` | `0.25` / `0.55` | 纵向/角加速度上限，单位 m/s²、rad/s²。关注起停冲击、转向跟随。 |
| `min_turning_radius` | `0.0` | 当前允许原地转向。 |
| `allow_init_with_backwards_motion` | `true` | 允许使用倒车运动初始化轨迹。 |
| `xy_goal_tolerance` | `0.08` | 到点位置容差为 8 cm；缩小前先确认定位和控制精度。 |
| `yaw_goal_tolerance` | `0.05` | 到点朝向容差约 2.9°；太小时可能反复转向。 |
| `free_goal_vel` | `false` | 规划要求终点停车。 |
| `complete_global_plan` | `true` | 完成全局路径的要求；调试中先保持。 |
| `odom_topic` | `/odometry/filtered` | 读取 EKF 里程计；先确认话题持续更新。 |

到点容差属于导航器的判定条件；如果比赛任务仍不切换到下一站，还需检查任务控制器自己的判定条件，不能只改 TEB。

## 3. TEB：轨迹、避障与优化

| 参数 | 当前值 | 调试关注点 |
| --- | --- | --- |
| `dt_ref` / `dt_hysteresis` | `0.25` / `0.10` | 轨迹时间离散及自动调整滞回，单位 s；更细的离散通常增加计算量。 |
| `max_global_plan_lookahead_dist` | `1.0` | 参与局部优化的全局路径前瞻长度，也受局部地图大小限制。 |
| `global_plan_overwrite_orientation` | `true` | 根据路径方向处理局部子目标朝向。 |
| `feasibility_check_no_poses` | `5` | 前部轨迹可行性检查的位姿数量。 |
| `min_obstacle_dist` | `0.04` | 优化期望的障碍间距；结合当前 polygon 车体模型理解，不是车中心到障碍的距离，也不是实际间距保证。 |
| `inflation_dist` | `0.12` | TEB 障碍惩罚缓冲距离，应大于 `min_obstacle_dist`。 |
| `include_costmap_obstacles` | `true` | 将代价地图障碍纳入局部优化。 |
| `costmap_obstacles_behind_robot_dist` | `1.0` | 纳入规划的车后方代价地图障碍距离。 |
| `weight_obstacle` / `weight_inflation` | `50.0` / `0.20` | 障碍距离及缓冲区惩罚权重。 |
| `weight_optimaltime` | `1.0` | 对时间最优的重视程度。 |
| `weight_kinematics_nh` | `100.0` | 非完整运动学约束权重；与当前允许小幅横移的设定一起检查。 |
| `weight_kinematics_forward_drive` | `10.0` | 前进行驶偏好。 |
| `no_inner_iterations` / `no_outer_iterations` | `5` / `4` | 优化迭代数；增加时同步检查 CPU 和控制周期。 |
| `enable_homotopy_class_planning` | `false` | 当前关闭多拓扑候选路径规划，先固定此项减少变量。 |

速度限制和避障参数涉及优化权重，不能把配置数值当作每一时刻的实际运动保证。结合 `/cmd_vel`、`/odometry/filtered` 和实测最小障碍间距验收。参数语义参见 [TEB 上游参数定义](https://github.com/rst-tu-dortmund/teb_local_planner/blob/noetic-devel/cfg/TebLocalPlannerReconfigure.cfg)。

## 4. 车体轮廓与代价地图

`costmap_common.yaml` 的 `footprint` 和 TEB 的 `footprint_model.vertices` 当前一致：

```yaml
[[-0.155, -0.141], [-0.155, 0.141], [0.155, 0.141], [0.155, -0.141]]
```

对应长 0.310 m、宽 0.282 m 的近似矩形。costmap 另有 `footprint_padding: 0.005`。修改车体尺寸时同步检查两个轮廓；不要通过缩小真实轮廓来让规划路径穿过窄道。

| 参数相对路径 | 全局当前值 | 局部当前值 | 作用与调试关注点 |
| --- | --- | --- | --- |
| `global_frame` | `map` | `map` | 局部地图也使用 map，AMCL 跳变会影响局部规划。 |
| `robot_base_frame` | `base_footprint` | `base_footprint` | 机器人基准坐标系。 |
| `update_frequency` | `5.0` Hz | `10.0` Hz | 内部地图更新频率。 |
| `publish_frequency` | `2.0` Hz | `5.0` Hz | 发布频率；提高它不等于提高内部更新频率。 |
| `rolling_window` | `false` | `true` | 局部地图窗口跟随机器人。 |
| `width` / `height` | 随静态地图 | `2.0` / `2.0` m | 局部观察窗口；增加会扩大计算范围。 |
| `resolution` | common 配置 `0.02` m | `0.02` m | 当前静态地图也为 0.02 m；全局静态层还会按地图尺寸/分辨率调整。 |
| `transform_tolerance` | `0.5` s | `0.5` s | costmap 对位姿 TF 时效的容忍；TF 故障先查时钟和发布链。 |
| `static_layer.map_topic` | `/planning_map` | `/planning_map` | 两者都包含虚拟禁区。 |
| `static_layer.subscribe_to_updates` | `false` | `false` | 不订阅地图增量更新。 |
| `obstacle_layer.laser_scan_sensor.obstacle_range` | `3.0` m | `2.5` m | 激光障碍标记范围。 |
| `obstacle_layer.laser_scan_sensor.raytrace_range` | `4.0` m | `3.0` m | 射线清除障碍的范围。 |
| `inflation_layer.inflation_radius` | `0.15` m | `0.15` m | 障碍膨胀传播范围；不等于车体外额外留出 15 cm。 |
| `inflation_layer.cost_scaling_factor` | `4.0` | `4.0` | 越大代价衰减越快，越小衰减越慢。 |

当前轮廓加 padding 后的内切半径约 0.146 m，膨胀半径为 0.15 m，留给外围代价梯度的范围很窄。若希望路径更早避开墙边，可优先观察膨胀范围，再调整衰减；这是基于当前几何尺寸的判断，仍需实测。衰减公式见 [InflationLayer 实现](https://github.com/ros-planning/navigation/blob/noetic-devel/costmap_2d/include/costmap_2d/inflation_layer.h)。

注意加载层级：common 文件先加载到两个 costmap 根命名空间，随后加载各自配置。显式插件使用 `obstacle_layer/...` 配置，激光范围应优先改上表中的位置，不要只改 common 顶层的 `obstacle_range` / `raytrace_range`。common 中 `laser_scan_sensor.inf_is_valid: true` 也没有显式写入各自的 `obstacle_layer.laser_scan_sensor`；需要依赖无穷远激光清障时，应核实插件命名空间和实际版本行为。参见 [ObstacleLayer 参数读取](https://github.com/ros-planning/navigation/blob/noetic-devel/costmap_2d/plugins/obstacle_layer.cpp)。

## 5. move_base：频率、超时和恢复

| 参数 | 当前值 | 作用与调试关注点 |
| --- | --- | --- |
| `planner_frequency` | `2.0` Hz | 全局路径重规划频率。 |
| `controller_frequency` | `10.0` Hz | 局部控制目标频率，周期 0.1 s；关注 missed rate 警告。 |
| `planner_patience` / `controller_patience` | `5.0` / `5.0` s | 规划/控制失败后的等待阈值。 |
| `recovery_behavior_enabled` | `true` | 启用恢复行为。 |
| `clearing_rotation_allowed` | `false` | 默认清障恢复不允许旋转；不禁止正常导航转向。 |
| `oscillation_timeout` | `10.0` s | 振荡检测超时。 |
| `oscillation_distance` | `0.10` m | 重置振荡计时所需移动距离。 |

不要先用增大超时掩盖“定位不准、禁区封路、局部轨迹不可行”。先在 RViz 确认全局路径、局部轨迹和障碍轮廓。

## 6. AMCL：定位

| 参数 | 当前值 | 作用与调试关注点 |
| --- | --- | --- |
| `min_particles` / `max_particles` | `300` / `2000` | 粒子数量上下限；增加会增加计算量。 |
| `kld_err` | `0.02` | 自适应采样误差控制，减小通常需要更多粒子。 |
| `update_min_d` / `update_min_a` | `0.02` m / `0.03` rad | 触发滤波更新的位移/转角阈值；当前已经较小。 |
| `laser_max_beams` | `60` | 每次定位使用的激光束数。 |
| `laser_min_range` / `laser_max_range` | `0.10` / `8.0` m | 定位使用的激光距离范围。 |
| `laser_sigma_hit` | `0.20` m | 激光匹配模型的误差尺度。 |
| `odom_model_type` | `omni-corrected` | 全向里程计模型，适用于麦轮底盘。 |
| `odom_alpha1`～`odom_alpha5` | 均为 `0.10` | 里程计运动噪声系数；按直行、旋转、侧移误差调节，不是速度参数。 |
| `recovery_alpha_slow` / `recovery_alpha_fast` | `0.0` / `0.0` | 当前关闭基于平均权重的随机粒子恢复机制。 |
| `transform_tolerance` | `0.5` s | AMCL 发布 TF 的未来有效时间，与 costmap 同名参数含义不同。 |

先在 RViz 用 **2D Pose Estimate** 设置初始位姿，观察 `/scan` 是否与 `/map` 的墙体对齐，再进行导航。AMCL 使用定位地图，不能为修复定位直接把虚拟禁区画入定位地图。参数语义参见 [AMCL 上游定义](https://github.com/ros-planning/navigation/blob/noetic-devel/amcl/cfg/AMCL.cfg)。

## 7. 禁区和建图配置

`forbidden_zones.yaml` 中有 `parking_area`、`area_1`、`area_2`。`points` 是 map 坐标系下以米为单位的多边形顶点，按边界顺序填写，避免自交。文件不会由导航 launch 直接加载，修改后必须重新生成规划地图：

```bash
cd /workspace/car_2026
python3 src/robot_navigation/scripts/generate_planning_map.py
```

此命令读取 `localization_map.pgm/.yaml` 和禁区配置，覆盖 `maps/planning_map.pgm`、`maps/planning_map.yaml`；随后重启导航使地图服务器重新加载。在 RViz 检查 `/planning_map` 是否符合预期。

`cartographer_2d.lua` 仅在建图时使用，修改它不会改变当前 AMCL 导航。常用项：

| 参数 | 当前值 | 调试用途 |
| --- | --- | --- |
| `TRAJECTORY_BUILDER_2D.min_range` / `max_range` | `0.10` / `8.0` m | 建图激光范围。 |
| `TRAJECTORY_BUILDER_2D.use_imu_data` / `options.use_odometry` | `true` / `true` | 使用 IMU 和里程计，先检查输入质量。 |
| `TRAJECTORY_BUILDER_2D.submaps.num_range_data` | `35` | 子图插入的测距数据数量相关设置。 |
| `TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window` | `0.1` m | 实时相关匹配的平移搜索窗口。 |
| `POSE_GRAPH.optimize_every_n_nodes` | `35` | 后端优化间隔。 |
| `POSE_GRAPH.constraint_builder.min_score` / `global_localization_min_score` | `0.60` / `0.70` | 约束匹配得分门槛。 |

当前 Cartographer 的 `published_frame = "odom"`、`provide_odom_frame = false`，EKF 提供 `odom → base_footprint`。建图与 AMCL 导航应分开运行，避免同时发布 `map → odom`。

## 8. 修改、生效和验收命令

以下在 ROS Noetic 容器内执行。Gazebo 已运行时直接复用，不要重复启动仿真。修改本目录 YAML/Lua 通常无需重新编译；文件保存不会自动刷新已运行节点。

若通过 `src/tools/start.sh` 或 GUI 启动，导航已包含在 `environment.launch` 中，不要再独立启动同名导航节点。需要单独调参时，先停止集中环境，再按 [工作空间 README](../../../README.md) 使用分终端流程。当前 `navigation.launch` 只暴露 `use_sim_time`；更换地图需检查其规划地图路径以及所包含 `localization.launch` 的定位地图路径，不能直接向导航入口传入不存在的 `map_file` 参数。

```bash
cd /workspace/car_2026
source /opt/ros/noetic/setup.bash
source /opt/cartographer_ws/install_isolated/setup.bash
source devel/setup.bash --extend
```

先在原导航终端按 `Ctrl+C` 停止导航，再加载修改后的配置：

```bash
roslaunch robot_navigation navigation.launch
```

新终端加载同样环境后，核实参数：

```bash
rosparam get /move_base/TebLocalPlannerROS/max_vel_x
rosparam get /move_base/TebLocalPlannerROS/xy_goal_tolerance
rosparam get /move_base/local_costmap/inflation_layer
rosparam get /move_base/local_costmap/obstacle_layer/laser_scan_sensor
rosparam get /amcl/update_min_d
```

需要在线尝试时，可运行以下命令；只调整界面实际提供的参数，确认后将值写回对应 YAML。单独执行 `rosparam set` 不保证节点重新读取配置。

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

以下持续观察命令分别运行，按 `Ctrl+C` 结束：

```bash
rostopic hz /scan
rostopic hz /odometry/filtered
rostopic echo /cmd_vel
rostopic echo /move_base/status
rosrun tf tf_echo map base_footprint
```

在 RViz 使用 **2D Nav Goal** 依次测试直行、直角转弯、窄通道、终点调头。记录成功率、耗时、终点位置/角度误差、最小障碍间距、振荡次数和控制周期警告。测试时确认没有其他任务节点同时发送导航目标或争用 `/cmd_vel`。

| 现象 | 优先排查与调整顺序 |
| --- | --- |
| 激光与墙体错位、机器人位置跳变 | 初始位姿 → TF/时钟 → 里程计/IMU → AMCL；暂不调 TEB 权重。 |
| 直线走得慢 | `/cmd_vel` 与里程计对比 → 障碍/转弯约束 → `max_vel_x`、`acc_lim_x`。 |
| 转弯慢或过冲 | 实际角速度 → `max_vel_theta`、`acc_lim_theta` → 前瞻长度。 |
| 窄道进不去 | 禁区边界 → 实际轮廓 → 两个 costmap 膨胀 → TEB 障碍间距。 |
| 路径贴墙 | 定位和轮廓 → 膨胀范围/衰减 → TEB 障碍间距及权重。 |
| 到终点后反复微调 | 定位抖动 → `xy_goal_tolerance`、`yaw_goal_tolerance` → 角速度/加速度。 |
| 控制周期超时 | 传感器/CPU → 局部窗口大小、轨迹离散、迭代数；不盲目提高控制频率。 |
| 修改参数后没变化 | 配置文件路径 → 运行时命名空间 → 动态参数覆盖 → 是否已重启对应节点。 |
