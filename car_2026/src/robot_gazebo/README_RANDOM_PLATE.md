# 随机车牌生成

`generate_random_plates.py` 为 Gazebo 场景中的 `car_plate2` 和 `car_plate3` 生成随机车牌纹理，`car_plate1` 使用固定车牌。车牌在 Gazebo 启动前生成，一轮运行中保持不变。

## 生成纹理

在 Docker 容器内执行：

```bash
cd /workspace/car_2026
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/robot_gazebo/scripts/generate_random_plates.py
```

命令覆盖以下纹理和生成记录：

- `models/car_plate2/materials/textures/car_plate2.jpg`
- `models/car_plate3/materials/textures/car_plate3.jpg`
- `results/generated_plates.json`

以上路径均相对于 `robot_gazebo` 包。可选参数通过以下命令查看：

```bash
python3 src/robot_gazebo/scripts/generate_random_plates.py --help
```

## 生成后启动仿真

关闭正在运行的 Gazebo，再执行：

```bash
bash src/robot_gazebo/scripts/start_simulation.sh
```

该脚本先生成纹理，再终止容器内已有的 Gazebo 进程，最后启动 `simulation.launch`。需要继续使用当前纹理时直接运行：

```bash
roslaunch robot_gazebo simulation.launch
```

导航、视觉与任务控制器在其他终端启动，参见 [工作空间说明](../../README.md)。
