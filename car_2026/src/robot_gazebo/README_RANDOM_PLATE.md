# 随机车牌文件替换说明

以下命令仅在 Docker 容器中执行，会覆盖现有车牌纹理和生成记录；保持当前场景时直接使用 `roslaunch robot_gazebo simulation.launch`。`start_simulation.sh` 还会终止容器中已有 Gazebo 进程，不要在运行中的场景上重复调用。日常分终端启动见 [工作空间说明](../../README.md)。

## 本次新增/替换

- `scripts/generate_random_plates.py`
- `scripts/start_simulation.sh`
- `models/car_plate2/materials/scripts/car_plate2.material`
- `models/car_plate2/materials/textures/car_plate2.jpg`
- `models/car_plate3/materials/scripts/car_plate3.material`
- `models/car_plate3/materials/textures/car_plate3.jpg`

`car_plate1` 不修改，继续作为固定车牌。

## 单独测试随机车牌

```bash
cd /workspace/car_2026
python3 src/robot_gazebo/scripts/generate_random_plates.py
```

生成结果会记录到：

`src/robot_gazebo/results/generated_plates.json`

## 一键生成并启动 Gazebo

```bash
chmod +x src/robot_gazebo/scripts/start_simulation.sh
./src/robot_gazebo/scripts/start_simulation.sh
```

随机车牌在 Gazebo 启动前生成，一轮仿真中保持不变。
