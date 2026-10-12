# 随机车牌生成

`generate_random_plates.py` 为 Gazebo 场景中的 `car_plate2` 和 `car_plate3` 生成随机车牌纹理，`car_plate1` 使用固定车牌。车牌在 Gazebo 启动前生成，一轮运行中保持不变。

## 生成纹理

在 Docker 容器内执行：

```bash
cd /workspace/car_2026
source /opt/ros/noetic/setup.bash
source /opt/cartographer_ws/install_isolated/setup.bash
source devel/setup.bash --extend
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

脚本只使用 Python 标准库，但会通过 HTTP 调用在线车牌图片接口，并发送随机生成的号码。默认接口地址在脚本 `DEFAULT_API_URL` 中，可用 `--api-url` 替换；这里不保证在线服务可用。

| 参数 | 默认值 / 行为 |
|---|---|
| `--timeout` | 每次请求超时 6 秒 |
| `--retries` | 每张车牌最多尝试 3 次，每次尝试重新生成号码 |
| `--seed` | 未设置；指定整数可固定随机序列，网络失败重试仍会影响最终结果 |

成功获取图片后原子替换该张 JPEG。请求失败且原纹理存在时保留旧文件，对应生成记录为 null；没有可用旧纹理时退出报错。两张纹理逐张处理，并非成组事务；不能将 null 解读为“无车牌”，也不能把记录当作视觉识别结果。工作空间是宿主机挂载目录，生成成功会修改仓库中已有纹理。

## 生成后启动仿真

完成生成后，直接启动仿真以使用当前纹理：

```bash
roslaunch robot_gazebo simulation.launch rviz:=true
```

完整比赛基础环境可使用：

```bash
bash src/tools/start.sh
```

`start.sh` 会再次生成随机车牌并启动 `environment.launch`（仿真、导航、三路视觉和 RViz）；需要保留当前纹理时直接启动 `roslaunch robot_competition environment.launch`。这两种环境入口不要与已运行的 Gazebo 重复启动。

旧辅助脚本 `scripts/start_simulation.sh` 仍保留：它先生成纹理，再用 `pkill -9` 终止 Gazebo 进程，最后仅启动仿真；脚本还保留 Melodic 环境探测，不是当前 Noetic 的集中启动入口。日常使用上面的 launch 或 `src/tools/start.sh`。

任务控制器需在定位确认后单独启动，参见 [工作空间说明](../../README.md)。修改纹理后重启 Gazebo 使材质重新加载；GUI 人员/车牌快照来自任务证据，不来自本生成记录。
