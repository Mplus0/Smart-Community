# Docker 部署与运行

Docker 镜像提供 ROS、Gazebo、Cartographer 和视觉推理环境。工作空间通过目录挂载与宿主机共享。

## 环境与目录

| 项目 | 配置 |
|---|---|
| 容器系统 | Ubuntu 20.04 |
| ROS / 仿真 | ROS Noetic / Gazebo 11 |
| 镜像 / 容器 | `smart-community:latest` / `smart-community-dev` |
| Compose 服务 / 用户 | `ros` / `developer`（UID/GID 默认 1000） |
| 工作空间 | `/workspace/car_2026` |
| Cartographer | `/opt/cartographer_ws/install_isolated` |
| 视觉 Python | `/home/developer/.venvs/robot-perception/bin/python` |

宿主机需要 Docker Engine、Compose 插件和图形显示环境。默认 Compose 使用 host 网络、NVIDIA GPU 设备及 `/dev/dri`，需要可用的 NVIDIA 驱动和容器 GPU 支持；X11 套接字挂载到容器。视觉依赖为 CPU 版本，GPU 设备可见不代表推理使用 CUDA。

## 首次部署

在仓库根目录打开宿主机终端：

```bash
cd docker
docker compose build ros
xhost +local:docker
docker compose up -d ros
docker compose exec --user developer ros bash
```

镜像构建需要网络下载 ROS、Cartographer 和 Python 依赖。Compose 的构建上下文为仓库根目录，`Dockerfile.dockerignore` 限制发送给构建器的内容。也可在仓库根目录使用 `docker build -f docker/Dockerfile -t smart-community:latest .` 构建同名镜像。

## 编译工作空间

以下命令在容器内执行：

```bash
cd /workspace/car_2026
source /opt/ros/noetic/setup.bash
source /opt/cartographer_ws/install_isolated/setup.bash
catkin_make -j2
source devel/setup.bash --extend
```

之后每个新容器终端加载上述两个 ROS 环境，再执行 `source /workspace/car_2026/devel/setup.bash --extend`。`--extend` 保留 Cartographer 的包搜索路径。

仿真、导航、视觉和比赛任务按 [工作空间说明](../car_2026/README.md) 分终端启动。`simulation.launch` 使用 roslaunch 管理 ROS Master 和 Gazebo，无需另开一个空 Gazebo 实例。

## 模型与 Python 环境

catkin 与任务主程序使用系统 Python 3.8。Dockerfile 创建 Python 3.10.18 视觉环境，并按 [依赖锁文件](../car_2026/src/robot_perception/docs/requirements-cpu.lock.txt) 安装 CPU 推理依赖；不要全局激活视觉 venv 来编译 ROS。

模型通过工作空间挂载提供，不打包进镜像。人物、车牌和交通灯模型的路径及部署方式见 [视觉包说明](../car_2026/src/robot_perception/README.md)。使用节点的同一用户在容器内准备车牌缓存：

```bash
cd /workspace/car_2026
source /opt/ros/noetic/setup.bash
"$HOME/.venvs/robot-perception/bin/python" src/robot_perception/scripts/prepare_hyperlpr_models.py \
  --source /workspace/car_2026/src/robot_perception/models/hyperlpr3
"$HOME/.venvs/robot-perception/bin/python" src/robot_perception/scripts/prepare_hyperlpr_models.py \
  --source /workspace/car_2026/src/robot_perception/models/hyperlpr3 --verify-only
```

同哈希的缓存文件可复用，冲突文件不会被覆盖。容器用户目录没有持久挂载，重建容器后需要重新准备 `~/.hyperlpr3`。

## 日常启停

宿主机在仓库的 `docker/` 目录执行：

```bash
xhost +local:docker
docker compose start ros
docker compose exec --user developer ros bash
```

也可从任意宿主机目录进入正在运行的容器：

```bash
docker exec -it --user developer smart-community-dev bash
```

停止时先结束任务与仿真，再在宿主机执行：

```bash
docker compose stop ros
```

停止容器不会删除镜像或挂载的工作空间。修改镜像依赖后，先结束容器中的任务，再更新：

```bash
docker compose build ros
docker compose up -d --force-recreate ros
```

重建保留挂载的工作空间，但容器内未挂载的个人文件和手工安装内容不会自动迁移。

## 常用诊断

以下命令在宿主机执行：

```bash
docker compose ps
docker compose logs --tail=100 ros
docker compose exec --user developer ros nvidia-smi
docker images smart-community
```

图形窗口无法显示时检查宿主机 `DISPLAY`、X11 授权和 `/tmp/.X11-unix` 挂载。文件权限与宿主机不匹配时检查 Compose 的 `USER_UID`、`USER_GID`。缓存和任务日志的路径见 [工作空间说明](../car_2026/README.md)。
