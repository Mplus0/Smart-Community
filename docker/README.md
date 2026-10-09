# Smart Community Docker 使用说明

本目录用于构建和运行 Smart Community 项目的统一 ROS1 开发环境。

## 环境

-   宿主机：Ubuntu 24.04
-   容器：Ubuntu 20.04
-   ROS：ROS Noetic
-   Gazebo：Gazebo 11
-   镜像：`smart-community:latest`
-   容器：`smart-community-dev`
-   Docker Compose 服务：`ros`
-   ROS 工作空间：`/workspace/car_2026`
-   支持：NVIDIA GPU、X11 图形界面

## 第一次使用

进入 Docker 配置目录：

``` bash
cd ~/workspace/smart-community/docker
```

构建镜像：

``` bash
docker compose build
```

第一次构建需要下载 ROS、Gazebo 和相关依赖，耗时可能较长。

创建并启动开发容器：

``` bash
xhost +local:docker
docker compose up -d
```

进入容器：

``` bash
docker compose exec ros bash
```

## 日常启动

进入 Docker 配置目录：

``` bash
cd ~/workspace/smart-community/docker
```

允许容器访问 X11：

``` bash
xhost +local:docker
```

启动已有容器：

``` bash
docker compose start
```

进入容器：

``` bash
docker compose exec ros bash
```

日常启动不会重新下载 ROS、Gazebo 或其他已经安装的依赖。

## ROS 工作空间

容器中的工作空间：

``` text
/workspace/car_2026
```

对应宿主机项目目录：

``` text
smart-community/car_2026
```

进入工作空间并编译：

``` bash
cd /workspace/car_2026
source /opt/ros/noetic/setup.bash
source /opt/cartographer_ws/install_isolated/setup.bash
catkin_make
source devel/setup.bash --extend
```

启动 ROS Master：

``` bash
roscore
```

启动 RViz：

``` bash
rviz
```

启动 Gazebo：

``` bash
gazebo
```

## 停止容器

退出容器：

``` bash
exit
```

停止开发容器：

``` bash
docker compose stop
```

`docker compose stop` 不会删除镜像、容器或项目源码。下次执行
`docker compose start` 即可继续开发。

## 常用命令

查看运行中的容器：

``` bash
docker ps
```

查看所有容器：

``` bash
docker ps -a
```

查看本地镜像：

``` bash
docker images
```

查看容器中的 NVIDIA GPU：

``` bash
docker compose exec ros nvidia-smi
```

修改 Dockerfile 或系统依赖后，重新构建并创建容器：

``` bash
docker compose build
docker compose down
docker compose up -d
```

## 注意事项

-   日常开发不需要反复执行 `docker compose build`。
-   修改 Dockerfile 或镜像依赖后才需要重新构建镜像。
-   `car_2026/build/` 和 `car_2026/devel/` 是 catkin
    编译产物，不应提交到 Git。
-   `car_2026`
    通过目录挂载与宿主机同步，源码不会因为停止或删除容器而丢失。
-   日常开发使用容器中的 `developer` 用户，不要使用 root 用户。
-   不要随意执行 `docker system prune`，避免误删镜像和构建缓存。

## P2-A 视觉隔离环境随镜像构建

Dockerfile 现在会安装 Python 3.10.18 与已验证的 CPU 视觉依赖，环境路径为
`/home/developer/.venvs/robot-perception`。系统 Python 3.8、ROS、catkin 的默认解释器和 PATH 不变，不全局激活 venv。此修改尚未实际重建镜像；Dockerfile 中包含依赖一致性与 ROS 消息导入检查，构建时失败会直接中止，不执行模型推理。

构建直接读取 `car_2026/src/robot_perception/docs/requirements-cpu.lock.txt`，不维护第二份依赖清单。Compose 构建上下文已改为仓库根目录，使用 `docker/Dockerfile.dockerignore` 白名单，仅发送 Dockerfile 与锁文件，排除工作空间代码、模型、ZIP、缓存和编译产物。该命名遵循 [Docker 的 Dockerfile 专用 ignore 规则](https://docs.docker.com/build/concepts/context/#filename-and-location)。

首次构建该层需要下载 Python 和 CPU Torch 等依赖，下载时间取决于网络。后续依赖锁文件与前面的构建层未变时可复用 Docker 缓存。镜像内保留 Python 解释器及 venv，清理的只是 uv 下载缓存。

你准备重建时，在宿主机执行（会替换开发容器，请先结束其中的仿真/终端任务）：

```bash
cd ~/workspace/smart-community/docker
docker compose build ros
docker compose up -d --force-recreate ros
docker compose exec ros bash
```

无需先 `down`；工作空间仍使用原有 bind mount。若不使用 Compose，从仓库根目录执行 `docker build -f docker/Dockerfile -t smart-community:latest .`，不要再以 `docker/` 子目录作为构建上下文。

新容器内确认环境并准备 HyperLPR 缓存：

```bash
source /opt/ros/noetic/setup.bash
cd /workspace/car_2026
"$HOME/.venvs/robot-perception/bin/python" src/robot_perception/scripts/prepare_hyperlpr_models.py \
  --source /workspace/car_2026/src/robot_perception/models/hyperlpr3
"$HOME/.venvs/robot-perception/bin/python" src/robot_perception/scripts/prepare_hyperlpr_models.py \
  --source /workspace/car_2026/src/robot_perception/models/hyperlpr3 --verify-only
```

模型文件继续由工作空间挂载提供，不复制进镜像，不在构建中导入 HyperLPR 或联网下载模型。每次重建容器后，需要为实际运行用户重新准备 `~/.hyperlpr3` 缓存；重复执行会复用相同文件，不覆盖冲突缓存。旧容器中未挂载的其他临时文件和手工安装内容不会自动迁入新容器。
