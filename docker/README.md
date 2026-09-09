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
catkin_make
source devel/setup.bash
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
