# 实车 Web 工作台

工作台提供系统状态、ROS 启动项管理、实时日志和急停入口。后端还提供 ROS 遥控、初始位姿与导航目标、相机源与检测器查询、相机流和巡检结果下载接口。地图 PNG、部分前端页面以及视觉与任务联动仍处于调试阶段。

## 启动

按 [实车环境说明](../README.md#终端环境) 加载 ROS、Cartographer 和工作空间，首次部署先编译 ROS 包。在 `robot_ws/` 中安装 Python 依赖：

```bash
python3 -m pip install -r requirements.txt
```

通过 ROS 启动：

```bash
roslaunch community_bringup infra.launch port:=8000
```

也可以使用脚本管理独立进程：

```bash
bash infra/run.sh start
bash infra/run.sh status
bash infra/run.sh stop
```

浏览器访问 `http://<小车IP>:8000/`。默认监听 `0.0.0.0:8000`，可通过 ROS launch 参数 `bind`、`port`、`readonly` 和 `token` 配置。

| 环境变量 | 作用 |
|---|---|
| `INFRA_PORT` | 监听端口，默认 8000 |
| `INFRA_BIND` | 监听地址，默认 `0.0.0.0` |
| `INFRA_READONLY` | 设置为 `1` 启用只读模式 |
| `INFRA_TOKEN` | 为受保护的操作配置令牌 |
| `INFRA_RUNTIME_DIR` | 工作台日志和快照根目录，默认 `~/.ros/community/infra` |

## 测试

在 `robot_ws/` 中运行：

```bash
python3 -m pytest infra/tests -q
```

状态总线测试可脱离 ROS 运行，启动管理器的进程组测试需要 POSIX 环境。[开发批次记录](BATCH_2_5_SUMMARY.md)
