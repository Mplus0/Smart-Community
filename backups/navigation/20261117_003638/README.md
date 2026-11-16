# 导航备份

来源：小车 `mowen` 的 `/home/mowen/workspace/Smart-Community/robot_ws`。

- 当前运行地图：`robot_ws/src/community_bringup/maps/map.yaml`（定位）和 `map_nav.yaml`（导航），对应 PGM 一并保存。
- 航点文件：`robot_ws/src/community_mission/config/route.yaml`。
- 同时保存 `square_4p2m*` 测试地图；它们不是备份时正在运行的地图。
- `mission_params_snapshot.yaml` 为 ROS 参数服务器中保留的任务参数，不代表任务正在执行。
- `manifest.json` 记录采集时间、文件来源、校验值及导航启动命令。

校验文件：在本备份目录执行 `sha256sum -c SHA256SUMS`。

恢复时先停止导航及任务，再将所需地图 YAML、对应图片和航点文件复制回原路径。备份操作未更改实车文件或运行状态。
