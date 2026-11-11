#!/usr/bin/env bash
# 在宿主机复用 Compose 容器显示实车 RViz。
set -euo pipefail

usage() {
    echo "用法: $0 [slam|navigation] <小车IP> <本机局域网IP>"
    echo "示例: $0 slam 192.168.31.59 192.168.31.83"
}

if [[ ${1:-} == -h || ${1:-} == --help ]]; then
    usage
    exit 0
fi
mode=slam
if [[ ${1:-} == slam || ${1:-} == navigation ]]; then
    mode=$1
    shift
fi
if [[ $# != 2 ]]; then
    usage >&2
    exit 2
fi
robot_ip=$1
local_ip=$2
: "${DISPLAY:?请在本地 Ubuntu 图形桌面终端运行}"

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
compose=(docker compose -f "$repo_root/docker/docker-compose.yml")

# 只启动已有容器；挂载更新由用户按 Docker README 单独执行。
"${compose[@]}" start ros
"${compose[@]}" exec -T --user developer ros \
    test -r "/workspace/robot_ws/src/community_bringup/launch/rviz/${mode}_rviz.launch" || {
    echo "容器缺少 RViz 文件，请按 docker/README.md 更新工作空间挂载。" >&2
    exit 1
}
xhost +si:localuser:"$(id -un)"
"${compose[@]}" exec --user developer \
    -e DISPLAY="$DISPLAY" \
    -e ROS_MASTER_URI="http://${robot_ip}:11311" \
    -e ROS_IP="$local_ip" \
    ros bash --noprofile --norc -c \
    'unset ROS_HOSTNAME; source /opt/ros/noetic/setup.bash; exec roslaunch "/workspace/robot_ws/src/community_bringup/launch/rviz/${1}_rviz.launch"' \
    bash "$mode"
