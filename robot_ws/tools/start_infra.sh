#!/bin/bash
# start_infra.sh - 启动 Web 工作台（roslaunch 方式，会自动拉起 roscore）
# 用法: ./start_infra.sh [--port 8000] [--bind 0.0.0.0] [--token SECRET] [--readonly]

PORT=8000
BIND=0.0.0.0
TOKEN=""
READONLY=false

usage() {
    echo "用法: $0 [--port 端口] [--bind 地址] [--token 口令] [--readonly]"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --port)     PORT="${2:-}"; shift 2 ;;
        --port=*)   PORT="${1#*=}"; shift ;;
        --bind)     BIND="${2:-}"; shift 2 ;;
        --bind=*)   BIND="${1#*=}"; shift ;;
        --token)    TOKEN="${2:-}"; shift 2 ;;
        --token=*)  TOKEN="${1#*=}"; shift ;;
        --readonly) READONLY=true; shift ;;
        -h|--help)  usage; exit 0 ;;
        *) echo "未知参数: $1"; usage; exit 2 ;;
    esac
done

if ! [[ "$PORT" =~ ^[0-9]+$ ]] || (( PORT < 1 || PORT > 65535 )); then
    echo "端口无效: '$PORT'"; exit 2
fi
if [[ -z "$BIND" ]]; then
    echo "--bind 不能为空"; exit 2
fi

# ROS 的 setup.bash 不兼容 set -u，环境加载完成后再开启严格模式。
source /opt/ros/noetic/setup.bash
source /home/mowen/cartographer_ws/install_isolated/setup.bash 2>/dev/null || true
cd "$(dirname "$(readlink -f "$0")")"/..
source devel/setup.bash --extend
set -euo pipefail

echo "Starting infra workbench on $BIND:$PORT (readonly=$READONLY)"
echo "Access at: http://<robot-ip>:$PORT/"
ARGS=(port:="$PORT" bind:="$BIND" readonly:="$READONLY")
if [[ -n "$TOKEN" ]]; then
    ARGS+=(token:="$TOKEN")
fi
exec roslaunch community_bringup infra.launch "${ARGS[@]}"
