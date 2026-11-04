#!/usr/bin/env bash
# 先停止比赛，再停止由新版 start.sh 启动的基础环境
set -e

stop_launch() {
    local name=$1 pattern=$2 pids pid i alive
    pids=$(pgrep -u "$(id -u)" -f "$pattern" || true)
    if [[ -z $pids ]]; then
        echo "[INFO] $name 未运行"
        return 0
    fi

    echo "[STOP] $name"
    for pid in $pids; do
        kill -INT "$pid" 2>/dev/null || true
    done

    for ((i=0; i<40; i++)); do
        alive=0
        for pid in $pids; do
            if kill -0 "$pid" 2>/dev/null; then
                alive=1
            fi
        done
        if ((alive == 0)); then
            echo "[OK] $name 已停止"
            return 0
        fi
        sleep 0.5
    done

    echo "[ERROR] $name 未在 20 秒内退出，请手动检查：$pids" >&2
    return 1
}

stop_launch '比赛主程序' '[/]roslaunch robot_competition competition[.]launch'
stop_launch '仿真、导航和视觉' '[/]roslaunch robot_competition environment[.]launch'
echo '[OK] 停止操作完成'