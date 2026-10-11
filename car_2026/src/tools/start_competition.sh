#!/usr/bin/env bash
# 人工确认定位后启动正式比赛；不重新启动基础环境
set -e

WORKSPACE=/workspace/car_2026
source /opt/ros/noetic/setup.bash
source /opt/cartographer_ws/install_isolated/setup.bash
source "$WORKSPACE/devel/setup.bash" --extend
cd "$WORKSPACE"

echo '[CHECK] 请在 RViz 确认 AMCL 定位、朝向及路线安全。'
read -r -p '输入 yes 开始比赛：' answer
if [[ $answer != yes ]]; then
    echo '[INFO] 已取消比赛启动'
    exit 0
fi

echo '[START] 正式比赛开始（Ctrl+C 结束）'
exec roslaunch robot_competition competition.launch navigation_test_mode:=false