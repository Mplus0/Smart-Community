#!/usr/bin/env bash
# 启动基础环境：随机车牌、Gazebo、导航、视觉（不启动比赛）
set -e

WORKSPACE=/workspace/car_2026
source /opt/ros/noetic/setup.bash
source /opt/cartographer_ws/install_isolated/setup.bash
source "$WORKSPACE/devel/setup.bash" --extend
cd "$WORKSPACE"

echo '[PLATE] 生成随机车牌'
python3 src/robot_gazebo/scripts/generate_random_plates.py

echo '[START] 启动仿真、导航与视觉'
exec roslaunch robot_competition environment.launch