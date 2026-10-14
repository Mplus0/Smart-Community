#!/usr/bin/env bash
# 启动基础环境：随机车牌、Gazebo、导航、视觉（不启动比赛）
set -e

WORKSPACE=/workspace/car_2026
source /opt/ros/noetic/setup.bash
source /opt/cartographer_ws/install_isolated/setup.bash
source "$WORKSPACE/devel/setup.bash" --extend
cd "$WORKSPACE"

echo '[CHECK] 准备并校验本地车牌模型缓存'
PYTHONNOUSERSITE=1 /home/developer/.venvs/robot-perception/bin/python \
    src/robot_perception/scripts/prepare_hyperlpr_models.py \
    --source "$WORKSPACE/src/robot_perception/models/hyperlpr3"

echo '[PLATE] 生成随机车牌'
python3 src/robot_gazebo/scripts/generate_random_plates.py

echo '[START] 启动仿真、导航与视觉'
exec roslaunch robot_competition environment.launch
