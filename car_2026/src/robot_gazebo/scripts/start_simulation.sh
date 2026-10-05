#!/usr/bin/env bash

set -e

# ============================================================
# Smart Community Gazebo Simulation Launcher
# ============================================================

WORKSPACE="/workspace/car_2026"
ROBOT_GAZEBO="$WORKSPACE/src/robot_gazebo"

echo "=========================================="
echo " Smart Community Simulation"
echo "=========================================="

# ------------------------------------------------------------
# 1. Enter workspace
# ------------------------------------------------------------

cd "$WORKSPACE"


# ------------------------------------------------------------
# 2. Load ROS / Catkin environment
# ------------------------------------------------------------

if [ -f "/opt/ros/melodic/setup.bash" ]; then
    source /opt/ros/melodic/setup.bash
fi

if [ -f "$WORKSPACE/devel/setup.bash" ]; then
    source "$WORKSPACE/devel/setup.bash"
else
    echo "[ERROR] devel/setup.bash not found."
    echo "Please run:"
    echo "  cd $WORKSPACE"
    echo "  catkin_make"
    exit 1
fi


# ------------------------------------------------------------
# 3. Generate random license plates
# ------------------------------------------------------------

echo
echo "[1/3] Generating random license plates..."

python3 "$ROBOT_GAZEBO/scripts/generate_random_plates.py"

echo "[OK] License plates ready."


# ------------------------------------------------------------
# 4. Clean old Gazebo processes
# ------------------------------------------------------------

echo
echo "[2/3] Cleaning old Gazebo processes..."

pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 gazebo 2>/dev/null || true

sleep 1

echo "[OK] Gazebo processes cleaned."


# ------------------------------------------------------------
# 5. Start ROS + Gazebo
# ------------------------------------------------------------

echo
echo "[3/3] Starting simulation..."
echo

exec roslaunch robot_gazebo simulation.launch