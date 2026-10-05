#!/usr/bin/env bash
set -e

WORKSPACE="/workspace/car_2026"
PKG="$WORKSPACE/src/robot_gazebo"
WORLD="$PKG/worlds/competition_field.world"

cd "$WORKSPACE"

# Generate / refresh car_plate2 and car_plate3 before Gazebo loads textures.
python3 "$PKG/scripts/generate_random_plates.py"

# Load catkin environment when available.
if [ -f "$WORKSPACE/devel/setup.bash" ]; then
    source "$WORKSPACE/devel/setup.bash"
fi

export GAZEBO_MODEL_PATH="$PKG/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="$WORKSPACE/devel/lib:${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=""

exec gazebo --verbose "$WORLD"
