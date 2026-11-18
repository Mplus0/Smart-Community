#!/bin/bash
# start_infra.sh - Start the infra workbench web interface
# Usage: ./start_infra.sh [--port 8000] [--bind 0.0.0.0]

set -euo pipefail

# Load ROS environment
source /opt/ros/noetic/setup.bash
source /home/mowen/cartographer_ws/install_isolated/setup.bash 2>/dev/null || true
cd "$(dirname "$(readlink -f "$0")")"/..
source devel/setup.bash --extend

# Parse arguments
PORT="${1:-8000}"
BIND="${2:-0.0.0.0}"

echo "Starting infra workbench on $BIND:$PORT"
echo "Access at: http://<robot-ip>:$PORT/"

# Start workbench via roslaunch or direct
if command -v roslaunch &> /dev/null; then
    roslaunch community_bringup infra.launch port:="$PORT" bind:="$BIND"
else
    python -m infra.server --port "$PORT" --host "$BIND"
fi
