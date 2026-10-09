#!/usr/bin/env bash
# Run manually inside the existing Docker container only. No real navigation goals.
set -eo pipefail
test -f /.dockerenv || { echo 'Run inside the existing ROS Docker container' >&2; exit 1; }
source /opt/ros/noetic/setup.bash
cd /workspace/car_2026
perception_python=/home/developer/.venvs/robot-perception/bin/python
test -x "$perception_python"
test -s src/robot_perception/models/traffic_light_best.pt || {
  echo 'Copy models/traffic_light/best.pt to robot_perception/models/traffic_light_best.pt first' >&2
  exit 1
}
# Required for roslaunch in the source workspace; install(PROGRAMS) handles installed layout.
chmod +x src/robot_perception/scripts/traffic_light_classification_node.py
python3 src/robot_perception/tests/check_static.py
catkin_make -j2
source devel/setup.bash
"$perception_python" src/robot_perception/tests/check_environment.py
"$perception_python" src/robot_perception/tests/test_adapters.py
TRAFFIC_LIGHT_MODEL="$PWD/src/robot_perception/models/traffic_light_best.pt" \
  YOLO_AUTOINSTALL=false YOLO_OFFLINE=true \
  "$perception_python" src/robot_perception/tests/test_traffic_light.py
bash src/robot_competition/tests/run_docker_checks.sh
# Parse only: defaults must remain dual, optional mode must have all three nodes.
dual=$(roslaunch --nodes robot_perception perception.launch)
triple=$(roslaunch --nodes robot_perception perception.launch enable_traffic_light:=true)
printf '%s\n' "$dual" | grep -qx '/person_detection'
printf '%s\n' "$dual" | grep -qx '/plate_recognition'
if printf '%s\n' "$dual" | grep -q '/traffic_light_classification'; then exit 1; fi
printf '%s\n' "$triple" | grep -qx '/person_detection'
printf '%s\n' "$triple" | grep -qx '/plate_recognition'
printf '%s\n' "$triple" | grep -qx '/traffic_light_classification'
echo 'PASS: P2-A, traffic-light tests, P1/P2-B, dual/triple launch parsing'
