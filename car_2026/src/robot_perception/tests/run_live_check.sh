#!/usr/bin/env bash
# Inside Docker only. Reuses running Gazebo; does not launch navigation or send motion.
set -eo pipefail
test -f /.dockerenv || { echo 'Run inside the existing ROS Docker container' >&2; exit 1; }
source /opt/ros/noetic/setup.bash
cd /workspace/car_2026
source devel/setup.bash
p2a_python=${1:-$HOME/.venvs/robot-perception/bin/python}
traffic_enabled=${2:-false}
if [[ "$traffic_enabled" != true && "$traffic_enabled" != false ]]; then
  echo 'Second argument must be true (three streams) or false (two streams)' >&2
  exit 1
fi
if rosnode list | grep -Eq '^/(person_detection|plate_recognition|traffic_light_classification)$'; then
  echo 'Existing perception node detected; stop it yourself before this test.' >&2
  exit 1
fi
roslaunch robot_perception perception.launch python:="$p2a_python" enable_traffic_light:="$traffic_enabled" > /tmp/p2a-live-launch.log 2>&1 &
p2a_launch_pid=$!
trap 'kill -INT "$p2a_launch_pid" 2>/dev/null || true; wait "$p2a_launch_pid" 2>/dev/null || true' EXIT
for attempt in {1..90}; do
  kill -0 "$p2a_launch_pid"
  if rostopic list | grep -q '^/perception/plates_json$' && rostopic list | grep -q '^/perception/person_detections_json$'; then
    if [[ "$traffic_enabled" == false ]] || rostopic list | grep -q '^/perception/traffic_light_json$'; then
      break
    fi
  fi
  sleep 1
done
observer_args=()
if [[ "$traffic_enabled" == true ]]; then observer_args+=(--traffic-light); fi
"$p2a_python" src/robot_perception/tests/observe_topics.py --seconds 30 \
  --output "/tmp/p2a-live-traffic-$traffic_enabled" "${observer_args[@]}"
