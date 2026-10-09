#!/usr/bin/env bash
# Run only inside the ROS Docker container. Dedicated master, no physical/simulation goals.
set -eo pipefail
source /opt/ros/noetic/setup.bash
cd /workspace/car_2026
export ROS_MASTER_URI=http://127.0.0.1:11329
export ROS_HOSTNAME=127.0.0.1
unset ROS_IP
export ROS_HOME
ROS_HOME=$(mktemp -d /tmp/robot-competition-tests.XXXXXX)
if rosparam list >/dev/null 2>&1; then
  echo 'Refusing to use an existing ROS master on test port 11329' >&2
  exit 1
fi
roscore -p 11329 > "$ROS_HOME/master.log" 2>&1 &
master_pid=$!
trap 'kill "$master_pid" 2>/dev/null || true; wait "$master_pid" 2>/dev/null || true' EXIT
for attempt in {1..50}; do
  if rosparam list >/dev/null 2>&1; then break; fi
  sleep 0.1
done
kill -0 "$master_pid"
rosparam list >/dev/null
python3 -m py_compile src/robot_competition/scripts/{main_controller,waypoint_manager,waypoint_recorder,perception_client,task_processor,result_manager,traffic_wait}.py
python3 src/robot_competition/tests/test_p1.py
python3 src/robot_competition/tests/test_p2b.py
python3 src/robot_competition/tests/test_p2c.py
source devel/setup.bash
rosrun robot_competition waypoint_manager.py src/robot_competition/config/waypoints.yaml
rosrun robot_competition waypoint_recorder.py --help
printf 'route: {frame_id: map, points: []}\n' > "$ROS_HOME/empty_route.yaml"
rosrun robot_competition main_controller.py _waypoints_file:="$ROS_HOME/empty_route.yaml" _results_root:="$ROS_HOME/results"
roslaunch --nodes robot_competition competition.launch
