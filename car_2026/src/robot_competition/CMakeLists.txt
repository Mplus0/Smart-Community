cmake_minimum_required(VERSION 3.0.2)
project(robot_competition)
find_package(catkin REQUIRED COMPONENTS
  rospy actionlib actionlib_msgs move_base_msgs geometry_msgs tf2_ros sensor_msgs std_msgs)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/main_controller.py
  scripts/waypoint_manager.py
  scripts/waypoint_recorder.py
  scripts/perception_client.py
  scripts/task_processor.py
  scripts/result_manager.py
  scripts/traffic_wait.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md P2B.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
