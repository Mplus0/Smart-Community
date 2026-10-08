cmake_minimum_required(VERSION 3.0.2)
project(robot_perception)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs std_msgs)
catkin_package(CATKIN_DEPENDS rospy sensor_msgs std_msgs)
# Preserve the interpreter chosen by the caller or launch prefix.
install(PROGRAMS scripts/person_detection_node.py scripts/plate_recognition_node.py
  scripts/traffic_light_node.py scripts/prepare_hyperlpr_models.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES scripts/traffic_light_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config models docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
