cmake_minimum_required(VERSION 3.0.2)
project(robot_perception)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs std_msgs)
catkin_package()
# Keep env shebangs: inference uses a separate interpreter, selected by launch-prefix.
install(PROGRAMS
  scripts/person_detection_node.py
  scripts/plate_recognition_node.py
  scripts/prepare_hyperlpr_models.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch models docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md NOTICE.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
