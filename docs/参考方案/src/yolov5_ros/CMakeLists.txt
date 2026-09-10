cmake_minimum_required(VERSION 3.0.2)
project(yolov5_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  geometry_msgs
  yolov5_ros_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge geometry_msgs yolov5_ros_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

# 安装可执行脚本
catkin_install_python(PROGRAMS
  scripts/yolo_v5.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装 launch 与权重目录
install(DIRECTORY launch weights yolov5
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
