cmake_minimum_required(VERSION 3.0.2)
project(mbot_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  move_base_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  tf
)

catkin_package(
  CATKIN_DEPENDS rospy actionlib move_base_msgs geometry_msgs nav_msgs std_srvs tf
)

include_directories(${catkin_INCLUDE_DIRS})

# 安装可执行脚本
catkin_install_python(PROGRAMS
  scripts/waypoint_nav.py
  scripts/multi_goal_nav.py
  scripts/listen_nav_goal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装 launch / config / param / map 资源
install(DIRECTORY launch config param map
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
