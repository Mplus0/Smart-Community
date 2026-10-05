cmake_minimum_required(VERSION 3.0.2)

project(robot_navigation)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY
    launch
    config
    maps
    rviz
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
