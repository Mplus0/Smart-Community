cmake_minimum_required(VERSION 3.0.2)
project(robot_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
  launch
  meshes
  urdf
  docs
  preview
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
