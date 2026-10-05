cmake_minimum_required(VERSION 3.0.2)
project(robot_gazebo)

# 使用 C++11
add_compile_options(-std=c++11)

# ==================================================
# Catkin dependencies
# ==================================================
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  xacro
)

# ==================================================
# Gazebo
# ==================================================
find_package(gazebo REQUIRED)

# ==================================================
# Catkin package
# ==================================================
catkin_package(
  CATKIN_DEPENDS
    gazebo_ros
    xacro

  DEPENDS
    gazebo
)

# ==================================================
# Include directories
# ==================================================
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Gazebo library directories
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Gazebo compilation flags
set(CMAKE_CXX_FLAGS
  "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}"
)

# ==================================================
# Traffic light Gazebo plugin
# ==================================================
add_library(
  traffic_light_plugin
  SHARED
  src/traffic_light_plugin.cpp
)

target_link_libraries(
  traffic_light_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

# ==================================================
# Install
# ==================================================

# Install Gazebo plugin
install(
  TARGETS
    traffic_light_plugin

  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install launch files
install(
  DIRECTORY
    launch/

  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install Gazebo models
install(
  DIRECTORY
    models/

  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

# Install world files
install(
  DIRECTORY
    worlds/

  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)