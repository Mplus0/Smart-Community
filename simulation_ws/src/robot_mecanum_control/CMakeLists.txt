cmake_minimum_required(VERSION 3.0.2)
project(robot_mecanum_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)
find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp
  DEPENDS gazebo
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
add_definitions(${GAZEBO_CXX_FLAGS})

# Gazebo Classic planar-motion plugin driving only base_footprint.
add_library(robotcar_planar_move
  src/robotcar_planar_move.cpp
)
target_link_libraries(robotcar_planar_move
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/mecanum_cmd_vel.py
  scripts/mecanum_wheel_odometry.py
  scripts/mecanum_math.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS robotcar_planar_move
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY config launch docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
