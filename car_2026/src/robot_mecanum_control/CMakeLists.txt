cmake_minimum_required(VERSION 3.0.2)
project(robot_mecanum_control)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/mecanum_cmd_vel.py
  scripts/mecanum_wheel_odometry.py
  scripts/mecanum_math.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
