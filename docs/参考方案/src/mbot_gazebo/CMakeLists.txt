cmake_minimum_required(VERSION 3.0.2)
project(mbot_gazebo)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

# 安装 launch 与 world 文件
install(DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
