cmake_minimum_required(VERSION 3.0.2)
project(mbot_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)

catkin_package(
  CATKIN_DEPENDS urdf xacro
)

include_directories(${catkin_INCLUDE_DIRS})

# 安装 URDF/Xacro 与网格文件，供其他包通过 $(find mbot_description) 引用
install(DIRECTORY urdf meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
