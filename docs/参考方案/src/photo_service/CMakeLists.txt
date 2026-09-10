cmake_minimum_required(VERSION 3.0.2)
project(photo_service)

find_package(catkin REQUIRED COMPONENTS
  rospy
  cv_bridge
  sensor_msgs
  std_msgs
  std_srvs
  smach
  smach_ros
  actionlib
  move_base_msgs
  geometry_msgs
  message_generation
)

# 声明并生成自定义服务 TakePhoto.srv
add_service_files(
  FILES
  TakePhoto.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    cv_bridge
    sensor_msgs
    std_msgs
    std_srvs
    smach
    smach_ros
    actionlib
    move_base_msgs
    geometry_msgs
    message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

# 安装可执行脚本
catkin_install_python(PROGRAMS
  scripts/photo_server.py
  scripts/smach_photo_nav.py
  scripts/publish_image.py
  scripts/detect.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装 launch 与照片输出目录
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY photo
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
