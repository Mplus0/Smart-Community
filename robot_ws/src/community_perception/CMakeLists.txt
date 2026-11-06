cmake_minimum_required(VERSION 3.0.2)
project(community_perception)
find_package(catkin REQUIRED COMPONENTS cv_bridge message_generation rospy sensor_msgs std_msgs)
add_service_files(FILES CountFallStand.srv LicensePlateOCR.srv ObjectDetection.srv)
generate_messages(DEPENDENCIES sensor_msgs std_msgs)
catkin_package(CATKIN_DEPENDS cv_bridge message_runtime rospy sensor_msgs std_msgs)
catkin_install_python(PROGRAMS scripts/fall_counter_server.py scripts/license_plate_server.py scripts/yolo_detector_service.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch models config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# 同目录辅助模块同时提供给 devel 中的节点入口与 install 空间。
foreach(module sort.py)
  configure_file(scripts/${module} ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/${module} COPYONLY)
  install(FILES scripts/${module} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()
