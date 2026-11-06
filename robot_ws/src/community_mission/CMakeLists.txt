cmake_minimum_required(VERSION 3.0.2)
project(community_mission)
find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs community_perception cv_bridge dynamic_reconfigure message_generation move_base_msgs rospy sensor_msgs std_msgs)
add_service_files(FILES Speech.srv StartRecording.srv StopRecording.srv)
generate_messages(DEPENDENCIES sensor_msgs std_msgs)
catkin_package(CATKIN_DEPENDS actionlib actionlib_msgs community_perception cv_bridge dynamic_reconfigure message_runtime move_base_msgs rospy sensor_msgs std_msgs)
catkin_install_python(PROGRAMS scripts/mission_node.py scripts/speech_server.py scripts/video_recorder_node.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# 同目录辅助模块同时提供给 devel 中的节点入口与 install 空间。
foreach(module inspection.py rules.py services.py)
  configure_file(scripts/${module} ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/${module} COPYONLY)
  install(FILES scripts/${module} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()
