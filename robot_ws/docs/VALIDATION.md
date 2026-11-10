# 静态检查报告

功能包：8；完整性校验：121 个文件。

检查 Python 语法、本地模块/服务导入、依赖、XML/YAML、资源引用、入口、include 参数、服务生成、scripts 安装声明及旧结构移除。

## 项目问题

Integrity mismatch: src/community_perception/models/people.onnx
Integrity mismatch: src/community_mission/config/route.yaml
Integrity mismatch: src/community_mission/scripts/mission_node.py
Integrity mismatch: src/wit_ros_imu/scripts/wit_normal_ros.py
Integrity mismatch: src/community_mission/scripts/inspection.py
Integrity mismatch: src/community_mission/scripts/rules.py
Integrity mismatch: src/community_bringup/CMakeLists.txt
Integrity mismatch: src/community_bringup/package.xml
Integrity mismatch: src/community_bringup/launch/hardware.launch
Integrity mismatch: src/community_mission/scripts/result_manager.py
Integrity mismatch: src/community_bringup/launch/community.launch
Integrity mismatch: src/community_bringup/launch/finals.launch
Integrity mismatch: src/community_perception/config/detectors.yaml
Integrity mismatch: src/community_mission/config/runtime.yaml

## 厂商原有问题（未修改）

src\robot_description\urdf\mowen.urdf.xacro: no element found: line 60, column 111
src\wit_ros_imu\demo\wit_normal.py: Missing parentheses in call to 'print'. Did you mean print(...)?
src\wit_ros_imu\scripts\wit_can_ros.py: unindent does not match any outer indentation level
src\wit_ros_imu\scripts\wit_hcan_ros.py: unindent does not match any outer indentation level
src\wit_ros_imu\scripts\wit_hmodbus_ros.py: unindent does not match any outer indentation level
src\wit_ros_imu\scripts\wit_modbus_ros.py: unindent does not match any outer indentation level

未安装 ROS、未编译、未启动节点；外部依赖、生成服务和运行行为待工控机验证。
