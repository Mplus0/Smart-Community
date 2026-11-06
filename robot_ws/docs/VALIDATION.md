# 静态检查与提交前验证

2026-11-07 在 mowen 的 ROS Noetic 环境验证现有编译产物。

## 验证结果

- tools/test_python_migration.py：38/38通过。
- community.launch / finals.launch：roslaunch --nodes解析成功，分别21/22个节点。
- 三个入口均默认使用 /dev/imu，wit_normal_ros.py 具有执行权限。
- git diff --check：通过。
- 完整性工具检查8个包、121个历史基线文件，返回码1；不能记为全部通过。

## 完整性差异分类

对 tools/validate_workspace.py 报告的差异逐项核对：
- 57个文本文件：仅LF/CRLF差异，恢复CRLF后哈希与历史基线一致。
- 3个启动文件：本次将imu_port默认值从/dev/ttyUSB1改为/dev/imu。
- 1个历史文件缺失：src/robot_description/export.log。
- 除上述完整性差异外，本次工具未报告其他项目结构/资源错误。
- 历史基线未重写，未通过更新哈希掩盖差异。

变更文件：
- src/community_bringup/launch/hardware.launch
- src/community_bringup/launch/community.launch
- src/community_bringup/launch/finals.launch

## 已知历史警告

默认未使用的 wit_normal demo 有Python2语法，部分CAN/Modbus脚本有缩进错误。
robot_description/urdf/mowen.urdf.xacro 不完整；当前入口使用 mowen.urdf。
这些文件内容本次未修改。

## 报告边界

validate_workspace.py 会重新生成本文件，且其尾部包含历史固定措辞“未安装ROS、未编译、未启动节点”；
该措辞不描述本次目标机环境。本报告补充本次实际验证，不修改工具或历史基线。
默认启动解析不代表导航/全路线运行通过；实车结果见 [ACCEPTANCE.md](ACCEPTANCE.md)。
