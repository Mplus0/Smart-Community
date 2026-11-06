# 原始 start_robot.sh 对照结果（2026-11-07）

实际运行 /home/mowen/start_robot.sh，未修改原脚本或旧源码。
当前 robot_ws 底盘节点事先停止；用户确认驱动轮架空及现场可断电。

## 最终补充

后续恢复正确USB连接与绑定后，当前程序收到正常反馈并完成架空运动。以下记录发生在修复之前，不能作为当前仍故障的结论。详见 [验收记录](ACCEPTANCE.md)。

## 启动条件与排除干扰

首次从未加载 ROS 的 SSH 环境直接启动，脚本第2行 roscore 找不到命令；
两组 roslaunch 同时尝试启动 master，出现地址冲突。
预加载 ROS 后再次运行，脚本后台并发启动 roscore 和两组 roslaunch，出现 run_id 冲突。
最终先启动并等待 ROS master 就绪，再执行原脚本，旧业务节点正常注册。
原脚本内部额外启动的 roscore 报已有 master 并退出，两个 launch 使用既有 master；
这一条件已明确记录，不将前两次启动失败作为硬件结论。

## 稳定运行观察

- 旧 /car_control（newt.py）订阅 /cmd_vel。
- 旧 /pubv（pubv.py）注册为 /vel_raw 发布者，但5秒观察窗口内没有消息。
- /cmd_vel 存在 move_base 发布者，但4秒观察窗口内没有收到消息。
  不能凭发布者存在认定发出了运动指令，本次不是有效的旧程序运动响应对照。
- 雷达报告 open_port /dev/lslidar ERROR。
- IMU 报 /dev/ttyUSB1 不存在，无法打开串口。
- 无法形成完整导航所需数据/TF，因此不能据此判断导航算法是否正确。

## 结论

无反馈不只出现在 robot_ws，当前旧系统也无法提供速度反馈；
没有证据将故障单独归因于当前重构代码。
共同设备条件存在明确问题（雷达/IMU缺失），底盘则仍须区分接线、供电、固件/控制模式与硬件故障。
不能因为旧系统也失败就直接判定硬件损坏，也不能据此证明当前代码完全无问题。
此前旧发送函数和当前 aa_bb 的离线字节对比一致，详见 BASE_PROTOCOL_REVIEW.md。

## 收尾与证据

停止 /control 和 /move_base，再以20Hz发送全零Twist约2秒；
随后关闭本次原脚本及子进程。停止指令发送不代替现场停车观察。
证据位于 ~/.ros/community/acceptance_20261107/：
original_start_robot.log、original_start_robot_sourced.log、
original_start_robot_stable.log、original_start_robot_stable_snapshot.txt。
本次未刷固件、未更改协议或导航参数。
