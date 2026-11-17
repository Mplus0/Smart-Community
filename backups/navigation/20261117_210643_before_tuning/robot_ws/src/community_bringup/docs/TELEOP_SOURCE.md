# 键盘控制脚本来源

- 上游：https://github.com/ros-teleop/teleop_twist_keyboard
- 文件：teleop_twist_keyboard.py；版本：1.0.0。
- 本次从小车已安装的 ROS Noetic 官方二进制包提取：
  ros-noetic-teleop-twist-keyboard 1.0.0-1focal.20250519.232347。
- 上游 package.xml 声明 BSD 许可证；作者 Graylin Trevor Jay，维护者 Austin Hendrix。
- 本地修改：移除旧式 roslib.load_manifest；默认线速度 0.1 m/s、
  角速度 0.3 rad/s、重复发送 10 Hz、无按键 0.6 秒回零，加入来源注释。
- 保留上游按键、全向移动、调速及退出发送零速度逻辑。
- 必须在交互终端使用 rosrun，SSH 连接需分配 TTY。不由后台 roslaunch 启动。
