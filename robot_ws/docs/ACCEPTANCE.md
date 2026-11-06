# 实车验收记录

日期：2026-11-06 至 2026-11-07。主机 mowen，ROS Noetic / Python 3。
用户已完成编译；本次未重新编译，使用现有 devel 验证。
结论：正确 USB 连接和端口绑定下，底盘反馈及架空低速运动正常。
导航、完整巡检路线、地面速度精度和失联停车未验收。

## 已执行结果

| 项目 | 结果与范围 |
|---|---|
| 启动解析 | community.launch / finals.launch 分别解析21/22个节点，非整套运行通过证明 |
| ROS/Python依赖 | 六种自定义服务及 rospy、cv_bridge、OpenCV、NumPy、pyserial、ONNX Runtime、PaddleOCR 可导入 |
| 离线回归 | 38项通过，含任务规则、状态转换、结果保存及异常处理 |
| 雷达 | /scan 约10.02Hz |
| IMU | /wit/imu 约10.03Hz，时间戳递增；修复驱动执行权限 |
| 底盘与里程计 | /vel_raw、/odom_raw 约20Hz；零速8秒各收到159条消息 |
| 架空运动 | 前进指令0.10m/s持续约1秒，随后连续发送20次零速；各收到99条速度/里程计消息；前向反馈峰值0.083m/s，末尾连续5帧三轴为0；用户确认正常转动并停止 |
| 相机 | 640×480，约29.7Hz，实际截图可显示；仅对本次采集链路通过 |
| 六模型YOLO | 真实图像服务调用成功，每次约0.53–0.65秒；识别准确率按用户要求暂不验收 |
| OCR | 实际调用约1.01秒，返回未识别到车牌；正样本效果暂不验收 |
| 录像 | 启停成功，149帧、30fps、约4.97秒，可解码；H264不可用时回退编码成功 |
| 视频倒伏统计 | 实际录像调用成功，返回0/0，耗时约32.04秒；准确率与实时性未验收 |
| 中文语音 | USB扬声器及ROS服务均由用户确认正常；使用临时cmn语音与PulseAudio配置，非默认配置重启验收 |

架空反馈速度为底盘回传值，轮式里程计积分不代表地面实际位移。
本次只验证前进低速和主动发送零速后的停止，不覆盖横移、转向、负载运行或断链停车。

## 根因排查与修复

初始雷达、IMU、相机有数据，但底盘无反馈。重新接线期间，原串口集线器整体断开；
工控机只识别到一个直连CH340。该接口在原始串口、当前ROS及旧程序对照下均无反馈。
用户将工控机改接另一块连接传感器的板子后，系统恢复Hub、两个CH340与雷达。
进一步被动读取及驱动验证确认：
- IMU：/dev/imu → ttyUSB0，父设备1-1.1；9600波特率有55开头帧。
- 底盘：/dev/carserial → ttyUSB1，父设备1-1.2；115200波特率有有效AA BB反馈。
- 雷达：/dev/lslidar → ttyACM0，父设备1-1.4。

此前无反馈随USB连接和绑定恢复而解决，没有依据归因于当前反馈解析代码或认定控制板损坏。
实际两块板子的型号和分工未由实物/厂家确认。
详见 [协议对比](BASE_PROTOCOL_REVIEW.md) 和 [原启动脚本对照](ORIGINAL_START_COMPARISON.md)。

仓库修改：
1. hardware/community/finals.launch 的 imu_port 默认改为 /dev/imu。
2. wit_normal_ros.py 增加执行权限，不改厂商驱动内容。
3. 更新使用说明、架构图与验收文档。

主机配置修改（不随仓库提交）：
- 备份并更新 carserial.rules、imu.rules，按当前物理USB父路径绑定；雷达规则沿用。
- mowen 加入 audio 组；此前临时声卡ACL会消失，组权限在新会话中生效。
- 纯历史Python缓存归档到 /tmp/robot_ws_review_cache_20261106；未移动源码。

## 语音与其他限制

语音默认 mb-cn1 所需 mbrola / mbrola-cn1 未安装，曾出现服务返回成功但实际无声。
接入USB扬声器、解决audio组权限后，以内置cmn合成，直接播放与ROS服务均现场确认正常。
本次PulseAudio以临时配置加载实际USB声卡，/speech_server/voice临时设为cmn；
这些设置未持久化到启动流程，重启或重新插拔后的自动恢复仍待验证。

原始 start_robot.sh 的并发启动存在master竞争；预先就绪master后才能稳定完成对照。
原始导航在传感器缺失时无法取得必要TF，未用于评价当前导航正确性。
旧usb_cam在对照结束附近有进程退出/段错误记录；长期运行和反复启停仍需检查。
requests提示urllib3/chardet版本不匹配，未在本轮服务调用中阻断。

## 证据与后续验收

原始测试输出保留在目标机 ~/.ros/community/acceptance_20261107/，不提交运行日志和媒体：
- results.json、camera.jpg、test.mp4：服务及媒体测试。
- protocol_comparison.json：新旧发送函数的5组离线字节对比。
- original_start_robot*.log / *_snapshot.txt：原始脚本运行记录。
- restored_hub_zero_probe.json：正确USB连接后的零速度反馈。
- restored_low_speed_probe_010.json：最终架空运动完整采样。

后续依次验收：语音持久启动；横移/转向及停车；地图与TF；单目标导航；完整路线。
识别效果暂不验收。任何后续运动测试均需现场条件就绪。
