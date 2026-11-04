#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import threading
import os
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from community_mission.srv import StartRecording, StartRecordingResponse
from community_mission.srv import StopRecording, StopRecordingResponse

class VideoRecorderNode:
    def __init__(self):
        rospy.init_node('video_recorder')
        
        # ROS参数
        self.camera_topic = rospy.get_param('~camera_topic', '/usb_cam/image_raw')
        self.default_save_dir = rospy.get_param('~default_save_dir', '~/videos')
        
        # 视频参数
        self.frame_width = 640
        self.frame_height = 480
        self.fps = 30.0  # 使用浮点数确保精度
        
        # 初始化变量
        self.bridge = CvBridge()
        self.is_recording = False
        self.video_writer = None
        self.lock = threading.Lock()
        self.last_frame = None
        
        # 性能监控变量
        self.frame_count = 0
        self.frame_drop_count = 0
        self.recording_start_time = None
        self.last_frame_time = None
        self.recording_file_path = ""
        
        # 创建服务
        self.start_service = rospy.Service(
            rospy.get_name()+'/start_recording', 
            StartRecording, 
            self.handle_start_recording
        )
        
        self.stop_service = rospy.Service(
            rospy.get_name()+'/stop_recording', 
            StopRecording, 
            self.handle_stop_recording
        )
        
        # 订阅摄像头话题 - 使用queue_size=1防止堆积，buff_size设置大一些
        rospy.Subscriber(
            self.camera_topic, 
            Image, 
            self.image_callback, 
            queue_size=1, 
            buff_size=2**24  # 16MB缓冲区
        )
        
        rospy.loginfo("视频录制节点已初始化")
        rospy.loginfo(f"监听摄像头话题: {self.camera_topic}")
        rospy.loginfo(f"视频参数: {self.frame_width}x{self.frame_height} @ {self.fps}fps")
        rospy.loginfo("可用服务:")
        rospy.loginfo(f"  rosservice call {rospy.get_name()}/start_recording \"file_path: '/path/to/save/video.mp4'\"")
        rospy.loginfo(f"  rosservice call {rospy.get_name()}/stop_recording")
        
        # 注册节点关闭时的清理函数
        rospy.on_shutdown(self.cleanup)
    
    def image_callback(self, msg):
        """图像回调函数 - 优化版本，减少处理延迟"""
        # 如果不在录制状态，直接返回
        if not self.is_recording:
            return
        
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn_once(f"图像转换失败: {e}")
            with self.lock:
                self.frame_drop_count += 1
            return
        
        # 使用锁保护视频写入器，防止多线程冲突
        with self.lock:
            if self.is_recording and self.video_writer is not None:
                try:
                    self.video_writer.write(cv_image)
                    self.frame_count += 1
                    
                    # 每100帧输出一次调试信息，监控写入频率
                    if self.frame_count % 100 == 0:
                        if self.last_frame_time is not None:
                            now = time.time()
                            elapsed = now - self.last_frame_time
                            actual_fps = 100.0 / elapsed if elapsed > 0 else 0
                            rospy.logdebug(f"[录制中] 已写入 {self.frame_count} 帧 | 最近100帧实际FPS: {actual_fps:.1f}")
                        self.last_frame_time = time.time()
                        
                except Exception as e:
                    rospy.logerr(f"写入视频帧失败: {e}")
                    self.frame_drop_count += 1
    
    def handle_start_recording(self, req):
        """处理开始录制请求"""
        response = StartRecordingResponse()
        
        # 检查是否已经在录制
        if self.is_recording:
            response.success = False
            response.message = "已经在录制视频，请先停止当前录制"
            rospy.logwarn(response.message)
            return response
        
        # 检查是否有有效帧（通过话题检查）
        rospy.loginfo("等待摄像头数据...")
        try:
            # 等待第一帧消息，最多等待2秒
            test_msg = rospy.wait_for_message(self.camera_topic, Image, timeout=2.0)
            rospy.loginfo("摄像头数据流正常")
        except rospy.ROSException:
            response.success = False
            response.message = f"无法收到摄像头话题 {self.camera_topic} 的数据，请检查摄像头是否正常工作"
            rospy.logerr(response.message)
            return response
        
        # 获取文件路径
        file_path = req.file_path
        
        # 如果未提供路径，使用默认路径和时间戳
        if not file_path or file_path.strip() == "":
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            file_name = f"video_{timestamp}.mp4"
            save_dir = os.path.expanduser(self.default_save_dir)
            file_path = os.path.join(save_dir, file_name)
        else:
            # 处理用户输入的 ~/ 路径
            if file_path.startswith("~/"):
                file_path = os.path.expanduser(file_path)
            else:
                file_path = os.path.abspath(file_path)
        
        try:
            # 确保目录存在
            save_dir = os.path.dirname(file_path)
            if save_dir and not os.path.exists(save_dir):
                os.makedirs(save_dir, exist_ok=True)
                rospy.loginfo(f"创建目录: {save_dir}")
            
            # 检查文件扩展名，确保是.mp4
            if not file_path.lower().endswith('.mp4'):
                original_path = file_path
                file_path += '.mp4'
                rospy.logwarn(f"添加扩展名: {original_path} -> {file_path}")
            
            # 检查文件是否已存在
            if os.path.exists(file_path):
                rospy.logwarn(f"文件已存在，将覆盖: {file_path}")
            
            # 尝试获取一帧图像来确定实际尺寸
            rospy.loginfo("获取图像尺寸...")
            try:
                # 等待一帧图像
                image_msg = rospy.wait_for_message(self.camera_topic, Image, timeout=1.0)
                cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
                height, width = cv_image.shape[:2]
                rospy.loginfo(f"检测到实际分辨率: {width}x{height}")
            except Exception as e:
                rospy.logwarn(f"无法获取实际图像尺寸，使用预设值: {e}")
                width, height = self.frame_width, self.frame_height
            
            # 创建视频写入器 - 使用更兼容的编码器
            # 尝试多种编码器，按兼容性排序
            codec_tried = []
            fourcc = None
            
            # 编码器选项列表（按优先级排序）
            codec_options = [
                ('avc1', cv2.VideoWriter_fourcc(*'avc1')),  # H.264编码，最兼容
                ('h264', cv2.VideoWriter_fourcc(*'h264')),  # H.264另一种表示
                ('X264', cv2.VideoWriter_fourcc(*'X264')),  # X264编码
                ('mp4v', cv2.VideoWriter_fourcc(*'mp4v')),  # 原始MP4编码
            ]
            
            for codec_name, codec_value in codec_options:
                try:
                    video_writer = cv2.VideoWriter(file_path, codec_value, self.fps, (width, height))
                    if video_writer.isOpened():
                        fourcc = codec_value
                        rospy.loginfo(f"使用编码器: {codec_name}")
                        break
                    else:
                        codec_tried.append(codec_name)
                        video_writer.release()
                except Exception as e:
                    codec_tried.append(f"{codec_name}(失败)")
                    continue
            
            if fourcc is None:
                response.success = False
                response.message = f"无法创建视频文件，所有编码器尝试失败: {', '.join(codec_tried)}"
                rospy.logerr(response.message)
                return response
            
            # 开始录制
            with self.lock:
                self.is_recording = True
                self.video_writer = video_writer
                self.recording_file_path = file_path
                self.frame_count = 0
                self.frame_drop_count = 0
                self.recording_start_time = time.time()
                self.last_frame_time = None
            
            response.success = True
            response.message = f"开始录制视频到: {file_path}"
            rospy.loginfo(f"开始录制视频: {file_path}")
            rospy.loginfo(f"视频参数: {width}x{height} @ {self.fps}fps")
            
        except Exception as e:
            response.success = False
            response.message = f"开始录制失败: {str(e)}"
            rospy.logerr(f"开始录制失败: {e}")
            # 确保在异常情况下清理资源
            with self.lock:
                if self.video_writer is not None:
                    self.video_writer.release()
                    self.video_writer = None
                self.is_recording = False
        
        return response
    
    def handle_stop_recording(self, req):
        """处理停止录制请求"""
        response = StopRecordingResponse()
        
        if not self.is_recording:
            response.success = False
            response.message = "当前没有在录制视频"
            rospy.logwarn(response.message)
            return response
        
        try:
            with self.lock:
                if self.is_recording:
                    self.is_recording = False
                    
                    if self.video_writer is not None:
                        self.video_writer.release()
                        self.video_writer = None
                        rospy.loginfo("视频写入器已正确释放")
                    
                    # 计算并打印详细的性能报告
                    recording_end_time = time.time()
                    total_wall_time = recording_end_time - self.recording_start_time
                    total_recorded_time = self.frame_count / self.fps if self.fps > 0 else 0
                    
                    # 生成性能报告
                    rospy.loginfo("=" * 60)
                    rospy.loginfo("视频录制性能报告")
                    rospy.loginfo(f"  文件路径: {self.recording_file_path}")
                    rospy.loginfo(f"  节点记录的时长 (墙钟): {total_wall_time:.2f} 秒")
                    rospy.loginfo(f"  预设帧率 FPS: {self.fps}")
                    rospy.loginfo(f"  成功写入帧数: {self.frame_count}")
                    rospy.loginfo(f"  丢失/丢弃帧数: {self.frame_drop_count}")
                    rospy.loginfo(f"  视频文件理论时长 (帧数/FPS): {total_recorded_time:.2f} 秒")
                    
                    if total_wall_time > 0:
                        efficiency = (total_recorded_time / total_wall_time) * 100
                        rospy.loginfo(f"  实际录制效率: {efficiency:.1f}%")
                        
                        if efficiency < 80:
                            rospy.logwarn("录制效率较低，可能丢帧严重！")
                            rospy.logwarn("建议检查: 1) 摄像头发布频率 2) 系统负载 3) 磁盘IO")
                    
                    rospy.loginfo("=" * 60)
                    
                    # 检查文件是否成功创建
                    if os.path.exists(self.recording_file_path):
                        file_size = os.path.getsize(self.recording_file_path)
                        file_size_mb = file_size / (1024 * 1024)
                        rospy.loginfo(f"文件大小: {file_size_mb:.2f} MB")
                        
                        # 验证文件完整性
                        if file_size == 0:
                            rospy.logwarn("警告: 生成的视频文件大小为0字节！")
                        elif file_size_mb < 0.1:  # 小于100KB
                            rospy.logwarn(f"警告: 视频文件异常小 ({file_size_mb:.2f} MB)，可能未正确写入")
                    else:
                        rospy.logwarn(f"警告: 视频文件未创建: {self.recording_file_path}")
                    
                    # 构建响应消息
                    response.success = True
                    response.message = (f"视频录制已停止。\n"
                                      f"写入 {self.frame_count} 帧，"
                                      f"时长 {total_recorded_time:.2f}秒。\n"
                                      f"保存到: {self.recording_file_path}")
                    
                    # 重置状态变量
                    self.recording_file_path = ""
                    self.frame_count = 0
                    self.frame_drop_count = 0
                    self.recording_start_time = None
                    self.last_frame_time = None
        
        except Exception as e:
            rospy.logerr(f"停止录制时发生错误: {e}")
            response.success = False
            response.message = f"停止录制失败: {str(e)}"
        
        return response
    
    def cleanup(self):
        """清理资源 - 节点关闭时调用"""
        rospy.loginfo("视频录制节点关闭中...")
        
        with self.lock:
            if self.is_recording and self.video_writer is not None:
                rospy.logwarn("节点关闭时正在录制，强制停止并保存视频...")
                
                try:
                    self.video_writer.release()
                    rospy.loginfo(f"强制保存视频到: {self.recording_file_path}")
                    
                    # 检查文件
                    if os.path.exists(self.recording_file_path):
                        file_size = os.path.getsize(self.recording_file_path)
                        rospy.loginfo(f"最终文件大小: {file_size / (1024*1024):.2f} MB")
                    else:
                        rospy.logwarn("警告: 视频文件可能未正确保存")
                        
                except Exception as e:
                    rospy.logerr(f"强制保存视频失败: {e}")
                
                self.video_writer = None
                self.is_recording = False
        
        rospy.loginfo("视频录制节点已关闭")
    
    def run(self):
        """运行节点"""
        rospy.spin()

if __name__ == '__main__':
    try:
        node = VideoRecorderNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"视频录制节点启动失败: {e}")