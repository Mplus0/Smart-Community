#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
拍照服务节点：提供 /take_photo 服务，订阅 /kinect/rgb/image_raw 并保存为 JPG。
通过时间戳去重，避免相机掉帧导致保存到旧帧或重复帧。
"""
import os
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from photo_service.srv import TakePhoto, TakePhotoResponse

# 获取包路径（保存照片到包内 photo 目录）
try:
    import rospkg
    _PKG_PATH = rospkg.RosPack().get_path('photo_service')
except Exception:
    _PKG_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class PhotoService(object):
    def __init__(self):
        rospy.init_node('photo_service', anonymous=False)
        rospy.loginfo("启动拍照服务节点...")

        self.image_topic = "/kinect/rgb/image_raw"
        self.bridge = CvBridge()

        self.base_dir = os.path.join(_PKG_PATH, "photo")
        if not os.path.exists(self.base_dir):
            os.makedirs(self.base_dir)
        rospy.loginfo("照片将保存至: %s" % self.base_dir)

        self.service = rospy.Service("/take_photo", TakePhoto, self.handle_take_photo)
        rospy.loginfo("拍照服务已注册: /take_photo")

    def handle_take_photo(self, req):
        folder = req.folder
        filename = req.filename
        rospy.loginfo("收到拍照请求: 文件夹=%s, 文件名=%s" % (folder, filename))

        # 等待最新一帧图像（丢弃时间戳未变化的旧帧）
        try:
            rospy.loginfo("等待最新图像帧...")
            img_msg = None
            last_stamp = rospy.Time(0)
            for _ in range(20):  # 最多等 20 次，每次 1 秒
                new_msg = rospy.wait_for_message(self.image_topic, Image, timeout=1.0)
                if new_msg.header.stamp != last_stamp:
                    img_msg = new_msg
                    break
                else:
                    rospy.logwarn("丢弃旧图像帧（时间戳未变化）...")
                last_stamp = new_msg.header.stamp

            if img_msg is None:
                raise Exception("未获取到新图像帧")
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except Exception as e:
            rospy.logerr("无法接收图像: %s" % e)
            return TakePhotoResponse(False, "无法接收图像")

        save_dir = os.path.join(self.base_dir, folder)
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        save_path = os.path.join(save_dir, filename + ".jpg")
        try:
            cv2.imwrite(save_path, cv_image)
            rospy.loginfo("已保存: %s" % save_path)
            return TakePhotoResponse(True, "已保存到: " + save_path)
        except Exception as e:
            rospy.logerr("保存失败: %s" % e)
            return TakePhotoResponse(False, "保存失败: %s" % e)


if __name__ == "__main__":
    node = PhotoService()
    rospy.spin()
