#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线图片发布节点：把本地文件夹中的图片逐张发布到 /kinect/rgb/image_raw，
用于在没有 Gazebo 仿真的情况下测试拍照服务 / 识别流程。
"""
import os
import glob
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

try:
    import rospkg
    _PKG_PATH = rospkg.RosPack().get_path('photo_service')
except Exception:
    _PKG_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def publish_images():
    rospy.init_node('image_publisher', anonymous=True)
    pub = rospy.Publisher('/kinect/rgb/image_raw', Image, queue_size=1)
    bridge = CvBridge()

    # 图片文件夹路径（默认包内 photo/photo1）
    folder_path = rospy.get_param('~folder_path',
                                  os.path.join(_PKG_PATH, 'photo', 'photo1'))

    img_files = sorted(
        glob.glob(os.path.join(folder_path, "*.jpg")) +
        glob.glob(os.path.join(folder_path, "*.png")) +
        glob.glob(os.path.join(folder_path, "*.jpeg"))
    )

    if not img_files:
        rospy.logerr("文件夹中没有找到图片: %s" % folder_path)
        return

    rospy.loginfo("找到 %d 张图片，开始发布..." % len(img_files))
    rate = rospy.Rate(0.2)  # 每张图片间隔 5 秒

    for img_path in img_files:
        if rospy.is_shutdown():
            break
        img = cv2.imread(img_path)
        if img is None:
            rospy.logwarn("无法读取图片: %s" % img_path)
            continue
        msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "kinect_frame_optical"
        pub.publish(msg)
        rospy.loginfo("已发布图片: %s" % os.path.basename(img_path))
        rate.sleep()

    rospy.loginfo("所有图片已发布完毕。")


if __name__ == '__main__':
    try:
        publish_images()
    except rospy.ROSInterruptException:
        pass
