#!/usr/bin/env python3
"""Capture the instrument camera in MJPEG mode at the requested resolution."""
import time

import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image


def main():
    rospy.init_node('ugreen_4k')
    device = rospy.get_param('~video_device')
    width = rospy.get_param('~width', 1920)
    height = rospy.get_param('~height', 1080)
    publisher = rospy.Publisher('~image_raw', Image, queue_size=1)
    bridge = CvBridge()
    cv2.setNumThreads(1)
    camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
    try:
        if not camera.isOpened():
            raise RuntimeError(f'无法打开仪表相机 {device}，请检查设备或采集占用')
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        camera.set(cv2.CAP_PROP_FPS, 30)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        next_publish = 0.0
        while not rospy.is_shutdown():
            ok, frame = camera.read()
            if not ok or frame.shape != (height, width, 3):
                raise RuntimeError(f'仪表相机 {device} 未输出 {width}×{height} 图像')
            now = time.monotonic()
            if publisher.get_num_connections() and now >= next_publish:
                message = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                message.header.stamp = rospy.Time.now()
                message.header.frame_id = 'ugreen_4k_optical_frame'
                publisher.publish(message)
                next_publish = now + 0.085
    except (RuntimeError, cv2.error) as error:
        rospy.logfatal('%s', error)
        return 1
    finally:
        camera.release()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
