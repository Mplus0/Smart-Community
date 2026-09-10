#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLOv5 ROS 检测节点：订阅图像话题，推理后发布 BoundingBoxes 与带框图像。
"""
import cv2
import torch
import rospy
import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes


class YoloDect:
    def __init__(self):
        # 加载参数
        yolov5_path = rospy.get_param('/yolov5_path', '')
        weight_path = rospy.get_param('~weight_path', '')
        image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        pub_topic = rospy.get_param('~pub_topic', '/yolov5/BoundingBoxes')
        self.camera_frame = rospy.get_param('~camera_frame', '')
        conf = float(rospy.get_param('~conf', '0.5'))

        # 加载本地 YOLOv5 模型（source='local'）
        self.model = torch.hub.load(yolov5_path, 'custom', path=weight_path, source='local')

        # 设备选择
        if rospy.get_param('/use_cpu', 'false'):
            self.model.cpu()
        else:
            self.model.cuda()

        self.model.conf = conf
        self.getImageStatus = False
        self.classes_colors = {}

        # 图像订阅
        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback,
                                          queue_size=1, buff_size=52428800)

        # 输出发布
        self.position_pub = rospy.Publisher(pub_topic, BoundingBoxes, queue_size=1)
        self.image_pub = rospy.Publisher('/yolov5/detection_image', Image, queue_size=1)

        # 等待图像
        while not self.getImageStatus and not rospy.is_shutdown():
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)

    def image_callback(self, image):
        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = image.header
        self.boundingBoxes.image_header = image.header
        self.getImageStatus = True

        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)

        results = self.model(self.color_image)
        boxs = results.pandas().xyxy[0].values
        self.dectshow(self.color_image, boxs, image.height, image.width)

        cv2.waitKey(3)

    def dectshow(self, org_img, boxs, height, width):
        img = org_img.copy()

        for idx, box in enumerate(boxs):
            boundingBox = BoundingBox()
            boundingBox.probability = np.float64(box[4])
            boundingBox.xmin = np.int64(box[0])
            boundingBox.ymin = np.int64(box[1])
            boundingBox.xmax = np.int64(box[2])
            boundingBox.ymax = np.int64(box[3])
            boundingBox.num = np.int16(idx)
            boundingBox.Class = box[-1]

            if box[-1] in self.classes_colors:
                color = self.classes_colors[box[-1]]
            else:
                color = np.random.randint(0, 183, 3)
                self.classes_colors[box[-1]] = color

            cv2.rectangle(img, (int(box[0]), int(box[1])),
                          (int(box[2]), int(box[3])),
                          (int(color[0]), int(color[1]), int(color[2])), 2)

            text_pos_y = box[1] + 30 if box[1] < 20 else box[1] - 10
            cv2.putText(img, str(box[-1]),
                        (int(box[0]), int(text_pos_y) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

            self.boundingBoxes.bounding_boxes.append(boundingBox)

        self.position_pub.publish(self.boundingBoxes)
        self.publish_image(img, height, width)
        cv2.imshow('YOLOv5', img)

    def publish_image(self, imgdata, height, width):
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = self.camera_frame
        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'bgr8'
        image_temp.data = np.array(imgdata).tobytes()
        image_temp.header = header
        image_temp.step = width * 3
        self.image_pub.publish(image_temp)


def main():
    rospy.init_node('yolov5_ros', anonymous=True)
    YoloDect()
    rospy.spin()


if __name__ == "__main__":
    main()
