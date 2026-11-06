#!/usr/bin/env python3
# YOLO 检测服务：模型推理与 ROS 服务入口。

import os
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import onnxruntime as ort
from community_perception.srv import ObjectDetection, ObjectDetectionResponse
from collections import Counter

class YOLOv8DetectorService:
    def __init__(self):
        self.bridge = CvBridge()

        # 从参数服务器获取配置
        self.model_path = rospy.get_param('~model_path', './models/best.onnx')
        self.conf_threshold = rospy.get_param('~conf_threshold', 0.5)
        self.iou_threshold = rospy.get_param('~iou_threshold', 0.4)

        # 从参数服务器获取类别映射
        # 支持多种格式：列表、字典、字符串
        class_names_param = rospy.get_param('~class_names', None)

        if class_names_param is None:
            # 默认类别（向后兼容）
            self.class_names = {
                0: "peopleInside",
                1: "peopleOutside",
                2: "object3",
                3: "object4",
                4: "object5"
            }
        elif isinstance(class_names_param, list):
            # 列表格式：[class0_name, class1_name, ...]
            self.class_names = {i: name for i, name in enumerate(class_names_param)}
        elif isinstance(class_names_param, dict):
            # 字典格式：{0: "class0", 1: "class1", ...}
            self.class_names = class_names_param
        else:
            rospy.logwarn("未知的类别名称格式，使用默认类别")
            self.class_names = {
                0: "peopleInside",
                1: "peopleOutside",
                2: "object3",
                3: "object4",
                4: "object5"
            }

        self.nc = len(self.class_names)

        # 模型名称（可选，用于日志）
        self.model_name = rospy.get_param('~model_name', os.path.basename(self.model_path))

        # 保存配置
        self.save_dir = rospy.get_param('~save_dir', os.path.expanduser('~/.ros/community/yolo_onnx_ros'))
        self.save_enabled = rospy.get_param('~save_enabled', True)
        self.save_idx = 0
        os.makedirs(self.save_dir, exist_ok=True)

        # 指定要保存的类别（可选）
        self.save_classes = rospy.get_param('~save_classes', [0, 1])  # 默认保存人员类别

        # 检查模型文件
        if not os.path.isfile(self.model_path):
            rospy.logerr(f"模型文件未找到: {self.model_path}")
            rospy.logerr(f"当前工作目录: {os.getcwd()}")
            rospy.signal_shutdown("模型文件未找到")
            return

        try:
            # 根据系统配置选择执行提供者
            providers = []
            if ort.get_device() == 'GPU':
                rospy.loginfo("检测到GPU，尝试使用CUDA执行")
                providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
            else:
                providers = ['CPUExecutionProvider']

            self.session = ort.InferenceSession(
                self.model_path,
                providers=providers
            )
            rospy.loginfo(f"模型 '{self.model_name}' 加载成功")
            rospy.loginfo(f"使用执行提供者: {self.session.get_providers()}")
        except Exception as e:
            rospy.logerr(f"模型加载失败: {str(e)}")
            rospy.signal_shutdown("模型加载失败")
            return

        # 获取模型信息
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [output.name for output in self.session.get_outputs()]

        self.input_shape = self.session.get_inputs()[0].shape
        self.input_height, self.input_width = self.input_shape[2:4]

        # 服务名称（可选参数）
        service_name = rospy.get_param('~service_name', '~/detect_objects')

        # 创建服务
        self.service = rospy.Service(service_name, ObjectDetection, self.handle_detection_request)

        rospy.loginfo(f"YOLOv8检测器服务初始化完成")
        rospy.loginfo(f"模型: {self.model_name}")
        rospy.loginfo(f"输入尺寸: {self.input_width}x{self.input_height}")
        rospy.loginfo(f"类别数量: {self.nc}")
        rospy.loginfo(f"类别: {list(self.class_names.values())}")
        rospy.loginfo(f"服务名称: {service_name}")

    def handle_detection_request(self, req):
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(req.image, desired_encoding='bgr8')

            # 推理
            input_tensor = self.preprocess(cv_image)
            outputs = self.session.run(self.output_names, {self.input_name: input_tensor})
            detections = self.postprocess(outputs[0], cv_image.shape)

            # 统计结果
            class_counts = self.count_objects(detections)

            # 构建响应
            response = ObjectDetectionResponse()

            for detection in detections:
                response.detected_classes.append(detection['class_name'])
                response.confidences.append(detection['score'])
                response.bboxes.append(detection['bbox'][0])
                response.bboxes.append(detection['bbox'][1])
                response.bboxes.append(detection['bbox'][2])
                response.bboxes.append(detection['bbox'][3])

            for class_name, count in class_counts.items():
                response.class_names.append(class_name)
                response.class_counts.append(count)

            response.success = True
            response.message = f"检测到 {len(detections)} 个物体"

            rospy.logdebug(f"服务处理完成: {response.message}")

            # 保存检测结果图像（如果启用）
            if self.save_enabled and len(detections) > 0:
                self.save_detection_image(cv_image, detections)

            return response

        except Exception as e:
            rospy.logerr(f"处理检测请求时出错: {str(e)}")
            response = ObjectDetectionResponse()
            response.success = False
            response.message = f"处理失败: {str(e)}"
            return response

    def save_detection_image(self, cv_image, detections):
        """保存检测结果图像"""
        try:
            # 检查是否有需要保存的类别
            save_required = any(d['class_id'] in self.save_classes for d in detections)
            if not save_required:
                return

            vis = cv_image.copy()

            # 绘制所有检测框
            for d in detections:
                # 为不同类别分配不同颜色
                color = self.get_class_color(d['class_id'])
                x1, y1, x2, y2 = d['bbox']
                cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
                label = f"{d['class_name']}:{d['score']:.2f}"
                cv2.putText(vis, label, (x1, y1-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # 保存图像
            timestamp = rospy.Time.now().to_sec()
            save_path = os.path.join(
                self.save_dir,
                f"{self.model_name}_{self.save_idx:03d}_{timestamp:.0f}.jpg"
            )
            cv2.imwrite(save_path, vis)

            rospy.loginfo(f"保存检测图: {save_path} (检测到 {len(detections)} 个物体)")
            self.save_idx += 1

        except Exception as e:
            rospy.logwarn(f"保存图像失败: {str(e)}")

    def get_class_color(self, class_id):
        """为不同类别分配不同颜色"""
        colors = [
            (0, 255, 0),    # 绿色 - class 0
            (0, 0, 255),    # 红色 - class 1
            (255, 0, 0),    # 蓝色 - class 2
            (255, 255, 0),  # 青色 - class 3
            (255, 0, 255),  # 紫色 - class 4
            (0, 255, 255),  # 黄色 - class 5
            (128, 0, 0),    # 深蓝 - class 6
            (0, 128, 0),    # 深绿 - class 7
            (0, 0, 128),    # 深红 - class 8
        ]
        return colors[class_id % len(colors)]

    def count_objects(self, detections):
        class_names = [detection['class_name'] for detection in detections]
        return dict(Counter(class_names))

    def preprocess(self, image):
        input_img = cv2.resize(image, (self.input_width, self.input_height))
        input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2RGB)
        input_img = input_img.transpose(2, 0, 1)
        input_tensor = input_img[np.newaxis, :, :, :].astype(np.float32) / 255.0
        return input_tensor

    def postprocess(self, output, orig_shape):
        outputs = np.transpose(output, (0, 2, 1))

        boxes = []
        scores = []
        class_ids = []

        orig_h, orig_w = orig_shape[:2]
        input_h, input_w = self.input_height, self.input_width

        for i in range(outputs.shape[1]):
            detection = outputs[0, i, :]
            classes_scores = detection[4:4+self.nc]
            max_score = np.max(classes_scores)
            class_id = np.argmax(classes_scores)

            if max_score >= self.conf_threshold:
                cx, cy, w, h = detection[0], detection[1], detection[2], detection[3]
                x1 = int((cx - w / 2) * orig_w / input_w)
                y1 = int((cy - h / 2) * orig_h / input_h)
                x2 = int((cx + w / 2) * orig_w / input_w)
                y2 = int((cy + h / 2) * orig_h / input_h)

                x1 = max(0, min(x1, orig_w - 1))
                y1 = max(0, min(y1, orig_h - 1))
                x2 = max(0, min(x2, orig_w - 1))
                y2 = max(0, min(y2, orig_h - 1))

                bbox_width = x2 - x1
                bbox_height = y2 - y1

                if bbox_width > 5 and bbox_height > 5:
                    boxes.append([x1, y1, x2, y2])
                    scores.append(max_score)
                    class_ids.append(class_id)

        if len(boxes) > 0:
            boxes_xywh = []
            for box in boxes:
                x1, y1, x2, y2 = box
                boxes_xywh.append([x1, y1, x2 - x1, y2 - y1])

            indices = cv2.dnn.NMSBoxes(boxes_xywh, scores, self.conf_threshold, self.iou_threshold)

            final_boxes = []
            final_scores = []
            final_class_ids = []

            if len(indices) > 0:
                for i in indices.flatten():
                    final_boxes.append(boxes[i])
                    final_scores.append(scores[i])
                    final_class_ids.append(class_ids[i])

            detections = []
            for i in range(len(final_boxes)):
                detections.append({
                    'class_id': final_class_ids[i],
                    'class_name': self.class_names.get(final_class_ids[i], f"class_{final_class_ids[i]}"),
                    'score': final_scores[i],
                    'bbox': final_boxes[i]
                })

            return detections

        return []

def main():
    rospy.init_node('yolov8_detector_service', anonymous=True)

    try:
        # 获取节点名称（可以用于多实例区分）
        node_name = rospy.get_name()
        rospy.loginfo(f"启动检测器节点: {node_name}")

        detector_service = YOLOv8DetectorService()

        rospy.loginfo("YOLOv8检测器服务节点已启动")
        rospy.loginfo("等待检测请求...")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("节点被用户中断")
    except Exception as e:
        rospy.logerr(f"节点启动失败: {str(e)}")

if __name__ == '__main__':
    main()
