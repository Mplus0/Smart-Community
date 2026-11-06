#!/usr/bin/env python3
# 倒伏统计服务；同目录 sort.py 提供目标跟踪。
import rospy
import cv2
import numpy as np
import onnxruntime as ort
import sys
import os
from sort import Sort
# import sort
from community_perception.srv import CountFallStand, CountFallStandResponse


class SimpleFallCounter:
    def __init__(self, model_path="byc.onnx", conf_thres=0.8, iou_thres=0.5):
        """
        简化的跌倒计数器
        """
        # 类定义
        self.CLASSES = {int(k): v for k, v in rospy.get_param('~class_names', {'0': 'stand', '1': 'fall'}).items()}

        # 从参数服务器获取配置
        model_path = rospy.get_param('~model_path', model_path)
        self.conf_thres = rospy.get_param('~conf_thres', conf_thres)
        self.iou_thres = rospy.get_param('~iou_thres', iou_thres)

        # ROI配置
        self.roi_enabled = rospy.get_param('~roi_enabled', True)
        self.roi_center_x = rospy.get_param('~roi_center_x', 0.5)
        self.roi_center_y = rospy.get_param('~roi_center_y', 0.6)
        self.roi_width_ratio = rospy.get_param('~roi_width_ratio', 0.8)
        self.roi_height_ratio = rospy.get_param('~roi_height_ratio', 0.9)

        # 抽帧配置
        self.frame_stride = rospy.get_param('~frame_stride', 3)

        rospy.loginfo(f"加载模型: {model_path}")

        # 初始化ONNX Runtime
        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        try:
            self.session = ort.InferenceSession(model_path, providers=providers)
            rospy.loginfo("使用GPU加速")
        except Exception:
            rospy.loginfo("使用CPU推理")
            self.session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])

        # 获取输入信息
        inp = self.session.get_inputs()[0]
        self.input_name = inp.name
        shape = inp.shape

        if len(shape) == 4:
            self.in_h, self.in_w = int(shape[2]), int(shape[3])
        else:
            self.in_h, self.in_w = 416, 416

        rospy.loginfo(f"模型输入尺寸: {self.in_w}x{self.in_h}")

    def iou_xyxy(self, box1, box2):
        """计算IoU"""
        # 确保box2是列表而不是NumPy数组
        if isinstance(box2, np.ndarray):
            box2 = box2.tolist()

        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])

        inter_w = max(0.0, x2 - x1)
        inter_h = max(0.0, y2 - y1)
        inter = inter_w * inter_h

        if inter == 0:
            return 0.0

        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union = area1 + area2 - inter + 1e-6

        return inter / union

    def preprocess(self, img, img_h, img_w):
        """图像预处理"""
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_resized = cv2.resize(img_rgb, (self.in_w, self.in_h))
        img_fp32 = img_resized.astype(np.float32) / 255.0
        img_chw = np.transpose(img_fp32, (2, 0, 1))
        img_batched = np.expand_dims(img_chw, axis=0)
        return img_batched, img_h, img_w

    def decode_yolo(self, outputs, img_h, img_w):
        """解码YOLO输出"""
        out = outputs[0]
        out = np.squeeze(out)
        out = out.T

        boxes_xywh = []
        scores = []
        class_ids = []

        x_scale = img_w / self.in_w
        y_scale = img_h / self.in_h

        for i in range(out.shape[0]):
            cls_scores = out[i, 4:]
            score = np.max(cls_scores)

            if score < self.conf_thres:
                continue

            cls_id = int(np.argmax(cls_scores))
            x, y, w, h = out[i, 0:4]

            cx = x * x_scale
            cy = y * y_scale
            bw = w * x_scale
            bh = h * y_scale

            x1 = cx - bw / 2
            y1 = cy - bh / 2

            boxes_xywh.append([x1, y1, bw, bh])
            scores.append(float(score))
            class_ids.append(cls_id)

        return boxes_xywh, scores, class_ids

    def nms(self, boxes_xywh, scores):
        """非极大值抑制"""
        if len(boxes_xywh) == 0:
            return []

        boxes_cv2 = []
        for (x, y, w, h) in boxes_xywh:
            boxes_cv2.append([int(x), int(y), int(w), int(h)])

        idxs = cv2.dnn.NMSBoxes(boxes_cv2, scores, self.conf_thres, self.iou_thres)

        if len(idxs) == 0:
            return []

        return idxs.flatten().tolist()

    def compute_roi(self, img_h, img_w):
        """计算ROI区域"""
        if not self.roi_enabled:
            return None

        cx = self.roi_center_x * img_w
        cy = self.roi_center_y * img_h
        w = self.roi_width_ratio * img_w
        h = self.roi_height_ratio * img_h

        x1 = int(max(0, cx - w / 2))
        y1 = int(max(0, cy - h / 2))
        x2 = int(min(img_w - 1, cx + w / 2))
        y2 = int(min(img_h - 1, cy + h / 2))

        return (x1, y1, x2, y2)

    def is_box_in_roi(self, box_xywh, roi_rect):
        """判断检测框是否在ROI内"""
        if roi_rect is None:
            return True

        x, y, w, h = box_xywh
        cx = x + w / 2
        cy = y + h / 2
        x1, y1, x2, y2 = roi_rect

        return (cx >= x1) and (cx <= x2) and (cy >= y1) and (cy <= y2)

    def count_video(self, video_path):
        """
        计算视频中的跌倒和站立数量
        返回: (fall_count, stand_count, message, success)
        """
        # 检查文件是否存在
        if not os.path.exists(video_path):
            return 0, 0, f"视频文件不存在: {video_path}", False

        # 打开视频
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            return 0, 0, f"无法打开视频文件: {video_path}", False

        # 获取视频信息
        fps = cap.get(cv2.CAP_PROP_FPS)
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

        if total_frames == 0:
            cap.release()
            return 0, 0, "视频文件为空或损坏", False

        rospy.loginfo(f"开始处理视频: {video_path}")
        rospy.loginfo(f"视频信息: 总帧数={total_frames}, FPS={fps}")
        rospy.loginfo(f"处理参数: ROI={self.roi_enabled}, 抽帧={self.frame_stride}")

        # 初始化追踪器和计数器
        tracker = Sort(max_age=rospy.get_param('~tracker_max_age', 20),
                       min_hits=rospy.get_param('~tracker_min_hits', 3),
                       iou_threshold=rospy.get_param('~tracker_iou_threshold', 0.3))
        track_class = {}  # track_id -> class_id
        fall_count = 0
        stand_count = 0

        frame_idx = 0
        processed_count = 0

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frame_idx += 1
            img_h, img_w = frame.shape[:2]

            # 抽帧处理
            if frame_idx % self.frame_stride != 0:
                continue

            processed_count += 1

            # 处理进度反馈
            if processed_count % 50 == 0:
                progress = frame_idx / total_frames * 100
                rospy.loginfo(f"处理进度: {progress:.1f}% ({frame_idx}/{total_frames})")

            # 预处理
            inp, img_h, img_w = self.preprocess(frame, img_h, img_w)

            # 计算ROI
            roi_rect = self.compute_roi(img_h, img_w)

            # 推理
            outputs = self.session.run(None, {self.input_name: inp})
            boxes_xywh, scores, class_ids = self.decode_yolo(outputs, img_h, img_w)

            # NMS
            keep = self.nms(boxes_xywh, scores)

            # 过滤ROI内的检测框
            dets_xyxy = []
            kept_scores = []
            kept_cls_ids = []
            kept_xywh = []

            for i in keep:
                box = boxes_xywh[i]
                if not self.is_box_in_roi(box, roi_rect):
                    continue

                x, y, w, h = box
                dets_xyxy.append([x, y, x + w, y + h])
                kept_scores.append(scores[i])
                kept_cls_ids.append(class_ids[i])
                kept_xywh.append([x, y, w, h])

            # 更新追踪器 - 修复：检查列表长度而不是列表本身
            if len(dets_xyxy) > 0:
                dets_xyxy_np = np.array(dets_xyxy, dtype=np.float32)
                scores_np = np.array(kept_scores, dtype=np.float32)
                dets_for_sort = np.concatenate([dets_xyxy_np, scores_np[:, None]], axis=1)
                tracks = tracker.update(dets_for_sort)
            else:
                tracks = tracker.update(np.empty((0, 5)))

            # 处理追踪结果并计数
            for t in tracks:
                x1, y1, x2, y2, tid = t
                tid = int(tid)

                # 匹配检测框获取类别
                best_idx = -1
                best_iou = 0.0
                # 修复：只有当有检测框时才进行匹配
                if len(dets_xyxy) > 0:
                    for i, det_box in enumerate(dets_xyxy):
                        iou = self.iou_xyxy([x1, y1, x2, y2], det_box)
                        if iou > best_iou:
                            best_iou = iou
                            best_idx = i

                if best_idx != -1:
                    cls_id = kept_cls_ids[best_idx]

                    # 如果是第一次出现的track_id，计数
                    if tid not in track_class:
                        track_class[tid] = cls_id
                        if self.CLASSES.get(cls_id) == 'fall':
                            fall_count += 1
                        elif self.CLASSES.get(cls_id) == 'stand':
                            stand_count += 1

        # 释放视频
        cap.release()

        # 返回结果
        total_count = fall_count + stand_count
        message = f"处理完成: fall={fall_count}, stand={stand_count}, total={total_count}"
        rospy.loginfo(message)

        return fall_count, stand_count, message, True


class FallCounterServer:
    def __init__(self):
        rospy.init_node('fall_counter_server')

        # 初始化计数器
        self.counter = SimpleFallCounter()

        # 创建服务
        self.service = rospy.Service('count_fall_stand', CountFallStand, self.handle_count_request)

        rospy.loginfo("跌倒计数服务已启动")
        rospy.loginfo("等待视频处理请求...")

    def handle_count_request(self, req):
        """处理计数请求"""
        rospy.loginfo(f"收到视频计数请求: {req.video_path}")

        try:
            # 计算视频中的跌倒和站立数量
            fall_count, stand_count, message, success = self.counter.count_video(req.video_path)

            return CountFallStandResponse(
                fall_count=fall_count,
                stand_count=stand_count,
                message=message,
                success=success
            )

        except Exception as e:
            rospy.logerr(f"处理视频时发生错误: {str(e)}")

            return CountFallStandResponse(
                fall_count=0,
                stand_count=0,
                message=f"处理错误: {str(e)}",
                success=False
            )


if __name__ == "__main__":
    try:
        server = FallCounterServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("服务节点已关闭")
