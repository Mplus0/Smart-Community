#!/usr/bin/env python3
"""ROS1 camera subscriber and YOLO26 community-person detector."""

import json
import math
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
import torch
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


EXPECTED_NAMES = {0: "community_person", 1: "non_community_person"}
DISPLAY_NAMES = {0: "C", 1: "NC"}
BOX_COLORS = {0: (0, 220, 0), 1: (0, 165, 255)}  # BGR


class ImageDecodeError(ValueError):
    pass


def decode_bgr(msg):
    channels = {"bgr8": 3, "rgb8": 3, "mono8": 1, "bgra8": 4, "rgba8": 4}
    count = channels.get(msg.encoding)
    if count is None:
        raise ImageDecodeError("unsupported image encoding: " + str(msg.encoding))
    if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * count:
        raise ImageDecodeError("invalid image dimensions or row step")
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size != msg.height * msg.step:
        raise ImageDecodeError("image buffer length does not match height * step")
    pixels = raw.reshape(msg.height, msg.step)[:, :msg.width * count]
    pixels = pixels.reshape(msg.height, msg.width, count)
    conversions = {
        "rgb8": cv2.COLOR_RGB2BGR,
        "mono8": cv2.COLOR_GRAY2BGR,
        "bgra8": cv2.COLOR_BGRA2BGR,
        "rgba8": cv2.COLOR_RGBA2BGR,
    }
    if msg.encoding == "mono8":
        pixels = pixels[:, :, 0]
    if msg.encoding in conversions:
        pixels = cv2.cvtColor(pixels, conversions[msg.encoding])
    return np.ascontiguousarray(pixels).copy()


def encode_bgr(pixels, header):
    pixels = np.ascontiguousarray(pixels, dtype=np.uint8)
    result = Image()
    result.header = header
    result.height, result.width = pixels.shape[:2]
    result.encoding = "bgr8"
    result.step = result.width * 3
    result.is_bigendian = 0
    result.data = pixels.tobytes()
    return result


def header_json(header):
    return {
        "seq": int(header.seq),
        "stamp": {"secs": int(header.stamp.secs), "nsecs": int(header.stamp.nsecs)},
        "frame_id": str(header.frame_id),
    }


def draw_detection(image, detection):
    x1, y1, x2, y2 = detection["bbox_xyxy"]
    x1i, y1i = int(round(x1)), int(round(y1))
    x2i, y2i = int(round(x2)), int(round(y2))
    class_id = detection["class_id"]
    color = BOX_COLORS[class_id]
    label = "{} {:.2f}".format(DISPLAY_NAMES[class_id], detection["confidence"])
    cv2.rectangle(image, (x1i, y1i), (x2i, y2i), color, 2)
    font = cv2.FONT_HERSHEY_SIMPLEX
    (text_width, text_height), baseline = cv2.getTextSize(label, font, 0.55, 2)
    left = max(0, min(x1i, image.shape[1] - text_width - 8))
    top = y1i - text_height - baseline - 8
    if top < 0:
        top = min(y1i + 3, max(0, image.shape[0] - text_height - baseline - 8))
    cv2.rectangle(image, (left, top), (left + text_width + 8,
                  top + text_height + baseline + 8), (0, 0, 0), -1)
    cv2.putText(image, label, (left + 4, top + text_height + 3), font,
                0.55, color, 2, cv2.LINE_AA)


class PersonDetectionNode:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/camera/color/image_raw")
        self.image_topic = rospy.get_param("~image_topic", "/perception/person_image")
        self.json_topic = rospy.get_param(
            "~json_topic", "/perception/person_detections_json")
        self.model_path = str(rospy.get_param("~model", ""))
        self.device = str(rospy.get_param("~device", "0"))
        self.imgsz = int(rospy.get_param("~imgsz", 640))
        self.confidence = float(rospy.get_param("~confidence", 0.25))
        self.iou = float(rospy.get_param("~iou", 0.45))
        self.max_det = int(rospy.get_param("~max_det", 100))
        self.max_rate = float(rospy.get_param("~max_rate", 5.0))
        self.stale_seconds = float(rospy.get_param("~stale_seconds", 1.5))
        self.warmup = bool(rospy.get_param("~warmup", True))

        numeric = (self.confidence, self.iou, self.max_rate, self.stale_seconds)
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("confidence, iou, rate and stale timeout must be finite")
        if not 0.0 <= self.confidence <= 1.0 or not 0.0 <= self.iou <= 1.0:
            raise ValueError("confidence and iou must be between 0 and 1")
        if self.imgsz < 32 or self.max_det < 1 or self.max_rate <= 0 or self.stale_seconds <= 0:
            raise ValueError("invalid image size, max_det, rate or stale timeout")
        if len({rospy.resolve_name(name) for name in
                (self.input_topic, self.image_topic, self.json_topic)}) != 3:
            raise ValueError("input, annotated image and JSON topics must differ")
        if not Path(self.model_path).is_file():
            raise FileNotFoundError("YOLO weights not found: " + self.model_path)
        if self.device != "cpu" and not torch.cuda.is_available():
            raise RuntimeError("CUDA device requested but torch.cuda.is_available() is false")

        rospy.loginfo("Loading personnel YOLO model: %s", self.model_path)
        self.model = YOLO(self.model_path)
        raw_names = self.model.names
        if isinstance(raw_names, dict):
            self.names = {int(key): str(value) for key, value in raw_names.items()}
        else:
            self.names = {index: str(value) for index, value in enumerate(raw_names)}
        if self.names != EXPECTED_NAMES:
            raise ValueError("model class mapping mismatch: expected {}, got {}".format(
                EXPECTED_NAMES, self.names))

        if self.device != "cpu":
            gpu_name = torch.cuda.get_device_name(int(self.device))
            rospy.loginfo("Using CUDA device %s: %s", self.device, gpu_name)
        else:
            rospy.loginfo("Using CPU inference")

        if self.warmup:
            warm_image = np.zeros((480, 640, 3), dtype=np.uint8)
            self.model.predict(source=warm_image, imgsz=self.imgsz,
                               device=self.device, verbose=False, max_det=1)
            rospy.loginfo("YOLO warmup complete")

        self.image_pub = rospy.Publisher(self.image_topic, Image, queue_size=1)
        self.json_pub = rospy.Publisher(self.json_topic, String, queue_size=1)
        self.lock = threading.Lock()
        self.latest = [None, 0, 0.0]
        self.processed_generation = 0
        self.last_process_wall = 0.0
        self.stale_reported = False
        self.last_header = None
        self.subscriber = rospy.Subscriber(
            self.input_topic, Image, self._receive, queue_size=1,
            buff_size=8 * 1024 * 1024)
        rospy.loginfo("Person detector ready: %s -> %s and %s",
                      self.input_topic, self.image_topic, self.json_topic)

    def _receive(self, msg):
        with self.lock:
            self.latest[0] = msg
            self.latest[1] += 1
            self.latest[2] = time.monotonic()

    def _publish_json(self, header, width, height, valid, reason,
                      processing_ms, detections):
        payload = {
            "schema_version": 1,
            "engine": "yolo26_person_detector",
            "header": header_json(header) if header is not None else None,
            "width": int(width),
            "height": int(height),
            "frame_valid": bool(valid),
            "reason": str(reason),
            "processing_ms": float(processing_ms),
            "score_kind": "yolo_model_confidence",
            "detections": detections,
        }
        self.json_pub.publish(String(data=json.dumps(
            payload, ensure_ascii=False, separators=(",", ":"), allow_nan=False)))

    def _infer(self, message):
        start = time.perf_counter()
        try:
            image = decode_bgr(message)
        except ImageDecodeError as exc:
            rospy.logwarn_throttle(5.0, "Cannot decode camera frame: %s", exc)
            self._publish_json(message.header, message.width, message.height,
                               False, "decode_error", 0.0, [])
            return
        height, width = image.shape[:2]
        try:
            results = self.model.predict(
                source=image, imgsz=self.imgsz, conf=self.confidence,
                iou=self.iou, device=self.device, verbose=False,
                max_det=self.max_det)
            result = results[0]
            detections = []
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    class_id = int(box.cls[0].detach().cpu().item())
                    if class_id not in EXPECTED_NAMES:
                        continue
                    confidence = float(box.conf[0].detach().cpu().item())
                    raw_xyxy = box.xyxy[0].detach().cpu().tolist()
                    x1 = max(0.0, min(float(width), float(raw_xyxy[0])))
                    y1 = max(0.0, min(float(height), float(raw_xyxy[1])))
                    x2 = max(0.0, min(float(width), float(raw_xyxy[2])))
                    y2 = max(0.0, min(float(height), float(raw_xyxy[3])))
                    detections.append({
                        "class_id": class_id,
                        "label": self.names[class_id],
                        "display_label": DISPLAY_NAMES[class_id],
                        "confidence": round(confidence, 6),
                        "bbox_xyxy": [round(x1, 2), round(y1, 2),
                                      round(x2, 2), round(y2, 2)],
                    })

            annotated = image.copy()
            for detection in detections:
                draw_detection(annotated, detection)
        except Exception as exc:
            rospy.logerr_throttle(5.0, "YOLO inference failed: %s", exc)
            self.image_pub.publish(encode_bgr(image, message.header))
            self._publish_json(message.header, width, height, False,
                               "inference_error", 0.0, [])
            return
        processing_ms = (time.perf_counter() - start) * 1000.0
        self.image_pub.publish(encode_bgr(annotated, message.header))
        self._publish_json(message.header, width, height, True, "ok",
                           processing_ms, detections)
        rospy.loginfo_throttle(
            5.0, "Person detections=%d, %.1f ms", len(detections), processing_ms)
        self.last_header = message.header

    def _publish_invalid(self, header, reason):
        self._publish_json(header, 0, 0, False, reason, 0.0, [])

    def spin(self):
        sleep_seconds = min(0.02, 1.0 / self.max_rate)
        period = 1.0 / self.max_rate
        while not rospy.is_shutdown():
            now = time.monotonic()
            with self.lock:
                message, generation, arrival = self.latest
            if message is not None and generation > self.processed_generation:
                if now - self.last_process_wall >= period:
                    self.processed_generation = generation
                    self.last_process_wall = now
                    self.stale_reported = False
                    try:
                        self._infer(message)
                    except Exception as exc:
                        rospy.logerr_throttle(5.0, "Person node error: %s", exc)
                        self._publish_invalid(message.header, "inference_error")
            elif now - arrival > self.stale_seconds and not self.stale_reported:
                rospy.logwarn("Camera input stale; publishing invalid detection state")
                self._publish_invalid(self.last_header, "camera_stale")
                self.stale_reported = True
            time.sleep(sleep_seconds)


def main():
    rospy.init_node("person_detection")
    node = PersonDetectionNode()
    node.spin()


if __name__ == "__main__":
    main()
