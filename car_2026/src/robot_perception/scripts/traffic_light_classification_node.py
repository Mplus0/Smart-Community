#!/usr/bin/env python3
"""ROI classification only: a class score is not evidence that a light exists."""

import json
import math
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO

# Reuse the existing byte/stride-safe adapters without changing either detector.
from person_detection_node import decode_bgr, encode_bgr, header_json


ENGINE = "yolo11n_traffic_light_classifier"
LABELS = {"red", "yellow", "green"}


def validate_model(model):
    if model.task != "classify":
        raise ValueError("Traffic-light weights must have task=classify")
    raw = model.names
    names = ({int(k): str(v) for k, v in raw.items()} if isinstance(raw, dict)
             else dict(enumerate(map(str, raw))))
    if set(names) != {0, 1, 2} or set(names.values()) != LABELS:
        raise ValueError("Expected exactly red/yellow/green, got {!r}".format(names))
    return names


def validate_roi(roi):
    if not isinstance(roi, (list, tuple)) or len(roi) != 4:
        raise ValueError("roi must be [left, top, right, bottom] in relative coordinates")
    coords = tuple(float(value) for value in roi)
    if not all(math.isfinite(value) for value in coords):
        raise ValueError("roi coordinates must be finite")
    x1, y1, x2, y2 = coords
    if not (0 <= x1 < x2 <= 1 and 0 <= y1 < y2 <= 1):
        raise ValueError("roi must be ordered and within [0, 1]")
    return coords


def crop_roi(image, roi):
    height, width = image.shape[:2]
    x1, y1, x2, y2 = validate_roi(roi)
    bounds = [int(x1 * width), int(y1 * height), int(x2 * width), int(y2 * height)]
    left, top, right, bottom = bounds
    if left >= right or top >= bottom:
        raise ValueError("roi is empty at this image resolution")
    return image[top:bottom, left:right].copy(), bounds


def classify(model, names, roi_image, confidence, device="cpu"):
    result = model.predict(source=roi_image, imgsz=224, device=device, verbose=False)[0]
    if result.probs is None:
        raise ValueError("Classification result has no probabilities")
    probabilities = result.probs.data.detach().cpu().numpy()
    if (probabilities.shape != (3,) or not np.isfinite(probabilities).all()
            or np.any(probabilities < 0) or np.any(probabilities > 1)
            or not np.isclose(probabilities.sum(), 1.0, atol=1e-3)):
        raise ValueError("Invalid classification probabilities")
    index = int(np.argmax(probabilities))
    score = float(probabilities[index])
    if score < confidence:
        return "UNKNOWN", score, "low_confidence"
    return names[index].upper(), score, "ok"


def freshness_reason(message, arrival, wall_now, ros_now_ns, timeout):
    """Check both clocks; a paused /clock must not keep a received frame alive."""
    if wall_now - arrival > timeout:
        return "camera_stale"
    source_ns = message.header.stamp.to_nsec()
    if source_ns <= 0 or ros_now_ns <= 0:
        return "invalid_stamp"
    age_ns = ros_now_ns - source_ns
    if age_ns < 0:
        return "future_stamp"
    if age_ns > timeout * 1e9:
        return "camera_stale"
    return "ok"


def make_payload(header, roi, label, confidence, reason, processing_ms):
    valid = reason == "ok" and label in {"RED", "YELLOW", "GREEN"}
    return {
        "schema_version": 1, "engine": ENGINE,
        "header": header_json(header) if header is not None else None,
        "frame_valid": valid, "reason": reason,
        "roi_xyxy": roi, "label": label if valid else "UNKNOWN",
        "confidence": float(confidence), "processing_ms": float(processing_ms),
        "score_kind": "conditional_class_probability",
        "presence_verified": False,
    }


def annotate(image, bounds, payload):
    annotated = image.copy()
    colors = {"RED": (0, 0, 255), "YELLOW": (0, 255, 255),
              "GREEN": (0, 220, 0), "UNKNOWN": (180, 180, 180)}
    color = colors[payload["label"]]
    if bounds is not None:
        x1, y1, x2, y2 = bounds
        cv2.rectangle(annotated, (x1, y1), (x2 - 1, y2 - 1), color, 2)
    text = "{} {:.3f} | {} | presence unverified".format(
        payload["label"], payload["confidence"], payload["reason"])
    cv2.putText(annotated, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, color, 1, cv2.LINE_AA)
    return annotated


class TrafficLightClassificationNode:
    def __init__(self):
        self.roi = validate_roi(rospy.get_param("~roi", [0.0, 0.0, 1.0, 0.5]))
        self.confidence = float(rospy.get_param("~confidence", 0.8))
        self.max_rate = float(rospy.get_param("~max_rate", 5.0))
        self.stale_seconds = float(rospy.get_param("~stale_seconds", 1.5))
        self.device = str(rospy.get_param("~device", "cpu"))
        if (not all(math.isfinite(v) for v in (self.confidence, self.max_rate, self.stale_seconds))
                or not 0 <= self.confidence <= 1 or self.max_rate <= 0 or self.stale_seconds <= 0):
            raise ValueError("Invalid confidence, max_rate or stale_seconds")
        if int(rospy.get_param("~imgsz", 224)) != 224:
            raise ValueError("This model integration requires imgsz=224")
        model_path = Path(str(rospy.get_param("~model", "")))
        if not model_path.is_file():
            raise FileNotFoundError("Traffic-light model not found: " + str(model_path))
        # Infer the task from the checkpoint; passing task=classify could hide a wrong model.
        self.model = YOLO(str(model_path))
        self.names = validate_model(self.model)
        input_topic = rospy.get_param("~input_topic", "/camera/color/image_raw")
        image_topic = rospy.get_param("~image_topic", "/perception/traffic_light_image")
        json_topic = rospy.get_param("~json_topic", "/perception/traffic_light_json")
        if len({rospy.resolve_name(t) for t in (input_topic, image_topic, json_topic)}) != 3:
            raise ValueError("Input, output image and JSON topics must differ")
        self.image_pub = rospy.Publisher(image_topic, Image, queue_size=1)
        self.json_pub = rospy.Publisher(json_topic, String, queue_size=1)
        self.lock = threading.Lock()
        self.latest = (None, 0, time.monotonic())
        self.processed_generation = 0
        self.last_process_wall = -math.inf
        self.last_stamp_ns = 0
        self.stale_reported = False
        self.subscriber = rospy.Subscriber(input_topic, Image, self._receive,
                                           queue_size=1, buff_size=8 * 1024 * 1024)
        rospy.loginfo("Traffic-light classifier: %s; names=%s; device=%s; ROI=%s",
                      model_path, self.names, self.device, self.roi)
        rospy.logwarn("Classification cannot establish traffic-light presence; not a crossing permission")

    def _receive(self, message):
        with self.lock:
            self.latest = (message, self.latest[1] + 1, time.monotonic())

    def _publish(self, message, bounds=None, label="UNKNOWN", confidence=0.0,
                 reason="camera_stale", processing_ms=0.0, image=None):
        header = message.header if message is not None else None
        payload = make_payload(header, bounds, label, confidence, reason, processing_ms)
        if image is not None:
            self.image_pub.publish(encode_bgr(annotate(image, bounds, payload), header))
        self.json_pub.publish(String(data=json.dumps(payload, separators=(",", ":"), allow_nan=False)))

    def _freshness(self, message, arrival):
        return freshness_reason(message, arrival, time.monotonic(),
                                rospy.Time.now().to_nsec(), self.stale_seconds)

    def _infer(self, message, arrival):
        started = time.perf_counter()
        reason = self._freshness(message, arrival)
        if reason != "ok":
            self._publish(message, reason=reason)
            return
        stamp_ns = message.header.stamp.to_nsec()
        if stamp_ns <= self.last_stamp_ns:
            self._publish(message, reason="non_increasing_stamp")
            return
        self.last_stamp_ns = stamp_ns
        try:
            image = decode_bgr(message)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Traffic-light image decode failed: %s", exc)
            self._publish(message, reason="decode_error",
                          processing_ms=(time.perf_counter() - started) * 1000)
            return
        bounds = None
        try:
            roi_image, bounds = crop_roi(image, self.roi)
            label, confidence, reason = classify(
                self.model, self.names, roi_image, self.confidence, self.device)
        except Exception as exc:
            rospy.logerr_throttle(5.0, "Traffic-light inference failed: %s", exc)
            label, confidence, reason = "UNKNOWN", 0.0, "inference_error"
        # Slow inference must never turn an expired source frame into valid GREEN.
        fresh = self._freshness(message, arrival)
        if fresh != "ok":
            label, confidence, reason = "UNKNOWN", 0.0, fresh
        self._publish(message, bounds, label, confidence, reason,
                      (time.perf_counter() - started) * 1000, image)

    def step(self):
        now = time.monotonic()
        if now - self.last_process_wall < 1.0 / self.max_rate:
            return
        with self.lock:
            message, generation, arrival = self.latest
        if message is not None and generation > self.processed_generation:
            self.processed_generation = generation
            self.last_process_wall = now
            self.stale_reported = False
            try:
                self._infer(message, arrival)
            except Exception as exc:
                rospy.logerr_throttle(5.0, "Traffic-light node error: %s", exc)
                self._publish(message, reason="inference_error")
        elif now - arrival > self.stale_seconds and not self.stale_reported:
            self.last_process_wall = now
            self._publish(message, reason="camera_stale")
            self.stale_reported = True

    def spin(self):
        while not rospy.is_shutdown():
            self.step()
            time.sleep(min(0.02, 1.0 / self.max_rate))


def main():
    rospy.init_node("traffic_light_classification")
    TrafficLightClassificationNode().spin()


if __name__ == "__main__":
    main()
