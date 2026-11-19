#!/usr/bin/env python3
"""Detect overheated floors in four equal vertical temperature-image regions."""
from collections import deque
import threading
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from community_perception.srv import MeasureBuildingTemperature, MeasureBuildingTemperatureResponse


def floor_temperatures(image):
    """Return finite maxima for floors 4, 3, 2, 1; None means no valid data."""
    if image.ndim != 2 or image.shape[0] == 0 or image.shape[1] < 4:
        raise ValueError("温度矩阵尺寸无效")
    edges = np.linspace(0, image.shape[1], 5, dtype=int)
    maxima = []
    for left, right in zip(edges[:-1], edges[1:]):
        region = image[:, left:right]
        valid = region[np.isfinite(region)]
        maxima.append(float(valid.max()) if valid.size else None)
    return maxima


def draw_preview(temperatures, maxima, streaks, confirm_frames):
    """Render the same temperature matrix used for detection; never crop the view."""
    valid = np.isfinite(temperatures)
    gray = np.zeros(temperatures.shape, dtype=np.uint8)
    if valid.any():
        low, high = temperatures[valid].min(), temperatures[valid].max()
        gray[valid] = ((temperatures[valid] - low) * 255 / max(float(high - low), 1.0)).astype(np.uint8)
    heatmap = cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO)
    heatmap[~valid] = (60, 60, 60)
    height, width = temperatures.shape
    scale, header = 4, 100
    heatmap = cv2.resize(heatmap, (width * scale, height * scale), interpolation=cv2.INTER_NEAREST)
    canvas = np.zeros((height * scale + header, width * scale, 3), dtype=np.uint8)
    canvas[header:] = heatmap
    edges = np.linspace(0, width, 5, dtype=int) * scale
    for index, (left, right) in enumerate(zip(edges[:-1], edges[1:])):
        temperature = maxima[index]
        if temperature is None:
            state, color = "NO DATA", (160, 160, 160)
        elif streaks[index] >= confirm_frames:
            state, color = "ALARM", (0, 0, 255)
        elif streaks[index]:
            state, color = "CHECKING", (0, 220, 255)
        else:
            state, color = "NORMAL", (0, 220, 0)
        cv2.rectangle(canvas, (left, header), (right - 1, canvas.shape[0] - 1), color, 2)
        for row, text in enumerate(("Floor %d" % (4 - index),
                                    "-- C" if temperature is None else "%.1f C" % temperature, state)):
            cv2.putText(canvas, text, (left + 10, 25 + row * 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv2.LINE_AA)
    return canvas


class BuildingDTemperature:
    def __init__(self):
        self.threshold = float(rospy.get_param("~threshold", 40.0))
        self.confirm_frames = int(rospy.get_param("~confirm_frames", 3))
        self.timeout = float(rospy.get_param("~timeout", 3.0))
        if not np.isfinite(self.threshold) or self.confirm_frames < 1 or not 0 < self.timeout < float("inf"):
            raise ValueError("threshold 须为有限温度，confirm_frames 和 timeout 须大于 0")
        self.bridge = CvBridge()
        self.frames = threading.Condition()
        self.valid_stamps = deque(maxlen=self.confirm_frames)
        self.latest_measurement = None
        self.streaks = [0] * 4
        self.last_received = time.monotonic()
        self.last_result = None
        self.result_pub = rospy.Publisher("~result", String, queue_size=1)
        self.preview_pub = rospy.Publisher("~image", Image, queue_size=1)
        self.subscriber = rospy.Subscriber(
            rospy.get_param("~temperature_topic", "/thermal_camera/temperature"),
            Image, self.receive, queue_size=1, buff_size=2**20)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.check_timeout)
        self.measure_service = rospy.Service("~measure", MeasureBuildingTemperature, self.measure)
        rospy.loginfo("楼宇D检测：从左到右 4/3/2/1 楼，温度 > %.1f℃，连续 %d 帧确认",
                      self.threshold, self.confirm_frames)

    def publish(self, result):
        self.result_pub.publish(String(data=result))
        if result != self.last_result:
            rospy.loginfo(result)
            self.last_result = result

    def receive(self, message):
        try:
            if message.encoding != "32FC1":
                raise ValueError("需要 32FC1 摄氏度温度矩阵，不能使用伪彩色图像")
            temperatures = self.bridge.imgmsg_to_cv2(message, "passthrough")
            maxima = floor_temperatures(temperatures)
        except (CvBridgeError, ValueError) as exc:
            with self.frames:
                self.streaks = [0] * 4
                self.valid_stamps.clear()
                self.publish("楼宇D温度数据无效")
                self.publish_unavailable("INVALID DATA")
            rospy.logwarn_throttle(2.0, str(exc))
            return
        with self.frames:
            now = time.monotonic()
            if now - self.last_received > self.timeout:
                self.streaks = [0] * 4
                self.valid_stamps.clear()
            stamp = message.header.stamp
            if self.valid_stamps and stamp <= self.valid_stamps[-1]:
                return
            self.last_received = now
            abnormal, pending, invalid, abnormal_floors = [], [], [], []
            for index, temperature in enumerate(maxima):
                floor = 4 - index
                if temperature is None:
                    self.streaks[index] = 0
                    invalid.append("楼宇D {}楼温度数据无效".format(floor))
                elif temperature > self.threshold:
                    self.streaks[index] = min(self.streaks[index] + 1, self.confirm_frames)
                    if self.streaks[index] >= self.confirm_frames:
                        abnormal.append("楼宇D {}楼温度异常".format(floor))
                        abnormal_floors.append(floor)
                    else:
                        pending.append("楼宇D {}楼超温确认中".format(floor))
                else:
                    self.streaks[index] = 0
            if invalid:
                self.valid_stamps.clear()
            elif not stamp.is_zero():
                self.valid_stamps.append(stamp)
            self.latest_measurement = None if pending or invalid else MeasureBuildingTemperatureResponse(
                success=True, abnormal_floors=abnormal_floors[::-1], max_temperatures_c=maxima[::-1])
            self.frames.notify_all()
            self.publish("\n".join(abnormal + pending + invalid) or "楼宇D温度正常")
            preview = self.bridge.cv2_to_imgmsg(
                draw_preview(temperatures, maxima, self.streaks, self.confirm_frames), "bgr8")
            preview.header = message.header
            self.preview_pub.publish(preview)
            rospy.loginfo_throttle(2.0, "各层最高温度 ℃（4/3/2/1）：" +
                                   ", ".join("无效" if t is None else "%.1f" % t for t in maxima))

    def measure(self, _request):
        """Wait for a complete, settled measurement from frames captured after the request."""
        started = rospy.Time.now()
        deadline = time.monotonic() + self.timeout
        with self.frames:
            while not rospy.is_shutdown():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                if (self.latest_measurement is not None
                        and len(self.valid_stamps) == self.confirm_frames
                        and self.valid_stamps[0] > started
                        and time.monotonic() - self.last_received <= self.timeout):
                    return self.latest_measurement
                self.frames.wait(min(remaining, 0.1))
        return MeasureBuildingTemperatureResponse(success=False, message="未获得完整且已确认的楼宇D温度结果")

    def publish_unavailable(self, text):
        canvas = np.zeros((724, 824, 3), dtype=np.uint8)
        cv2.putText(canvas, text, (180, 350), cv2.FONT_HERSHEY_SIMPLEX,
                    1.2, (0, 0, 255), 2, cv2.LINE_AA)
        image = self.bridge.cv2_to_imgmsg(canvas, "bgr8")
        image.header.stamp = rospy.Time.now()
        self.preview_pub.publish(image)

    def check_timeout(self, event):
        with self.frames:
            if time.monotonic() - self.last_received > self.timeout:
                self.streaks = [0] * 4
                self.valid_stamps.clear()
                self.publish("楼宇D温度数据超时")
                self.publish_unavailable("DATA TIMEOUT")
                rospy.logwarn_throttle(5.0, "楼宇D检测未收到有效温度帧，请检查红外采集节点")


if __name__ == "__main__":
    rospy.init_node("building_d_temperature")
    try:
        detector = BuildingDTemperature()
        rospy.spin()
    except ValueError as exc:
        rospy.logfatal(str(exc))
        raise SystemExit(1)
