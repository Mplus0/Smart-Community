#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Collect traffic-light images from ROS by hand.

Keys (focus the preview window):
  1: RED     2: YELLOW     3: GREEN
  S: save   Q/Esc: quit
Mouse:
  Left-click a color button to select the class;
  left-click elsewhere in the image to save. Right-click does nothing.
"""

import argparse
from datetime import datetime
from pathlib import Path
import threading
import time

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


CLASSES = {ord('1'): 'red', ord('2'): 'yellow', ord('3'): 'green'}
BUTTONS = [(10, 113, 'red'), (123, 243, 'yellow'), (253, 373, 'green')]


class CameraCapture:
    def __init__(self, topic):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame = None
        self.received_at = None
        self.subscriber = rospy.Subscriber(
            topic, Image, self._on_image, queue_size=1, buff_size=8 * 1024 * 1024
        )

    def _on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except (CvBridgeError, ValueError) as exc:
            rospy.logwarn_throttle(5.0, 'Unable to decode camera frame: %s', exc)
            return
        with self.lock:
            self.frame = frame.copy()
            self.received_at = time.monotonic()

    def latest(self):
        with self.lock:
            if self.frame is None:
                return None, None
            return self.frame.copy(), self.received_at


def main():
    parser = argparse.ArgumentParser(description='Capture classified traffic-light photos')
    parser.add_argument('--image-topic', default='/camera/color/image_raw')
    parser.add_argument('--output-dir', default=None,
                        help='Dataset root, e.g. /workspace/car_2026/datasets/traffic_light')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('traffic_light_camera_capture', anonymous=True)
    # The intended location of this script is car_2026/src/robot_competition/scripts/.
    workspace = Path(__file__).resolve().parents[3]
    output = (Path(args.output_dir).expanduser().resolve() if args.output_dir
              else workspace / 'datasets' / 'traffic_light')
    for name in CLASSES.values():
        (output / name).mkdir(parents=True, exist_ok=True)

    capture = CameraCapture(args.image_topic)
    selected = 'red'
    save_requested = False
    title = 'Traffic Light Capture - 1/2/3 Class, S Save, Q Quit'

    def on_mouse(event, x, y, flags, userdata):
        nonlocal selected, save_requested
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if 0 <= y <= 46:
            for x_start, x_end, name in BUTTONS:
                if x_start <= x <= x_end:
                    selected = name
                    print('Selected:', selected, flush=True)
                    return
        save_requested = True

    print('Running script:', Path(__file__).resolve(), flush=True)
    print('Camera topic:', args.image_topic, flush=True)
    print('Dataset root:', output, flush=True)
    print('Keys: 1=RED, 2=YELLOW, 3=GREEN, S=SAVE, Q=QUIT', flush=True)
    print('Mouse: left-click a color label to select; left-click image to save.', flush=True)

    cv2.namedWindow(title, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(title, on_mouse)
    try:
        while not rospy.is_shutdown():
            frame, arrival = capture.latest()
            if frame is not None:
                preview = frame.copy()
                height, width = preview.shape[:2]
                cv2.rectangle(preview, (0, 0), (width, min(height - 1, 47)),
                              (27, 27, 27), -1)
                for left, right, name in BUTTONS:
                    if left >= width:
                        continue
                    color = (0, 0, 255) if name == 'red' else (
                        (0, 255, 255) if name == 'yellow' else (0, 200, 0))
                    cv2.rectangle(preview, (left, 5), (min(right, width - 1), 42),
                                  color, 2 if selected == name else 1)
                    cv2.putText(preview, name.upper(), (left + 8, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
                if width >= 495:
                    cv2.putText(preview, 'S: SAVE', (385, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255),
                                2, cv2.LINE_AA)
                cv2.imshow(title, preview)

            key = cv2.waitKeyEx(30)
            if key in (ord('q'), ord('Q'), 27):
                break
            if key in CLASSES:
                selected = CLASSES[key]
                print('Selected:', selected, flush=True)
            if key in (ord('s'), ord('S')):
                save_requested = True

            if save_requested:
                save_requested = False
                # Grab a fresh unannotated frame, not the image with GUI controls.
                frame, arrival = capture.latest()
                if frame is None or arrival is None:
                    print('No image received; nothing saved.', flush=True)
                    continue
                if time.monotonic() - arrival > 2.0:
                    print('Camera frame is stale; nothing saved.', flush=True)
                    continue
                filename = datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.jpg'
                path = output / selected / filename
                if cv2.imwrite(str(path), frame):
                    print('SAVED [{}]: {}'.format(selected.upper(), path), flush=True)
                else:
                    rospy.logerr('Image write failed: %s', path)
    finally:
        capture.subscriber.unregister()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
