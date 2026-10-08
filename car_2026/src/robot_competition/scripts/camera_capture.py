#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Manual traffic-light dataset capture (ROS Noetic).

In the preview window: 1=red, 2=yellow, 3=green, S=save, Q/Esc=quit.
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

LABELS = {ord('1'): 'red', ord('2'): 'yellow', ord('3'): 'green'}
DEFAULT_OUTPUT = Path(__file__).resolve().parents[3] / 'datasets' / 'traffic_light'


class CameraCapture:
    def __init__(self, topic):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame = None
        self.arrival = None
        self.subscriber = rospy.Subscriber(topic, Image, self.receive, queue_size=1,
                                           buff_size=8 * 1024 * 1024)

    def receive(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except (CvBridgeError, ValueError) as exc:
            rospy.logwarn_throttle(5.0, 'Image decoding failed: %s', exc)
            return
        with self.lock:
            self.frame = frame.copy()
            self.arrival = time.monotonic()

    def latest(self):
        with self.lock:
            return ((None, None) if self.frame is None
                    else (self.frame.copy(), self.arrival))


def main():
    parser = argparse.ArgumentParser(description='Manual traffic-light photo capture')
    parser.add_argument('--image-topic', default='/camera/color/image_raw')
    parser.add_argument('--output-dir', default=None)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('traffic_light_camera_capture', anonymous=True)
    output = (Path(args.output_dir).expanduser().resolve() if args.output_dir
              else DEFAULT_OUTPUT)
    capture = CameraCapture(args.image_topic)
    selected = 'red'
    title = 'Traffic light capture (1/2/3 class, S save, Q quit)'

    print('Camera:', args.image_topic)
    print('Output:', output)
    print('Keys: 1=red, 2=yellow, 3=green, S=save, Q=quit')
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    try:
        while not rospy.is_shutdown():
            frame, arrival = capture.latest()
            if frame is not None:
                preview = frame.copy()
                cv2.putText(preview, 'Class: ' + selected.upper() + ' | S: save',
                            (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (255, 255, 255), 2, cv2.LINE_AA)
                cv2.imshow(title, preview)

            key = cv2.waitKey(30) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            if key in LABELS:
                selected = LABELS[key]
                print('Selected:', selected)
            elif key in (ord('s'), ord('S')):
                if frame is None or arrival is None:
                    print('No image received; not saved.')
                    continue
                if time.monotonic() - arrival > 2.0:
                    print('Camera image is stale; not saved.')
                    continue
                folder = output / selected
                folder.mkdir(parents=True, exist_ok=True)
                filename = datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.jpg'
                path = folder / filename
                if cv2.imwrite(str(path), frame):
                    print('Saved:', path)
                else:
                    rospy.logerr('Unable to save image: %s', path)
    finally:
        capture.subscriber.unregister()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
