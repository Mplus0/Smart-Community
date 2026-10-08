#!/usr/bin/env python3
"""ROS1 camera lamp states. Image evidence only; no motion commands."""
import json
import math
import threading
import time

import cv2
import numpy as np
import rospy
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image

from traffic_light_detector import Detector, ConfirmStates


def decode(msg):
    n = {'bgr8': 3, 'rgb8': 3, 'mono8': 1, 'rgba8': 4, 'bgra8': 4}.get(msg.encoding)
    if not n or msg.width <= 0 or msg.height <= 0 or msg.step < msg.width*n:
        raise ValueError('Unsupported encoding or invalid dimensions/step')
    raw = np.frombuffer(msg.data, np.uint8)
    if len(raw) != msg.height*msg.step:
        raise ValueError('Image buffer length mismatch')
    image = raw.reshape(msg.height, msg.step)[:, :msg.width*n].reshape(msg.height, msg.width, n)
    codes = {'rgb8': cv2.COLOR_RGB2BGR, 'mono8': cv2.COLOR_GRAY2BGR,
             'rgba8': cv2.COLOR_RGBA2BGR, 'bgra8': cv2.COLOR_BGRA2BGR}
    if msg.encoding == 'mono8':
        image = image[:, :, 0]
    if msg.encoding in codes:
        image = cv2.cvtColor(image, codes[msg.encoding])
    return np.ascontiguousarray(image).copy()


def encode(image, header):
    msg = Image()
    msg.header = header
    msg.height, msg.width = image.shape[:2]
    msg.encoding, msg.step, msg.is_bigendian = 'bgr8', msg.width*3, 0
    msg.data = np.ascontiguousarray(image, dtype=np.uint8).tobytes()
    return msg


def text_panel(image, text, x, y, color=(255, 255, 255), scale=0.6):
    """Draw a high-contrast label; y is the top edge of the background."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    (width, height), baseline = cv2.getTextSize(text, font, scale, 2)
    x = max(0, min(int(x), max(0, image.shape[1]-width-8)))
    y = max(0, min(int(y), max(0, image.shape[0]-height-baseline-8)))
    cv2.rectangle(image, (x, y), (min(image.shape[1]-1, x+width+8),
                                min(image.shape[0]-1, y+height+baseline+8)), (0, 0, 0), -1)
    cv2.putText(image, text, (x+4, y+height+4), font,
                scale, color, 2, cv2.LINE_AA)


def main():
    rospy.init_node('traffic_light_recognition')
    defaults = dict(min_saturation=55, candidate_value=25, bright_value=85,
                    min_lamp_area=10.0, min_color_fraction=0.20,
                    min_dark_fraction=0.12, min_bright_fraction=0.10,
                    min_mean_value=80.0, min_activation_margin=0.04,
                    max_candidates=36, max_groups=4)
    params = {key: rospy.get_param('~'+key, value) for key, value in defaults.items()}
    for key in ('min_saturation', 'candidate_value', 'bright_value'):
        if not 0 <= params[key] <= 255:
            raise ValueError('Invalid byte threshold '+key)
    for key in ('min_color_fraction', 'min_dark_fraction', 'min_bright_fraction', 'min_activation_margin'):
        if not 0 <= params[key] <= 1:
            raise ValueError('Invalid ratio '+key)
    if params['min_lamp_area'] <= 0 or not 0 <= params['min_mean_value'] <= 255:
        raise ValueError('Invalid area/brightness')
    if any(not math.isfinite(float(value)) for value in params.values()):
        raise ValueError('Nonfinite parameter')
    for key in ('max_candidates', 'max_groups'):
        if not isinstance(params[key], int) or params[key] < 1:
            raise ValueError('Invalid count '+key)
    max_rate = float(rospy.get_param('~max_rate', 5.0))
    stale = float(rospy.get_param('~stale_seconds', 1.0))
    confirm_frames = int(rospy.get_param('~green_confirm_frames', 3))
    max_gap = float(rospy.get_param('~track_max_gap', 0.6))
    if not all(math.isfinite(v) and v > 0 for v in (max_rate, stale, max_gap)) or confirm_frames < 1:
        raise ValueError('Invalid rate, timeout or confirmation count')
    topics = [rospy.get_param('~'+key, value) for key, value in
              [('input_topic', '/camera/color/image_raw'),
               ('image_topic', '/perception/traffic_light_image'),
               ('result_topic', '/perception/traffic_light_json')]]
    if len({rospy.resolve_name(topic) for topic in topics}) != 3:
        raise ValueError('Topic names must differ')
    image_pub = rospy.Publisher(topics[1], Image, queue_size=1)
    json_pub = rospy.Publisher(topics[2], String, queue_size=1)
    detector = Detector(params)
    confirmer = ConfirmStates(confirm_frames, max_gap)
    lock, latest = threading.Lock(), [None]

    def receive(msg):
        with lock:
            latest[0] = (msg, time.monotonic())

    subscriber = rospy.Subscriber(topics[0], Image, receive, queue_size=1, buff_size=8*1024*1024)
    last_arrival = time.monotonic()
    last_source, last_stamp = None, None
    next_time = 0.0
    stale_announced = False

    def publish(image, source, groups, frame_valid, reason, elapsed):
        header = source.header if source is not None else Header()
        payload = dict(schema_version=1, engine='opencv_hsv_horizontal_triplet',
                       header=dict(seq=header.seq, stamp=dict(secs=header.stamp.secs,
                                   nsecs=header.stamp.nsecs), frame_id=header.frame_id),
                       width=image.shape[1], height=image.shape[0],
                       frame_valid=frame_valid, reason=reason, processing_ms=elapsed,
                       score_kind='heuristic_not_probability', traffic_lights=groups)
        if not frame_valid:
            text_panel(image, 'UNKNOWN', 8, 8, (0, 200, 255))
        else:
            if not groups:
                text_panel(image, 'UNKNOWN', 8, 8, (0, 200, 255))
            else:
                states = [group['state'] for group in groups]
                summary = ' / '.join(states)
                color = ({'RED': (0, 0, 255), 'YELLOW': (0, 255, 255),
                          'GREEN': (0, 255, 0), 'UNKNOWN': (0, 200, 255)}[states[0]]
                         if len(states) == 1 else (255, 255, 255))
                text_panel(image, summary, 8, 8, color)
        json_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, allow_nan=False)))
        image_pub.publish(encode(image, header))
        rospy.loginfo_throttle(2, 'frame_valid=%s reason=%s states=%s',
                               frame_valid, reason, [g['state'] for g in groups])

    rospy.loginfo('Ready: input=%s outputs=%s,%s max_rate=%.1f; no cmd_vel publisher', *topics, max_rate)
    while not rospy.is_shutdown():
        now = time.monotonic()
        if now < next_time:
            time.sleep(min(0.02, next_time-now))
            continue
        with lock:
            item, latest[0] = latest[0], None
        if now-last_arrival > stale and not stale_announced:
            confirmer.clear()
            shape = (last_source.height, last_source.width, 3) if last_source is not None else (480, 640, 3)
            publish(np.zeros(shape, np.uint8), last_source, [], False, 'camera_stale', 0.0)
            stale_announced = True
        if item is None:
            time.sleep(0.02)
            continue
        source, arrived = item
        stamp = source.header.stamp.to_nsec()
        if now-arrived > stale:
            confirmer.clear()
            continue
        if last_stamp is not None and stamp <= last_stamp:
            # Duplicates/reset clear temporal evidence rather than confirming old green.
            confirmer.clear()
            if stamp == last_stamp:
                continue
        last_stamp, stale_announced, last_arrival = stamp, False, arrived
        next_time = now+1.0/max_rate
        started = time.perf_counter()
        try:
            image = decode(source)
            last_source = source
            groups = confirmer.update(detector.detect(image), now)
            elapsed = (time.perf_counter()-started)*1000
            publish(image, source, groups, True,
                    'ok' if groups else 'no_group', elapsed)
        except Exception as error:
            confirmer.clear()
            rospy.logerr_throttle(5, 'Frame failed: %s', str(error))
            publish(np.zeros((480, 640, 3), np.uint8), source, [], False, 'frame_error', 0.0)
    subscriber.unregister()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
