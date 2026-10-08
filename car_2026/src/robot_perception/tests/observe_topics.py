#!/usr/bin/env python3
"""Read-only wall-time observation of camera and real inference topics; no motion."""
import argparse
import json
from pathlib import Path
import statistics
import threading
import time

import cv2
import numpy as np
import psutil
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--seconds', type=float, default=30)
parser.add_argument('--output', default='/tmp/p2a-observation')
parser.add_argument('--camera-topic', default='/camera/color/image_raw')
args = parser.parse_args(rospy.myargv()[1:])
output = Path(args.output)
output.mkdir(parents=True, exist_ok=True)
rospy.init_node('p2a_observer', anonymous=True)
lock = threading.RLock()
streams = {name: [] for name in ('camera', 'person', 'plates', 'person_image', 'plates_image')}
errors = []
source_stamps = set()
images = {}
json_records = []
processes = []
for proc in psutil.process_iter(['pid', 'cmdline']):
    command = proc.info['cmdline'] or []
    if any(Path(arg).name in ('person_detection_node.py', 'plate_recognition_node.py') for arg in command):
        proc.cpu_percent()
        processes.append(proc)


def stamp(header):
    return header.stamp.secs, header.stamp.nsecs


def image_callback(message, key):
    with lock:
        streams[key].append({'wall': time.monotonic(), 'stamp': stamp(message.header)})
        if key == 'camera':
            source_stamps.add(stamp(message.header))
        images[key] = message


def json_callback(message, key):
    now = time.monotonic()
    try:
        payload = json.loads(message.data)
        assert payload['schema_version'] == 1
        objects = payload['detections' if key == 'person' else 'plates']
        for item in objects:
            assert 0 <= item['confidence'] <= 1
            x1, y1, x2, y2 = item['bbox_xyxy']
            assert 0 <= x1 < x2 <= payload['width']
            assert 0 <= y1 < y2 <= payload['height']
            if key == 'person':
                assert (item['class_id'], item['label'], item['display_label']) in (
                    (0, 'community_person', 'C'), (1, 'non_community_person', 'NC'))
            else:
                assert isinstance(item['text'], str)
        with lock:
            streams[key].append({'wall': now, 'payload': payload})
            json_records.append({'stream': key, 'payload': payload})
    except Exception as error:
        with lock:
            errors.append(key + ': ' + repr(error))


subscriptions = [rospy.Subscriber(args.camera_topic, Image, image_callback, 'camera', queue_size=1)]
for key, topic in [('person', '/perception/person_detections_json'), ('plates', '/perception/plates_json')]:
    subscriptions.append(rospy.Subscriber(topic, String, json_callback, key, queue_size=20))
for key, topic in [('person_image', '/perception/person_image'), ('plates_image', '/perception/plates_image')]:
    subscriptions.append(rospy.Subscriber(topic, Image, image_callback, key, queue_size=1))
started = time.monotonic()
while time.monotonic() - started < args.seconds and not rospy.is_shutdown():
    time.sleep(0.05)
with lock:
    report = {'duration_wall_seconds': time.monotonic() - started, 'streams': {}, 'errors': errors}
    for key, records in streams.items():
        summary = {'messages': len(records)}
        if len(records) > 1:
            summary['observed_hz'] = (len(records)-1)/(records[-1]['wall']-records[0]['wall'])
        if key in ('person', 'plates'):
            payloads = [record['payload'] for record in records]
            summary['objects_per_message'] = [len(p['detections' if key == 'person' else 'plates']) for p in payloads]
            summary['invalid_reasons'] = [p['reason'] for p in payloads if p.get('frame_valid') is False]
            times = [p['processing_ms'] for p in payloads if p.get('frame_valid', True)]
            summary['processing_ms_mean'] = statistics.mean(times) if times else None
            summary['processing_ms_max'] = max(times) if times else None
            summary['last_payload'] = payloads[-1] if payloads else None
            summary['source_stamp_matches'] = sum(
                (p['header']['stamp']['secs'], p['header']['stamp']['nsecs']) in source_stamps
                for p in payloads if p.get('header'))
            if key == 'plates':
                summary['texts'] = sorted({item['text'] for p in payloads for item in p['plates']})
        report['streams'][key] = summary
    report['processes'] = []
    for proc in processes:
        try:
            report['processes'].append({'pid': proc.pid, 'cpu_percent': proc.cpu_percent(),
                                        'rss_mib': proc.memory_info().rss/(1024*1024)})
        except psutil.Error:
            pass
    for key, message in images.items():
        count = {'rgb8': 3, 'bgr8': 3}.get(message.encoding)
        if count:
            data = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
            pixels = data[:, :message.width*count].reshape(message.height, message.width, count)
            if message.encoding == 'rgb8':
                pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
            cv2.imwrite(str(output / (key + '.png')), pixels)
    (output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    (output / 'messages.json').write_text(json.dumps(json_records, ensure_ascii=False, indent=2))
print(json.dumps(report, ensure_ascii=False, indent=2))
for subscription in subscriptions:
    subscription.unregister()
rospy.signal_shutdown('observation complete')
if errors or not streams['camera'] or not streams['person'] or not streams['plates']:
    raise SystemExit(1)
