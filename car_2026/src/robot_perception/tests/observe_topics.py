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
parser.add_argument('--traffic-light', action='store_true', help='Require and observe the third visual stream')
args = parser.parse_args(rospy.myargv()[1:])
output = Path(args.output)
output.mkdir(parents=True, exist_ok=True)
rospy.init_node('p2a_observer', anonymous=True)
lock = threading.RLock()
streams = {name: [] for name in ('camera', 'person', 'plates', 'person_image', 'plates_image')}
if args.traffic_light:
    streams.update({'traffic_light': [], 'traffic_light_image': []})
errors = []
source_stamps = set()
source_dimensions = {}
source_wall_times = {}
images = {}
json_records = []
processes = []
resource_samples = {}
for proc in psutil.process_iter(['pid', 'cmdline']):
    command = proc.info['cmdline'] or []
    if any(Path(arg).name in ('person_detection_node.py', 'plate_recognition_node.py',
                             'traffic_light_classification_node.py') for arg in command):
        proc.cpu_percent()
        processes.append(proc)
        resource_samples[proc.pid] = []


def stamp(header):
    return header.stamp.secs, header.stamp.nsecs


def image_callback(message, key):
    with lock:
        streams[key].append({'wall': time.monotonic(), 'stamp': stamp(message.header)})
        if key == 'camera':
            source_stamps.add(stamp(message.header))
            source_dimensions[stamp(message.header)] = (message.width, message.height)
            source_wall_times[stamp(message.header)] = streams[key][-1]['wall']
        streams[key][-1]['dimensions'] = (message.width, message.height)
        images[key] = message


def json_callback(message, key):
    now = time.monotonic()
    try:
        payload = json.loads(message.data)
        assert payload['schema_version'] == 1
        if key == 'traffic_light':
            assert payload['engine'] == 'yolo11n_traffic_light_classifier'
            assert payload['label'] in ('RED', 'YELLOW', 'GREEN', 'UNKNOWN')
            assert type(payload['frame_valid']) is bool
            assert payload['presence_verified'] is False
            assert 0 <= payload['confidence'] <= 1
            assert np.isfinite(payload['processing_ms']) and payload['processing_ms'] >= 0
            if payload['frame_valid']:
                assert payload['reason'] == 'ok' and payload['label'] != 'UNKNOWN'
                assert payload['roi_xyxy'] is not None
            else:
                assert payload['label'] == 'UNKNOWN' and payload['reason'] != 'ok'
            if payload['roi_xyxy'] is not None:
                x1, y1, x2, y2 = payload['roi_xyxy']
                assert 0 <= x1 < x2 and 0 <= y1 < y2
            objects = []
        else:
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
            header = payload.get('header')
            source_age_ms = None
            if header:
                source_ns = header['stamp']['secs'] * 1000000000 + header['stamp']['nsecs']
                source_age_ms = (rospy.Time.now().to_nsec() - source_ns) / 1000000
            streams[key].append({'wall': now, 'payload': payload, 'source_age_ms': source_age_ms})
            json_records.append({'stream': key, 'payload': payload})
    except Exception as error:
        with lock:
            errors.append(key + ': ' + repr(error))


subscriptions = [rospy.Subscriber(args.camera_topic, Image, image_callback, 'camera', queue_size=1)]
json_topics = [('person', '/perception/person_detections_json'), ('plates', '/perception/plates_json')]
image_topics = [('person_image', '/perception/person_image'), ('plates_image', '/perception/plates_image')]
if args.traffic_light:
    json_topics.append(('traffic_light', '/perception/traffic_light_json'))
    image_topics.append(('traffic_light_image', '/perception/traffic_light_image'))
for key, topic in json_topics:
    subscriptions.append(rospy.Subscriber(topic, String, json_callback, key, queue_size=20))
for key, topic in image_topics:
    subscriptions.append(rospy.Subscriber(topic, Image, image_callback, key, queue_size=1))
started = time.monotonic()
last_resource_sample = started
while time.monotonic() - started < args.seconds and not rospy.is_shutdown():
    if time.monotonic() - last_resource_sample >= 1.0:
        last_resource_sample = time.monotonic()
        for proc in processes:
            try:
                resource_samples[proc.pid].append((proc.cpu_percent(), proc.memory_info().rss / (1024 * 1024)))
            except psutil.Error:
                pass
    time.sleep(0.05)
with lock:
    report = {'duration_wall_seconds': time.monotonic() - started, 'streams': {}, 'errors': errors}
    for key, records in streams.items():
        summary = {'messages': len(records)}
        if len(records) > 1:
            summary['observed_hz'] = (len(records)-1)/(records[-1]['wall']-records[0]['wall'])
        if key in ('person', 'plates', 'traffic_light'):
            payloads = [record['payload'] for record in records]
            if key != 'traffic_light':
                summary['objects_per_message'] = [len(p['detections' if key == 'person' else 'plates']) for p in payloads]
            else:
                summary['labels'] = {label: sum(p['label'] == label for p in payloads)
                                     for label in ('RED', 'YELLOW', 'GREEN', 'UNKNOWN')}
                summary['presence_verified'] = False
            summary['invalid_reasons'] = [p['reason'] for p in payloads if p.get('frame_valid') is False]
            times = [p['processing_ms'] for p in payloads if p.get('frame_valid', True)]
            summary['processing_ms_mean'] = statistics.mean(times) if times else None
            summary['processing_ms_max'] = max(times) if times else None
            summary['processing_ms_p95'] = float(np.percentile(times, 95)) if times else None
            ages = [r['source_age_ms'] for r in records if r['source_age_ms'] is not None]
            summary['source_age_ms_mean'] = statistics.mean(ages) if ages else None
            summary['source_age_ms_p95'] = float(np.percentile(ages, 95)) if ages else None
            observed_delays = []
            for record in records:
                header = record['payload'].get('header')
                if header:
                    key_stamp = (header['stamp']['secs'], header['stamp']['nsecs'])
                    if key_stamp in source_wall_times:
                        delay = (record['wall'] - source_wall_times[key_stamp]) * 1000
                        if delay >= 0:
                            observed_delays.append(delay)
            summary['observer_camera_to_json_ms_mean'] = statistics.mean(observed_delays) if observed_delays else None
            summary['observer_camera_to_json_ms_p95'] = float(np.percentile(observed_delays, 95)) if observed_delays else None
            summary['last_payload'] = payloads[-1] if payloads else None
            summary['source_stamp_matches'] = sum(
                (p['header']['stamp']['secs'], p['header']['stamp']['nsecs']) in source_stamps
                for p in payloads if p.get('header'))
            if key == 'plates':
                summary['texts'] = sorted({item['text'] for p in payloads for item in p['plates']})
        if key.endswith('_image'):
            comparable = [r for r in records if r['stamp'] in source_dimensions]
            summary['source_dimension_matches'] = sum(
                r['dimensions'] == source_dimensions[r['stamp']] for r in comparable)
            summary['comparable_frames'] = len(comparable)
            if any(r['dimensions'] != source_dimensions[r['stamp']] for r in comparable):
                errors.append(key + ': output resolution differs from source')
        report['streams'][key] = summary
    report['processes'] = []
    for proc in processes:
        try:
            samples = resource_samples[proc.pid]
            report['processes'].append({'pid': proc.pid, 'command': proc.cmdline(),
                                        'cpu_percent_mean': statistics.mean(s[0] for s in samples) if samples else None,
                                        'cpu_percent_peak': max((s[0] for s in samples), default=None),
                                        'rss_mib_peak': max((s[1] for s in samples), default=None),
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
if errors or any(not records for records in streams.values()):
    raise SystemExit(1)
