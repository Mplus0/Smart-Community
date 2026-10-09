#!/usr/bin/env python3
"""Bounded traffic-light task consuming the existing classifier JSON only."""
from collections import deque
import json
import threading
import time

import rospy
from std_msgs.msg import String

from perception_client import finite, integer, header_key, _unique_mapping

DEFAULTS = dict(traffic_timeout_sec=35.0, green_confirm_frames=3,
                min_confidence=0.8, max_source_age_sec=1.5, settle_sec=0.5)


def traffic_settings(config):
    if not isinstance(config, dict) or set(config) - set(DEFAULTS):
        raise ValueError('invalid traffic_light settings')
    settings = dict(DEFAULTS, **config)
    for key in ('traffic_timeout_sec', 'max_source_age_sec'):
        finite(settings[key], key, 0.001)
    finite(settings['settle_sec'], 'settle_sec', 0)
    finite(settings['min_confidence'], 'min_confidence', 0, 1)
    integer(settings['green_confirm_frames'], 'green_confirm_frames', 1)
    if settings['settle_sec'] >= settings['traffic_timeout_sec']:
        raise ValueError('settle_sec must be less than traffic_timeout_sec')
    return settings


def parse_frame(raw):
    if not isinstance(raw, str) or len(raw.encode('utf-8')) > 16384:
        raise ValueError('oversized traffic JSON')
    def invalid_constant(value):
        raise ValueError('nonfinite JSON: ' + value)
    p = json.loads(raw, object_pairs_hook=_unique_mapping, parse_constant=invalid_constant)
    if not isinstance(p, dict) or type(p.get('schema_version')) is not int or p['schema_version'] != 1:
        raise ValueError('invalid traffic schema')
    if p.get('engine') != 'yolo11n_traffic_light_classifier':
        raise ValueError('unexpected traffic engine')
    if (p.get('score_kind') != 'conditional_class_probability' or
            p.get('presence_verified') is not False):
        raise ValueError('invalid classifier metadata')
    finite(p.get('confidence'), 'confidence', 0, 1)
    finite(p.get('processing_ms'), 'processing_ms', 0)
    label = p.get('label')
    if type(p.get('frame_valid')) is not bool:
        raise ValueError('invalid frame_valid')
    # Low-confidence UNKNOWN is normal. Camera/decode/inference errors are faults.
    if not ((p['frame_valid'] and label in ('RED', 'YELLOW', 'GREEN') and p.get('reason') == 'ok') or
            (not p['frame_valid'] and label == 'UNKNOWN' and p.get('reason') == 'low_confidence')):
        raise ValueError('traffic source fault: ' + str(p.get('reason')))
    stamp, source = header_key(p.get('header'))
    roi = p.get('roi_xyxy')
    if not isinstance(roi, list) or len(roi) != 4:
        raise ValueError('invalid traffic ROI')
    for v in roi:
        integer(v, 'ROI coordinate')
    if not (roi[0] < roi[2] and roi[1] < roi[3]):
        raise ValueError('empty traffic ROI')
    return stamp, source, p


class TrafficWait:
    def __init__(self, settings, results, check_running, stop,
                 topic='/perception/traffic_light_json', subscribe=True,
                 wall=time.monotonic, ros_ns=None, sleep=time.sleep):
        self.settings = traffic_settings(settings)
        self.results, self.check_running, self.stop = results, check_running, stop
        self.wall, self.sleep = wall, sleep
        self.ros_ns = ros_ns or (lambda: rospy.Time.now().to_nsec())
        self.lock = threading.RLock()
        self.queue = deque()
        self.accepting = False
        self.overflow = False
        self.highwater = 0  # Retained across waypoints, never reuse a consumed source.
        self.subscriber = (rospy.Subscriber(topic, String, self.receive, queue_size=128,
                                          buff_size=2097152) if subscribe else None)

    def receive(self, message):
        with self.lock:
            if not self.accepting:
                return
            if len(self.queue) >= 128:
                self.overflow = True
                return
            self.queue.append((self.wall(), message.data))

    def execute(self, point):
        started = self.wall()
        deadline = started + self.settings['traffic_timeout_sec']
        result = self.results.begin_task(point, 1, self.settings)
        result.update(released=False, green_streak=0, wait_duration_sec=0.0,
                      last_label='UNKNOWN', rejected_frames=0)
        source = None
        last_ros = self.ros_ns()
        latest_stamp, latest_arrival = 0, 0
        try:
            # Timeout includes the settling interval. No pre-settle frames are cached.
            while self.wall() - started < self.settings['settle_sec']:
                self.check_running()
                self.stop()
                if self.wall() >= deadline:
                    raise RuntimeError('TRAFFIC_TIMEOUT')
                self.sleep(min(0.05, max(0, deadline - self.wall())))
            with self.lock:
                self.queue.clear()
                self.overflow = False
                window_ns = self.ros_ns()
                if window_ns <= 0 or window_ns < last_ros:
                    raise RuntimeError('invalid traffic window clock')
                last_ros = window_ns
                self.accepting = True
            while True:
                self.check_running()
                self.stop()
                now, ros_now = self.wall(), self.ros_ns()
                if now >= deadline:
                    raise RuntimeError('TRAFFIC_TIMEOUT')
                if ros_now < last_ros:
                    raise RuntimeError('traffic clock moved backwards')
                last_ros = ros_now
                with self.lock:
                    if self.overflow:
                        raise RuntimeError('traffic input queue overflow')
                    frames = list(self.queue)
                    self.queue.clear()
                now, ros_now = self.wall(), self.ros_ns()
                if ros_now < last_ros:
                    raise RuntimeError('traffic clock moved backwards')
                for arrival, raw in frames:
                    try:
                        stamp, frame_id, payload = parse_frame(raw)
                        # In-flight inference captured before the new window is expected.
                        # Discard it, without turning a normal window boundary into a fault.
                        if stamp <= window_ns:
                            result['rejected_frames'] += 1
                            result['green_streak'] = 0
                            self.results.event('TRAFFIC_REJECTED', {'waypoint_id': point['id'],
                                'reason': 'previous_window', 'wait_duration_sec': now-started})
                            continue
                        if stamp <= self.highwater:
                            raise ValueError('duplicate or reversed traffic frame')
                        if (stamp > ros_now or ros_now - stamp > self.settings['max_source_age_sec'] * 1e9 or
                                now - arrival > self.settings['max_source_age_sec']):
                            raise ValueError('stale or future traffic frame')
                        identity = (frame_id, tuple(payload['roi_xyxy']))
                        if source is not None and source != identity:
                            raise ValueError('traffic source or ROI changed')
                    except Exception as exc:
                        result['rejected_frames'] += 1
                        result['green_streak'] = 0
                        self.results.event('TRAFFIC_REJECTED', {'waypoint_id': point['id'],
                                           'reason': str(exc), 'wait_duration_sec': now-started})
                        raise
                    source = identity
                    self.highwater = stamp
                    # A gap in source or receipt time breaks consecutive confirmation.
                    if latest_stamp and (stamp-latest_stamp > self.settings['max_source_age_sec']*1e9 or
                                         arrival-latest_arrival > self.settings['max_source_age_sec']):
                        result['green_streak'] = 0
                    latest_stamp, latest_arrival = stamp, arrival
                    green = (payload['frame_valid'] and payload['label'] == 'GREEN' and
                             payload['confidence'] >= self.settings['min_confidence'])
                    result['green_streak'] = result['green_streak'] + 1 if green else 0
                    result['valid_frames'] += int(payload['frame_valid'])
                    result['last_label'] = payload['label']
                    result['last_recognition'] = payload
                    observation = dict(payload=payload, wait_duration_sec=now-started,
                                       green_streak=result['green_streak'], released=False)
                    # Full observations in mission.log; bounded recent evidence in JSON.
                    result['raw_frames'].append(observation)
                    del result['raw_frames'][:-128]
                    self.results.event('TRAFFIC_OBSERVATION', dict(waypoint_id=point['id'], **observation))
                with self.lock:
                    # Account for everything already received before committing release.
                    if (not self.queue and not self.overflow and
                            result['green_streak'] >= self.settings['green_confirm_frames'] and
                            self.wall() < deadline and
                            0 <= self.ros_ns()-latest_stamp <= self.settings['max_source_age_sec']*1e9 and
                            self.wall()-latest_arrival <= self.settings['max_source_age_sec']):
                        self.accepting = False
                        result.update(status='SUCCESS', reason='confirmed consecutive green frames', released=True)
                        break
                self.sleep(0.05)
        except Exception as exc:
            result.update(status='ERROR', reason=str(exc), released=False)
            raise
        finally:
            with self.lock:
                self.accepting = False
                self.queue.clear()
            result['wait_duration_sec'] = self.wall() - started
            result['task_duration_sec'] = result['wait_duration_sec']
            self.stop()
            self.results.record_task(result)

    def close(self):
        if self.subscriber is not None:
            self.subscriber.unregister()
