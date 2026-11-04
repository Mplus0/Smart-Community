#!/usr/bin/env python3
"""Bounded ROS result/image buffers. No inference dependencies or motion commands."""
from collections import Counter, OrderedDict
from dataclasses import dataclass, field
import json
import math
import threading
import time

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

LABELS = {0: ('community_person', 'C'), 1: ('non_community_person', 'NC')}
TOPICS = {
    'person_json': '/perception/person_detections_json',
    'person_image': '/perception/person_image',
    'plate_json': '/perception/plates_json',
    'plate_image': '/perception/plates_image',
}


class InvalidFrame(ValueError):
    pass


class WindowClockError(RuntimeError):
    pass


def integer(value, name, minimum=0):
    if type(value) is not int or value < minimum:
        raise InvalidFrame(name + ' must be an integer >= ' + str(minimum))
    return value


def finite(value, name, minimum=0, maximum=float('inf')):
    if type(value) not in (int, float) or not math.isfinite(value) or not minimum <= value <= maximum:
        raise InvalidFrame(name + ' outside finite range')
    return float(value)


def header_key(header):
    if not isinstance(header, dict) or not isinstance(header.get('stamp'), dict):
        raise InvalidFrame('missing source header/stamp')
    integer(header.get('seq'), 'header.seq')
    secs = integer(header['stamp'].get('secs'), 'stamp.secs')
    nsecs = integer(header['stamp'].get('nsecs'), 'stamp.nsecs')
    frame = header.get('frame_id')
    if nsecs >= 1000000000 or secs * 1000000000 + nsecs == 0:
        raise InvalidFrame('invalid or zero source stamp')
    if not isinstance(frame, str) or not frame.strip() or len(frame) > 256:
        raise InvalidFrame('invalid frame_id')
    return secs * 1000000000 + nsecs, frame


def image_key(message, max_bytes):
    key = header_key({'seq': message.header.seq, 'stamp': {
        'secs': message.header.stamp.secs, 'nsecs': message.header.stamp.nsecs},
        'frame_id': message.header.frame_id})
    if (message.encoding != 'bgr8' or not 0 < message.width <= 8192 or
            not 0 < message.height <= 8192 or message.step < message.width * 3 or
            len(message.data) != message.height * message.step or len(message.data) > max_bytes):
        raise InvalidFrame('invalid bgr8 annotated image dimensions/step/data size')
    return key


def _unique_mapping(pairs):
    output = {}
    for key, value in pairs:
        if key in output:
            raise InvalidFrame('duplicate JSON field: ' + key)
        output[key] = value
    return output


def validate_json(raw, kind, max_bytes=262144):
    if not isinstance(raw, str) or len(raw.encode('utf-8')) > max_bytes:
        raise InvalidFrame('JSON exceeds byte limit')
    payload = json.loads(raw, object_pairs_hook=_unique_mapping,
                         parse_constant=lambda value: (_ for _ in ()).throw(InvalidFrame(value)))
    if not isinstance(payload, dict) or type(payload.get('schema_version')) is not int or payload['schema_version'] != 1:
        raise InvalidFrame('expected schema_version=1')
    if kind == 'person':
        if type(payload.get('frame_valid')) is not bool or not isinstance(payload.get('reason'), str):
            raise InvalidFrame('missing person validity/reason')
        if not payload['frame_valid']:
            raise InvalidFrame('person frame invalid: ' + payload['reason'])
        if payload['reason'] != 'ok':
            raise InvalidFrame('valid person frame must have reason=ok')
    key = header_key(payload.get('header'))
    width = integer(payload.get('width'), 'width', 1)
    height = integer(payload.get('height'), 'height', 1)
    if width > 8192 or height > 8192:
        raise InvalidFrame('image dimensions too large')
    finite(payload.get('processing_ms'), 'processing_ms')
    expected_engine = 'yolo26_person_detector' if kind == 'person' else 'hyperlpr3'
    if payload.get('engine') != expected_engine:
        raise InvalidFrame('unexpected engine')
    objects = payload.get('detections' if kind == 'person' else 'plates')
    if not isinstance(objects, list) or len(objects) > 100:
        raise InvalidFrame('expected at most 100 objects per frame')
    for item in objects:
        if not isinstance(item, dict):
            raise InvalidFrame('object must be a mapping')
        finite(item.get('confidence'), 'confidence', 0, 1)
        box = item.get('bbox_xyxy')
        if not isinstance(box, list) or len(box) != 4:
            raise InvalidFrame('bbox must contain four pixel coordinates')
        x1, y1, x2, y2 = [finite(v, 'bbox coordinate') for v in box]
        if not (x1 < x2 <= width and y1 < y2 <= height):
            raise InvalidFrame('invalid bbox extent')
        if kind == 'person':
            class_id = integer(item.get('class_id'), 'class_id')
            if class_id not in LABELS or (item.get('label'), item.get('display_label')) != LABELS[class_id]:
                raise InvalidFrame('person C/NC mapping mismatch')
        else:
            text = item.get('text')
            if not isinstance(text, str) or not text.strip() or len(text) > 64:
                raise InvalidFrame('invalid plate text')
            # HyperLPR uses -1 for UNKNOWN; keep the producer's type metadata.
            integer(item.get('plate_type_id'), 'plate_type_id', -1)
    return key, payload


@dataclass
class Window:
    kind: str
    start_ros_ns: int
    start_wall: float
    epoch: int
    max_source_age_sec: float
    baseline: Counter
    frame_id: str = ''
    dimensions: tuple = ()
    last_ns: int = 0
    rejected: Counter = field(default_factory=Counter)


class PerceptionClient:
    def __init__(self, topics=None, max_cache_frames=24, max_cache_age_sec=10.0,
                 max_cache_bytes=67108864, max_image_bytes=8388608,
                 subscribe=True, wall=time.monotonic, ros_ns=None):
        self.wall = wall
        self.ros_ns = ros_ns or (lambda: rospy.Time.now().to_nsec())
        self.max_frames = integer(max_cache_frames, 'max_cache_frames', 1)
        self.max_age = finite(max_cache_age_sec, 'max_cache_age_sec', 0.001)
        self.max_bytes = integer(max_cache_bytes, 'max_cache_bytes', 1)
        self.max_image_bytes = integer(max_image_bytes, 'max_image_bytes', 1)
        if self.max_image_bytes > self.max_bytes:
            raise ValueError('max_image_bytes cannot exceed max_cache_bytes')
        self.lock = threading.RLock()
        self.jsons = {kind: OrderedDict() for kind in ('person', 'plate')}
        self.images = {kind: OrderedDict() for kind in ('person', 'plate')}
        self.errors = {kind: Counter() for kind in ('person', 'plate')}
        self.highwater = {'person': 0, 'plate': 0}
        self.last_clock = 0
        self.epoch = 0
        self.subscribers = []
        if subscribe:
            selected = dict(TOPICS, **(topics or {}))
            if len(set(rospy.resolve_name(v) for v in selected.values())) != 4:
                raise ValueError('four perception topics must be distinct')
            for kind in ('person', 'plate'):
                self.subscribers.append(rospy.Subscriber(selected[kind + '_json'], String,
                    lambda msg, k=kind: self.receive_json(k, msg.data), queue_size=1, buff_size=524288))
                self.subscribers.append(rospy.Subscriber(selected[kind + '_image'], Image,
                    lambda msg, k=kind: self.receive_image(k, msg), queue_size=1,
                    buff_size=self.max_image_bytes + 65536))

    def _clock(self, now):
        if now < self.last_clock:
            self.epoch += 1
            for store in (self.jsons, self.images):
                for cache in store.values():
                    cache.clear()
            self.highwater = {'person': 0, 'plate': 0}
        self.last_clock = now

    def _prune(self, now):
        for store in (self.jsons, self.images):
            for cache in store.values():
                while cache and (len(cache) > self.max_frames or now - next(iter(cache.values()))[0] > self.max_age):
                    cache.popitem(last=False)
        # Images are the dominant allocation; additionally enforce a global byte ceiling.
        while sum(v[2] for cache in self.images.values() for v in cache.values()) > self.max_bytes:
            candidates = [(next(iter(cache.values()))[0], kind) for kind, cache in self.images.items() if cache]
            self.images[min(candidates)[1]].popitem(last=False)

    def _invalid(self, kind, reason, category):
        with self.lock:
            self.errors[kind][category] += 1
        rospy.logwarn_throttle(5.0, 'Perception %s rejected: %s', kind, reason)

    def receive_json(self, kind, raw):
        arrived = self.wall()
        try:
            key, payload = validate_json(raw, kind)
            with self.lock:
                now = self.ros_ns()
                self._clock(now)
                self._prune(arrived)
                if now <= 0 or key[0] > now:
                    self.errors[kind]['future_or_zero_clock'] += 1
                    return False
                if key[0] <= self.highwater[kind]:
                    self.errors[kind]['duplicate_or_reversed'] += 1
                    return False
                self.highwater[kind] = key[0]
                self.jsons[kind][key] = (arrived, payload)
                self._prune(arrived)
            return True
        except Exception as error:
            self._invalid(kind, str(error), 'invalid_json')
            return False

    def receive_image(self, kind, message):
        arrived = self.wall()
        try:
            key = image_key(message, self.max_image_bytes)
            with self.lock:
                self._clock(self.ros_ns())
                self._prune(arrived)
                if key not in self.images[kind]:
                    self.images[kind][key] = (arrived, message, len(message.data))
                self._prune(arrived)
            return True
        except Exception as error:
            self._invalid(kind, str(error), 'invalid_image')
            return False

    def begin_window(self, kind, max_source_age_sec):
        with self.lock:
            now = self.ros_ns()
            self._clock(now)
            if now <= 0:
                raise WindowClockError('cannot open task window with zero ROS time')
            # Establish a receipt barrier even if two windows share the same ROS time.
            # Keep the high watermark so replaying a consumed stamp remains invalid.
            self.jsons[kind].clear()
            self.images[kind].clear()
            return Window(kind, now, self.wall(), self.epoch, max_source_age_sec, self.errors[kind].copy())

    def drain(self, window):
        with self.lock:
            now, wall = self.ros_ns(), self.wall()
            self._clock(now)
            if self.epoch != window.epoch:
                raise WindowClockError('ROS clock reset during perception task')
            self._prune(wall)
            frames = []
            for key, (arrival, payload) in self.jsons[window.kind].items():
                # A monotonic watermark avoids retaining every old cache key in the window.
                if key[0] <= window.last_ns:
                    continue
                window.last_ns = key[0]
                if (arrival < window.start_wall or key[0] < window.start_ros_ns or
                        now - key[0] > int(window.max_source_age_sec * 1e9) or
                        wall - arrival > window.max_source_age_sec):
                    window.rejected['stale_or_previous_window'] += 1
                    continue
                dims = (payload['width'], payload['height'])
                if window.frame_id and (window.frame_id != key[1] or window.dimensions != dims):
                    window.rejected['source_changed'] += 1
                    continue
                window.frame_id, window.dimensions = key[1], dims
                frames.append({'key': key, 'arrival_wall': arrival, 'payload': payload})
            return frames

    def matching_image(self, window, frame):
        with self.lock:
            self._prune(self.wall())
            item = self.images[window.kind].get(frame['key'])
            if item is None or item[0] < window.start_wall:
                return None
            message = item[1]
            payload = frame['payload']
            if message.width != payload['width'] or message.height != payload['height']:
                return None
            return message

    def diagnostics(self, window):
        with self.lock:
            output = dict(self.errors[window.kind] - window.baseline)
            output.update(window.rejected)
            return output

    def close(self):
        for sub in self.subscribers:
            sub.unregister()
        self.subscribers.clear()
