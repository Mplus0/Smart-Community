#!/usr/bin/env python3
"""Atomic mission results and limited, source-matched annotated-image evidence."""
from collections import Counter
from datetime import datetime, timezone
import copy
import json
import os
from pathlib import Path
import tempfile
import time
import uuid

import cv2
import numpy as np
import rospy

from perception_client import image_key

COMPLETE = ('SUCCESS', 'EMPTY_VALID')


def atomic_bytes(path, data):
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=str(path.parent), prefix='.' + path.name, delete=False) as stream:
            temporary = stream.name
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, str(path))
        temporary = None
    finally:
        if temporary is not None:
            os.unlink(temporary)


def counts_for(result):
    if result is None or result['status'] not in COMPLETE:
        return None
    counts = Counter(d['label'] for d in result['detections'])
    return {'community_count': counts['community_person'], 'non_community_count': counts['non_community_person']}


def summarize_people(route, tasks, counting):
    points = [p for p in route if p['task'] == 'person']
    latest = {task['waypoint_id']: task for task in tasks if task['task_type'] == 'person'}
    assignments = counting.get('assignments', {})
    confirmed = counting.get('non_overlapping_partitions_confirmed', False)
    by_point = {p['id']: {'area': p.get('area', ''), 'partition': assignments.get(p['id']),
                           'status': latest[p['id']]['status'] if p['id'] in latest else 'NOT_EXECUTED',
                           'counts': counts_for(latest.get(p['id']))} for p in points}

    def group(ids):
        values = [by_point[key]['counts'] for key in ids]
        all_complete = all(v is not None for v in values)
        declared = bool(confirmed and ids and all(assignments.get(key) and by_point[key]['area'] for key in ids))
        duplicate = len(ids) > 1 and not declared
        observed = {name: sum(v[name] for v in values if v is not None)
                    for name in ('community_count', 'non_community_count')}
        return {'waypoint_ids': ids, 'coverage_complete': bool(ids) and all_complete,
                'potential_duplicate': duplicate, 'observed_counts_not_unique': observed,
                'counts': observed if ids and all_complete and not duplicate else None,
                'counting_scope': ('declared_non_overlapping_partitions' if declared else
                                   'single_observation' if len(ids) == 1 else 'unresolved_overlapping_observations')}
    areas = {}
    for p in points:
        areas.setdefault(p.get('area', ''), []).append(p['id'])
    return {'by_waypoint': by_point, 'by_area': {area: group(ids) for area, ids in areas.items()},
            'total': group([p['id'] for p in points]),
            'note': 'Counts cover configured observations only. No cross-view ReID or proof of physical coverage.'}


def validate_counting(counting, route):
    if not isinstance(counting, dict) or set(counting) - {'assignments', 'non_overlapping_partitions_confirmed'}:
        raise ValueError('invalid person counting config')
    if type(counting.get('non_overlapping_partitions_confirmed', False)) is not bool:
        raise ValueError('non_overlapping_partitions_confirmed must be boolean')
    assignments = counting.get('assignments', {})
    if not isinstance(assignments, dict):
        raise ValueError('counting assignments must map waypoint IDs to unique physical partition IDs')
    people = {p['id']: p for p in route if p['task'] == 'person'}
    if any(key not in people or not isinstance(value, str) or not value.strip() for key, value in assignments.items()):
        raise ValueError('invalid counting waypoint or partition ID')
    if len(set(assignments.values())) != len(assignments):
        raise ValueError('one counting partition must have exactly one designated observation waypoint')
    if counting.get('non_overlapping_partitions_confirmed') and (
            set(assignments) != set(people) or any(not p.get('area', '').strip() for p in people.values())):
        raise ValueError('confirmed counting requires an area and unique partition for every person waypoint')


class ResultManager:
    def __init__(self, root, wall=time.monotonic):
        root = Path(root).expanduser()
        if not root.is_absolute():
            raise ValueError('results_root must be absolute, preferably under /workspace/car_2026/results')
        self.wall, self.started = wall, wall()
        run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ_') + uuid.uuid4().hex[:8]
        self.path = root / run_id
        (self.path / 'images').mkdir(parents=True, exist_ok=False)
        self.route, self.counting = [], {}
        self.sequence = 0
        self.data = {'schema_version': 1, 'run_id': run_id, 'status': 'RUNNING',
                     'started_at_utc': datetime.now(timezone.utc).isoformat(),
                     'waypoint_order': [], 'waypoint_executions': [], 'tasks': [], 'pending_task': None,
                     'person_summary': {}, 'plates': [], 'mission_duration_sec': 0.0,
                     'error': None}
        self.save()
        self.event('RUN_STARTED', {'run_id': run_id, 'results_directory': str(self.path)})
        rospy.loginfo('Mission result directory: %s', self.path)

    def set_route(self, route, counting):
        validate_counting(counting, route)
        self.route, self.counting = copy.deepcopy(route), copy.deepcopy(counting)
        self.data['waypoint_order'] = [p['id'] for p in route]
        self.data['counting_policy'] = self.counting
        self.save()

    def event(self, event, payload):
        line = json.dumps({'event': event, 'elapsed_sec': self.wall()-self.started, 'data': payload},
                          ensure_ascii=False, allow_nan=False)
        with (self.path / 'mission.log').open('a', encoding='utf-8') as stream:
            stream.write(line + '\n')
            stream.flush()
            os.fsync(stream.fileno())

    def save(self):
        self.data['mission_duration_sec'] = self.wall() - self.started
        self.data['person_summary'] = summarize_people(self.route, self.data['tasks'], self.counting)
        latest = {task['waypoint_id']: task for task in self.data['tasks'] if task['task_type'] == 'plate'}
        self.data['plates'] = [{'waypoint_id': p['id'], 'area': p.get('area', ''),
                               'task_execution_id': latest[p['id']]['task_execution_id'] if p['id'] in latest else None,
                               'status': latest[p['id']]['status'] if p['id'] in latest else 'NOT_EXECUTED',
                               'recognized_plates': latest[p['id']]['plates'] if p['id'] in latest else []}
                              for p in self.route if p['task'] == 'plate']
        atomic_bytes(self.path / 'mission_results.json',
                     json.dumps(self.data, ensure_ascii=False, allow_nan=False, indent=2).encode('utf-8'))

    def begin_waypoint(self, point, index):
        self.data['waypoint_executions'].append({'index': index, 'waypoint_id': point['id'],
            'task_type': point['task'], 'x': point['x'], 'y': point['y'], 'yaw': point['yaw'],
            'status': 'RUNNING', 'started_elapsed_sec': self.wall()-self.started})
        self.save()

    def end_waypoint(self, status, reason=''):
        if self.data['waypoint_executions'] and self.data['waypoint_executions'][-1]['status'] == 'RUNNING':
            record = self.data['waypoint_executions'][-1]
            record.update(status=status, reason=reason,
                          duration_sec=self.wall()-self.started-record['started_elapsed_sec'])
            self.save()

    def begin_task(self, point, attempt, settings):
        self.sequence += 1
        result = {'task_execution_id': 'task_{:04d}_{}'.format(self.sequence, uuid.uuid4().hex[:8]),
                  'recognition_order': self.sequence, 'run_id': self.data['run_id'], 'waypoint_id': point['id'],
                  'task_type': point['task'], 'area': point.get('area', ''), 'attempt': attempt,
                  'status': 'RUNNING', 'reason': '', 'valid_frames': 0, 'detections': [], 'plates': [],
                  'raw_frames': [], 'evidence': [], 'image_missing': False, 'task_duration_sec': 0.0,
                  'settings': dict(settings)}
        self.data['pending_task'] = copy.deepcopy(result)
        self.save()
        self.event('TASK_STARTED', result)
        return result

    def save_evidence(self, result, header, message, frame_index):
        key = image_key(message, 8388608)
        expected = (header['stamp']['secs']*1000000000 + header['stamp']['nsecs'], header['frame_id'])
        if key != expected:
            raise ValueError('refusing mismatched annotated image')
        pixels = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        pixels = np.ascontiguousarray(pixels[:, :message.width*3].reshape(message.height, message.width, 3))
        ok, encoded = cv2.imencode('.jpg', pixels, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if not ok:
            raise OSError('annotated JPEG encoding failed')
        name = '{}_{}_{}.jpg'.format(result['task_execution_id'], result['task_type'], len(result['evidence'])+1)
        relative = 'images/' + name
        atomic_bytes(self.path / relative, encoded.tobytes())
        result['evidence'].append({'source_header': copy.deepcopy(header), 'frame_index': frame_index,
                                   'image_path': relative, 'annotated_topic_image': True,
                                   'contains_all_raw_frame_detections': True})

    def record_task(self, result):
        # JSON and both logs share this one decision object; no second recognition judgement.
        self.data['tasks'].append(copy.deepcopy(result))
        self.data['pending_task'] = None
        self.save()
        self.event('TASK_RESULT', result)
        logger = rospy.loginfo if result['status'] in COMPLETE else rospy.logwarn
        logger('TASK_RESULT %s', json.dumps(result, ensure_ascii=False, allow_nan=False))

    def finish(self, status, error=None):
        self.data.update(status=status, error=error)
        self.save()
        self.event('MISSION_' + status, {'error': error, 'person_summary': self.data['person_summary'],
                                        'plates': self.data['plates']})
        rospy.loginfo('MISSION_%s person_summary=%s plates=%s', status,
                      json.dumps(self.data['person_summary'], ensure_ascii=False),
                      json.dumps(self.data['plates'], ensure_ascii=False))
