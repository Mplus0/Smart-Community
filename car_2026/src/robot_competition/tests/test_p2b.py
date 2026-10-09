#!/usr/bin/env python3
"""Synthetic interface/mission tests only; run on the isolated test ROS master."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String

from perception_client import PerceptionClient, WindowClockError, validate_json
from result_manager import ResultManager, atomic_bytes, summarize_people, validate_counting
from task_processor import TaskProcessor, fuse, task_settings
import test_p1 as p1
from test_p1 import point


def detection(class_id=0, box=None, confidence=0.9):
    return {'class_id': class_id, 'label': ('community_person', 'non_community_person')[class_id],
            'display_label': ('C', 'NC')[class_id], 'confidence': confidence,
            'bbox_xyxy': box or [2, 2, 16, 30]}


def plate(text='京A12345', box=None, confidence=0.9):
    return {'text': text, 'confidence': confidence, 'plate_type_id': 0, 'bbox_xyxy': box or [2, 2, 30, 20]}


def payload(ns, kind='person', objects=None, frame='camera', valid=True):
    data = {'schema_version': 1, 'engine': 'yolo26_person_detector' if kind == 'person' else 'hyperlpr3',
            'header': {'seq': 7, 'stamp': {'secs': ns//1000000000, 'nsecs': ns%1000000000}, 'frame_id': frame},
            'width': 64, 'height': 48, 'processing_ms': 20.0}
    data['detections' if kind == 'person' else 'plates'] = objects if objects is not None else [detection() if kind == 'person' else plate()]
    if kind == 'person':
        data.update(frame_valid=valid, reason='ok' if valid else 'camera_stale')
    return data


def image_for(data, seq=999):
    h = data['header']
    return Image(header=Header(seq=seq, stamp=rospy.Time(h['stamp']['secs'], h['stamp']['nsecs']), frame_id=h['frame_id']),
                 height=data['height'], width=data['width'], encoding='bgr8', is_bigendian=0,
                 step=data['width']*3+4, data=bytes([80])*((data['width']*3+4)*data['height']))


class FakeTime:
    def __init__(self):
        self.value, self.offset, self.events = 100.0, 0, []

    def wall(self):
        return self.value

    def ros(self):
        return int(round((self.value-90)*1e9)) + self.offset

    def sleep(self, seconds):
        self.value += seconds
        for at, callback in list(self.events):
            if self.value + 1e-8 >= at:
                self.events.remove((at, callback))
                callback()


class ClientTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeTime()
        self.client = PerceptionClient(subscribe=False, wall=self.clock.wall, ros_ns=self.clock.ros)

    def send(self, data):
        return self.client.receive_json('person', json.dumps(data))

    def test_exact_nanoseconds_previous_window_and_duplicates(self):
        start = self.clock.ros()
        self.assertTrue(self.send(payload(start-1)))
        window = self.client.begin_window('person', 2)
        self.assertEqual(self.client.drain(window), [])
        self.assertTrue(self.send(payload(start)))
        self.assertFalse(self.send(payload(start)))
        self.assertEqual(len(self.client.drain(window)), 1)
        self.assertEqual(self.client.drain(window), [])
        next_window = self.client.begin_window('person', 2)
        # Previously consumed stamps cannot be delivered into a new task, even at equal ROS time.
        self.assertFalse(self.send(payload(start)))
        self.assertEqual(self.client.drain(next_window), [])

    def test_image_match_stamp_frame_not_seq(self):
        window = self.client.begin_window('person', 2)
        data = payload(self.clock.ros())
        self.send(data)
        frame = self.client.drain(window)[0]
        wrong = payload(self.clock.ros()-1)
        self.client.receive_image('person', image_for(wrong))
        self.assertIsNone(self.client.matching_image(window, frame))
        wrong = payload(self.clock.ros(), frame='other')
        self.client.receive_image('person', image_for(wrong))
        self.assertIsNone(self.client.matching_image(window, frame))
        self.client.receive_image('person', image_for(data, seq=999))
        self.assertEqual(self.client.matching_image(window, frame).header.seq, 999)

    def test_invalid_json_mapping_coordinates_and_nan(self):
        base = payload(self.clock.ros())
        invalid = []
        for change in ({'schema_version': True}, {'frame_valid': False}, {'width': '64'}):
            candidate = copy.deepcopy(base)
            candidate.update(change)
            invalid.append(candidate)
        for change in ({'confidence': float('nan')}, {'confidence': True}, {'bbox_xyxy': [20, 0, 1, 2]},
                       {'label': 'unknown'}, {'class_id': False}):
            candidate = copy.deepcopy(base)
            candidate['detections'][0].update(change)
            invalid.append(candidate)
        for candidate in invalid:
            self.assertFalse(self.send(candidate))
        self.assertFalse(self.client.receive_json('person', '{"schema_version":1,"schema_version":1}'))
        self.assertEqual(self.client.jsons['person'], {})

    def test_hyperlpr_unknown_type_is_valid_metadata(self):
        data = payload(self.clock.ros(), 'plate')
        data['plates'][0]['plate_type_id'] = -1
        _, accepted = validate_json(json.dumps(data), 'plate')
        self.assertEqual(accepted['plates'][0]['plate_type_id'], -1)

    def test_reversed_future_stale_and_clock_reset(self):
        window = self.client.begin_window('person', 0.1)
        self.assertFalse(self.send(payload(self.clock.ros()+1)))
        self.assertTrue(self.send(payload(self.clock.ros())))
        self.assertFalse(self.send(payload(self.clock.ros()-1)))
        self.clock.sleep(0.2)
        self.assertEqual(self.client.drain(window), [])
        self.clock.offset = -1000000000
        with self.assertRaises(WindowClockError):
            self.client.drain(window)

    def test_cache_frame_age_and_byte_caps(self):
        client = PerceptionClient(subscribe=False, wall=self.clock.wall, ros_ns=self.clock.ros,
                                  max_cache_frames=2, max_cache_age_sec=0.2, max_cache_bytes=20000,
                                  max_image_bytes=10000)
        for _ in range(5):
            self.clock.sleep(0.02)
            data = payload(self.clock.ros())
            client.receive_json('person', json.dumps(data))
            client.receive_image('person', image_for(data))
            client.receive_image('plate', image_for(data))
        self.assertLessEqual(len(client.jsons['person']), 2)
        self.assertLessEqual(sum(v[2] for cache in client.images.values() for v in cache.values()), 20000)
        self.clock.sleep(0.3)
        window = client.begin_window('person', 1)
        self.assertEqual(client.drain(window), [])
        self.assertFalse(client.images['person'])

    def test_malformed_image_and_source_change(self):
        window = self.client.begin_window('person', 2)
        data = payload(self.clock.ros())
        self.send(data)
        self.assertEqual(len(self.client.drain(window)), 1)
        for change in ({'encoding': 'rgb8'}, {'step': 1}, {'data': b'x'}, {'height': 0}):
            image = image_for(data)
            for key, value in change.items():
                setattr(image, key, value)
            self.assertFalse(self.client.receive_image('person', image))
        self.clock.sleep(0.02)
        self.send(payload(self.clock.ros(), frame='different_camera'))
        self.assertEqual(self.client.drain(window), [])
        self.assertEqual(self.client.diagnostics(window)['source_changed'], 1)


class TaskTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.clock = FakeTime()
        self.client = PerceptionClient(subscribe=False, wall=self.clock.wall, ros_ns=self.clock.ros)
        self.results = ResultManager(self.tmp.name, wall=self.clock.wall)
        self.config = {'defaults': {'settle_sec': 0.0, 'task_timeout_sec': 0.18,
            'observation_sec': 0.06, 'min_valid_frames': 3, 'min_support_frames': 2,
            'image_match_timeout_sec': 0.04, 'task_retry_count': 0,
            'plate_text_annotation_confirmed': True}}
        self.check = lambda: None

    def tearDown(self):
        self.tmp.cleanup()

    def schedule(self, kind='person', objects=None, valid=True, image=True, times=(0.02, 0.04, 0.06), repeated=False):
        stamp = [None]
        for index, delta in enumerate(times):
            def emit(i=index):
                current_objects = objects[i] if objects is not None else None
                stamp[0] = stamp[0] if repeated and stamp[0] else self.clock.ros()
                data = payload(stamp[0], kind, current_objects, valid=valid)
                self.client.receive_json(kind, json.dumps(data))
                if image:
                    self.client.receive_image(kind, image_for(data))
            self.clock.events.append((100.0+delta, emit))

    def run_task(self, kind='person'):
        p = point('observation', kind, area='block_01')
        self.results.set_route([p], {})
        processor = TaskProcessor(self.client, self.results, self.check, self.config, self.clock.sleep)
        return processor.execute(p)

    def test_person_repeated_box_and_classes(self):
        objects = [[detection(), detection(1, [40, 2, 55, 30])] for _ in range(3)]
        self.schedule(objects=objects)
        result = self.run_task()
        self.assertEqual(result['status'], 'SUCCESS')
        self.assertEqual(len(result['detections']), 2)
        self.assertEqual(result['valid_frames'], 3)
        self.assertTrue(result['evidence'])
        for evidence in result['evidence']:
            image = cv2.imread(str(self.results.path/evidence['image_path']))
            self.assertEqual(image.shape, (48, 64, 3))
            self.assertEqual(evidence['source_header'], result['raw_frames'][evidence['frame_index']]['header'])
        data = json.loads((self.results.path/'mission_results.json').read_text())
        self.assertEqual(data['person_summary']['total']['counts'], {'community_count': 1, 'non_community_count': 1})
        self.assertIn('TASK_RESULT', (self.results.path/'mission.log').read_text())

    def test_valid_empty(self):
        self.schedule(objects=[[], [], []])
        result = self.run_task()
        self.assertEqual(result['status'], 'EMPTY_VALID')
        self.assertEqual(self.results.data['person_summary']['total']['counts']['community_count'], 0)

    def test_invalid_is_not_zero(self):
        self.schedule(valid=False)
        result = self.run_task()
        self.assertEqual(result['status'], 'FAILED_INVALID')
        self.assertIsNone(self.results.data['person_summary']['total']['counts'])

    def test_silence_timeout_and_repeated_frames(self):
        result = self.run_task()
        self.assertEqual(result['status'], 'FAILED_TIMEOUT')
        self.assertEqual(result['valid_frames'], 0)
        self.schedule(times=(0.20, 0.22, 0.24), repeated=True)
        result = self.run_task()
        self.assertEqual(result['status'], 'FAILED_TIMEOUT')
        self.assertLessEqual(result['valid_frames'], 1)

    def test_image_missing_fails(self):
        self.schedule(image=False)
        result = self.run_task()
        self.assertEqual(result['status'], 'FAILED_IMAGE_MISSING')
        self.assertEqual(result['decision_status'], 'SUCCESS')
        self.assertFalse(result['evidence'])

    def test_late_image_and_representative_fallback(self):
        objects = [[detection(box=[2+i, 2, 16+i, 30])] for i in range(3)]
        self.schedule(objects=objects, image=False)
        def late_image():
            data = payload(10020000000, objects=objects[0])
            self.client.receive_image('person', image_for(data))
        self.clock.events.append((100.10, late_image))
        result = self.run_task()
        self.assertEqual(result['status'], 'SUCCESS')
        target = result['detections'][0]
        self.assertEqual(target['representative_frame_index'], 0)
        self.assertEqual(target['bbox_xyxy'], objects[0][0]['bbox_xyxy'])
        self.assertEqual(target['evidence_image_path'], result['evidence'][0]['image_path'])

    def test_settle_discards_pre_window_messages(self):
        self.config['defaults']['settle_sec'] = 0.1
        self.schedule()
        result = self.run_task()
        self.assertEqual(result['status'], 'FAILED_TIMEOUT')
        self.assertEqual(result['valid_frames'], 0)

    def test_invalid_settings_fail_before_task(self):
        for overrides in ({'min_valid_frames': True}, {'max_valid_frames': 129}, {'vote_fraction': float('nan')},
                          {'task_timeout_sec': 0.001}, {'unknown': 1}):
            with self.assertRaises(ValueError):
                task_settings({'defaults': overrides}, 'person')

    def test_image_encoding_failure_preserves_previous_result(self):
        self.schedule()
        first = self.run_task()
        self.schedule(times=(0.12, 0.14, 0.16))
        with patch('result_manager.cv2.imencode', return_value=(False, None)):
            with self.assertRaises(OSError):
                self.run_task()
        data = json.loads((self.results.path/'mission_results.json').read_text())
        self.assertEqual(data['tasks'][0]['task_execution_id'], first['task_execution_id'])
        self.assertEqual(data['tasks'][-1]['status'], 'FAILED_INTERRUPTED')

    def test_chinese_plate_vote_and_raw_text(self):
        self.schedule('plate', [[plate(' 京a12345 ')], [plate('京A12345')], [plate('京A12345')]])
        result = self.run_task('plate')
        self.assertEqual(result['status'], 'SUCCESS')
        self.assertEqual(result['plates'][0]['text'], '京A12345')
        self.assertEqual(result['plates'][0]['support_frames'], 3)
        self.assertEqual(result['raw_frames'][0]['plates'][0]['text'], ' 京a12345 ')

    def test_plate_multiple_vehicles_ambiguous(self):
        self.schedule('plate', [[plate(), plate('粤B67890', [40, 2, 60, 20])] for _ in range(3)])
        result = self.run_task('plate')
        self.assertEqual(result['status'], 'AMBIGUOUS')
        self.assertFalse(result['plates'])

    def test_plate_conflicting_strings_ambiguous(self):
        self.schedule('plate', [[plate('京A12345')], [plate('京A12346')], [plate('京A12347')]])
        self.assertEqual(self.run_task('plate')['status'], 'AMBIGUOUS')

    def test_low_confidence_not_empty(self):
        self.schedule('plate', [[plate(confidence=0.1)] for _ in range(3)])
        self.assertEqual(self.run_task('plate')['status'], 'FAILED_UNSTABLE')

    def test_plate_empty_and_dropout_distinct(self):
        self.schedule('plate', [[], [], []])
        self.assertEqual(self.run_task('plate')['status'], 'EMPTY_VALID')
        self.assertEqual(self.run_task('plate')['status'], 'FAILED_TIMEOUT')

    def test_chinese_annotation_confirmation_required(self):
        self.config['defaults']['plate_text_annotation_confirmed'] = False
        self.schedule('plate')
        self.assertEqual(self.run_task('plate')['status'], 'FAILED_IMAGE_ANNOTATION')

    def test_retry_new_execution_and_no_old_evidence(self):
        self.config['defaults']['task_retry_count'] = 1
        self.schedule(times=(0.22, 0.24, 0.26))
        result = self.run_task()
        self.assertEqual(result['status'], 'SUCCESS')
        self.assertEqual(len(self.results.data['tasks']), 2)
        self.assertEqual(self.results.data['tasks'][0]['status'], 'FAILED_TIMEOUT')
        self.assertNotEqual(self.results.data['tasks'][0]['task_execution_id'], result['task_execution_id'])

    def test_shutdown_interrupt_persisted(self):
        def check():
            if self.clock.wall() > 100.03:
                raise RuntimeError('ROS shutdown')
        self.check = check
        with self.assertRaisesRegex(RuntimeError, 'shutdown'):
            self.run_task()
        data = json.loads((self.results.path/'mission_results.json').read_text())
        self.assertEqual(data['tasks'][0]['status'], 'FAILED_INTERRUPTED')
        self.assertIsNone(data['pending_task'])

    def test_clock_reset_interrupt_persisted(self):
        self.clock.events.append((100.04, lambda: setattr(self.clock, 'offset', -1000000000)))
        with self.assertRaises(WindowClockError):
            self.run_task()
        self.assertEqual(self.results.data['tasks'][0]['status'], 'FAILED_INTERRUPTED')

    def test_atomic_snapshot_unique_runs_and_pending_task(self):
        second = ResultManager(self.tmp.name)
        self.assertNotEqual(second.path, self.results.path)
        pending = self.results.begin_task(point('pending', 'person'), 1, {})
        data = json.loads((self.results.path/'mission_results.json').read_text())
        self.assertEqual(data['pending_task']['task_execution_id'], pending['task_execution_id'])
        original = (self.results.path/'mission_results.json').read_bytes()
        with patch('result_manager.os.replace', side_effect=OSError('disk failure')):
            with self.assertRaises(OSError):
                atomic_bytes(self.results.path/'mission_results.json', b'broken')
        self.assertEqual((self.results.path/'mission_results.json').read_bytes(), original)


class CountingTests(unittest.TestCase):
    def test_overlap_failure_and_declared_partitions(self):
        points = [point('a', 'person', area='block'), point('b', 'person', area='block')]
        tasks = [{'waypoint_id': p['id'], 'task_type': 'person', 'status': 'SUCCESS',
                  'detections': [detection()]} for p in points]
        summary = summarize_people(points, tasks, {})
        self.assertTrue(summary['total']['potential_duplicate'])
        self.assertIsNone(summary['total']['counts'])
        self.assertEqual(summary['total']['observed_counts_not_unique']['community_count'], 2)
        policy = {'non_overlapping_partitions_confirmed': True, 'assignments': {'a': 'left', 'b': 'right'}}
        validate_counting(policy, points)
        self.assertEqual(summarize_people(points, tasks, policy)['total']['counts']['community_count'], 2)
        tasks[-1]['status'] = 'FAILED_TIMEOUT'
        self.assertIsNone(summarize_people(points, tasks, policy)['total']['counts'])
        policy['assignments']['b'] = 'left'
        with self.assertRaises(ValueError):
            validate_counting(policy, points)


class MissionIntegrationTests(unittest.TestCase):
    setUpClass = classmethod(p1.RosTests.setUpClass.__func__)
    execute = classmethod(p1.RosTests.execute.__func__)
    setUp = p1.RosTests.setUp
    tearDown = p1.RosTests.tearDown
    run_route = p1.RosTests.run_route

    def test_failed_task_continues_next_goal_and_finishes(self):
        self.assertEqual(self.run_route([point('p', 'person'), point('next')]), 0)
        self.assertEqual(len(self.goals), 2)
        result = json.loads((self.controller.results.path/'mission_results.json').read_text())
        self.assertEqual(result['status'], 'FINISH')
        self.assertEqual(result['tasks'][0]['status'], 'FAILED_TIMEOUT')

    def test_real_ros_synthetic_publishers_success_and_image_correlation(self):
        rospy.set_param('~task_config', {'defaults': {'settle_sec': 0.0, 'task_timeout_sec': 3.0,
            'observation_sec': 0.15, 'min_valid_frames': 3, 'min_support_frames': 2,
            'task_retry_count': 0, 'image_match_timeout_sec': 0.5, 'plate_text_annotation_confirmed': True}})
        publishers = {}
        for kind, jt, it in [('person', 'person_detections_json', 'person_image'), ('plate', 'plates_json', 'plates_image')]:
            publishers[kind] = (rospy.Publisher('/perception/'+jt, String, queue_size=1),
                                rospy.Publisher('/perception/'+it, Image, queue_size=1))
        stop = threading.Event()
        def publish():
            while not stop.is_set():
                for kind, (jp, ip) in publishers.items():
                    data = payload(rospy.Time.now().to_nsec(), kind)
                    # Publish image first to cover the asynchronous order as well.
                    ip.publish(image_for(data))
                    jp.publish(String(data=json.dumps(data, ensure_ascii=False)))
                time.sleep(0.05)
        thread = threading.Thread(target=publish)
        thread.start()
        try:
            self.assertEqual(self.run_route([point('p', 'person'), point('car', 'plate'), point('next')]), 0)
            tasks = self.controller.results.data['tasks']
            self.assertEqual([t['status'] for t in tasks], ['SUCCESS', 'SUCCESS'])
            self.assertEqual(tasks[1]['plates'][0]['text'], '京A12345')
            self.assertEqual(len(self.goals), 3)
        finally:
            stop.set()
            thread.join(2)
            for pair in publishers.values():
                for pub in pair:
                    pub.unregister()

    def test_navigation_error_and_traffic_guard(self):
        type(self).behavior = ['abort', 'abort']
        self.assertEqual(self.run_route([point(), point('after')]), 1)
        self.assertEqual(self.controller.results.data['status'], 'ERROR')
        self.assertFalse(self.controller.results.data['tasks'])
        self.assertEqual(self.run_route([point('light', 'traffic_light', stop_before_line=True), point('after')]), 1)
        self.assertIn('TRAFFIC_TIMEOUT', self.controller.results.data['error'])
        self.assertFalse(self.controller.results.data['tasks'][-1]['released'])

    def test_task_paused_clock_stops_next_navigation(self):
        rospy.set_param('~task_config', {'defaults': {'task_timeout_sec': 3.0, 'observation_sec': 0.1,
                        'settle_sec': 0.0, 'task_retry_count': 0}})
        original = p1.Controller.task
        def paused(controller, point):
            controller.use_sim_time = True
            controller.clock_timeout = 0.08
            controller.clock_changed = time.monotonic()
            controller.last_clock = 10.0
            with patch('main_controller.rospy.Time.now', return_value=rospy.Time(10)):
                original(controller, point)
        with patch.object(p1.Controller, 'task', paused):
            self.assertEqual(self.run_route([point('p', 'person'), point('next')]), 1)
        self.assertEqual(len(self.goals), 1)
        self.assertIn('paused', self.controller.results.data['error'])
        self.assertEqual(self.controller.results.data['tasks'][0]['status'], 'FAILED_INTERRUPTED')

    def test_task_shutdown_callback_stops_next_navigation(self):
        original = p1.Controller.task
        def stopped(controller, point):
            controller.shutdown()
            original(controller, point)
        with patch.object(p1.Controller, 'task', stopped):
            self.assertEqual(self.run_route([point('p', 'person'), point('next')]), 1)
        self.assertEqual(len(self.goals), 1)
        self.assertEqual(self.controller.results.data['status'], 'ERROR')
        self.assertEqual(self.controller.results.data['tasks'][0]['status'], 'FAILED_INTERRUPTED')


if __name__ == '__main__':
    rospy.init_node('p1_test', disable_signals=True)
    try:
        unittest.main(verbosity=2)
    finally:
        rospy.signal_shutdown('P2-B synthetic tests finished')
