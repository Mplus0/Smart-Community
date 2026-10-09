#!/usr/bin/env python3
"""Docker-only deterministic task tests; no navigation or model execution."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from traffic_wait import TrafficWait, traffic_settings, parse_frame
from result_manager import ResultManager


def frame(stamp, label='GREEN', confidence=0.8, **changes):
    ns = int(round(stamp * 1e9))
    p = dict(schema_version=1, engine='yolo11n_traffic_light_classifier',
             header=dict(seq=1, stamp=dict(secs=ns//10**9, nsecs=ns % 10**9), frame_id='camera'),
             frame_valid=label != 'UNKNOWN', reason='low_confidence' if label == 'UNKNOWN' else 'ok',
             roi_xyxy=[0, 0, 640, 240], label=label, confidence=confidence, processing_ms=2.0,
             score_kind='conditional_class_probability', presence_verified=False)
    p.update(changes)
    return json.dumps(p)


class Clock:
    def __init__(self):
        self.now = 100.0
        self.events = []
        self.client = None

    def sleep(self, seconds):
        self.now += seconds
        while self.events and self.events[0][0] <= self.now + 1e-8:
            _, raw = self.events.pop(0)
            self.client.receive(SimpleNamespace(data=raw))


class Records:
    def __init__(self):
        self.tasks, self.events = [], []

    def begin_task(self, point, attempt, settings):
        return dict(waypoint_id=point['id'], raw_frames=[], valid_frames=0)

    def event(self, name, payload):
        self.events.append((name, copy.deepcopy(payload)))

    def record_task(self, result):
        self.tasks.append(copy.deepcopy(result))


class TrafficTests(unittest.TestCase):
    def setUp(self):
        self.clock, self.records = Clock(), Records()
        self.stops = 0
        def stop():
            self.stops += 1
        self.client = TrafficWait({}, self.records, lambda: None, stop, subscribe=False,
                                  wall=lambda: self.clock.now,
                                  ros_ns=lambda: int(round(self.clock.now * 1e9)), sleep=self.clock.sleep)
        self.clock.client = self.client
        self.point = dict(id='light_01', task='traffic_light')

    def schedule(self, labels):
        for i, label in enumerate(labels):
            t = self.clock.now + 0.7 + i*0.1
            self.clock.events.append((t+0.01, frame(t, label)))

    def test_green_releases_at_exact_confidence_threshold(self):
        self.schedule(['GREEN']*3)
        self.client.execute(self.point)
        r = self.records.tasks[-1]
        self.assertTrue(r['released'])
        self.assertEqual(r['green_streak'], 3)
        self.assertLess(r['wait_duration_sec'], 1.1)
        self.assertGreater(self.stops, 3)

    def test_red_yellow_unknown_and_low_confidence_break_streak(self):
        for label in ('RED', 'YELLOW', 'UNKNOWN', 'LOW'):
            with self.subTest(label=label):
                self.setUp()
                self.schedule(['GREEN', 'GREEN', label if label != 'LOW' else 'GREEN', 'GREEN', 'GREEN', 'GREEN'])
                if label == 'LOW':
                    t, _ = self.clock.events[2]
                    self.clock.events[2] = (t, frame(t-0.01, confidence=0.799))
                self.client.execute(self.point)
                self.assertEqual(len(self.records.tasks[-1]['raw_frames']), 6)
                self.assertTrue(self.records.tasks[-1]['released'])

    def test_default_35_second_timeout_no_data_or_wait_colors(self):
        for label in (None, 'RED', 'YELLOW', 'UNKNOWN'):
            with self.subTest(label=label):
                self.setUp()
                if label:
                    self.schedule([label]*100)
                with self.assertRaisesRegex(RuntimeError, 'TRAFFIC_TIMEOUT'):
                    self.client.execute(self.point)
                r = self.records.tasks[-1]
                self.assertFalse(r['released'])
                self.assertEqual(r['status'], 'ERROR')
                self.assertGreaterEqual(r['wait_duration_sec'], 35)
                self.assertLess(r['wait_duration_sec'], 35.06)

    def test_invalid_stale_future_duplicate_and_source_faults_fail_closed(self):
        cases = ['{', '[]', frame(99), frame(110), frame(100.7, confidence=float('nan')),
                 frame(100.7, confidence=True), frame(100.7, frame_valid=False),
                 frame(100.7, label='UNKNOWN', frame_valid=False, reason='camera_stale'),
                 frame(100.7, header=None), frame(100.7, schema_version=True),
                 frame(100.7, engine='other'), frame(100.7).replace('"seq": 1', '"seq": 1, "seq": 2')]
        for raw in cases:
            with self.subTest(raw=raw):
                self.setUp()
                self.clock.events = [(100.75, raw)]
                with self.assertRaises((ValueError, RuntimeError)):
                    self.client.execute(self.point)
                self.assertFalse(self.records.tasks[-1]['released'])
                self.assertEqual(self.records.tasks[-1]['rejected_frames'], 1)
        for raw in (frame(100.7), frame(100.6), frame(100.8, roi_xyxy=[1, 0, 640, 240])):
            self.setUp()
            self.clock.events = [(100.75, frame(100.7)), (100.85, raw)]
            with self.assertRaises(ValueError):
                self.client.execute(self.point)
            self.assertFalse(self.records.tasks[-1]['released'])

    def test_pre_window_and_previous_waypoint_cannot_release(self):
        self.client.receive(SimpleNamespace(data=frame(100)))  # Outside task: ignored.
        self.clock.events = [(100.1, frame(100.1)), (100.2, frame(100.2)), (100.3, frame(100.3))]
        self.schedule(['GREEN']*3)
        self.client.execute(self.point)
        self.assertEqual(len(self.records.tasks[-1]['raw_frames']), 3)
        self.clock.events = [(self.clock.now+0.8, frame(100.9))]
        with self.assertRaisesRegex(RuntimeError, 'TIMEOUT'):
            self.client.execute(dict(id='light_02', task='traffic_light'))
        self.assertFalse(self.records.tasks[-1]['released'])

    def test_delayed_pre_window_frame_is_discarded_then_new_green_releases(self):
        self.clock.events = [(100.65, frame(100.4))]
        self.schedule(['GREEN']*3)
        self.client.execute(self.point)
        self.assertTrue(self.records.tasks[-1]['released'])
        self.assertEqual(self.records.tasks[-1]['rejected_frames'], 1)

    def test_expired_current_window_frame_and_deadline_green(self):
        self.clock.events = [(103, frame(100.7))]
        with self.assertRaisesRegex(ValueError, 'stale'):
            self.client.execute(self.point)
        self.setUp()
        self.clock.events = [(135, frame(134.9+i*0.01)) for i in range(3)]
        with self.assertRaisesRegex(RuntimeError, 'TIMEOUT'):
            self.client.execute(self.point)
        self.assertFalse(self.records.tasks[-1]['released'])

    def test_later_red_in_same_batch_prevents_release(self):
        self.clock.events = [(101, frame(100.7+i*0.01, label))
                             for i, label in enumerate(['GREEN']*3+['RED'])]
        with self.assertRaisesRegex(RuntimeError, 'TIMEOUT'):
            self.client.execute(self.point)
        self.assertEqual(self.records.tasks[-1]['green_streak'], 0)

    def test_gap_breaks_confirmation(self):
        self.clock.events = [(100.75, frame(100.7)), (100.85, frame(100.8)), (103, frame(102.99))]
        with self.assertRaisesRegex(RuntimeError, 'TIMEOUT'):
            self.client.execute(self.point)
        self.assertEqual(self.records.tasks[-1]['green_streak'], 1)

    def test_clock_reset_shutdown_and_overflow_fail_closed(self):
        self.client.ros_ns = lambda: int((self.clock.now if self.clock.now < 101 else 99)*1e9)
        with self.assertRaisesRegex(RuntimeError, 'backwards'):
            self.client.execute(self.point)
        self.setUp()
        def shutdown():
            if self.clock.now >= 101:
                raise RuntimeError('shutdown')
        self.client.check_running = shutdown
        with self.assertRaisesRegex(RuntimeError, 'shutdown'):
            self.client.execute(self.point)
        self.setUp()
        self.clock.events = [(101, frame(100.7+i*0.001)) for i in range(129)]
        with self.assertRaisesRegex(RuntimeError, 'overflow'):
            self.client.execute(self.point)
        self.assertFalse(self.records.tasks[-1]['released'])

    def test_persistent_json_and_log(self):
        with tempfile.TemporaryDirectory() as directory:
            records = ResultManager(directory, wall=lambda: self.clock.now)
            records.set_route([self.point], {})
            self.client.results = records
            self.schedule(['RED', 'GREEN', 'GREEN', 'GREEN'])
            self.client.execute(self.point)
            data = json.loads((records.path/'mission_results.json').read_text())
            result = data['tasks'][-1]
            self.assertTrue(result['released'])
            self.assertEqual(result['last_recognition']['label'], 'GREEN')
            self.assertGreater(result['wait_duration_sec'], 0.5)
            log = [json.loads(line) for line in (records.path/'mission.log').read_text().splitlines()]
            self.assertEqual([e['data'] for e in log if e['event'] == 'TASK_RESULT'][-1], result)

    def test_settings_reject_invalid_values(self):
        for overrides in ({'green_confirm_frames': True}, {'green_confirm_frames': 0},
                          {'traffic_timeout_sec': 0}, {'settle_sec': 35},
                          {'min_confidence': float('nan')}, {'max_source_age_sec': -1}):
            with self.assertRaises(ValueError):
                traffic_settings(overrides)


if __name__ == '__main__':
    unittest.main(verbosity=2)
