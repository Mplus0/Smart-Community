#!/usr/bin/env python3
"""Check route selection and navigation completion without ROS or robot hardware."""
import ast
import copy
import math
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace, MethodType
import unittest
from unittest.mock import patch
import tempfile
import threading
import time
from contextlib import nullcontext

import yaml

WORKSPACE = Path(__file__).resolve().parents[1]
SCRIPTS = WORKSPACE / 'src/community_mission/scripts'
sys.path.insert(0, str(SCRIPTS))
from rules import validate_route, obstacle_route, Detection, detection_records, ocr_detection_records
from result_manager import ResultManager


class MissionRouteTests(unittest.TestCase):
    def setUp(self):
        self.route = yaml.safe_load((SCRIPTS.parent / 'config/route.yaml').read_text(encoding='utf-8'))

    def method(self, name, filename='mission_node.py', class_name='Mission', **scope):
        tree = ast.parse((SCRIPTS / filename).read_text(encoding='utf-8'))
        mission = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == class_name)
        method = next(node for node in mission.body if isinstance(node, ast.FunctionDef) and node.name == name)
        namespace = dict(rospy=SimpleNamespace(loginfo=lambda *args: None, sleep=lambda duration: None),
                         detection_records=detection_records, ocr_detection_records=ocr_detection_records)
        namespace.update(scope)
        exec(compile(ast.Module(body=[method], type_ignores=[]), filename, 'exec'), namespace)
        return namespace[name]

    def test_new_clear_route_tasks_and_parking_photo(self):
        route = yaml.safe_load((SCRIPTS.parent / 'config/route_clear.yaml').read_text())
        validate_route(route)
        self.assertNotIn('navigation_branches', route)
        self.assertEqual([p['id'] for p in route['points']],
                         [f'wp_{i:02d}' for i in range(1, 40) if i != 36])
        expected = {
            'traffic_light': ['wp_01', 'wp_24'],
            'person': ['wp_02', 'wp_06', 'wp_07', 'wp_14', 'wp_17', 'wp_18'],
            'fire': ['wp_19', 'wp_22', 'wp_34'],
            'plate': ['wp_27', 'wp_28', 'wp_31'], 'trash': ['wp_29'],
            'vehicle_photo': ['wp_35'], 'record_start': [], 'record_stop': [], 'obstacle': [],
            'temperature': ['wp_08'], 'meter': ['wp_11']}
        for task, ids in expected.items():
            self.assertEqual([p['id'] for p in route['points'] if p['task'] == task], ids)
        self.assertEqual([p['area'] for p in route['points'] if p['task'] == 'person'],
                         ['A', 'A', 'B', 'B', 'B', 'A'])
        self.assertEqual([p['parking_lot'] for p in route['points'] if p['task'] == 'plate'], [1, 2, 3])
        self.assertEqual(route['points'][34]['y'], 2.3729)
        self.assertNotIn('wp_36', [p['id'] for p in route['points']])

    def test_new_clear_route_completes_without_branching(self):
        route = yaml.safe_load((SCRIPTS.parent / 'config/route_clear.yaml').read_text())
        for mode in ('clear', 'blocked'):
            transitions, visited = [], []
            obj = SimpleNamespace(route=copy.deepcopy(route), index=0, single_waypoint='',
                                  obstacle_mode=mode,
                                  transition=lambda state, reason: transitions.append(state))
            for _ in route['points']:
                visited.append(obj.route['points'][obj.index]['id'])
                self.method('next_point')(obj)
            self.assertEqual(visited, [p['id'] for p in route['points']])
            self.assertEqual(transitions[-1], 'FINISH')

    def test_blocked_route_shares_clear_tail_and_preserves_recorded_tasks(self):
        route = yaml.safe_load((SCRIPTS.parent / 'config/route_blocked.yaml').read_text())
        clear = yaml.safe_load((SCRIPTS.parent / 'config/route_clear.yaml').read_text())
        validate_route(route)
        self.assertEqual(len(route['points']), 37)
        start = next(i for i, p in enumerate(route['points'])
                     if p['task'] == 'plate' and p['parking_lot'] == 1)
        common = next(i for i, p in enumerate(clear['points'])
                      if p['task'] == 'plate' and p['parking_lot'] == 1)
        self.assertEqual(route['points'][start:], clear['points'][common:])
        by_id = {p['id']: p for p in route['points']}
        self.assertEqual(by_id['blocked_wp_01']['task'], 'traffic_light')
        self.assertEqual(by_id['blocked_wp_13']['task'], 'fire')
        self.assertEqual(by_id['blocked_wp_13']['yaw'], 0.0001)
        self.assertEqual(by_id['blocked_wp_15']['task'], 'traffic_light')
        self.assertEqual(by_id['blocked_wp_15']['yaw'], 1.7333)
        self.assertEqual(by_id['blocked_wp_22']['task'], 'temperature')
        self.assertEqual(by_id['blocked_wp_25']['task'], 'meter')
        self.assertEqual(by_id['blocked_wp_08']['area'], 'B')
        self.assertEqual(by_id['blocked_wp_08']['yaw'], 1.6255)
        self.assertEqual(by_id['blocked_wp_14']['area'], 'B')
        self.assertEqual(by_id['blocked_wp_14']['yaw'], -0.0514)
        self.assertEqual(by_id['blocked_wp_10']['task'], 'fire')
        self.assertEqual(by_id['blocked_wp_10']['x'], 2.4101)
        self.assertEqual([p['id'] for p in route['points'] if p['task'] == 'traffic_light'],
                         ['blocked_wp_01', 'blocked_wp_15'])
        obj = SimpleNamespace(route=copy.deepcopy(route), index=0, single_waypoint='',
                              transition=lambda state, reason: events.append(state))
        events = []
        for _ in route['points']:
            self.method('next_point')(obj)
        self.assertEqual(events[-1], 'FINISH')
        self.assertEqual(obj.route['points'][-1]['id'], 'wp_39')

    def test_obstacle_detection_selects_main_state_machine_branch(self):
        for box, expected in [((5, 0, 15, 10), 'clear'), ((40, 0, 60, 10), 'blocked')]:
            with self.subTest(expected=expected):
                obj = SimpleNamespace(
                    route=copy.deepcopy(self.route), index=4, single_waypoint='',
                    obstacle_mode='clear' if expected == 'blocked' else 'blocked',
                    services=SimpleNamespace(detect=lambda detector, image: [Detection('obstacle', box, .9)]),
                    check_running=lambda: None, obstacle_clear_x_ratio=.35,
                    announce=lambda text: None,
                    transition=lambda *args: None)
                select = self.method('select_obstacle_route', detections=lambda result: result,
                                     obstacle_route=obstacle_route)
                select(obj, SimpleNamespace(width=100))
                self.assertEqual(obj.obstacle_mode, expected)
                self.method('next_point')(obj)
                self.assertEqual(obj.route['points'][5]['id'],
                                 'wp_06' if expected == 'clear' else 'blocked_wp_06')

    def test_navigation_only_skips_obstacle_service_and_uses_manual_branch(self):
        for mode in ('clear', 'blocked'):
            states = []
            obj = SimpleNamespace(route=copy.deepcopy(self.route), index=4,
                                  navigation_only=True, single_waypoint='', obstacle_mode=mode,
                                  check_running=lambda: None,
                                  results=SimpleNamespace(waypoint=lambda *args: None),
                                  transition=lambda state, reason: states.append(state))
            self.method('task')(obj)
            self.assertEqual(states[-1], 'NEXT_POINT')
            self.method('next_point')(obj)
            self.assertEqual(obj.index, 5)
            self.assertEqual(obj.route['points'][5]['id'],
                             'wp_06' if mode == 'clear' else 'blocked_wp_06')

    def test_branches_cover_confirmed_common_tasks(self):
        from collections import Counter
        by_id = {p['id']: p for p in self.route['points']}
        coverage = []
        for mode, order in self.route['navigation_branches'].items():
            selected = self.route['points'][:5] + [by_id[name] for name in order]
            tasks = Counter((p['task'], p.get('area'), p.get('parking_lot'))
                            for p in selected if p['task'] != 'navigation')
            self.assertEqual(sum(v for (kind, _, _), v in tasks.items() if kind == 'traffic_light'), 2)
            self.assertEqual(sum(v for (kind, _, _), v in tasks.items() if kind == 'fire'), 3)
            self.assertEqual(tasks[('temperature', None, None)], 1)
            self.assertEqual(tasks[('meter', None, None)], 1)
            coverage.append(tasks)
        self.assertEqual(coverage[0], coverage[1])
        clear = yaml.safe_load((SCRIPTS.parent / 'config/route_clear.yaml').read_text())['points']
        blocked = yaml.safe_load((SCRIPTS.parent / 'config/route_blocked.yaml').read_text())['points']
        def tasks(points):
            return Counter((p['task'], p.get('area'), p.get('parking_lot'))
                           for p in points if p['task'] != 'navigation')
        self.assertEqual(tasks(clear), tasks(blocked))

    def test_camera_assignments_match_both_route_requests(self):
        primary = {'clear': {1, 5, 19, 24, 27, 28, 29, 31, 35, 36},
                   'blocked': {1, 5, 8, 10, 14, 15, 27, 28, 29, 31, 35, 36}}
        by_id = {p['id']: p for p in self.route['points']}
        for mode in ('clear', 'blocked'):
            source = yaml.safe_load((SCRIPTS.parent / f'config/route_{mode}.yaml').read_text())
            validate_route(source)
            for point in source['points']:
                if point['task'] in ('navigation', 'temperature'):
                    self.assertNotIn('camera', point)
                    continue
                expected = 'main' if int(point['id'].rsplit('_', 1)[1]) in primary[mode] else 'meter'
                self.assertEqual(point['camera'], expected, point['id'])
            for name in self.route['navigation_branches'][mode]:
                point = by_id[name]
                original = next(p for p in source['points'] if p['id'] == name)
                self.assertEqual(point.get('camera'), original.get('camera'), name)
        self.assertEqual(by_id['wp_02']['camera'], 'meter')
        self.assertEqual(by_id['wp_05']['camera'], 'main')
        self.assertEqual(by_id['wp_34']['camera'], 'meter')

    def test_camera_switch_discards_previous_source_frames(self):
        import threading
        subscriptions, stopped = [], []
        main, meter = '/main/image', '/meter/image'
        obj = SimpleNamespace(condition=threading.Condition(), active_image_topic=main,
                              latest='old frame', sequence=8, check_running=lambda: None,
                              subscriber=SimpleNamespace(unregister=lambda: stopped.append(main)),
                              point_image_topic=lambda point: meter if point['camera'] == 'meter' else main)
        callback = self.method('image_callback')
        obj.image_callback = lambda image, topic: callback(obj, image, topic)
        def subscribe(topic, image_type, function, **kwargs):
            subscriptions.append((topic, kwargs['callback_args']))
            return SimpleNamespace(unregister=lambda: stopped.append(topic))
        select = self.method('select_camera', Image=object)
        select.__globals__['rospy'].Subscriber = subscribe
        select(obj, {'id': 'wp_02', 'camera': 'meter'})
        self.assertEqual(subscriptions, [(meter, meter)])
        self.assertIsNone(obj.latest)
        self.assertEqual(obj.sequence, 0)
        callback(obj, 'late main frame', main)
        self.assertIsNone(obj.latest)
        callback(obj, 'new meter frame', meter)
        self.assertEqual(obj.latest, 'new meter frame')
        self.assertEqual(obj.sequence, 1)
        select(obj, {'id': 'wp_06', 'camera': 'meter'})
        self.assertEqual(len(subscriptions), 1)
        select(obj, {'id': 'wp_19', 'camera': 'main'})
        self.assertIsNone(obj.latest)
        callback(obj, 'late meter frame', meter)
        self.assertIsNone(obj.latest)
        callback(obj, 'new main frame', main)
        self.assertEqual(obj.latest, 'new main frame')
        self.assertEqual(stopped, [main, meter])

    def test_camera_topics_use_launch_overrides(self):
        obj = SimpleNamespace(image_topic='/custom/main', meter_image_topic='/custom/meter')
        resolve = self.method('point_image_topic')
        self.assertEqual(resolve(obj, {'camera': 'main'}), '/custom/main')
        self.assertEqual(resolve(obj, {'camera': 'meter'}), '/custom/meter')
        self.assertEqual(resolve(obj, {}), '/custom/main')
        self.assertEqual(resolve(obj, {}, default='meter'), '/custom/meter')

    def test_task_selects_camera_before_acquiring_recognition_frame(self):
        calls = []
        image = object()
        point = {'id': 'wp_06', 'task': 'person', 'area': 'A', 'camera': 'meter'}
        obj = SimpleNamespace(
            route={'points': [point]}, index=0, navigation_only=False,
            check_running=lambda: None, data_dir=Path('/unused'),
            select_camera=lambda selected: calls.append(('camera', selected['camera'])),
            fresh_image=lambda: calls.append(('frame', image)) or image,
            bridge=SimpleNamespace(imgmsg_to_cv2=lambda frame, **kwargs: frame),
            run_inspection=lambda point, kind, frame, path: calls.append(('recognition', frame)),
            results=SimpleNamespace(waypoint=lambda *args: None), transition=lambda *args: None)
        task = self.method('task', cv2=SimpleNamespace(imwrite=lambda *args: True))
        task.__globals__['rospy'].sleep = lambda duration: calls.append(('wait', duration))
        task(obj)
        self.assertEqual(calls, [('camera', 'meter'), ('wait', 1.0), ('frame', image), ('recognition', image)])

    def test_thermal_task_does_not_switch_visible_light_camera(self):
        calls = []
        def fail_switch(point):
            self.fail('Thermal task must use its measurement service')
        obj = SimpleNamespace(
            route={'points': [{'id': 'wp_08', 'task': 'temperature'}]}, index=0,
            navigation_only=False, check_running=lambda: None, select_camera=fail_switch,
            measure_temperature=lambda *a: {'announcement': ''}, data_dir=Path('/unused'),
                               read_meter=lambda *a: {'temperature_c': 20, 'humidity_percent': 50, 'electricity_kwh': '1'},
            results=SimpleNamespace(execute=lambda kind, point, callback, *a, **kw: calls.append(kind) or callback(*a),
                                    waypoint=lambda *args: None),
            transition=lambda *args: None)
        task = self.method('task')
        task.__globals__['rospy'].sleep = lambda duration: calls.append(('wait', duration))
        task(obj)
        self.assertEqual(calls, [('wait', 1.0), 'temperature'])

    def test_meter_task_waits_before_reading_new_measurements(self):
        calls = []
        obj = SimpleNamespace(
            route={'points': [{'id': 'wp_11', 'task': 'meter'}]}, index=0,
            navigation_only=False, check_running=lambda: None, data_dir=Path('/unused'),
            read_meter=lambda *args: calls.append('meter'),
            results=SimpleNamespace(execute=lambda kind, point, callback, *args, **kwargs: callback(*args),
                                    waypoint=lambda *args: None), transition=lambda *args: None)
        task = self.method('task')
        task.__globals__['rospy'].sleep = lambda duration: calls.append(('wait', duration))
        task(obj)
        self.assertEqual(calls, [('wait', 1.0), 'meter'])

    def test_meter_deadline_and_valid_results(self):
        for scenario in ('no_frames', 'invalid', 'valid', 'late', 'ocr_error'):
            with self.subTest(scenario=scenario):
                clock = SimpleNamespace(now=0., callback=None, stamp=0, unsubscribed=False)
                saved = []
                def deliver():
                    if scenario != 'no_frames':
                        clock.stamp += 1
                        clock.callback(SimpleNamespace(header=SimpleNamespace(stamp=clock.stamp)))
                class Frames:
                    def __enter__(self):
                        return self
                    def __exit__(self, *args):
                        pass
                    def wait(self, duration):
                        clock.now += duration
                        deliver()
                    def notify(self):
                        pass
                def subscribe(topic, message, callback, **kwargs):
                    clock.callback = callback
                    deliver()
                    return SimpleNamespace(unregister=lambda: setattr(clock, 'unsubscribed', True))
                def read_electricity(image, deadline):
                    if scenario == 'late':
                        clock.now = deadline
                    else:
                        clock.now += .1
                    if scenario == 'ocr_error':
                        return None, 'OCR unavailable'
                    return SimpleNamespace(plate_texts=['001.2'], confidences=[.9],
                                           bboxes=[20, 30, 80, 70], status_message='ok'), ''
                reader = SimpleNamespace(read=lambda frame: (
                    {'temperature': None if scenario == 'invalid' else 25., 'humidity': 50.},
                    'overlay', 'aligned'))
                module = SimpleNamespace(__file__='/unused/meter_reader.py', Reader=lambda path: reader,
                                         quantize_reading=lambda kind, value: value,
                                         save_image=lambda path, image: saved.append(str(path)))
                ros = SimpleNamespace(get_param=lambda key, default: default,
                                      Time=SimpleNamespace(now=lambda: 0), Subscriber=subscribe,
                                      logwarn=lambda *args: None)
                obj = SimpleNamespace(route={'points': [{'id': 'wp_11'}]}, index=0,
                                      point_image_topic=lambda *args, **kwargs: '/meter',
                                      check_running=lambda: None,
                                      read_electricity_until=read_electricity,
                                      bridge=SimpleNamespace(imgmsg_to_cv2=lambda *args, **kwargs: 'frame'))
                read = self.method('read_meter', rospy=ros, math=math, Path=Path,
                                   threading=SimpleNamespace(Condition=Frames),
                                   time=SimpleNamespace(monotonic=lambda: clock.now), Image=object,
                                   save_annotation=lambda frame, record, path, font: saved.append(str(path)),
                                   cv2=SimpleNamespace(putText=lambda *args: None, FONT_HERSHEY_SIMPLEX=0))
                with patch.dict(sys.modules, meter_reader=module):
                    result = read(obj, 'raw.jpg', 'overlay.jpg')
                self.assertTrue(clock.unsubscribed)
                if scenario == 'valid':
                    self.assertEqual(result['electricity_kwh'], '001.2')
                    self.assertNotIn('skipped', result)
                    self.assertLess(clock.now, 20.)
                else:
                    self.assertTrue(result['skipped'])
                    self.assertEqual(result['timeout_sec'], 20.)
                    self.assertTrue(result['reason'].startswith('METER_TIMEOUT:'))
                    self.assertGreaterEqual(clock.now, 20.)
                    self.assertLess(clock.now, 20.3)
                expected = [] if scenario == 'no_frames' else ['raw.jpg', 'overlay.jpg']
                if scenario == 'valid':
                    expected.append('raw_electricity_annotated.jpg')
                self.assertEqual(saved, expected)

    def test_meter_ocr_wait_is_bounded_and_cancellation_propagates(self):
        read = self.method('read_electricity_until', threading=threading, time=time)
        gate = threading.Event()
        obj = SimpleNamespace(check_running=lambda: None,
                              services=SimpleNamespace(read_electricity=lambda image: gate.wait()))
        started = time.monotonic()
        try:
            result, reason = read(obj, object(), started + .05)
            self.assertIsNone(result)
            self.assertIn('超时', reason)
            self.assertLess(time.monotonic() - started, 1.)
            def cancel():
                raise RuntimeError('CANCELLED')
            obj.check_running = cancel
            with self.assertRaisesRegex(RuntimeError, 'CANCELLED'):
                read(obj, object(), time.monotonic() + 1.)
        finally:
            gate.set()

    def test_skipped_meter_is_recorded_and_advances_to_next_point(self):
        from result_manager import ResultManager
        point = {'id': 'wp_11', 'task': 'meter'}
        result = {'skipped': True, 'reason': 'METER_TIMEOUT: no valid results', 'timeout_sec': 20.}
        transitions = []
        with tempfile.TemporaryDirectory() as directory:
            results = ResultManager(directory, [point], report=lambda *args: None)
            results.waypoint(point, 'RUNNING')
            obj = SimpleNamespace(route={'points': [point]}, index=0, navigation_only=False,
                                  check_running=lambda: None, data_dir=results.path, results=results,
                                  read_meter=lambda *args: result,
                                  transition=lambda state, reason: transitions.append(state))
            self.method('task')(obj)
            self.assertEqual(transitions, ['NEXT_POINT'])
            self.assertEqual(results.data['operations'][0]['status'], 'SKIPPED')
            self.assertEqual(results.data['planned_tasks'][0]['status'], 'SKIPPED')
            results.finish('SUCCESS')
            self.assertEqual(results.data['summary']['skipped_tasks'], 1)
            self.assertEqual(results.data['summary']['failed_tasks'], 0)

    def test_no_settle_delay_for_navigation_only_or_navigation_waypoint(self):
        for navigation_only, kind in [(True, 'person'), (False, 'navigation')]:
            obj = SimpleNamespace(route={'points': [{'id': 'wp_03', 'task': kind}]}, index=0,
                                  navigation_only=navigation_only, check_running=lambda: None,
                                  results=SimpleNamespace(waypoint=lambda *args: None),
                                  transition=lambda *args: None)
            task = self.method('task')
            task.__globals__['rospy'].sleep = lambda duration: self.fail('Unexpected settle delay')
            task(obj)

    def test_cancel_during_settle_delay_prevents_frame_capture(self):
        cancelled = []
        def check():
            if cancelled:
                raise RuntimeError('CANCELLED')
        obj = SimpleNamespace(route={'points': [{'id': 'wp_06', 'task': 'person'}]}, index=0,
                              navigation_only=False, check_running=check,
                              select_camera=lambda point: None)
        task = self.method('task')
        task.__globals__['rospy'].sleep = lambda duration: cancelled.append(True)
        with self.assertRaisesRegex(RuntimeError, 'CANCELLED'):
            task(obj)

    def test_invalid_camera_config_is_rejected(self):
        for camera in ('rear', None, {}, 42):
            route = copy.deepcopy(self.route)
            route['points'][0]['camera'] = camera
            with self.subTest(camera=camera), self.assertRaises(ValueError):
                validate_route(route)

    def test_recording_detects_active_planner_without_changing_navigation_speed(self):
        for name in ('DWAPlannerROS', 'TebLocalPlannerROS'):
            obj = SimpleNamespace(runtime={'server_timeout': 30}, navigation_only=False, task_only=False,
                                  client=SimpleNamespace(wait_for_server=lambda duration: True),
                                  transition=lambda *args: None, planner=None)
            initialize = self.method('initialize')
            initialize.__globals__['rospy'].Duration = lambda value: value
            initialize.__globals__['rospy'].get_param = lambda key: 'package/' + name
            initialize(obj)
            self.assertEqual(obj.planner, '/move_base/' + name)

    def test_lateral_service_uses_only_parameters_supported_by_planner(self):
        for limits in ({'max_vel_y': .7}, {'max_vel_y': .08, 'min_vel_y': -.06}):
            requests = []
            def config(**kwargs):
                return SimpleNamespace(doubles=kwargs.get('doubles', []))
            def call(name, service, request, **kwargs):
                requests.append((name, {v.name: v.value for v in request.doubles}))
                values = limits if not request.doubles else requests[-1][1]
                return SimpleNamespace(config=config(doubles=[SimpleNamespace(name=k, value=v)
                                                              for k, v in values.items()]))
            obj = SimpleNamespace(config={'planner_service_timeout': 1}, call=call)
            scope = dict(filename='services.py', class_name='Services', Config=config,
                         Reconfigure=object, DoubleParameter=SimpleNamespace, math=math)
            get = self.method('get_lateral_limits', **scope)
            set_limits = self.method('set_lateral_limits', **scope)
            self.assertEqual(get(obj, '/move_base/planner'), limits)
            set_limits(obj, '/move_base/planner', limits)
            self.assertEqual(requests, [('/move_base/planner/set_parameters', {}),
                                        ('/move_base/planner/set_parameters', limits)])

    def test_lateral_service_rejects_unapplied_speed_limit(self):
        obj = SimpleNamespace(config={'planner_service_timeout': 1},
                              call=lambda *args, **kwargs: SimpleNamespace(config=SimpleNamespace(
                                  doubles=[SimpleNamespace(name='max_vel_y', value=.7)])))
        set_limits = self.method('set_lateral_limits', filename='services.py', class_name='Services',
                                 Config=lambda **kwargs: SimpleNamespace(**kwargs),
                                 DoubleParameter=SimpleNamespace, Reconfigure=object, math=math)
        with self.assertRaisesRegex(RuntimeError, 'did not apply'):
            set_limits(obj, '/move_base/TebLocalPlannerROS', {'max_vel_y': .15})

    def test_recording_limits_and_restores_original_planner_values(self):
        originals = [{'max_vel_y': .7}, {'max_vel_y': .08, 'min_vel_y': -.06},
                     {'max_vel_y': .3, 'min_vel_y': -.4}]
        for original in originals:
            events, current = [], dict(original)
            def set_limits(namespace, limits):
                events.append(('limits', dict(limits)))
                current.clear()
                current.update(limits)
            obj = SimpleNamespace(
                task_only=False, planner='/move_base/planner', runtime={'recording_lateral_speed': .15},
                saved_lateral_limits=None, recording=False, video_path=Path('/unused/video.mp4'),
                services=SimpleNamespace(get_lateral_limits=lambda namespace: dict(current),
                                         set_lateral_limits=set_limits,
                                         start_recording=lambda path: events.append(('start', str(path))),
                                         stop_recording=lambda: events.append(('stop', None))))
            restore = self.method('restore_lateral_limits')
            obj.restore_lateral_limits = lambda: restore(obj)
            self.method('start_recording')(obj)
            self.assertTrue(obj.recording)
            expected = {'max_vel_y': min(original['max_vel_y'], .15)}
            if 'min_vel_y' in original:
                expected['min_vel_y'] = max(original['min_vel_y'], -.15)
            self.assertEqual(current, expected)
            self.assertEqual(events[0], ('limits', expected))
            self.assertEqual(events[1][0], 'start')
            self.method('stop_recording')(obj)
            self.assertFalse(obj.recording)
            self.assertEqual(current, original)
            self.assertIsNone(obj.saved_lateral_limits)
            self.assertEqual(events[-2:], [('stop', None), ('limits', original)])

    def test_recording_start_failures_restore_speed_during_cleanup(self):
        for failure in ('limit', 'recorder'):
            events = []
            def set_limits(namespace, limits):
                events.append(('limits', dict(limits)))
                if failure == 'limit' and len(events) == 1:
                    raise RuntimeError('limit unavailable')
            def start(path):
                events.append(('start', None))
                raise RuntimeError('recorder unavailable')
            obj = SimpleNamespace(
                task_only=False, planner='/move_base/planner', runtime={'recording_lateral_speed': .15},
                saved_lateral_limits=None, recording=False, closed=False, stopping=False,
                condition=nullcontext(), video_path=Path('/unused/video.mp4'),
                client=SimpleNamespace(cancel_all_goals=lambda: events.append(('cancel', None))),
                subscriber=None, speech=None,
                services=SimpleNamespace(get_lateral_limits=lambda namespace: {'max_vel_y': .7},
                                         set_lateral_limits=set_limits, start_recording=start))
            restore = self.method('restore_lateral_limits')
            obj.restore_lateral_limits = lambda: restore(obj)
            with self.assertRaisesRegex(RuntimeError, failure):
                self.method('start_recording')(obj)
            self.assertFalse(obj.recording)
            if failure == 'limit':
                self.assertNotIn(('start', None), events)
            self.method('close')(obj)
            self.assertEqual(events[-1], ('limits', {'max_vel_y': .7}))
            self.assertIsNone(obj.saved_lateral_limits)
            self.assertTrue(obj.closed)

    def test_limit_failure_stops_state_machine_before_recording_navigation(self):
        events = []
        def fail_limits(namespace, limits):
            raise RuntimeError('limit unavailable')
        def forbidden(*args, **kwargs):
            self.fail('Must not record or navigate after failed speed limiting')
        obj = SimpleNamespace(
            route={'points': [{'id': 'wp_35', 'task': 'record_start'}]}, index=0,
            navigation_only=False, task_only=False, single_waypoint='', obstacle_mode='clear', check_running=lambda: None, state='INIT',
            planner='/move_base/TebLocalPlannerROS', saved_lateral_limits=None,
            recording=False, runtime={'recording_lateral_speed': .15},
            video_path=Path('/unused/video.mp4'),
            services=SimpleNamespace(get_lateral_limits=lambda ns: {'max_vel_y': .7},
                                     set_lateral_limits=fail_limits, start_recording=forbidden),
            data_dir=Path('/unused'),
            results=SimpleNamespace(log=lambda *a: None, run_id='test', execute=lambda kind, point, callback, **kwargs: callback(),
                                    finish=lambda state, error: events.append(('finish', state))))
        def transition(state, reason):
            obj.state, obj.reason = state, reason
        obj.transition = transition
        obj.initialize = lambda: transition('TASK', 'already at recording point')
        obj.navigate = forbidden
        obj.next_point = forbidden
        obj.start_recording = lambda: self.method('start_recording')(obj)
        obj.task = lambda: self.method('task')(obj)
        obj.stop_navigation = lambda: events.append(('stop', None))
        with self.assertRaisesRegex(RuntimeError, 'limit unavailable'):
            self.method('run')(obj)
        self.assertEqual(events, [('stop', None), ('finish', 'ERROR')])

    def test_shutdown_during_recording_stops_video_and_restores_limits(self):
        events = []
        obj = SimpleNamespace(
            planner='/move_base/planner', saved_lateral_limits={'max_vel_y': .7},
            recording=True, closed=False, stopping=False, condition=nullcontext(),
            client=SimpleNamespace(cancel_all_goals=lambda: events.append('cancel')),
            subscriber=None, speech=None,
            services=SimpleNamespace(stop_recording=lambda: events.append('stop'),
                                     set_lateral_limits=lambda ns, limits: events.append(dict(limits))))
        restore = self.method('restore_lateral_limits')
        obj.restore_lateral_limits = lambda: restore(obj)
        close = self.method('close')
        close(obj)
        self.assertEqual(events, ['cancel', 'stop', {'max_vel_y': .7}])
        self.assertIsNone(obj.saved_lateral_limits)
        close(obj)
        self.assertEqual(len(events), 3)

    def test_explicit_traffic_light_skip_does_not_read_camera_or_detector(self):
        checked = []
        obj = SimpleNamespace(skip_traffic_light=True,
                              check_running=lambda: checked.append(True))
        wait = self.method('wait_for_green')
        wait.__globals__['rospy'].logwarn = lambda *args: None
        result = wait(obj)
        self.assertTrue(result['released'])
        self.assertTrue(result['skipped'])
        self.assertEqual(result['reason'], 'skip_traffic_light')
        self.assertEqual(result['green_streak'], 0)
        self.assertEqual(result['detected_classes'], [])
        self.assertEqual(checked, [True])

    def test_normal_traffic_light_wait_still_requires_consecutive_green_frames(self):
        for labels in [('go', 'go', 'go'), ('go', 'stop', 'go', 'go', 'go')]:
            clock = SimpleNamespace(now=0., index=0)
            def fresh(timeout):
                clock.now += .1
                clock.index += 1
                return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
                    to_nsec=lambda: clock.index)))
            obj = SimpleNamespace(skip_traffic_light=False, light_timeout=2., image_timeout=1.,
                                  green_confirm_frames=3, check_running=lambda: None,
                                  fresh_image=fresh,
                                  services=SimpleNamespace(detect=lambda name, image: [
                                      Detection(labels[clock.index-1], (0, 0, 10, 10), .9)]))
            wait = self.method('wait_for_green', math=math, detections=lambda result: result,
                               time=SimpleNamespace(monotonic=lambda: clock.now))
            result = wait(obj)
            self.assertTrue(result['released'])
            self.assertFalse(result['skipped'])
            self.assertEqual(result['green_streak'], 3)
            self.assertEqual(clock.index, len(labels))

    def test_normal_traffic_light_wait_times_out_on_red(self):
        clock = SimpleNamespace(now=0., index=0)
        def fresh(timeout):
            clock.now += .1
            clock.index += 1
            return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
                to_nsec=lambda: clock.index)))
        obj = SimpleNamespace(skip_traffic_light=False, light_timeout=.4, image_timeout=1.,
                              green_confirm_frames=3, check_running=lambda: None,
                              fresh_image=fresh,
                              services=SimpleNamespace(detect=lambda *args: [
                                  Detection('stop', (0, 0, 10, 10), .9)]))
        wait = self.method('wait_for_green', math=math, detections=lambda result: result,
                           time=SimpleNamespace(monotonic=lambda: clock.now))
        with self.assertRaisesRegex(RuntimeError, 'TRAFFIC_TIMEOUT'):
            wait(obj)
        self.assertGreater(clock.index, 0)

    def test_initialize_single_map_ignores_old_closures(self):
        transitions = []
        obj = SimpleNamespace(
            route={'navigation_closures': {'clear': {}}},
            runtime={'server_timeout': 30.0}, navigation_only=True, task_only=False,
            client=SimpleNamespace(wait_for_server=lambda timeout: True),
            transition=lambda *args: transitions.append(args))
        initialize = self.method('initialize')
        initialize.__globals__['rospy'].Duration = lambda value: value
        initialize(obj)
        self.assertEqual(transitions[-1][0], 'NAVIGATE')

    def test_obstacle_selection_does_not_require_map_service(self):
        image = SimpleNamespace(width=640)
        result = {'mode': 'clear', 'center_x_ratio': 0.2}
        obj = SimpleNamespace(
            route={'navigation_closures': {'clear': {}}},
            services=SimpleNamespace(detect=lambda detector, frame: []),
            check_running=lambda: None, obstacle_clear_x_ratio=0.35, announce=lambda text: None)
        select = self.method('select_obstacle_route', detections=lambda objects: objects,
                             obstacle_route=lambda *args: result)
        self.assertEqual(select(obj, image), result)
        self.assertEqual(obj.obstacle_mode, 'clear')

    def test_both_branches_run_every_task_once_and_finish(self):
        validate_route(self.route)
        points = self.route['points']
        split = next(i for i, p in enumerate(points) if p['task'] == 'obstacle')
        by_id = {p['id']: p for p in points}
        for mode, count in [('clear', 38), ('blocked', 37)]:
            for navigation_only in (True, False):
                with self.subTest(mode=mode, navigation_only=navigation_only):
                    obj = SimpleNamespace(route=copy.deepcopy(self.route), index=split,
                                          single_waypoint='', obstacle_mode=mode,
                                          navigation_only=navigation_only,
                                          transition=lambda *args: None)
                    self.method('next_point')(obj)
                    selected = obj.route['points']
                    expected = points[:split + 1] + [by_id[name] for name in self.route['navigation_branches'][mode]]
                    self.assertEqual(selected, expected)
                    self.assertEqual(len(selected), count)
                    self.assertEqual(len({p['id'] for p in selected}), count)
                    self.assertEqual(selected[-1]['id'], 'wp_39')
                    # Subsequent state transitions visit the entire selected suffix once.
                    visited = [p['id'] for p in selected[:obj.index]]
                    events = []
                    obj.transition = lambda state, reason: events.append(state)
                    while True:
                        visited.append(obj.route['points'][obj.index]['id'])
                        self.method('next_point')(obj)
                        if events[-1] == 'FINISH':
                            break
                    self.assertEqual(visited, [p['id'] for p in selected])

    def test_navigation_wp04_does_not_trigger_branch(self):
        obj = SimpleNamespace(route=copy.deepcopy(self.route), index=3, single_waypoint='',
                              obstacle_mode='blocked', transition=lambda *args: None)
        self.assertEqual(obj.route['points'][3]['id'], 'wp_04')
        before = copy.deepcopy(obj.route['points'])
        self.method('next_point')(obj)
        self.assertEqual(obj.route['points'], before)
        self.assertEqual(obj.index, 4)

    def test_branch_trigger_can_use_another_id(self):
        trigger = next(p for p in self.route['points'] if p['task'] == 'obstacle')
        trigger['id'] = 'obstacle_observation'
        validate_route(self.route)
        obj = SimpleNamespace(route=self.route, index=4, single_waypoint='',
                              obstacle_mode='blocked', transition=lambda *args: None)
        self.method('next_point')(obj)
        self.assertEqual(obj.route['points'][5]['id'], 'blocked_wp_06')
        self.assertEqual(len(obj.route['points']), 37)

    def test_main_branches_use_source_waypoints_and_common_tail(self):
        by_id = {p['id']: p for p in self.route['points']}
        for mode in ('clear', 'blocked'):
            source = yaml.safe_load((SCRIPTS.parent / f'config/route_{mode}.yaml').read_text())['points']
            selected = [by_id[name] for name in self.route['navigation_branches'][mode]]
            self.assertEqual(selected, source[5:])
        prefix = self.route['points'][:5]
        self.assertEqual(prefix[-1]['task'], 'obstacle')
        self.assertEqual(prefix[0]['task'], 'traffic_light')
        self.assertEqual(prefix[1]['area'], 'A')
        suffixes = []
        for mode in ('clear', 'blocked'):
            order = self.route['navigation_branches'][mode]
            suffixes.append(order[order.index('wp_27'):])
        self.assertEqual(suffixes[0], suffixes[1])
        self.assertEqual(suffixes[0], [f'wp_{i:02d}' for i in range(27, 40) if i != 36])

    def test_fixed_parking_photo_in_both_branches(self):
        for mode, order in self.route['navigation_branches'].items():
            with self.subTest(mode=mode):
                self.assertEqual(order[order.index('wp_35') + 1], 'wp_37')
                selected = [p for p in self.route['points'] if p['id'] in order]
                self.assertEqual([p['id'] for p in selected if p['task'] == 'vehicle_photo'], ['wp_35'])
                self.assertFalse(any(p['task'] in ('record_start', 'record_stop') for p in selected))
                self.assertEqual(order[-1], 'wp_39')

    def test_parking_settle_holds_zero_speed_and_checks_cancellation(self):
        events = []
        clock = SimpleNamespace(now=0.0)
        def sleep(seconds):
            clock.now += seconds
        obj = SimpleNamespace(vehicle_settle_time=.12,
                              check_running=lambda: events.append('check'),
                              stop_pub=SimpleNamespace(publish=lambda value: events.append(value)))
        self.method('settle_vehicle_photo', time=SimpleNamespace(
            monotonic=lambda: clock.now, sleep=sleep), Twist=lambda: 'zero')(obj)
        self.assertGreaterEqual(clock.now, .12)
        self.assertGreaterEqual(events.count('zero'), 3)
        self.assertEqual(events, ['check', 'zero'] * events.count('zero'))
        def cancel():
            raise RuntimeError('CANCELLED')
        obj.check_running = cancel
        with self.assertRaisesRegex(RuntimeError, 'CANCELLED'):
            self.method('settle_vehicle_photo', time=SimpleNamespace(
                monotonic=lambda: clock.now), Twist=lambda: 'zero')(obj)

    def test_invalid_branch_configurations_are_rejected(self):
        mutations = []
        route = copy.deepcopy(self.route)
        route['navigation_branches']['blocked'].remove('blocked_wp_10')
        mutations.append(route)
        route = copy.deepcopy(self.route)
        route['navigation_branches']['blocked'].append('wp_39')
        mutations.append(route)
        route = copy.deepcopy(self.route)
        route['navigation_branches']['clear'][-1] = 'wp_38'
        mutations.append(route)
        route = copy.deepcopy(self.route)
        route['navigation_branches']['blocked'][0] = 'missing'
        mutations.append(route)
        route = copy.deepcopy(self.route)
        branch = route['navigation_branches']['blocked']
        branch.remove('wp_35')
        route['navigation_branches']['clear'].remove('wp_35')
        mutations.append(route)
        route = copy.deepcopy(self.route)
        next(p for p in route['points'] if p['id'] == 'wp_07')['task'] = 'obstacle'
        mutations.append(route)
        route = copy.deepcopy(self.route)
        route['navigation_branches']['blocked'][0] = 'wp_02'
        mutations.append(route)
        for index, route in enumerate(mutations):
            with self.subTest(index=index), self.assertRaises(ValueError):
                validate_route(route)

    def test_single_waypoint_does_not_expand_branch(self):
        events = []
        obj = SimpleNamespace(route=self.route, index=4, single_waypoint='wp_05',
                              transition=lambda state, reason: events.append(state))
        self.method('next_point')(obj)
        self.assertEqual(events, ['FINISH'])
        self.assertEqual(obj.index, 4)

    def test_task_only_rejects_navigation_even_if_called_directly(self):
        with self.assertRaisesRegex(RuntimeError, 'disabled'):
            self.method('move_to')(SimpleNamespace(task_only=True), (1, 2, 0))

    def test_execution_modes_must_be_boolean_and_exclusive(self):
        for navigation_only, task_only in [(True, True), (False, 'true'), (1, False)]:
            params = {'~navigation_only': navigation_only, '~task_only': task_only}
            rospy = SimpleNamespace(Publisher=lambda *a, **k: None,
                                    get_param=lambda key, default=None: params.get(key, default))
            with self.subTest(params=params), self.assertRaises(ValueError):
                self.method('__init__', rospy=rospy, String=object)(SimpleNamespace())

    def test_task_only_creates_no_action_client(self):
        params = {'~task_only': True, '~': copy.deepcopy(self.route),
                  '~mission': yaml.safe_load((SCRIPTS.parent / 'config/runtime.yaml').read_text())['mission'],
                  '~recognition_rules': {}}
        def forbidden(*args, **kwargs):
            self.fail('task_only touched navigation')
        with tempfile.TemporaryDirectory() as folder:
            params['~data_dir'] = folder
            rospy = SimpleNamespace(Publisher=lambda *a, **k: None, loginfo=lambda *a: None,
                                    get_param=lambda key, default=None: params.get(key, default),
                                    Subscriber=lambda *a, **k: None, on_shutdown=lambda cb: None)
            obj = SimpleNamespace(image_callback=lambda image: None, close=lambda: None)
            self.method('__init__', rospy=rospy, String=object, Image=object,
                        validate_route=validate_route, math=math, threading=threading,
                        ResultManager=ResultManager, actionlib=SimpleNamespace(SimpleActionClient=forbidden),
                        Services=lambda runtime: object(), CvBridge=lambda: object(),
                        SpeechQueue=lambda *a: object(), Inspection=lambda *a: object())(obj)
            self.assertIsNone(obj.client)
            self.assertTrue(obj.results.data['task_only'])

    def task_only_mission(self, folder, mode='clear', single='', fail=False):
        """Run real state handlers/result persistence with fake camera/services, no ROS."""
        def forbidden(*args, **kwargs):
            self.fail('task_only touched navigation/planner')
        route = copy.deepcopy(self.route)
        results = ResultManager(folder, route['points'], lambda *a: None)
        obj = SimpleNamespace(route=route, index=0, state='INIT', reason='',
                              single_waypoint=single, obstacle_mode=mode, navigation_only=False,
                              task_only=True, runtime={}, results=results, data_dir=results.path,
                               saved_lateral_limits=None, select_camera=lambda point: None,
                               measure_temperature=lambda *a: {'announcement': ''},
                               read_meter=lambda *a: {'temperature_c': 20, 'humidity_percent': 50, 'electricity_kwh': '1'},
                              client=None, planner='/move_base/DWAPlannerROS', stopping=False,
                              condition=threading.Condition(), recording=False, closed=False,
                              subscriber=None, speech=None, video_path=results.path/'videos/video.mp4',
                              fresh_image=lambda: object(),
                              bridge=SimpleNamespace(imgmsg_to_cv2=lambda *a, **k: object()),
                              services=SimpleNamespace(set_lateral_speed=forbidden,
                                                       start_recording=lambda path: None,
                                                       stop_recording=lambda: None),
                              check_running=lambda: None, move_to=forbidden,
                              run_inspection=lambda point, kind, *a: results.execute(
                                  kind, point, lambda: {'recognized': True}))
        if single:
            obj.index = next(i for i, p in enumerate(route['points']) if p['id'] == single)
        def transition(state, reason):
            obj.state, obj.reason = state, reason
            results.state(state, route['points'][obj.index], reason)
        obj.transition = transition
        def green():
            if fail:
                raise RuntimeError('TRAFFIC_TIMEOUT')
            return {'released': True}
        obj.wait_for_green = green
        obj.select_obstacle_route = lambda image, *args: {'mode': mode}
        for name in ['initialize', 'navigate', 'task', 'wait_traffic', 'next_point',
                     'start_recording', 'stop_recording', 'restore_lateral_limits', 'finish_recording',
                     'stop_navigation', 'close', 'run']:
            method = self.method(name, cv2=SimpleNamespace(imwrite=lambda *a: True))
            obj.__dict__[name] = MethodType(method, obj)
        return obj

    def test_task_only_runs_real_task_handlers_in_both_branches(self):
        for mode, count in [('clear', 38), ('blocked', 37)]:
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as folder:
                obj = self.task_only_mission(folder, mode)
                obj.run()
                obj.close()
                self.assertEqual(obj.state, 'FINISH')
                operations = obj.results.data['operations']
                expected = [(p['id'], p['task']) for p in obj.route['points']
                            if p['task'] != 'navigation']
                actual = [(p['waypoint_id'], p['task_type']) for p in operations
                          if p['task_type'] != 'vehicle_video']
                self.assertEqual(actual, expected)
                self.assertNotIn('navigation', [p['task_type'] for p in operations])
                self.assertEqual(obj.results.data['summary']['completed_waypoints'], count)
                self.assertFalse(obj.recording)
                self.assertTrue((obj.data_dir / 'mission_results.json').is_file())

    def test_task_only_single_point_and_error_cleanup(self):
        for fail in (False, True):
            with self.subTest(fail=fail), tempfile.TemporaryDirectory() as folder:
                obj = self.task_only_mission(folder, single='wp_01', fail=fail)
                if fail:
                    with self.assertRaisesRegex(RuntimeError, 'TRAFFIC_TIMEOUT'):
                        obj.run()
                else:
                    obj.run()
                obj.close()
                self.assertEqual(obj.state, 'ERROR' if fail else 'FINISH')
                ops = obj.results.data['operations']
                self.assertEqual([p['task_type'] for p in ops], ['traffic_light'])
                self.assertEqual(ops[0]['status'], 'FAILED' if fail else 'SUCCESS')
                self.assertTrue(all(p['status'] == 'NOT_EXECUTED'
                                    for p in obj.results.data['waypoints'][1:]))

    def test_navigation_waits_for_success_and_stops_on_failure(self):
        statuses = SimpleNamespace(PENDING=0, ACTIVE=1, PREEMPTED=2, SUCCEEDED=3,
                                   ABORTED=4, REJECTED=5, PREEMPTING=6,
                                   RECALLING=7, RECALLED=8, LOST=9)
        for states, timeout, error in [([0, 1, 1, 3], 10, None),
                                       ([1] * 10, .09, 'NAV_TIMEOUT'),
                                       ([4], 10, 'NAV_FAILED'), ([9], 10, 'NAV_LOST'),
                                       ([2], 10, 'NAV_CANCELLED')]:
            with self.subTest(states=states):
                clock = SimpleNamespace(now=0., sleeps=0)
                sent, stopped = [], []
                sequence = iter(states)
                def sleep(duration):
                    clock.now += duration
                    clock.sleeps += 1
                def goal():
                    return SimpleNamespace(target_pose=SimpleNamespace(
                        header=SimpleNamespace(), pose=SimpleNamespace(
                            position=SimpleNamespace(), orientation=SimpleNamespace())))
                move_to = self.method('move_to', math=math, MoveBaseGoal=goal,
                                      GoalStatus=statuses,
                                      time=SimpleNamespace(monotonic=lambda: clock.now, sleep=sleep))
                # The only ROS operation in this isolated method is the goal timestamp.
                move_to.__globals__['rospy'].Time = SimpleNamespace(now=lambda: 0)
                obj = SimpleNamespace(navigation_retries=0, goal_timeout=timeout, task_only=False,
                                      condition=nullcontext(), check_running=lambda: None,
                                      stop_navigation=lambda: stopped.append(True),
                                      client=SimpleNamespace(send_goal=lambda goal: sent.append(goal),
                                                             get_state=lambda: next(sequence)))
                if error:
                    with self.assertRaisesRegex(RuntimeError, error):
                        move_to(obj, (3.84, .41, 1.5708))
                    self.assertTrue(stopped)
                else:
                    self.assertEqual(move_to(obj, (3.84, .41, 1.5708))['outcome'], 'SUCCESS')
                    self.assertEqual(clock.sleeps, 3)
                    self.assertFalse(stopped)
                self.assertEqual(len(sent), 1)


if __name__ == '__main__':
    unittest.main()
