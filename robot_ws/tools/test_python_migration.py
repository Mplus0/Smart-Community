#!/usr/bin/env python3
"""Offline regression fixtures: business rules, odometry and mission ordering."""
import ast
import math
import yaml
import copy
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import tempfile
import json

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src/community_mission/scripts')]
from result_manager import ResultManager
from rules import (Detection, detections, people_counts, illegal_parking_count,
    vehicle_counts, trash_announcement, vehicle_announcement, validate_route)
# Bind the deployed YAML values to pure-rule tests.
from functools import partial
SETTINGS = yaml.safe_load((ROOT/'src/community_mission/config/recognition_rules.yaml').read_text(encoding='utf-8'))['recognition_rules']
RUNTIME = yaml.safe_load((ROOT/'src/community_mission/config/runtime.yaml').read_text(encoding='utf-8'))['mission']
people_counts = partial(people_counts, settings=SETTINGS)
illegal_parking_count = partial(illegal_parking_count, settings=SETTINGS)
vehicle_counts = partial(vehicle_counts, settings=SETTINGS)
trash_announcement = partial(trash_announcement, settings=SETTINGS)
# Extract the pure integrator from the combined node without importing ROS.
_tree = ast.parse((ROOT/'src/community_bringup/scripts/odometry_node.py').read_text(encoding='utf-8'))
_integrator = ast.Module(body=[n for n in _tree.body if isinstance(n, ast.ClassDef)
                               and n.name == 'OdometryIntegrator'], type_ignores=[])
_env = {'math': math}
exec(compile(_integrator, '<production odometry integrator>', 'exec'), _env)
OdometryIntegrator = _env['OdometryIntegrator']


class ConfigurationTests(unittest.TestCase):
    def test_yolo_profiles_and_explicit_overrides(self):
        config = yaml.safe_load((ROOT/'src/community_perception/config/detectors.yaml').read_text(encoding='utf-8'))
        tree = ast.parse((ROOT/'src/community_perception/scripts/yolo_detector_service.py').read_text(encoding='utf-8'))
        init = next(n for c in tree.body if isinstance(c, ast.ClassDef) for n in c.body
                    if isinstance(n, ast.FunctionDef) and n.name == '__init__')
        # Execute the production parameter-loading block, without loading a model.
        body = []
        for statement in init.body:
            body.append(statement)
            if isinstance(statement, ast.Assign) and any(isinstance(t, ast.Attribute) and t.attr == 'nc' for t in statement.targets):
                break
        code = compile(ast.Module(body=body, type_ignores=[]), '<YOLO settings>', 'exec')
        for name, settings in config['detectors'].items():
            for profile in ('standalone', 'mission'):
                for override in (None, 0.0, 0.63):
                    with self.subTest(name=name, profile=profile, override=override):
                        params = {'~model_name': name, '~profile': profile,
                                  '~detectors': config['detectors'], '~mission_confidence': config['mission_confidence']}
                        if override is not None: params['~conf_threshold'] = override
                        node = SimpleNamespace()
                        ros = SimpleNamespace(get_param=lambda k, *default: params[k] if k in params else default[0])
                        exec(code, dict(self=node, rospy=ros, CvBridge=Mock()))
                        expected = settings['conf_threshold'] if profile == 'standalone' else config['mission_confidence'][name]
                        self.assertEqual(node.conf_threshold, expected if override is None else override)
                        self.assertEqual(list(node.class_names.values()), settings['class_names'])
                        relative = settings['model_path'].replace('$(find community_perception)/', '')
                        self.assertTrue((ROOT/'src/community_perception'/relative).is_file())

    def test_total_detector_launch_forwards_thresholds(self):
        import xml.etree.ElementTree as E
        tree = E.parse(ROOT/'src/community_perception/launch/main_detector.launch')
        includes = tree.findall('.//include')
        self.assertEqual(len(includes), 6)
        for include in includes:
            args = {a.get('name'):a.get('value') for a in include.findall('arg')}
            self.assertEqual(args['profile'], 'mission')
            self.assertEqual(args['iou_threshold'], '$(arg iou_threshold)')
            self.assertIn('conf_threshold', args['conf_threshold'])

    def test_rule_yaml_changes_are_used_without_changing_defaults(self):
        from rules import people_counts as count_people
        objects = [Detection('good', (300, 200, 400, 240))]
        self.assertEqual(count_people(objects, 640, 480, SETTINGS)[0], 1)
        changed = dict(SETTINGS, person_y_ratio=0.9)
        self.assertEqual(count_people(objects, 640, 480, changed)[0], 0)
        self.assertEqual(len(SETTINGS['waste_names']), 36)
        self.assertEqual(SETTINGS['waste_names']['orther1'], '脏污衣物')
        self.assertEqual(RUNTIME['green_confirm_frames'], 3)
        self.assertEqual(RUNTIME['traffic_light_timeout'], 30.0)
        self.assertEqual(RUNTIME['navigation_retries'], 0)


class RulesTests(unittest.TestCase):
    def test_interleaved_classes_use_object_order(self):
        response = SimpleNamespace(detected_classes=['good', 'bad', 'good'],
            confidences=[0.9, 0.8, 0.7], bboxes=[0, 0, 10, 10, 20, 0, 30, 10, 40, 0, 50, 10],
            class_names=['good', 'bad'], class_counts=[2, 1])
        self.assertEqual([(d.label, d.box[0]) for d in detections(response)],
                         [('good', 0), ('bad', 20), ('good', 40)])
        response.bboxes.pop()
        with self.assertRaises(ValueError):
            detections(response)

    def test_roi_scales_and_clears_previous_people(self):
        full = [Detection('good', (600, 500, 800, 800)), Detection('bad', (600, 0, 800, 300))]
        half = [Detection(d.label, tuple(v/2 for v in d.box)) for d in full]
        self.assertEqual(people_counts(full, 1280, 960)[:2], (1, 0))
        self.assertEqual(people_counts(half, 640, 480)[:2], (1, 0))
        self.assertIsNone(people_counts([], 640, 480)[2])

    def test_illegal_parking_overlap_and_area(self):
        objects = [Detection('stand', (100, 300, 180, 400)),
                   Detection('stand', (110, 310, 170, 390)),
                   Detection('fall', (200, 300, 210, 310))]
        self.assertEqual(illegal_parking_count(objects, (80, 250, 300, 450), 640, 480), 1)
        self.assertEqual(illegal_parking_count(objects, None, 640, 480), 0)

    def test_vehicle_adjacent_boundary_deduplication(self):
        frames = [([Detection('stand', (0, 100, 100, 300)), Detection('fall', (400, 100, 500, 300))], 1000, 500),
                  ([Detection('stand', (800, 100, 1000, 300))], 1000, 500)]
        self.assertEqual(vehicle_counts(frames), (1, 1))
        self.assertEqual(vehicle_counts([]), (0, 0))

    def test_trash_repeated_same_class_and_misclassification(self):
        bins = [Detection('red_open', (0, 0, 100, 100)), Detection('red_open', (200, 0, 300, 100))]
        waste = [Detection('hazardous1', (30, 0, 70, 20)), Detection('kitchen1', (230, 0, 270, 20))]
        text = trash_announcement(bins, waste, 1280)
        self.assertEqual(text.count('有害垃圾桶'), 2)
        self.assertIn('创可贴投放正确', text)
        self.assertIn('橙皮投放错误', text)

    def test_vehicle_announcement_does_not_swap_counts(self):
        self.assertIn('正常7辆，倒伏2辆', vehicle_announcement({'A': 1, 'B': 0}, 7, 2))

    def test_route_rejects_invalid_ids_coordinates_tasks_and_recording(self):
        import json
        route = yaml.safe_load((ROOT/'src/community_mission/config/route.yaml').read_text(encoding='utf-8'))
        validate_route(route)
        for change in ('duplicate', 'coordinate', 'task', 'multiple', 'recording'):
            bad = copy.deepcopy(route)
            if change == 'duplicate': bad['points'][1]['id'] = bad['points'][0]['id']
            elif change == 'coordinate': bad['points'][0]['x'] = float('nan')
            elif change == 'task': bad['points'][0]['task'] = 'unknown'
            elif change == 'multiple': bad['points'][0]['task'] = ['traffic_light', 'person']
            else: next(p for p in bad['points'] if p['task'] == 'record_start')['task'] = 'navigation'
            with self.subTest(change=change), self.assertRaises(ValueError): validate_route(bad)



class ResultManagerTests(unittest.TestCase):
    def test_runs_are_isolated_and_results_include_timing_paths(self):
        with tempfile.TemporaryDirectory() as root:
            point = dict(id='p1', task='person')
            first = ResultManager(root, [point], Mock())
            second = ResultManager(root, [point], Mock())
            self.assertNotEqual(first.path, second.path)
            first.waypoint(point, 'RUNNING')
            first.execute('navigation', point, lambda: None)
            image = first.path/'photo/000.jpg'
            value = first.execute('person', point, lambda: {'people': 2}, paths=[image])
            self.assertEqual(value, {'people': 2})
            first.waypoint(point, 'SUCCESS')
            first.finish('FINISH')
            data = json.loads((first.path/'mission_results.json').read_text(encoding='utf-8'))
            self.assertEqual(data['summary'], dict(completed_waypoints=1, successful_tasks=1, failed_tasks=0))
            task = data['operations'][1]
            self.assertEqual(task['result'], {'people': 2})
            self.assertEqual(task['paths'], [str(image)])
            self.assertTrue(task['started_at'] and task['ended_at'])
            self.assertGreaterEqual(task['duration_sec'], 0)
            self.assertIn('FINISH', (first.path/'mission.log').read_text(encoding='utf-8'))
            self.assertEqual(second.data['status'], 'RUNNING')

    def test_failure_keeps_reason_and_unexecuted_tasks(self):
        with tempfile.TemporaryDirectory() as root:
            point = dict(id='p1', task='plate')
            results = ResultManager(root, [point, dict(id='p2', task='fire')], Mock())
            results.waypoint(point, 'RUNNING')
            def fail(): raise RuntimeError('OCR unavailable')
            with self.assertRaisesRegex(RuntimeError, 'OCR unavailable'):
                results.execute('plate', point, fail)
            results.finish('ERROR', 'OCR unavailable')
            results.finish('FINISH')  # Finalization is idempotent.
            data = json.loads((results.path/'mission_results.json').read_text(encoding='utf-8'))
            self.assertEqual(data['error'], 'OCR unavailable')
            self.assertEqual(data['summary']['failed_tasks'], 1)
            self.assertEqual(data['waypoints'][0]['status'], 'FAILED')
            self.assertEqual([t['status'] for t in data['planned_tasks']], ['FAILED', 'NOT_EXECUTED'])

    def test_recognition_status_is_saved_without_reinterpretation(self):
        with tempfile.TemporaryDirectory() as root:
            points = [dict(id=kind, task=kind) for kind in ('person', 'plate', 'fire')]
            results = ResultManager(root, points, Mock())
            for point, status in zip(points, ('SUCCESS', 'EMPTY_VALID', 'FAILED')):
                results.record_task(dict(waypoint_id=point['id'], task_type=point['task'], status=status,
                    reason='test', result={}, duration_sec=0.5, started_at='start', ended_at='end'), paths=['/image.jpg'])
            results.finish('ERROR', 'test')
            data = json.loads((results.path/'mission_results.json').read_text(encoding='utf-8'))
            self.assertEqual([r['status'] for r in data['operations']], ['SUCCESS', 'EMPTY_VALID', 'FAILED'])
            self.assertEqual(data['summary']['successful_tasks'], 2)
            self.assertEqual(data['summary']['failed_tasks'], 1)
            self.assertEqual(data['operations'][1]['duration_sec'], 0.5)

    def test_disk_write_error_does_not_change_callback_result(self):
        with tempfile.TemporaryDirectory() as root:
            results = ResultManager(root, [], Mock())
            with patch.object(Path, 'write_text', side_effect=OSError('disk full')):
                value = results.execute('fire', {'id': 'p'}, lambda: {'fire_count': 0})
            self.assertEqual(value, {'fire_count': 0})
            self.assertTrue(results.data['recording_errors'])



class WaypointTasksTests(unittest.TestCase):
    def test_all_tasks_share_success_empty_and_failure_contract(self):
        from inspection import Inspection
        kinds = ('person', 'plate', 'fire', 'trash', 'vehicle_photo', 'vehicle_video')
        point = dict(id='arbitrary_id', task='person', area='B', parking_lot=3)
        image = SimpleNamespace(width=640, height=480)
        for kind in kinds:
            point['task'] = 'record_stop' if kind == 'vehicle_video' else kind
            for expected in ('SUCCESS', 'EMPTY_VALID', 'FAILED'):
                with self.subTest(kind=kind, status=expected):
                    services, speech = Mock(), Mock()
                    def detect(name, frame):
                        label = {'people_detector': 'good', 'bicycle_detector': 'stand',
                                 'fire_detector': 'fire', 'trash_bin_detector': 'red_open',
                                 'waste_detector': 'hazardous1'}[name]
                        return SimpleNamespace(detected_classes=[label] if expected == 'SUCCESS' else [],
                            bboxes=[300, 300, 400, 450] if expected == 'SUCCESS' else [],
                            confidences=[0.9] if expected == 'SUCCESS' else [])
                    services.detect.side_effect = detect
                    services.ocr.return_value = SimpleNamespace(plate_texts=['AB123'] if expected == 'SUCCESS' else [])
                    services.count_video.return_value = SimpleNamespace(stand_count=2 if expected == 'SUCCESS' else 0, fall_count=0)
                    if expected == 'FAILED':
                        for method in (services.detect, services.ocr, services.count_video):
                            method.side_effect = RuntimeError('service unavailable')
                    inspection = Inspection({'points': [point]}, services, speech, SETTINGS)
                    record = inspection.execute(point, kind, image if kind != 'vehicle_video' else '/video.mp4')
                    self.assertEqual(set(record), {'waypoint_id', 'task_type', 'status', 'reason',
                                                  'result', 'duration_sec', 'started_at', 'ended_at'})
                    self.assertEqual(record['waypoint_id'], point['id'])
                    self.assertEqual(record['task_type'], kind)
                    self.assertEqual(record['status'], expected)
                    self.assertGreaterEqual(record['duration_sec'], 0)
                    self.assertTrue(record['reason'])
                    if expected == 'FAILED': self.assertIn('service unavailable', record['reason'])

    def test_fire_report_survives_building_ocr_failure(self):
        from inspection import Inspection
        services, speech = Mock(), Mock()
        services.detect.return_value = SimpleNamespace(detected_classes=['fire'],
            bboxes=[0, 0, 10, 10], confidences=[0.9])
        services.ocr.side_effect = RuntimeError('OCR unavailable')
        record = Inspection({'points': []}, services, speech, SETTINGS).execute({'id': 'fire'}, 'fire', object())
        self.assertEqual(record['status'], 'SUCCESS')
        self.assertEqual(record['result']['fire_count'], 1)
        self.assertIn('OCR unavailable', record['reason'])
        speech.put.assert_called_once_with('大厦发现火灾隐患1处')

    def test_area_and_parking_number_are_explicit(self):
        from inspection import Inspection
        point = dict(id='not_an_index', task='person', area='B', parking_lot=3)
        services, speech = Mock(), Mock()
        services.detect.return_value = SimpleNamespace(detected_classes=[], bboxes=[], confidences=[])
        services.ocr.return_value = SimpleNamespace(plate_texts=['AB123'])
        inspection = Inspection({'points': [point]}, services, speech, SETTINGS)
        inspection.inspect_people(point, SimpleNamespace(width=640, height=480))
        inspection.inspect_plate(dict(id='parking', task='plate', parking_lot=3), object())
        self.assertEqual(inspection.people_points, {'not_an_index'})
        self.assertIn('B街人数', speech.put.call_args_list[0].args[0])
        self.assertEqual(inspection.plates, {3: 'AB123'})
        self.assertTrue(speech.put.call_args.args[0].startswith('3号停车场'))


class OdometryTests(unittest.TestCase):
    def test_first_frame_and_linear_motion(self):
        odom = OdometryIntegrator()
        self.assertIsNone(odom.update(1000, 1, 0, 0))
        self.assertEqual(odom.update(1000.5, 1, 0, 0)[:3], (0.5, 0.0, 0.0))

    def test_rotation_scaling_and_bad_intervals(self):
        odom = OdometryIntegrator(scale_x=2)
        odom.update(0, 0, 0, 0)
        odom.update(0.5, 0, 0, math.pi)
        state = odom.update(1.0, 1, 0, 0)
        self.assertAlmostEqual(state[0], 0)
        self.assertAlmostEqual(state[1], 1)
        self.assertIsNone(odom.update(10, 50, 0, 0))
        self.assertIsNone(odom.update(9, 50, 0, 0))
        self.assertIsNone(odom.update(9.1, float('nan'), 0, 0))
        self.assertAlmostEqual(odom.y, 1)


class MissionTests(unittest.TestCase):
    def setUp(self):
        # Load only the production class with explicit test doubles for ROS dependencies.
        tree = ast.parse((ROOT/'src/community_mission/scripts/mission_node.py').read_text(encoding='utf-8'))
        module = ast.Module(body=[n for n in tree.body if isinstance(n, ast.ClassDef)], type_ignores=[])
        self.ros = SimpleNamespace(is_shutdown=lambda: False, sleep=lambda _: None,
                                   loginfo=Mock(), logwarn=Mock(), logerr=Mock(), Duration=lambda x: x)
        self.cv = SimpleNamespace(imwrite=Mock(return_value=True))
        env = dict(rospy=self.ros, cv2=self.cv, time=time, math=math, detections=detections)
        self.env = env
        exec(compile(module, '<production mission class>', 'exec'), env)
        self.node = env['Mission'].__new__(env['Mission'])
        self.node.state, self.node.index = 'INIT', 0
        self.node.runtime = RUNTIME
        self.node.video_path = Path('/offline/videos/video.mp4')
        self.node.stopping = False
        self.node.navigation_only = False
        self.node.obstacle_mode = 'blocked'
        self.node.condition = threading.Condition()
        self.node.navigation_retries, self.node.cancel_timeout = 0, 0.01
        self.node.green_confirm_frames = 3
        self.node.results = Mock()
        self.node.results.execute.side_effect = lambda kind, point, callback, *args, **kwargs: callback(*args)

    def test_point_task_completes_before_next_goal(self):
        n = self.prepare_route()
        n.route = {'points': [dict(id='person', x=0, y=0, yaw=0, task='person', area='A'),
                              dict(id='start', x=0, y=0, yaw=0, task='record_start'),
                              dict(id='stop', x=0, y=0, yaw=0, task='record_stop'),
                              dict(id='last', x=0, y=0, yaw=0, task='navigation')]}
        events = []
        n.move_to.side_effect = lambda _: events.append('move')
        n.inspection.execute.side_effect = lambda p, kind, source: (events.append(kind) or {'status': 'SUCCESS'})
        n.services.start_recording.side_effect = lambda _: events.append('start')
        n.services.stop_recording.side_effect = lambda: events.append('stop')
        n.run()
        self.assertEqual(events, ['move', 'person', 'move', 'start', 'move', 'stop', 'vehicle_video', 'move'])
        self.assertFalse(n.recording)

    def prepare_route(self):
        import json
        n = self.node
        n.stopping = False
        n.route = yaml.safe_load((ROOT/'src/community_mission/config/route.yaml').read_text(encoding='utf-8'))
        n.data_dir, n.planner = Path('/offline'), '/move_base/DWAPlannerROS'
        n.client, n.services, n.speech, n.inspection, n.bridge = Mock(), Mock(), Mock(), Mock(), Mock()
        n.move_to, n.wait_for_green, n.fresh_image = Mock(), Mock(), Mock()
        n.inspection.execute.return_value = {'status': 'SUCCESS'}
        n.transition = Mock(wraps=n.transition)
        return n

    def test_full_route_state_sequence(self):
        n = self.prepare_route()
        self.assertEqual(len(n.route['points']), 19)
        n.run()
        expected = ['INIT', 'NAVIGATE']
        for index in range(19):
            expected.extend(['TASK', 'NEXT_POINT', 'FINISH' if index == 18 else 'NAVIGATE'])
        self.assertEqual([c.args[0] for c in n.transition.call_args_list], expected)
        self.assertEqual([c.args[0] for c in n.move_to.call_args_list], [tuple(p[k] for k in ('x', 'y', 'yaw')) for p in n.route['points']])
        self.assertEqual([c.args[0]['id'] for c in n.inspection.execute.call_args_list if c.args[1] != 'vehicle_video'],
                         ['wp_02', 'wp_03', 'wp_05', 'wp_06', 'wp_07', 'wp_08',
                          'wp_12', 'wp_13', 'wp_14', 'wp_15', 'wp_16'])
        self.assertEqual((n.state, n.index), ('FINISH', 18))
        self.assertEqual(n.wait_for_green.call_count, 2)
        n.speech.drain.assert_called_once()

    def test_navigation_only_runs_original_route_without_tasks_or_services(self):
        n = self.prepare_route()
        n.navigation_only = True
        n.services = n.speech = n.inspection = n.subscriber = None
        n.closed = n.recording = False
        n.task = Mock(wraps=n.task)
        n.run()
        self.assertEqual((n.state, n.index), ('FINISH', 18))
        self.assertEqual([c.args[0] for c in n.move_to.call_args_list],
                         [tuple(p[k] for k in ('x', 'y', 'yaw')) for p in n.route['points']])
        self.assertEqual(n.task.call_count, 19)
        n.wait_for_green.assert_not_called()
        n.fresh_image.assert_not_called()
        self.assertEqual([c.args[0] for c in n.results.execute.call_args_list], ['navigation'] * 19)
        n.close()
        n.client.cancel_all_goals.assert_called_once()

    def test_navigation_only_clear_branch_visits_all_points_in_order(self):
        n = self.prepare_route()
        n.navigation_only, n.obstacle_mode = True, 'clear'
        n.services = n.speech = n.inspection = n.subscriber = None
        n.run()
        visited = [c.args[1]['id'] for c in n.results.execute.call_args_list]
        self.assertEqual(visited, [f'wp_{i:02d}' for i in
                                  (1, 2, 3, 4, 11, 10, 8, 6, 5, 7, 9, 12, 13, 14, 15, 16, 17, 18, 19)])
        self.assertEqual(n.state, 'FINISH')
        n.wait_for_green.assert_not_called()
        n.fresh_image.assert_not_called()

    def test_navigation_only_initializes_without_camera_or_perception_workers(self):
        route = yaml.safe_load((ROOT/'src/community_mission/config/route.yaml').read_text(encoding='utf-8'))
        params = dict(route, mission=RUNTIME)
        self.ros.get_param = lambda key, *default: (
            True if key == '~navigation_only' else params if key == '~' else
            RUNTIME if key == '~mission' else default[0])
        self.ros.on_shutdown = Mock()
        self.ros.Subscriber = Mock(side_effect=AssertionError('Camera must not be subscribed'))
        forbidden = Mock(side_effect=AssertionError('Perception worker must not be created'))
        results = Mock(path=Path('/offline'))
        dependencies = dict(validate_route=validate_route, threading=threading,
                            ResultManager=Mock(return_value=results),
                            actionlib=SimpleNamespace(SimpleActionClient=Mock()),
                            MoveBaseAction=object, Services=forbidden, CvBridge=forbidden,
                            SpeechQueue=forbidden, Inspection=forbidden)
        with patch.dict(self.env, dependencies):
            n = type(self.node)()
            self.assertIsNone(n.speech)
            self.assertIsNone(n.subscriber)
            self.assertIsNone(n.services)
            n.close()
        forbidden.assert_not_called()
        self.ros.Subscriber.assert_not_called()

    def test_navigation_only_stops_at_failed_waypoint(self):
        n = self.prepare_route()
        n.navigation_only = True
        n.services = n.speech = n.inspection = n.subscriber = None
        n.move_to.side_effect = [dict(outcome='SUCCESS'), RuntimeError('NAV_FAILED')]
        with self.assertRaisesRegex(RuntimeError, 'NAV_FAILED'):
            n.run()
        self.assertEqual((n.state, n.index), ('ERROR', 1))
        self.assertEqual(n.move_to.call_count, 2)
        n.client.cancel_goal.assert_called()

    def test_traffic_wait_runs_only_at_configured_point(self):
        n = self.prepare_route()
        n.route = {'points': [dict(id='light', x=0, y=0, yaw=0, task='traffic_light'),
                              dict(id='person', x=1, y=0, yaw=0, task='person', area='A')]}
        events = []
        n.move_to.side_effect = lambda _: events.append('navigate')
        n.wait_for_green.side_effect = lambda: events.append('green')
        n.inspection.execute.side_effect = lambda *args: (events.append('person') or {'status': 'SUCCESS'})
        n.run()
        self.assertEqual(events, ['navigate', 'green', 'navigate', 'person'])

    def test_failures_enter_error_without_advancing_or_retry(self):
        for phase in ('INIT', 'NAVIGATE', 'TRAFFIC_TASK', 'TASK'):
            with self.subTest(phase=phase):
                n = self.prepare_route()
                n.state, n.index = 'INIT', 0
                if phase == 'TASK': n.route['points'][0]['task'] = 'person'
                if phase == 'INIT': n.client.wait_for_server.return_value = False
                elif phase == 'NAVIGATE': n.move_to.side_effect = RuntimeError('navigation timeout')
                elif phase == 'TRAFFIC_TASK': n.wait_for_green.side_effect = RuntimeError('traffic timeout')
                else: n.inspection.execute.return_value = {'status': 'FAILED', 'reason': 'inspection failed'}
                with self.assertRaises(RuntimeError): n.run()
                self.assertEqual((n.state, n.index), ('ERROR', 0))
                self.assertTrue(n.stopping)
                n.client.cancel_goal.assert_called()
                self.assertTrue(n.reason)
                self.assertLessEqual(n.move_to.call_count, 1)
                n.speech.drain.assert_not_called()

    def test_non_traffic_points_never_invoke_green_wait(self):
        n = self.prepare_route()
        n.route = {'points': [dict(id='navigation', x=0, y=0, yaw=0, task='navigation'),
                              dict(id='people', x=1, y=0, yaw=0, task='person', area='A')]}
        n.run()
        n.wait_for_green.assert_not_called()
        n.fresh_image.assert_called_once()
        n.inspection.execute.assert_called_once()

    def test_shutdown_enters_error_before_navigation(self):
        n = self.prepare_route()
        self.ros.is_shutdown = lambda: True
        with self.assertRaisesRegex(RuntimeError, 'Mission stopped'): n.run()
        self.assertEqual(n.state, 'ERROR')
        n.move_to.assert_not_called()

    def test_mission_records_navigation_failure(self):
        n = self.prepare_route()
        with tempfile.TemporaryDirectory() as root:
            n.results = ResultManager(root, n.route['points'], Mock())
            n.move_to.side_effect = RuntimeError('navigation failed')
            with self.assertRaisesRegex(RuntimeError, 'navigation failed'): n.run()
            data = json.loads((n.results.path/'mission_results.json').read_text(encoding='utf-8'))
            self.assertEqual(data['status'], 'ERROR')
            self.assertEqual(data['operations'][0]['task_type'], 'navigation')
            self.assertEqual(data['operations'][0]['status'], 'FAILED')
            self.assertEqual(data['summary']['completed_waypoints'], 0)

    def test_empty_inspection_continues_but_failure_is_saved_before_stop(self):
        from inspection import Inspection
        for failed in (False, True):
            with self.subTest(failed=failed), tempfile.TemporaryDirectory() as root:
                n = self.prepare_route()
                n.state, n.index = 'INIT', 0
                point = dict(id='people', x=0, y=0, yaw=0, task='person', area='A')
                points = [point, dict(id='plate', x=1, y=0, yaw=0, task='plate', parking_lot=1)]
                n.route = {'points': points}
                n.results = ResultManager(root, points, Mock())
                n.data_dir = n.results.path
                n.move_to.return_value = None
                n.fresh_image.return_value = SimpleNamespace(width=640, height=480)
                n.services.detect.return_value = SimpleNamespace(detected_classes=[], bboxes=[], confidences=[])
                n.services.ocr.return_value = SimpleNamespace(plate_texts=[])
                if failed: n.services.detect.side_effect = RuntimeError('detector offline')
                n.inspection = Inspection(n.route, n.services, n.speech, SETTINGS)
                if failed:
                    with self.assertRaisesRegex(RuntimeError, 'detector offline'): n.run()
                    n.services.ocr.assert_not_called()
                else:
                    n.run()
                data = json.loads((n.results.path/'mission_results.json').read_text(encoding='utf-8'))
                self.assertEqual(data['status'], 'ERROR' if failed else 'FINISH')
                self.assertEqual([t['status'] for t in data['operations'] if t['task_type'] != 'navigation'],
                                 ['FAILED'] if failed else ['EMPTY_VALID', 'EMPTY_VALID'])

    def test_transition_logs_state_point_and_reason(self):
        n = self.prepare_route()
        n.index = 14
        n.transition('TASK', 'arrived')
        self.assertEqual((n.state, n.reason), ('TASK', 'arrived'))
        n.results.state.assert_called_with('TASK', n.route['points'][14], 'arrived')

    def test_stale_camera_frame_is_not_reused(self):
        n = self.node
        n.condition, n.sequence, n.latest, n.image_timeout = threading.Condition(), 1, 'old', 0.01
        with self.assertRaises(RuntimeError): n.fresh_image()
        timer = threading.Timer(0.02, n.image_callback, args=('new',))
        timer.start()
        self.assertEqual(n.fresh_image(timeout=0.5), 'new')
        timer.join()

    def test_shutdown_cancels_navigation_and_stops_recording(self):
        n = self.node
        n.closed, n.recording = False, True
        n.client, n.services, n.subscriber, n.speech = Mock(), Mock(), Mock(), Mock()
        n.close()
        n.close()
        n.client.cancel_all_goals.assert_called_once()
        n.services.stop_recording.assert_called_once()
        n.speech.close.assert_called_once()

    def test_navigation_terminal_states(self):
        n = self.node
        self.env['GoalStatus'] = SimpleNamespace(PENDING=0, ACTIVE=1, PREEMPTED=2,
            SUCCEEDED=3, ABORTED=4, REJECTED=5, PREEMPTING=6, RECALLING=7, RECALLED=8, LOST=9)
        self.env['math'] = math
        self.env['MoveBaseGoal'] = lambda: SimpleNamespace(target_pose=SimpleNamespace(
            header=SimpleNamespace(), pose=SimpleNamespace(position=SimpleNamespace(),
                                                           orientation=SimpleNamespace())))
        self.ros.Time = SimpleNamespace(now=lambda: 123)
        n.client, n.goal_timeout = Mock(), 0.001
        n.client.get_state.return_value = 3
        n.move_to((1, 2, math.pi))
        goal = n.client.send_goal.call_args.args[0]
        self.assertAlmostEqual(goal.target_pose.pose.orientation.z, 1)
        n.client.get_state.return_value = 4
        with self.assertRaisesRegex(RuntimeError, 'Navigation failed'):
            n.move_to((1, 2, 0))
        n.stopping = False
        n.client.cancel_goal.reset_mock()
        n.client.get_state.return_value = 1
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            n.move_to((1, 2, 0))
        n.client.cancel_goal.assert_called_once()

    def test_navigation_retries_only_after_cancel_confirmation(self):
        # Reuse goal construction fixtures from the terminal-state regression.
        self.test_navigation_terminal_states()
        n = self.node
        n.stopping = False
        n.navigation_retries = 1
        n.client.reset_mock()
        n.client.get_state.side_effect = [4, 4, 3]  # aborted, terminal confirmation, success
        result = n.move_to((0, 0, 0))
        self.assertEqual(result['attempts'], 2)
        self.assertEqual(n.client.send_goal.call_count, 2)
        n.client.cancel_goal.assert_called_once()
        calls = [c[0] for c in n.client.mock_calls]
        self.assertLess(calls.index('cancel_goal'), len(calls)-1-calls[::-1].index('send_goal'))

    def test_external_cancel_and_lost_never_retry(self):
        self.test_navigation_terminal_states()
        n = self.node
        n.navigation_retries = 2
        for state in (2, 6, 7, 8, 9):
            with self.subTest(state=state):
                n.stopping = False
                n.client.reset_mock()
                n.client.get_state.return_value = state
                with self.assertRaisesRegex(RuntimeError, 'NAV_CANCELLED|NAV_LOST'): n.move_to((0, 0, 0))
                n.client.send_goal.assert_called_once()
                self.assertTrue(n.stopping)
                with self.assertRaisesRegex(RuntimeError, 'CANCELLED'): n.move_to((0, 0, 0))
                n.client.send_goal.assert_called_once()

    def test_retry_refused_when_cancel_ack_times_out(self):
        self.test_navigation_terminal_states()
        n = self.node
        n.stopping, n.navigation_retries = False, 1
        n.client.reset_mock()
        n.client.get_state.return_value = 1
        with self.assertRaisesRegex(RuntimeError, 'CANCEL_TIMEOUT'): n.move_to((0, 0, 0))
        n.client.send_goal.assert_called_once()
        self.assertTrue(n.stopping)

    def test_navigation_retry_budget_is_finite(self):
        self.test_navigation_terminal_states()
        n = self.node
        n.stopping, n.navigation_retries = False, 2
        n.client.reset_mock()
        n.client.get_state.return_value = 4
        with self.assertRaisesRegex(RuntimeError, 'NAV_FAILED'): n.move_to((0, 0, 0))
        self.assertEqual(n.client.send_goal.call_count, 3)
        self.assertTrue(n.stopping)

    def test_green_count_parameter_and_invalid_detections(self):
        n = self.node
        n.light_timeout, n.image_timeout = 1.0, 0.1
        for labels, confidence in [([], []), (['yellow_light'], [0.9]),
                                  (['green_light', 'red_light'], [0.9, 0.9]),
                                  (['green_light'], [float('nan')])]:
            with self.subTest(labels=labels, confidence=confidence):
                stamps = iter(range(1, 100))
                n.fresh_image = lambda *args: SimpleNamespace(header=SimpleNamespace(
                    stamp=SimpleNamespace(to_nsec=lambda: next(stamps))))
                green = SimpleNamespace(detected_classes=['green_light'], bboxes=[0, 0, 10, 10], confidences=[0.9])
                invalid = SimpleNamespace(detected_classes=labels, bboxes=[0, 0, 10, 10]*len(labels), confidences=confidence)
                n.services = Mock()
                n.services.detect.side_effect = [green, invalid, green, green, green]
                self.assertTrue(n.wait_for_green()['released'])
                self.assertEqual(n.services.detect.call_count, 5)
        n.green_confirm_frames = 1
        n.services.detect.side_effect = None
        n.services.detect.return_value = green
        n.services.detect.reset_mock()
        self.assertEqual(n.wait_for_green()['green_streak'], 1)
        n.services.detect.assert_called_once()

    def test_green_confirmation_resets_on_non_green_and_duplicate_frames(self):
        n = self.node
        n.light_timeout, n.image_timeout = 1.0, 0.1
        stamps = iter([1, 2, 3, 4, 4, 5, 6, 7])
        n.fresh_image = lambda *args: SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(to_nsec=lambda: next(stamps))))
        labels = iter(['green_light', 'red_light', 'green_light', 'green_light',
                       'green_light', 'green_light', 'green_light'])
        n.services = Mock()
        n.services.detect.side_effect = lambda *args: SimpleNamespace(
            detected_classes=[next(labels)], bboxes=[0, 0, 10, 10], confidences=[0.9])
        result = n.wait_for_green()
        self.assertEqual(result['green_streak'], 3)
        self.assertEqual(n.services.detect.call_count, 7)

    def test_late_green_and_service_failure_do_not_release(self):
        n = self.node
        n.light_timeout, n.image_timeout, n.green_confirm_frames = 0.001, 1.0, 1
        n.fresh_image = lambda *args: SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(to_nsec=lambda: 1)))
        n.services = Mock()
        def late(*args):
            time.sleep(0.005)
            return SimpleNamespace(detected_classes=['green_light'], bboxes=[0, 0, 10, 10], confidences=[0.9])
        n.services.detect.side_effect = late
        with self.assertRaisesRegex(RuntimeError, 'TRAFFIC_TIMEOUT'): n.wait_for_green()
        n.light_timeout = 1.0
        n.services.detect.side_effect = RuntimeError('detector unavailable')
        with self.assertRaisesRegex(RuntimeError, 'detector unavailable'): n.wait_for_green()

    def test_traffic_light_requires_green_detection(self):
        n = self.node
        n.light_timeout, n.image_timeout = 0.1, 0.1
        stamps = iter(range(1, 100000))
        n.fresh_image = Mock(side_effect=lambda *args: SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(to_nsec=lambda: next(stamps)))))
        n.services = Mock()
        n.services.detect.return_value = SimpleNamespace(detected_classes=['green_light'],
            bboxes=[0, 0, 10, 10], confidences=[0.9])
        n.wait_for_green()
        n.services.detect.return_value = SimpleNamespace(detected_classes=[], bboxes=[], confidences=[])
        n.light_timeout = 0.001
        with self.assertRaisesRegex(RuntimeError, 'Traffic light timeout'):
            n.wait_for_green()


if __name__ == '__main__':
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__]))
    report = ('# Python 迁移离线回归\n\n'
              f'- 用例数：{result.testsRun}。\n- 失败：{len(result.failures)}；错误：{len(result.errors)}。\n'
              '- 覆盖：逐目标响应解析、ROI 缩放、违停框去重、垃圾投放、多照片车辆去重、'
              '播报计数、配置校验、首帧/异常时间/旋转里程计、到点任务顺序、图像新鲜度、'
              '导航成功/失败/超时、红绿灯条件、26 航点状态转换、单航点单任务及红绿灯按配置执行、错误终止、退出清理、结果目录隔离、任务记录、异常汇总、写入失败、六类识别任务统一返回结构、空结果和服务异常区分、连续绿灯与重复帧、导航取消确认及有限重试、YAML 模型配置/启动覆盖与规则参数读取。\n'
              '- 任务编排测试使用 ROS/服务测试替身；未启动 ROS，未加载模型，未连接硬件，未编译。\n\n'
              '复现：`python3 tools/test_python_migration.py`。\n')
    (ROOT/'docs/PYTHON_TESTS.md').write_text(report, encoding='utf-8')
    raise SystemExit(not result.wasSuccessful())
