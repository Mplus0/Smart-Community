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
from contextlib import nullcontext

import yaml

WORKSPACE = Path(__file__).resolve().parents[1]
SCRIPTS = WORKSPACE / 'src/community_mission/scripts'
sys.path.insert(0, str(SCRIPTS))
from rules import validate_route, obstacle_route, Detection, detection_records
from result_manager import ResultManager


class MissionRouteTests(unittest.TestCase):
    def setUp(self):
        self.route = yaml.safe_load((SCRIPTS.parent / 'config/route.yaml').read_text(encoding='utf-8'))

    def method(self, name, **scope):
        tree = ast.parse((SCRIPTS / 'mission_node.py').read_text(encoding='utf-8'))
        mission = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'Mission')
        method = next(node for node in mission.body if isinstance(node, ast.FunctionDef) and node.name == name)
        namespace = dict(rospy=SimpleNamespace(loginfo=lambda *args: None),
                         detection_records=detection_records)
        namespace.update(scope)
        exec(compile(ast.Module(body=[method], type_ignores=[]), 'mission_node.py', 'exec'), namespace)
        return namespace[name]

    def test_new_clear_route_tasks_and_recording(self):
        route = yaml.safe_load((SCRIPTS.parent / 'config/route_clear.yaml').read_text())
        validate_route(route)
        self.assertNotIn('navigation_branches', route)
        self.assertEqual([p['id'] for p in route['points']],
                         [f'wp_{i:02d}' for i in range(1, 40)])
        expected = {
            'traffic_light': ['wp_01', 'wp_24'],
            'person': ['wp_02', 'wp_06', 'wp_07', 'wp_14', 'wp_17', 'wp_18'],
            'fire': ['wp_19', 'wp_22', 'wp_34'],
            'plate': ['wp_27', 'wp_28', 'wp_31'], 'trash': ['wp_29'],
            'record_start': ['wp_35'], 'record_stop': ['wp_36'], 'obstacle': []}
        for task, ids in expected.items():
            self.assertEqual([p['id'] for p in route['points'] if p['task'] == task], ids)
        self.assertEqual([p['area'] for p in route['points'] if p['task'] == 'person'],
                         ['A', 'A', 'B', 'B', 'B', 'A'])
        self.assertEqual([p['parking_lot'] for p in route['points'] if p['task'] == 'plate'], [1, 2, 3])
        self.assertEqual(route['points'][34]['y'], 2.3729)
        self.assertEqual(route['points'][35]['y'], 0.8364)

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
        self.assertEqual(len(route['points']), 38)
        start = next(i for i, p in enumerate(route['points'])
                     if p['task'] == 'plate' and p['parking_lot'] == 1)
        common = next(i for i, p in enumerate(clear['points'])
                      if p['task'] == 'plate' and p['parking_lot'] == 1)
        self.assertEqual(route['points'][start:], clear['points'][common:])
        by_id = {p['id']: p for p in route['points']}
        self.assertEqual(by_id['blocked_wp_01']['task'], 'navigation')
        self.assertEqual(by_id['blocked_wp_13']['task'], 'navigation')
        self.assertEqual(by_id['blocked_wp_15']['task'], 'navigation')
        self.assertEqual(by_id['blocked_wp_08']['area'], 'B')
        self.assertEqual(by_id['blocked_wp_08']['yaw'], 1.6255)
        self.assertEqual(by_id['blocked_wp_14']['area'], 'B')
        self.assertEqual(by_id['blocked_wp_14']['yaw'], -0.0514)
        self.assertEqual(by_id['blocked_wp_10']['task'], 'fire')
        self.assertEqual(by_id['blocked_wp_10']['x'], 2.4101)
        self.assertNotIn('traffic_light', [p['task'] for p in route['points']])
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
            check_running=lambda: None, obstacle_clear_x_ratio=0.35)
        select = self.method('select_obstacle_route', detections=lambda objects: objects,
                             obstacle_route=lambda *args: result)
        self.assertEqual(select(obj, image), result)
        self.assertEqual(obj.obstacle_mode, 'clear')

    def test_both_branches_run_every_task_once_and_finish(self):
        validate_route(self.route)
        points = self.route['points']
        split = next(i for i, p in enumerate(points) if p['task'] == 'obstacle')
        by_id = {p['id']: p for p in points}
        for mode, count in [('clear', 39), ('blocked', 38)]:
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
        self.assertEqual(len(obj.route['points']), 38)

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
        self.assertEqual(suffixes[0], [f'wp_{i:02d}' for i in range(27, 40)])

    def test_recording_is_paired_and_uninterrupted_in_both_branches(self):
        for mode, order in self.route['navigation_branches'].items():
            with self.subTest(mode=mode):
                self.assertEqual(order[order.index('wp_35') + 1], 'wp_36')
                self.assertEqual(order[-1], 'wp_39')

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
        first, last = branch.index('wp_35'), branch.index('wp_36')
        branch[first], branch[last] = branch[last], branch[first]
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
                     'start_recording', 'stop_recording', 'finish_recording',
                     'stop_navigation', 'close', 'run']:
            method = self.method(name, cv2=SimpleNamespace(imwrite=lambda *a: True))
            obj.__dict__[name] = MethodType(method, obj)
        return obj

    def test_task_only_runs_real_task_handlers_in_both_branches(self):
        for mode, count in [('clear', 39), ('blocked', 38)]:
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as folder:
                obj = self.task_only_mission(folder, mode)
                obj.run()
                obj.close()
                self.assertEqual(obj.state, 'FINISH')
                operations = obj.results.data['operations']
                expected = [(p['id'], p['task']) for p in obj.route['points']
                            if p['task'] != 'navigation']
                # Stopping recording also performs vehicle-video inspection.
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
