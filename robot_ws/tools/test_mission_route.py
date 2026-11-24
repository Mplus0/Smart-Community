#!/usr/bin/env python3
"""Check route selection and navigation completion without ROS or robot hardware."""
import ast
import copy
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from contextlib import nullcontext

import yaml

WORKSPACE = Path(__file__).resolve().parents[1]
SCRIPTS = WORKSPACE / 'src/community_mission/scripts'
sys.path.insert(0, str(SCRIPTS))
from rules import validate_route


class MissionRouteTests(unittest.TestCase):
    def setUp(self):
        self.route = yaml.safe_load((SCRIPTS.parent / 'config/route.yaml').read_text(encoding='utf-8'))

    def method(self, name, **scope):
        tree = ast.parse((SCRIPTS / 'mission_node.py').read_text(encoding='utf-8'))
        mission = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'Mission')
        method = next(node for node in mission.body if isinstance(node, ast.FunctionDef) and node.name == name)
        namespace = dict(rospy=SimpleNamespace(loginfo=lambda *args: None), **scope)
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

    def test_initialize_single_map_ignores_old_closures(self):
        transitions = []
        obj = SimpleNamespace(
            route={'navigation_closures': {'clear': {}}},
            runtime={'server_timeout': 30.0}, navigation_only=True,
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
        expected = {p['id'] for p in points if p['task'] != 'navigation'}
        next_point = self.method('next_point')
        for mode, count in [('clear', 45), ('blocked', 57)]:
            for navigation_only in (True, False):
                with self.subTest(mode=mode, navigation_only=navigation_only):
                    obj = SimpleNamespace(route=copy.deepcopy(self.route), index=split,
                                          single_waypoint='', obstacle_mode=mode,
                                          navigation_only=navigation_only,
                                          transition=lambda *args: None)
                    next_point(obj)
                    selected = obj.route['points']
                    self.assertEqual(len(selected), count)
                    self.assertEqual([p['id'] for p in selected[split+1:]],
                                     self.route['navigation_branches'][mode])
                    tasks = [p['id'] for p in selected if p['task'] != 'navigation']
                    self.assertEqual(set(tasks), expected)
                    self.assertEqual(len(tasks), len(expected))
                    self.assertEqual(selected[-1]['id'], 'wp_41')

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
        self.assertEqual(obj.route['points'][5]['id'], 'wp_05')
        self.assertEqual(len(obj.route['points']), 57)

    def test_translation_legs_face_travel_except_clear_recording_area(self):
        points = self.route['points']
        split = next(i for i, p in enumerate(points) if p['task'] == 'obstacle')
        by_id = {p['id']: p for p in points}
        for mode, order in self.route['navigation_branches'].items():
            selected = points[:split + 1] + [by_id[name] for name in order]
            for start, end in zip(selected, selected[1:]):
                if mode == 'clear' and end['id'] in ('wp_36', 'wp_37'):
                    continue
                dx, dy = end['x'] - start['x'], end['y'] - start['y']
                if math.hypot(dx, dy) <= .001:
                    continue
                direction = math.atan2(dy, dx)
                with self.subTest(mode=mode, start=start['id'], end=end['id']):
                    for point in (start, end):
                        delta = point['yaw'] - direction
                        self.assertLess(abs(math.atan2(math.sin(delta), math.cos(delta))), .001)

    def test_clear_uses_direct_rotations_and_uninterrupted_recording(self):
        order = self.route['navigation_branches']['clear']
        self.assertEqual(order[order.index('wp_26') + 1], 'wp_28')
        self.assertNotIn('wp_34', order)
        self.assertNotIn('wp_35', order)
        self.assertEqual(order[order.index('wp_33') + 1:order.index('wp_36')],
                         ['via_turn_wp_33_to_wp_36'])
        self.assertEqual(order[order.index('wp_36') + 1], 'wp_37')
        by_id = {p['id']: p for p in self.route['points']}
        for first, second in [('wp_26', 'wp_28')]:
            self.assertEqual((by_id[first]['x'], by_id[first]['y']),
                             (by_id[second]['x'], by_id[second]['y']))
            delta = by_id[first]['yaw'] - by_id[second]['yaw']
            self.assertAlmostEqual(abs(math.atan2(math.sin(delta), math.cos(delta))),
                                   math.pi, places=4)
        self.assertEqual(by_id['wp_36']['x'], by_id['wp_37']['x'])
        self.assertEqual(by_id['wp_36']['yaw'], by_id['wp_37']['yaw'])

    def test_invalid_branch_configurations_are_rejected(self):
        mutations = []
        route = copy.deepcopy(self.route)
        route['navigation_branches']['blocked'].remove('wp_18')
        mutations.append(route)
        route = copy.deepcopy(self.route)
        route['navigation_branches']['blocked'].append('wp_41')
        mutations.append(route)
        route = copy.deepcopy(self.route)
        route['navigation_branches']['clear'][-1] = 'wp_40'
        mutations.append(route)
        route = copy.deepcopy(self.route)
        route['navigation_branches']['blocked'][0] = 'missing'
        mutations.append(route)
        route = copy.deepcopy(self.route)
        branch = route['navigation_branches']['blocked']
        first, last = branch.index('wp_36'), branch.index('wp_37')
        branch[first], branch[last] = branch[last], branch[first]
        mutations.append(route)
        route = copy.deepcopy(self.route)
        next(p for p in route['points'] if p['id'] == 'wp_07')['task'] = 'obstacle'
        mutations.append(route)
        for index, route in enumerate(mutations):
            with self.subTest(index=index), self.assertRaises(ValueError):
                validate_route(route)

    def test_single_waypoint_does_not_expand_branch(self):
        events = []
        obj = SimpleNamespace(route=self.route, index=4, single_waypoint='wp_04_obstacle',
                              transition=lambda state, reason: events.append(state))
        self.method('next_point')(obj)
        self.assertEqual(events, ['FINISH'])
        self.assertEqual(obj.index, 4)

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
                obj = SimpleNamespace(navigation_retries=0, goal_timeout=timeout,
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
