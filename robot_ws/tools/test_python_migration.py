#!/usr/bin/env python3
"""Offline regression fixtures: business rules, odometry and mission ordering."""
import ast
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src/community_mission/scripts')]
from rules import (Detection, detections, people_counts, illegal_parking_count,
    vehicle_counts, trash_announcement, vehicle_announcement, validate_route)
# Extract the pure integrator from the combined node without importing ROS.
_tree = ast.parse((ROOT/'src/community_base/scripts/odometry_node.py').read_text(encoding='utf-8'))
_integrator = ast.Module(body=[n for n in _tree.body if isinstance(n, ast.ClassDef)
                               and n.name == 'OdometryIntegrator'], type_ignores=[])
_env = {'math': math}
exec(compile(_integrator, '<production odometry integrator>', 'exec'), _env)
OdometryIntegrator = _env['OdometryIntegrator']


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

    def test_route_rejects_invalid_indices_and_nonfinite_coordinates(self):
        import json
        route = json.loads((ROOT/'src/community_mission/config/route.yaml').read_text(encoding='utf-8'))
        validate_route(route)
        route['record_stop_index'] = 999
        with self.assertRaises(ValueError): validate_route(route)
        route['record_stop_index'] = 23
        route['goals'][0][0] = float('nan')
        with self.assertRaises(ValueError): validate_route(route)


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
                                   logwarn=Mock(), logerr=Mock(), Duration=lambda x: x)
        self.cv = SimpleNamespace(imwrite=Mock(return_value=True))
        env = dict(rospy=self.ros, cv2=self.cv, time=time, detections=detections)
        self.env = env
        exec(compile(module, '<production mission class>', 'exec'), env)
        self.node = env['Mission'].__new__(env['Mission'])

    def test_point_tasks_complete_before_next_goal(self):
        n = self.node
        route = dict(goals=[(0, 0, 0)]*3, traffic_light_idx=[], photo_idx=[0],
                     license_plate_idx=[], fire_idx=[], trash_can_idx=[], electrocar_idx=[],
                     announcement_index=0, record_start_index=0, record_stop_index=1)
        n.route, n.data_dir, n.planner = route, Path('/offline'), '/move_base/DWAPlannerROS'
        n.client, n.services, n.speech, n.inspection, n.bridge = Mock(), Mock(), Mock(), Mock(), Mock()
        events = []
        n.move_to = lambda _: events.append('move')
        n.fresh_image = lambda: object()
        n.inspection.run_at.side_effect = lambda i, frame: events.append(f'inspect{i}')
        n.services.start_recording.side_effect = lambda _: events.append('record_start')
        n.services.stop_recording.side_effect = lambda: events.append('record_stop')
        n.inspection.inspect_video.side_effect = lambda _: events.append('count_video')
        n.run()
        self.assertEqual(events, ['move', 'inspect0', 'record_start', 'move', 'record_stop', 'count_video', 'move'])
        self.assertFalse(n.recording)

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
            SUCCEEDED=3, ABORTED=4, PREEMPTING=6, RECALLING=7)
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
        n.client.get_state.return_value = 1
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            n.move_to((1, 2, 0))
        n.client.cancel_goal.assert_called_once()

    def test_traffic_light_requires_green_detection(self):
        n = self.node
        n.light_timeout, n.image_timeout = 0.1, 0.1
        n.fresh_image = Mock(return_value='frame')
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
              '导航成功/失败/超时、红绿灯条件和退出清理。\n'
              '- 任务编排测试使用 ROS/服务测试替身；未启动 ROS，未加载模型，未连接硬件，未编译。\n\n'
              '复现：`python3 tools/test_python_migration.py`。\n')
    (ROOT/'docs/PYTHON_TESTS.md').write_text(report, encoding='utf-8')
    raise SystemExit(not result.wasSuccessful())
