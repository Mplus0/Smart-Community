"""Offline obstacle decisions and production mission branch integration."""
import unittest
from types import SimpleNamespace

from tools import test_python_migration as fixtures
from rules import Detection, obstacle_route, validate_route


class ObstacleRouteTests(unittest.TestCase):
    def test_sample_positions_and_resolution_scaling(self):
        for box, expected in (((146, 6, 533, 560), 'clear'),
                              ((866, 178, 1261, 558), 'blocked')):
            for scale in (1, 1 / 3):
                result = obstacle_route([Detection('obstacle', tuple(v * scale for v in box), 0.96)],
                                        1920 * scale, 0.35)
                self.assertEqual(result['mode'], expected)
        self.assertEqual(obstacle_route([Detection('obstacle', (30, 0, 40, 10), 0.9)],
                                        100, 0.35)['mode'], 'blocked')

    def test_empty_multiple_and_invalid_detections_fail(self):
        valid = Detection('obstacle', (10, 10, 30, 30), 0.9)
        for objects in ([], [valid, valid], [Detection('other', valid.box, 0.9)],
                        [Detection('obstacle', valid.box, float('nan'))],
                        [Detection('obstacle', (30, 10, 10, 30), 0.9)]):
            with self.subTest(objects=objects), self.assertRaises(ValueError):
                obstacle_route(objects, 100, 0.35)

    def prepare_mission(self, center_ratio):
        fixture = fixtures.MissionTests()
        fixture.setUp()
        fixture.env['obstacle_route'] = obstacle_route
        node = fixture.prepare_route()
        node.obstacle_clear_x_ratio = 0.35
        node.fresh_image.return_value = SimpleNamespace(width=1000, height=600)
        node.services.detect.return_value = SimpleNamespace(
            detected_classes=['obstacle'], confidences=[0.95],
            bboxes=[center_ratio * 1000 - 20, 100, center_ratio * 1000 + 20, 200])
        return node

    def test_full_mission_selects_branch_and_keeps_tasks(self):
        for center, mode in ((0.18, 'clear'), (0.55, 'blocked')):
            with self.subTest(mode=mode):
                node = self.prepare_mission(center)
                validate_route(node.route)
                original = {p['id']: p for p in node.route['points']}
                expected = [f'wp_{i:02d}' for i in range(1, 5)] + node.route['navigation_branches'][mode]
                node.run()
                visits = [call.args[1]['id'] for call in node.results.execute.call_args_list
                          if call.args[0] == 'navigation']
                self.assertEqual(visits, expected)
                self.assertEqual(len(set(visits)), 21)
                self.assertEqual(node.obstacle_mode, mode)
                self.assertEqual(node.state, 'FINISH')
                node.services.detect.assert_called_once_with('obstacle_detector', node.fresh_image.return_value)
                self.assertEqual(node.wait_for_green.call_count, 2)
                for call in node.inspection.execute.call_args_list:
                    point, kind = call.args[:2]
                    self.assertEqual(point, original[point['id']])
                    self.assertIn(kind, (point['task'], 'vehicle_video'))
                node.services.start_recording.assert_called_once()
                node.services.stop_recording.assert_called_once()

    def test_detection_failure_stops_at_wp04(self):
        for service_error in (False, True):
            with self.subTest(service_error=service_error):
                node = self.prepare_mission(0.18)
                node.services.detect.return_value = SimpleNamespace(
                    detected_classes=[], confidences=[], bboxes=[])
                if service_error:
                    node.services.detect.side_effect = RuntimeError('detector unavailable')
                with self.assertRaises((ValueError, RuntimeError)):
                    node.run()
                self.assertEqual(node.state, 'ERROR')
                self.assertEqual(node.index, 3)
                self.assertEqual(node.move_to.call_count, 4)
                node.client.cancel_goal.assert_called()

    def test_obstacle_task_requires_branch_configuration(self):
        node = self.prepare_mission(0.18)
        del node.route['navigation_branches']
        with self.assertRaisesRegex(ValueError, 'navigation_branches'):
            validate_route(node.route)

    def test_navigation_only_keeps_manual_branches_without_camera(self):
        for mode in ('clear', 'blocked'):
            with self.subTest(mode=mode):
                node = self.prepare_mission(0.18)
                node.navigation_only, node.obstacle_mode = True, mode
                node.run()
                visits = [call.args[1]['id'] for call in node.results.execute.call_args_list
                          if call.args[0] == 'navigation']
                self.assertEqual(visits, [f'wp_{i:02d}' for i in range(1, 5)]
                                 + node.route['navigation_branches'][mode])
                node.fresh_image.assert_not_called()
                node.services.detect.assert_not_called()
                node.inspection.execute.assert_not_called()


if __name__ == '__main__':
    unittest.main()
