#!/usr/bin/env python3
"""Check task output and saved results agree, without ROS or hardware."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/community_mission/scripts'))
from result_manager import ResultManager, task_summary


class MissionOutputTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.lines = []
        self.point = {'id': 'wp_06', 'task': 'person'}
        self.manager = ResultManager(self.directory.name, [self.point], self.lines.append)
        self.manager.data['task_only'] = True

    def test_success_result_evidence_and_final_paths(self):
        manager = self.manager
        photo = manager.path / 'photo/005.jpg'
        photo.write_bytes(b'fixture')
        manager.waypoint(self.point, 'RUNNING')
        result = {'area': 'A', 'people': 3, 'outside': 1, 'illegal_parking': 2}
        manager.execute('person', self.point, lambda: result, paths=[photo])
        manager.waypoint(self.point, 'SUCCESS')
        manager.finish('FINISH')
        text = '\n'.join(self.lines)
        self.assertIn('[任务开始] 航点=wp_06 类型=person', text)
        self.assertIn('状态=SUCCESS', text)
        self.assertIn('区域=A 人数=3 外来人员=1 违停=2', text)
        self.assertIn('[任务证据] 航点=wp_06 文件=' + str(photo), text)
        self.assertIn('[巡检结束] 状态=FINISH 模式=仅任务 完成航点=1', text)
        self.assertIn('[结果文件] ' + str(manager.path / 'mission_results.json'), text)
        self.assertIn('[事件日志] ' + str(manager.path / 'mission.log'), text)
        saved = json.loads((manager.path / 'mission_results.json').read_text())
        self.assertEqual(saved['operations'][0]['result'], result)
        for line in self.lines:
            self.assertIn(line, (manager.path / 'mission.log').read_text())

    def test_empty_result_is_valid_and_counted_separately(self):
        self.manager.record_task(dict(waypoint_id='wp_06', task_type='person', status='EMPTY_VALID',
                                      reason='未检测到有效目标', duration_sec=.2,
                                      result=dict(area='A', people=0, outside=0, illegal_parking=0)))
        self.manager.finish('FINISH')
        text = '\n'.join(self.lines)
        self.assertIn('状态=EMPTY_VALID', text)
        self.assertIn('任务成功=1（其中空结果=1） 任务失败=0', text)
        self.assertEqual(self.manager.data['planned_tasks'][0]['status'], 'EMPTY_VALID')

    def test_failure_is_not_hidden_or_converted_to_success(self):
        def fail():
            raise RuntimeError('detector unavailable\nconnection refused')
        self.manager.waypoint(self.point, 'RUNNING')
        with self.assertRaisesRegex(RuntimeError, 'detector unavailable'):
            self.manager.execute('person', self.point, fail)
        self.manager.finish('ERROR', 'detector unavailable\nconnection refused')
        text = '\n'.join(self.lines)
        self.assertIn('状态=FAILED', text)
        self.assertIn('结果=detector unavailable connection refused', text)
        self.assertIn('[巡检结束] 状态=ERROR', text)
        self.assertIn('任务失败=1', text)
        self.assertEqual(self.manager.data['operations'][0]['reason'],
                         'detector unavailable\nconnection refused')
        self.assertTrue(all('\n' not in line for line in self.lines))

    def test_missing_evidence_is_not_reported_as_saved(self):
        missing = self.manager.path / 'photo/missing.jpg'
        self.manager.execute('person', self.point, lambda: {}, paths=[missing])
        text = '\n'.join(self.lines)
        self.assertIn('[证据未生成]', text)
        self.assertNotIn('[任务证据]', text)

    def test_navigation_is_distinct_and_not_counted_as_task(self):
        self.manager.execute('navigation', self.point, lambda: {'outcome': 'SUCCESS', 'attempts': 1})
        self.manager.finish('FINISH')
        self.assertTrue(self.lines[0].startswith('[导航开始]'))
        self.assertTrue(self.lines[1].startswith('[导航结果]'))
        self.assertEqual(self.manager.data['summary']['successful_tasks'], 0)

    def test_compact_summary_does_not_drop_saved_details(self):
        result = {'raw_boxes': list(range(200))}
        self.manager.execute('unknown', self.point, lambda: result)
        self.assertIn('完整数据见结果文件', self.lines[-1])
        self.assertEqual(self.manager.data['operations'][0]['result'], result)
        self.assertIn('录像已开始', task_summary('record_start', None))
        self.assertIn('车牌=未检出', task_summary('plate', {'parking_lot': 1, 'plate': ''}))


if __name__ == '__main__':
    unittest.main()
