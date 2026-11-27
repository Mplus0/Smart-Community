#!/usr/bin/env python3
"""Exercise real HTTP/multipart reporting and failure isolation without ROS."""
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/community_mission/scripts'))
from judge_client import JudgeClient
from result_manager import ResultManager


class JudgeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.requests = []
        self.status = 200
        self.gate = threading.Event()
        self.gate.set()
        owner = self
        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                body = self.rfile.read(int(self.headers['Content-Length']))
                owner.requests.append((self.path, self.headers['Content-Type'], body))
                owner.gate.wait(2)
                self.send_response(owner.status)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({'ok': owner.status == 200}).encode())
            def log_message(self, *args):
                pass
        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)
        self.url = f'http://127.0.0.1:{self.server.server_port}'
        self.client = JudgeClient(self.url, '测试队', self.root, lambda *args: None, timeout=1)
        self.addCleanup(self.client.close)

    def record(self, kind, result, **kwargs):
        return dict(task_type=kind, result=result, status='SUCCESS', waypoint_id='wp_1', paths=[], **kwargs)

    def test_real_http_hello_then_utf8_multipart_annotated_image(self):
        raw, annotated = self.root/'raw.jpg', self.root/'raw_plate_annotated.jpg'
        raw.write_bytes(b'RAW')
        annotated.write_bytes(b'ANNOTATED')
        record = self.record('plate', {'parking_lot': 2, 'plate': '苏A12345'})
        record['paths'] = [str(raw), str(annotated)]
        self.client.submit(record)
        self.client.close()
        self.assertEqual([r[0] for r in self.requests], ['/api/hello', '/api/report'])
        self.assertEqual(json.loads(self.requests[0][2]), {'team': '测试队'})
        body = self.requests[1][2]
        self.assertIn('苏A12345'.encode(), body)
        self.assertIn('2号停车位'.encode(), body)
        self.assertIn(b'name="images"', body)
        self.assertIn(b'ANNOTATED', body)
        self.assertNotIn(b'RAW', body)
        self.assertEqual(json.loads(self.client.journal.read_text())['status'], 'sent')

    def test_failure_does_not_change_persisted_task_result(self):
        self.status = 403
        manager = ResultManager(self.root, [{'id': 'wp_1', 'task': 'plate'}], lambda *args: None)
        manager.on_result = self.client.submit
        result = {'parking_lot': 1, 'plate': '京A12345'}
        self.assertEqual(manager.execute('plate', {'id': 'wp_1'}, lambda: result), result)
        self.client.close()
        saved = json.loads((manager.path/'mission_results.json').read_text())
        self.assertEqual(saved['operations'][0]['status'], 'SUCCESS')
        self.assertEqual(saved['operations'][0]['result'], result)
        journal = json.loads(self.client.journal.read_text())
        self.assertEqual(journal['status'], 'failed')
        self.assertIn('403', journal['error'])

    def test_slow_network_does_not_block_submit(self):
        self.gate.clear()
        try:
            started = time.monotonic()
            self.client.submit(self.record('plate', {'parking_lot': 1, 'plate': 'A'}))
            self.assertLess(time.monotonic()-started, .1)
        finally:
            self.gate.set()

    def test_callback_failure_keeps_recognition_success(self):
        manager = ResultManager(self.root, [{'id': 'wp_1', 'task': 'plate'}], lambda *args: None)
        def fail(record):
            raise RuntimeError('queue unavailable')
        manager.on_result = fail
        self.assertEqual(manager.execute('plate', {'id': 'wp_1'}, lambda: {'ok': 1}), {'ok': 1})
        self.assertEqual(manager.data['operations'][0]['status'], 'SUCCESS')

    def test_people_repeated_waypoint_is_replaced_not_double_counted(self):
        record = self.record('person', dict(area='A', people=3, outside=1, illegal_parking=2))
        self.client.reports(record)
        record['result']['people'] = 4
        reports = self.client.reports(record)
        self.assertIn('共有4人', reports[0]['result'])
        record['waypoint_id'] = 'wp_2'
        record['result'] = dict(area='B', people=2, outside=0, illegal_parking=1)
        reports = self.client.reports(record)
        self.assertEqual([r['sub_id'] for r in reports], ['A街区', 'B街区'])
        self.assertTrue(all('共有6人' in r['result'] for r in reports))
        video = self.client.reports(self.record('vehicle_video', dict(stand_count=3, fall_count=1)))[0]
        self.assertEqual(video['task'], 'ebike')
        self.assertIn('A街区有2辆', video['result'])
        self.assertIn('B街区有1辆', video['result'])
        self.assertIn('正常3辆，倒伏1辆', video['result'])

    def test_unsupported_failed_and_skipped_results_are_not_uploaded(self):
        for kind in ('navigation', 'traffic_light', 'obstacle', 'record_start'):
            self.assertEqual(self.client.reports(self.record(kind, {})), [])
        for status in ('FAILED', 'SKIPPED'):
            record = self.record('plate', {'parking_lot': 1, 'plate': 'A'})
            record['status'] = status
            self.client.submit(record)
        self.client.close()
        self.assertEqual([r[0] for r in self.requests], ['/api/hello'])

    def test_parking_photo_reports_counts_and_current_evidence_without_accumulating(self):
        record = self.record('vehicle_photo', dict(stand_count=3, fall_count=1, total_count=4))
        record['paths'] = ['first_vehicle_photo_annotated.jpg']
        item = self.client.reports(record)[0]
        self.assertEqual(item['task'], 'ebike')
        self.assertIn('正常3辆，倒伏1辆', item['result'])
        self.assertEqual(item['paths'], record['paths'])
        record['result'] = dict(stand_count=1, fall_count=2, total_count=3)
        record['paths'] = ['second_vehicle_photo_annotated.jpg']
        item = self.client.reports(record)[0]
        self.assertIn('正常1辆，倒伏2辆', item['result'])
        self.assertEqual(item['paths'], record['paths'])
        record['result'] = dict(stand_count=0, fall_count=0, total_count=0)
        record['status'] = 'EMPTY_VALID'
        self.assertIn('正常0辆，倒伏0辆', self.client.reports(record)[0]['result'])

    def test_task_mapping_and_four_trash_bins(self):
        trash = dict(matches=[dict(bin_type='可回收', bin_state='开启', garbage=[{'type': '可回收'}], status='CORRECT') for _ in range(4)])
        reports = self.client.reports(self.record('trash', trash))
        self.assertEqual([r['sub_id'] for r in reports], ['1号桶', '2号桶', '3号桶', '4号桶'])
        self.assertTrue(all('投放正确' in r['result'] for r in reports))
        for kind, result, task in [
            ('fire', {'fire_count': 1, 'building': 'A大厦'}, 'fire'),
            ('temperature', {'abnormal_floors': [2, 3]}, 'temperature'),
            ('meter', dict(temperature_c=20, humidity_percent=50, electricity_kwh='100'), 'meter'),
            ('announcement', {'queued_text': '禁止直行'}, 'sign'),
        ]:
            self.assertEqual(self.client.reports(self.record(kind, result))[0]['task'], task)

    def test_sign_report_uploads_evidence_and_fire_report_contains_count(self):
        annotated = self.root/'004_obstacle_annotated.jpg'
        annotated.write_bytes(b'SIGN_EVIDENCE')
        sign = self.record('obstacle', dict(mode='blocked', announcement='前方是禁止直行指示牌'))
        sign['paths'] = [str(annotated)]
        self.client.submit(sign)
        self.client.submit(self.record('fire', dict(fire_count=5, building='晨光大厦')))
        self.client.close()
        self.assertEqual([r[0] for r in self.requests], ['/api/hello', '/api/report', '/api/report'])
        self.assertIn(b'SIGN_EVIDENCE', self.requests[1][2])
        self.assertIn('前方是禁止直行指示牌'.encode(), self.requests[1][2])
        self.assertIn('晨光大厦发现火灾隐患5个'.encode(), self.requests[2][2])
        zero = self.client.reports(self.record('fire', dict(fire_count=0, building='A楼')))[0]
        self.assertIn('A楼发现火灾隐患0个', zero['result'])


if __name__ == '__main__':
    unittest.main()
