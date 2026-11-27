#!/usr/bin/env python3
"""Exercise real annotation and mission persistence with fake detector services."""
import ast
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import cv2
import numpy as np
from PIL import Image
import yaml

SCRIPTS = Path(__file__).resolve().parents[1] / 'src/community_mission/scripts'
sys.path.insert(0, str(SCRIPTS))
from evidence import save_annotation
from inspection import Inspection
from result_manager import ResultManager
from rules import detections, detection_records, obstacle_route


def response(objects):
    return SimpleNamespace(detected_classes=[label for label, box in objects],
                           confidences=[.9] * len(objects),
                           bboxes=[value for label, box in objects for value in box])


class MissionEvidenceTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.folder = Path(directory.name)
        self.frame = np.full((360, 640, 3), (40, 60, 80), dtype=np.uint8)
        self.original = self.frame.copy()
        self.image = SimpleNamespace(width=640, height=360)
        self.calls = []
        self.empty = False
        self.lines = []
        self.font = next((str(p) for p in (
            Path('C:/Windows/Fonts/msyh.ttc'),
            Path('/System/Library/Fonts/Supplemental/Arial Unicode.ttf')) if p.is_file()), '')
        # Production fonts are auto-selected by save_annotation; the explicit font is optional.
        self.font = self.font if Path(self.font).is_file() else ''
        rospy = SimpleNamespace(loginfo=lambda *a: None, sleep=lambda duration: None,
                                get_param=lambda name, default='': self.font)
        source = ast.parse((SCRIPTS / 'mission_node.py').read_text())
        mission = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == 'Mission')
        namespace = dict(Path=Path, rospy=rospy, cv2=cv2, detections=detections,
                         detection_records=detection_records, obstacle_route=obstacle_route,
                         save_annotation=save_annotation)
        exec(compile(ast.Module(body=[mission], type_ignores=[]), 'mission_node.py', 'exec'), namespace)
        self.Mission = namespace['Mission']

    def detector(self, name, image):
        self.assertIs(image, self.image)
        self.calls.append(name)
        objects = {
            'people_detector': [('community_person', (100, 170, 220, 310)),
                                ('outsider', (250, 180, 340, 320))],
            'bicycle_detector': [('stand', (110, 240, 200, 340)), ('fall', (400, 240, 560, 320))],
            'fire_detector': [('fire', (100, 100, 250, 200))],
            'obstacle_detector': [('obstacle', (280, 100, 360, 240))],
        }
        return response([] if self.empty else objects[name])

    def make_mission(self, kind):
        point = dict(id='test_' + kind, task=kind, area='A', parking_lot=1)
        obj = self.Mission.__new__(self.Mission)
        obj.route, obj.index = {'points': [point]}, 0
        obj.navigation_only, obj.task_only = False, True
        obj.image_topic = obj.active_image_topic = '/usb_cam/image_raw'
        obj.meter_image_topic = '/ugreen_4k/image_raw'
        obj.results = ResultManager(self.folder, [point], self.lines.append)
        obj.data_dir = obj.results.path
        obj.bridge = SimpleNamespace(imgmsg_to_cv2=lambda *a, **k: self.frame.copy())
        def ocr(image):
            self.assertIs(image, self.image)
            self.calls.append('license_plate_ocr')
            return SimpleNamespace(plate_texts=[] if self.empty else ['粤B12345'],
                                   confidences=[] if self.empty else [.93],
                                   bboxes=[] if self.empty else [100, 170, 300, 230],
                                   status_message='未识别到车牌' if self.empty else '识别成功')
        services = SimpleNamespace(detect=self.detector, ocr=ocr,
                                   building_ocr=lambda image: SimpleNamespace(plate_texts=['A号楼']))
        settings = yaml.safe_load((SCRIPTS.parent / 'config/recognition_rules.yaml').read_text())['recognition_rules']
        obj.inspection = Inspection(obj.route, services, None, settings, obj.bridge, ocr_interval=0)
        obj.services, obj.obstacle_mode, obj.obstacle_clear_x_ratio = services, 'clear', .35
        obj.speech = SimpleNamespace(put=lambda text: self.calls.append('sign_speech'), drain=lambda: True)
        obj.check_running = lambda: None
        obj.fresh_image = lambda: self.image
        obj.transition = lambda state, reason: None
        obj.results.waypoint(point, 'RUNNING')
        return obj

    def assert_artifacts(self, obj, kind):
        raw = obj.data_dir / 'photo/000.jpg'
        annotated = obj.data_dir / f'photo/000_{kind}_annotated.jpg'
        self.assertTrue(raw.is_file())
        self.assertTrue(annotated.is_file())
        with Image.open(annotated) as image:
            self.assertEqual(image.width, 640)
            self.assertGreater(image.height, 360)  # caption cannot obscure original boxes
        operation = obj.results.data['operations'][-1]
        self.assertIn(str(raw), operation['paths'])
        self.assertIn(str(annotated), operation['paths'])
        saved = json.loads((obj.data_dir / 'mission_results.json').read_text())
        self.assertEqual(saved['operations'][-1]['paths'], operation['paths'])
        np.testing.assert_array_equal(self.frame, self.original)
        return annotated

    def test_five_tasks_save_original_and_annotation_without_extra_inference(self):
        for kind, expected_calls in [
            ('person', ['people_detector', 'bicycle_detector']),
            ('plate', ['license_plate_ocr']), ('fire', ['fire_detector']),
            ('obstacle', ['obstacle_detector', 'sign_speech']), ('vehicle_photo', ['bicycle_detector'])]:
            with self.subTest(kind=kind):
                self.calls.clear()
                obj = self.make_mission(kind)
                obj.task()
                self.assert_artifacts(obj, kind)
                self.assertEqual(self.calls, expected_calls)
                operation = obj.results.data['operations'][-1]
                self.assertEqual(operation['status'], 'SUCCESS')
                if kind == 'plate':
                    self.assertEqual(operation['result']['plate'], '粤B12345')
                    self.assertEqual(operation['result']['detections'][0]['bbox'], [100, 170, 300, 230])
                else:
                    self.assertTrue(operation['result']['detections'])
                if kind == 'obstacle':
                    self.assertEqual(obj.obstacle_mode, 'blocked')

    def test_empty_observations_still_save_annotations(self):
        self.empty = True
        for kind in ['person', 'plate', 'fire', 'vehicle_photo']:
            with self.subTest(kind=kind):
                obj = self.make_mission(kind)
                obj.task()
                self.assert_artifacts(obj, kind)
                self.assertEqual(obj.results.data['operations'][-1]['status'], 'EMPTY_VALID')

    def test_parking_photo_counts_states_and_saves_exact_result(self):
        obj = self.make_mission('vehicle_photo')
        obj.task()
        record = obj.results.data['operations'][-1]
        self.assertEqual({k: record['result'][k] for k in ('stand_count', 'fall_count', 'total_count')},
                         dict(stand_count=1, fall_count=1, total_count=2))
        self.assert_artifacts(obj, 'vehicle_photo')
        self.empty = True
        obj.task()
        self.assertEqual(obj.results.data['operations'][-1]['result']['total_count'], 0)

    def test_parking_photo_waits_for_stop_before_requesting_new_image(self):
        obj = self.make_mission('vehicle_photo')
        obj.task_only = False
        events = []
        obj.settle_vehicle_photo = lambda: events.append('stopped')
        def fresh_image():
            events.append('capture')
            return self.image
        obj.fresh_image = fresh_image
        obj.task()
        self.assertEqual(events, ['stopped', 'capture'])

    def test_parking_photo_counts_only_supported_vehicle_states(self):
        obj = self.make_mission('vehicle_photo')
        obj.services.detect = lambda *args: response([
            ('stand', (10, 10, 50, 100)), ('stand', (60, 10, 100, 100)),
            ('fall', (110, 10, 210, 50)), ('unknown', (220, 10, 260, 100))])
        obj.task()
        result = obj.results.data['operations'][-1]['result']
        self.assertEqual((result['stand_count'], result['fall_count'], result['total_count']), (2, 1, 3))

    def test_parking_detector_failure_is_not_reported_as_zero(self):
        obj = self.make_mission('vehicle_photo')
        def fail(*args):
            raise RuntimeError('detector offline')
        obj.services.detect = fail
        with self.assertRaisesRegex(RuntimeError, 'detector offline'):
            obj.task()
        record = obj.results.data['operations'][-1]
        self.assertEqual(record['status'], 'FAILED')
        self.assertNotIn('total_count', record['result'])

    def test_invalid_obstacle_still_saves_failure_annotation_and_stops(self):
        self.empty = True
        obj = self.make_mission('obstacle')
        with self.assertRaisesRegex(ValueError, 'exactly one obstacle'):
            obj.task()
        self.assert_artifacts(obj, 'obstacle')
        self.assertEqual(obj.results.data['operations'][-1]['status'], 'FAILED')
        self.assertEqual(obj.obstacle_mode, 'clear')
        self.assertNotIn('sign_speech', self.calls)

    def test_sign_result_and_voice_use_the_actual_detection(self):
        obj = self.make_mission('obstacle')
        spoken = []
        obj.speech.put = spoken.append
        obj.task()
        record = obj.results.data['operations'][-1]
        self.assertEqual(record['result']['announcement'], '前方是禁止直行指示牌')
        self.assertEqual(spoken, [record['result']['announcement']])
        self.assertIn('禁止直行', '\n'.join(self.lines))

    def test_thermal_task_saves_service_evidence_and_reports_its_source_stamp(self):
        obj = self.make_mission('temperature')
        source = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 123.5)))
        calls = []
        def measure():
            calls.append('measure')
            return SimpleNamespace(abnormal_floors=[2], max_temperatures_c=[25., 52., 25., 25.],
                                   evidence_image=source)
        def convert(image, **kwargs):
            self.assertIs(image, source)
            return self.frame.copy()
        obj.services.measure_building_temperature = measure
        obj.bridge.imgmsg_to_cv2 = convert
        obj.task()
        record = obj.results.data['operations'][-1]
        self.assertEqual(record['result']['image_stamp'], 123.5)
        self.assertEqual(record['result']['abnormal_floors'], [2])
        self.assertEqual(calls, ['measure'])
        self.assertEqual(len(record['paths']), 1)
        self.assertTrue(Path(record['paths'][0]).is_file())
        self.assertTrue(record['paths'][0].endswith('_temperature_annotated.jpg'))

    def test_detector_exception_preserves_failed_result_and_raw_image(self):
        obj = self.make_mission('fire')
        def fail(*args):
            raise RuntimeError('detector offline')
        obj.services.detect = fail
        with self.assertRaisesRegex(RuntimeError, 'detector offline'):
            obj.task()
        self.assert_artifacts(obj, 'fire')
        self.assertEqual(obj.results.data['operations'][-1]['status'], 'FAILED')

    def test_annotation_failure_does_not_change_recognition_outcome(self):
        obj = self.make_mission('plate')
        scope = obj.save_detection_evidence.__func__.__globals__
        scope['save_annotation'] = lambda *a: (_ for _ in ()).throw(OSError('disk full'))
        obj.task()
        record = obj.results.data['operations'][-1]
        self.assertEqual(record['status'], 'SUCCESS')
        self.assertEqual(record['result']['plate'], '粤B12345')
        self.assertEqual(record['evidence_errors'], ['disk full'])
        self.assertEqual(len(record['paths']), 1)
        self.assertTrue(any('[标注保存失败]' in line for line in self.lines))

    def test_invalid_boxes_are_clipped_or_skipped_and_source_is_unchanged(self):
        record = dict(waypoint_id='test', task_type='vehicle_photo', status='SUCCESS', reason='',
                      result={'detected_classes': ['stand'], 'detections': [
                          dict(label='stand', source='bicycle', confidence=.9, bbox=[-10, 40, 200, 300]),
                          dict(label='fall', source='bicycle', confidence=.8, bbox=[20, 20, 10, 10]),
                          dict(label='fall', source='bicycle', confidence=.8, bbox=[float('nan'), 0, 20, 20])]})
        path = self.folder / 'clipped.jpg'
        save_annotation(self.frame, record, path, self.font)
        self.assertTrue(path.is_file())
        np.testing.assert_array_equal(self.frame, self.original)
        with Image.open(path) as image:
            r, g, b = image.getpixel((199, 200))
            self.assertGreater(g, r + 35)  # actual green outline, not just a copied image


if __name__ == '__main__':
    unittest.main()
