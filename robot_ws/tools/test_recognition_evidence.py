#!/usr/bin/env python3
"""Exercise OCR coordinates and same-frame thermal evidence without ROS/model loading."""
import ast
from collections import deque
from pathlib import Path
import re
import threading
import time
from types import SimpleNamespace
import unittest

import cv2
import numpy as np

PERCEPTION = Path(__file__).resolve().parents[1] / 'src/community_perception/scripts'


def response(**values):
    fields = dict(plate_texts=[], confidences=[], bboxes=[], status_message='')
    fields.update(values)
    return SimpleNamespace(**fields)


def load_nodes(filename, names, **scope):
    tree = ast.parse((PERCEPTION / filename).read_text(encoding='utf-8'))
    nodes = [node for node in tree.body if isinstance(node, (ast.ClassDef, ast.FunctionDef))
             and node.name in names]
    exec(compile(ast.Module(body=nodes, type_ignores=[]), filename, 'exec'), scope)
    return scope


class RecognitionEvidenceTests(unittest.TestCase):
    def ocr_server(self, result):
        scope = load_nodes('license_plate_server.py', ['LicensePlateOCRServer'],
                           np=np, re=re, LicensePlateOCRResponse=response,
                           rospy=SimpleNamespace(logerr=lambda *args: None))
        server = scope['LicensePlateOCRServer'].__new__(scope['LicensePlateOCRServer'])
        server.processing_lock = threading.Lock()
        server.conf = .8
        server.bridge = SimpleNamespace(imgmsg_to_cv2=lambda *args: np.zeros((300, 500, 3), np.uint8))
        server.safe_ocr_process = lambda frame: result
        return server

    def test_plate_box_excludes_unrelated_text_and_restores_crop_offset(self):
        server = self.ocr_server(None)
        result = dict(texts=['广告', '粤B', '12345', '电话'], boxes=[
            [[0, 0], [30, 0], [30, 20], [0, 20]],
            [[40, 50], [80, 50], [80, 70], [40, 70]],
            [[82, 50], [170, 50], [170, 72], [82, 72]],
            [[10, 150], [90, 150], [90, 180], [10, 180]]])
        self.assertEqual(server.text_bbox(result, '粤B12345', (100, 5)), [140, 55, 270, 77])
        with self.assertRaises(ValueError):
            server.text_bbox(result, '京A99999')

    def test_energy_box_contains_only_matching_numeric_and_unit_regions(self):
        result = dict(texts=['123.4', 'kWh', '999'], scores=[.95, .91, .99], boxes=[
            [[20, 30], [90, 30], [90, 50], [20, 50]],
            [[30, 55], [80, 55], [80, 70], [30, 70]],
            [[300, 100], [350, 100], [350, 120], [300, 120]]])
        server = self.ocr_server(result)
        reading = server.handle_energy_request(SimpleNamespace(image=object()))
        self.assertEqual(reading.plate_texts, ['123.4'])
        self.assertEqual(reading.bboxes, [20, 30, 90, 70])
        result['texts'][1] = 'W'
        reading = server.handle_energy_request(SimpleNamespace(image=object()))
        self.assertEqual(reading.plate_texts, [])
        self.assertEqual(reading.bboxes, [])

    def test_thermal_measurement_returns_image_from_exact_confirmed_frame(self):
        class Stamp(float):
            def is_zero(self):
                return self == 0
            def to_sec(self):
                return float(self)
        ros = SimpleNamespace(Time=SimpleNamespace(now=lambda: Stamp(100)), loginfo=lambda *a: None,
                              is_shutdown=lambda: False, loginfo_throttle=lambda *a: None,
                              logwarn_throttle=lambda *a: None)
        scope = load_nodes('building_d_temperature.py',
            ['floor_temperatures', 'draw_preview', 'BuildingDTemperature'],
            np=np, cv2=cv2, rospy=ros, time=time, CvBridgeError=RuntimeError,
            String=lambda data: data, MeasureBuildingTemperatureResponse=response)
        detector = scope['BuildingDTemperature'].__new__(scope['BuildingDTemperature'])
        detector.threshold, detector.confirm_frames, detector.timeout = 40., 3, 3.0
        detector.frames = threading.Condition()
        detector.valid_stamps = deque(maxlen=3)
        detector.streaks = [0] * 4
        detector.latest_measurement, detector.last_result = None, None
        detector.last_received = time.monotonic()
        detector.bridge = SimpleNamespace(imgmsg_to_cv2=lambda msg, encoding: msg.data,
            cv2_to_imgmsg=lambda frame, encoding: SimpleNamespace(data=frame.copy(), encoding=encoding))
        previews = []
        detector.preview_pub = SimpleNamespace(publish=previews.append)
        detector.result_pub = SimpleNamespace(publish=lambda *a: None)
        temperatures = np.full((156, 206), 25., np.float32)
        temperatures[60:65, 65:70] = 52.
        for stamp in (101, 102, 103):
            detector.receive(SimpleNamespace(data=temperatures, encoding='32FC1',
                                            header=SimpleNamespace(stamp=Stamp(stamp))))
        reading = detector.measure(None)
        self.assertTrue(reading.success)
        self.assertEqual(reading.abnormal_floors, [3])
        self.assertEqual(reading.evidence_image.header.stamp, Stamp(103))
        self.assertIs(reading.evidence_image, previews[-1])
        # Confirmed hot spot has a red outline at its actual image position.
        self.assertEqual(reading.evidence_image.data[100 + 60*4, 65*4].tolist(), [0, 0, 255])
        self.assertEqual(reading.max_temperatures_c, [25., 25., 52., 25.])


if __name__ == '__main__':
    unittest.main()
