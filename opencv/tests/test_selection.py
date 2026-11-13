"""验证重复选择、逐步确认、返回编辑，以及取消时不写文件。"""

import json
import math
import tempfile
import unittest
from contextlib import ExitStack, contextmanager
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

from opencv.src import calibration
from opencv.src.core import validate_config
from opencv.src.selection import PointSelection, RectangleSelection, rectangle_to_roi


@contextmanager
def fake_gui(events):
    """向实际窗口循环注入鼠标回调与按键，避免自动测试依赖真实桌面。"""
    callbacks = {}
    events = iter(events)

    def wait_key(delay):
        action, key = next(events)
        if action is not None:
            action(callbacks["mouse"])
        return key

    def set_callback(window, callback):
        callbacks["mouse"] = callback

    with ExitStack() as stack:
        for name in ("namedWindow", "imshow", "destroyWindow"):
            stack.enter_context(patch.object(calibration.cv2, name))
        stack.enter_context(patch.object(calibration.cv2, "setMouseCallback", side_effect=set_callback))
        stack.enter_context(patch.object(calibration.cv2, "waitKey", side_effect=wait_key))
        stack.enter_context(patch.object(calibration.cv2, "getWindowProperty", return_value=1))
        yield


def click(x, y):
    # 100×100 原图在选点窗口显示为 150×150，提示栏不属于图片。
    return lambda callback: callback(cv2.EVENT_LBUTTONDOWN, round(x * 1.5),
                                     calibration.HEADER + round(y * 1.5), 0, None)


def drag(x1, y1, x2, y2):
    def action(callback):
        callback(cv2.EVENT_LBUTTONDOWN, x1, y1 + calibration.HEADER, 0, None)
        callback(cv2.EVENT_MOUSEMOVE, x2, y2 + calibration.HEADER, 0, None)
        callback(cv2.EVENT_LBUTTONUP, x2, y2 + calibration.HEADER, 0, None)
    return action


class SelectionTests(unittest.TestCase):
    def test_redrag_replaces_old_rectangle_and_supports_reverse_drag(self):
        state = RectangleSelection(300, 200)
        state.start(10, 10)
        state.finish(50, 50)
        state.start(260, 180)
        self.assertIsNone(state.confirm())
        state.finish(40, 50)
        self.assertEqual(state.confirm(), (40, 50, 260, 180))
        state.reset()
        self.assertIsNone(state.confirm())

    def test_roi_window_waits_for_enter_and_returns_last_rectangle(self):
        with fake_gui([(drag(10, 10, 100, 100), -1),
                       (drag(260, 180, 40, 50), -1), (None, 13)]):
            roi = calibration._select_roi(np.zeros((200, 300, 3), dtype=np.uint8))
        self.assertEqual(roi, [40, 50, 220, 130])

    def test_roi_empty_enter_does_not_confirm(self):
        with fake_gui([(None, 13), (drag(20, 20, 250, 190), 13)]):
            roi = calibration._select_roi(np.zeros((200, 300, 3), dtype=np.uint8))
        self.assertEqual(roi, [20, 20, 230, 170])

    def test_display_to_original_coordinates(self):
        self.assertEqual(rectangle_to_roi((10, 20, 90, 70), (300, 200), (100, 100)),
                         [30, 40, 240, 100])

    def test_reclick_does_not_advance_and_back_allows_edit(self):
        state = PointSelection(2)
        self.assertFalse(state.confirm())
        state.select(1, 2)
        state.select(3, 4)
        self.assertEqual(state.index, 0)
        self.assertEqual(state.points[0], [3, 4])
        state.confirm()
        state.select(5, 6)
        state.confirm()
        self.assertTrue(state.reviewing)
        state.back()
        state.select(7, 8)
        state.confirm()
        self.assertEqual(state.points, [[3, 4], [7, 8]])

    def test_actual_point_loop_reclick_enter_back_and_final_review(self):
        validator_calls = []
        events = [(None, 13), (click(20, 20), -1), (click(50, 50), 13),
                  (click(50, 20), -1), (click(50, 30), 13), (None, 8),
                  (click(50, 36), 13), (None, 13)]
        with fake_gui(events):
            points = calibration._click_points(np.zeros((100, 100, 3), dtype=np.uint8),
                                               ["center", "radius"], validator_calls.append)
        self.assertEqual(points, [[50, 50], [50, 36]])
        self.assertEqual(len(validator_calls), 1)

    def test_cancel_keeps_existing_file_and_creates_no_reference(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "meter.json"
            path.write_text("existing calibration", encoding="utf-8")
            with patch.object(calibration, "read_image", return_value=np.zeros((400, 400, 3), dtype=np.uint8)), \
                    fake_gui([(None, 27)]):
                with self.assertRaises(ValueError):
                    calibration.calibrate("unused.png", path, list(calibration.DEFAULT_VALUES), "clockwise")
            self.assertEqual(path.read_text(encoding="utf-8"), "existing calibration")
            self.assertFalse(path.with_suffix(".reference.png").exists())

    def test_saved_config_contains_range_and_default_major_ticks(self):
        values = list(calibration.DEFAULT_VALUES)
        points = [[200, 200], [200, 40]]
        for value in values:
            angle = math.radians(225 + (value + 30) * 270 / 80)
            points.append([200 + 160 * math.cos(angle), 200 + 160 * math.sin(angle)])

        def confirm(image, labels, validator):
            validator(points)
            return points

        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "meter.json"
            with patch.object(calibration, "read_image", return_value=np.zeros((400, 400, 3), dtype=np.uint8)), \
                    patch.object(calibration, "_click_points", side_effect=confirm):
                calibration.calibrate("unused.png", path, values, "clockwise", roi=[0, 0, 400, 400])
            config = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(config["scale"], {"min": -30, "max": 50, "step": 1})
            self.assertEqual([tick["value"] for tick in config["ticks"]], values)
            self.assertTrue(path.with_suffix(".reference.png").exists())
            validate_config(config)


if __name__ == "__main__":
    unittest.main()
