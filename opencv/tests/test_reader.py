"""用已知真值的合成表盘验证读数与失败路径，不能替代比赛实拍验收。"""

import copy
import math
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from opencv.src.core import adapt_config_to_frame_size, nearest_integer, read_frame, read_image, validate_config, write_image


def fixture(value=23, counterclockwise=False):
    image = np.full((400, 400, 3), 235, dtype=np.uint8)
    center, radius = (200, 200), 160
    start, sign = (45, -1) if counterclockwise else (225, 1)

    def point(angle, distance):
        angle = math.radians(angle)
        return [200 + distance * math.cos(angle), 200 + distance * math.sin(angle)]

    ticks = [{"value": v, "point": point(start + sign * (v + 20) * 3, radius)}
             for v in (-20, 10, 40, 70)]
    for progress in range(0, 271, 3):
        p1 = tuple(round(p) for p in point(start + sign * progress, radius * 0.92))
        p2 = tuple(round(p) for p in point(start + sign * progress, radius))
        cv2.line(image, p1, p2, (60, 60, 60), 1, cv2.LINE_AA)
    angle = (start + sign * (value + 20) * 3) % 360
    tip = tuple(round(p) for p in point(angle, radius * 0.85))
    cv2.line(image, center, tip, (15, 15, 15), 4, cv2.LINE_AA)
    cv2.circle(image, center, 12, (15, 15, 15), -1)
    config = {"version": 1, "frame_size": [400, 400], "roi": [0, 0, 400, 400],
              "center": [200, 200], "radius": radius,
              "direction": "counterclockwise" if counterclockwise else "clockwise",
              "ticks": ticks, "exclude_circles": [], "settings": {}}
    return image, config, angle


class ReaderTests(unittest.TestCase):
    def test_competition_range_all_81_integer_readings(self):
        for value in range(-30, 51):
            with self.subTest(value=value):
                # 沿用合成图绘制器，以同一 270° 量程模拟 -30～50 表盘。
                image, config, _ = fixture(-20 + (value + 30) * 90 / 80)
                config["scale"] = {"min": -30, "max": 50, "step": 1}
                config["ticks"] = []
                for tick_value in range(-30, 51, 10):
                    angle = math.radians(225 + (tick_value + 30) * 270 / 80)
                    config["ticks"].append({"value": tick_value,
                                            "point": [200 + 160 * math.cos(angle),
                                                      200 + 160 * math.sin(angle)]})
                result, _ = read_frame(image, config)
                self.assertEqual(result["status"], "ok", result)
                self.assertEqual(result["rounded"], value)

    def test_known_readings_including_zero_crossing_and_endpoints(self):
        for value in (-20, 0, 22.4, 22.6, 23, 40, 70):
            with self.subTest(value=value):
                image, config, _ = fixture(value)
                result, _ = read_frame(image, config)
                self.assertEqual(result["status"], "ok", result)
                self.assertAlmostEqual(result["reading"], value, delta=0.15)
                self.assertEqual(result["rounded"], nearest_integer(value))

    def test_counterclockwise_scale(self):
        image, config, _ = fixture(23, counterclockwise=True)
        result, _ = read_frame(image, config)
        self.assertEqual(result["status"], "ok", result)
        self.assertEqual(result["rounded"], 23)

    def test_partial_reflection_gap(self):
        image, config, angle = fixture(23)
        position = (round(200 + 90 * math.cos(math.radians(angle))),
                    round(200 + 90 * math.sin(math.radians(angle))))
        cv2.circle(image, position, 12, (255, 255, 255), -1)
        result, _ = read_frame(image, config)
        self.assertEqual(result["status"], "ok", result)
        self.assertEqual(result["rounded"], 23)

    def test_severe_blur_rejected(self):
        image, config, _ = fixture()
        image = cv2.GaussianBlur(image, (0, 0), 12)
        result, _ = read_frame(image, config)
        self.assertEqual(result["status"], "rejected")
        self.assertIsNone(result["rounded"])

    def test_needle_hidden_by_saturated_area_rejected(self):
        image, config, _ = fixture()
        cv2.circle(image, (200, 200), 145, (255, 255, 255), -1)
        result, _ = read_frame(image, config)
        self.assertEqual(result["status"], "rejected")
        self.assertIsNone(result["reading"])

    def test_two_equally_strong_needles_rejected(self):
        image, config, angle = fixture(25)
        second = math.radians(angle + 80)
        tip = (round(200 + 136 * math.cos(second)), round(200 + 136 * math.sin(second)))
        cv2.line(image, (200, 200), tip, (15, 15, 15), 4, cv2.LINE_AA)
        result, _ = read_frame(image, config)
        self.assertEqual(result["status"], "rejected", result)
        self.assertIsNone(result["rounded"])

    def test_unconfirmed_scale_never_returns_a_number(self):
        image, config, _ = fixture()
        config["ticks"] = []
        result, _ = read_frame(image, config)
        self.assertEqual(result["status"], "needs_calibration", result)
        self.assertIsNone(result["rounded"])

    def test_proportional_resolution_changes_preserve_reading_without_modifying_config(self):
        image, config, _ = fixture()
        original = copy.deepcopy(config)
        for size in (200, 600, 800):
            with self.subTest(size=size):
                result, debug = read_frame(cv2.resize(image, (size, size)), config)
                self.assertEqual(result["status"], "ok", result)
                self.assertEqual(result["rounded"], 23)
                self.assertEqual(debug.shape[:2], (size, size))
                self.assertTrue(result["resolution"]["coordinates_scaled"])
        self.assertEqual(config, original)

    def test_changed_aspect_ratio_rejected(self):
        image, config, _ = fixture()
        with self.assertRaises(ValueError):
            read_frame(cv2.resize(image, (200, 250)), config)

    def test_resizing_does_not_bypass_minimum_pointer_image_size(self):
        image, config, _ = fixture()
        result, _ = read_frame(cv2.resize(image, (80, 80)), config)
        self.assertEqual(result["status"], "rejected")
        self.assertEqual(result["metrics"]["radius_px"], 32)
        self.assertTrue(any("像素尺寸太小" in reason for reason in result["reasons"]))

    def test_nonzero_roi_offset_ticks_and_exclusion_circle_scale_together(self):
        image, config, _ = fixture()
        frame = np.full((600, 800, 3), 235, dtype=np.uint8)
        frame[100:500, 200:600] = image
        config["frame_size"] = [800, 600]
        config["roi"] = [200, 100, 400, 400]
        config["exclude_circles"] = [[100, 200, 15]]
        adapted = adapt_config_to_frame_size(config, [400, 300])
        self.assertEqual(adapted["roi"], [100, 50, 200, 200])
        self.assertEqual(adapted["center"], [100, 100])
        self.assertEqual(adapted["exclude_circles"], [[50, 100, 7.5]])
        result, _ = read_frame(cv2.resize(frame, (400, 300)), config)
        self.assertEqual(result["status"], "ok", result)
        self.assertEqual(result["rounded"], 23)

    def test_normal_image_resize_rounding_is_accepted(self):
        _, config, _ = fixture()
        config["frame_size"] = [3072, 4096]
        config["roi"] = [157, 1100, 2389, 2218]
        config["center"] = [1218.39, 1109]
        config["radius"] = 869.35
        config["ticks"] = []
        adapted = adapt_config_to_frame_size(config, [1279, 1706])
        self.assertEqual(adapted["frame_size"], [1279, 1706])
        self.assertAlmostEqual(adapted["radius"], 361.954, delta=0.01)

    def test_invalid_scale_order_and_nonfinite_input_rejected(self):
        _, config, _ = fixture()
        wrong = copy.deepcopy(config)
        wrong["ticks"][2]["value"] = -100
        with self.assertRaises(ValueError):
            validate_config(wrong)
        config["center"][0] = float("nan")
        with self.assertRaises(ValueError):
            validate_config(config)

    def test_chinese_paths(self):
        image, _, _ = fixture()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "仪表测试.png"
            write_image(path, image)
            self.assertTrue(np.array_equal(read_image(path), image))


if __name__ == "__main__":
    unittest.main()
