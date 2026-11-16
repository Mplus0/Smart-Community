#!/usr/bin/env python3
"""禁行图几何标定、坐标转换及输出保护的离线检查。"""

import math
from pathlib import Path
import tempfile
import unittest

import numpy as np
import yaml
from PIL import Image

from generate_forbidden_map import (FIELD_SIZE, fit_field_transform,
                                    generate_map, reference_zones)


class ForbiddenMapTests(unittest.TestCase):
    def corners(self, angle=.23, translation=(.4, .1)):
        rotation = np.array([[math.cos(angle), -math.sin(angle)],
                             [math.sin(angle), math.cos(angle)]])
        field = np.array([[0., 0.], [FIELD_SIZE, 0.],
                          [FIELD_SIZE, FIELD_SIZE], [0., FIELD_SIZE]])
        return field @ rotation.T + translation, rotation

    def test_rotated_translated_calibration(self):
        corners, expected = self.corners()
        rotation, translation, errors = fit_field_transform(corners)
        np.testing.assert_allclose(rotation, expected, atol=1e-12)
        np.testing.assert_allclose(translation, [.4, .1], atol=1e-12)
        self.assertLess(errors.max(), 1e-12)

    def test_reject_mirrored_and_wrong_size_field(self):
        corners, _ = self.corners()
        with self.assertRaises(ValueError):
            fit_field_transform(corners[[0, 3, 2, 1]])
        with self.assertRaises(ValueError):
            fit_field_transform(corners * 1.25)

    def test_dimensions_come_from_reference_image(self):
        zones = reference_zones()
        self.assertEqual(set(zones), {'parking_area', 'rectangle', 'l_area'})
        np.testing.assert_allclose(np.ptp(zones['rectangle'], axis=0), [1., 1.6], atol=.02)
        np.testing.assert_allclose(np.ptp(zones['l_area'], axis=0), [2.2, 2.8], atol=.02)
        np.testing.assert_allclose(np.ptp(zones['parking_area'], axis=0), [.6, 3.4], atol=.02)

    def check_generated_map(self, negate):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            free, occupied = (0, 255) if negate else (254, 0)
            source = np.full((400, 400), free, dtype=np.uint8)
            source[0, :] = 127
            source[20, 20] = occupied
            Image.fromarray(source).save(root / 'scan.pgm')
            info = {'image': 'scan.pgm', 'resolution': .02, 'origin': [-1., -1., .12],
                    'negate': negate, 'occupied_thresh': .65, 'free_thresh': .196}
            map_path = root / 'scan.yaml'
            map_path.write_text(yaml.safe_dump(info))
            original = (map_path.read_bytes(), (root / 'scan.pgm').read_bytes())
            corners, rotation = self.corners()
            outputs, report = generate_map(map_path, root / 'navigation', corners)
            result = np.array(Image.open(outputs['image']))
            saved_info = yaml.safe_load(outputs['map'].read_text())
            self.assertEqual(saved_info, dict(info, image='navigation.pgm'))
            self.assertEqual(result.shape, source.shape)
            self.assertEqual(result[20, 20], occupied)
            np.testing.assert_array_equal(result[0], source[0])

            def pixel(field_point):
                world = rotation @ np.asarray(field_point) + [.4, .1]
                a = info['origin'][2]
                inverse = np.array([[math.cos(a), math.sin(a)],
                                    [-math.sin(a), math.cos(a)]])
                grid = inverse @ (world - info['origin'][:2]) / info['resolution']
                return result[399 - int(grid[1]), int(grid[0])]

            self.assertEqual(pixel([3., 2.9]), occupied)  # 矩形内部
            self.assertEqual(pixel([1.6, 2.]), occupied)  # L 型竖条内部
            self.assertEqual(pixel([2.5, 1.]), occupied)  # L 型横条内部
            self.assertEqual(pixel([.4, 2.]), occupied)   # 左侧禁行区
            self.assertEqual(pixel([2.15, 1.6]), free)   # 通道
            self.assertEqual(pixel([.4, .4]), free)      # 起始停车位
            self.assertEqual(original, (map_path.read_bytes(), (root / 'scan.pgm').read_bytes()))
            self.assertTrue(outputs['preview'].exists())
            self.assertLess(max(report['corner_residuals_m']), 1e-12)
            previous = outputs['image'].read_bytes()
            with self.assertRaises(ValueError):
                generate_map(map_path, root / 'navigation', corners)
            self.assertEqual(previous, outputs['image'].read_bytes())

    def test_pipeline_with_rotated_map_origin(self):
        self.check_generated_map(negate=0)

    def test_pipeline_with_inverted_map(self):
        self.check_generated_map(negate=1)


if __name__ == '__main__':
    unittest.main()
