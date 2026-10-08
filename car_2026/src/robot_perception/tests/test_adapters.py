#!/usr/bin/env python3
"""Tests for ROS image adapters, annotation and offline cache integrity only."""
import hashlib
from io import BytesIO
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import rospy
from rospy.msg import serialize_message
import person_detection_node as person
import plate_recognition_node as plate
from prepare_hyperlpr_models import verify


class Adapters(unittest.TestCase):
    def test_padded_rgb_and_header(self):
        for module in (person, plate):
            message = Image(header=Header(seq=12, stamp=rospy.Time(123, 456), frame_id='camera'),
                            height=1, width=2, encoding='rgb8', step=8,
                            data=bytes([1, 2, 3, 4, 5, 6, 0, 0]))
            decoded = module.decode_bgr(message)
            self.assertEqual(decoded.tolist(), [[[3, 2, 1], [6, 5, 4]]])
            encoded = module.encode_bgr(decoded, message.header)
            self.assertEqual(encoded.header, message.header)
            self.assertEqual(encoded.step, 6)
            self.assertEqual(encoded.encoding, 'bgr8')
            serialize_message(BytesIO(), 99, encoded)
            self.assertEqual(encoded.header.seq, 99)
            self.assertEqual(message.header.seq, 12)
            self.assertEqual(message.header.stamp, encoded.header.stamp)

    def test_supported_encodings(self):
        for encoding, channels in [('mono8', 1), ('rgba8', 4), ('bgra8', 4), ('bgr8', 3)]:
            message = Image(height=2, width=2, encoding=encoding, step=2*channels,
                            data=bytes([30]*(4*channels)))
            for module in (person, plate):
                self.assertEqual(module.decode_bgr(message).shape, (2, 2, 3))

    def test_malformed_inputs(self):
        for module in (person, plate):
            for message in (Image(), Image(height=1, width=2, step=6, encoding='bgr8', data=b'x'),
                            Image(height=1, width=2, step=5, encoding='rgb8', data=b'12345')):
                with self.assertRaises(ValueError):
                    module.decode_bgr(message)

    def test_labels_and_annotations(self):
        self.assertEqual(person.EXPECTED_NAMES, {0: 'community_person', 1: 'non_community_person'})
        for class_id in (0, 1):
            image = np.zeros((120, 200, 3), dtype=np.uint8)
            person.draw_detection(image, {'bbox_xyxy': [30, 50, 120, 100], 'class_id': class_id,
                                          'confidence': 0.9})
            self.assertGreater(np.count_nonzero(image), 0)
        image = np.zeros((120, 200, 3), dtype=np.uint8)
        annotated = plate.annotate(image, [{'bbox_xyxy': [30, 50, 120, 100],
                                           'text': '京A12345', 'confidence': 0.9}], None)
        self.assertGreater(np.count_nonzero(annotated), 0)

    def test_cache_conflict_is_read_only(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'model.onnx'
            path.write_bytes(b'original')
            with self.assertRaises(ValueError):
                verify(path, {'bytes': 8, 'sha256': hashlib.sha256(b'different').hexdigest()})
            self.assertEqual(path.read_bytes(), b'original')
            with self.assertRaises(FileNotFoundError):
                verify(path.parent / 'missing', {'bytes': 8, 'sha256': ''})


if __name__ == '__main__':
    unittest.main(verbosity=2)
