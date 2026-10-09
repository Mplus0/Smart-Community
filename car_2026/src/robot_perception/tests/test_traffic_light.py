#!/usr/bin/env python3
"""Run in the existing ROS Python 3.10 container environment, never on the host."""
from io import BytesIO
import json
import os
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rospy
from rospy.msg import serialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import traffic_light_classification_node as traffic


def message(height=5, width=8):
    return Image(header=Header(seq=7, stamp=rospy.Time(10, 123), frame_id="camera"),
                 height=height, width=width, encoding="bgr8", step=width * 3,
                 data=np.arange(height * width * 3, dtype=np.uint8).tobytes())


def fake_model(values=(0.05, 0.9, 0.05)):
    result = SimpleNamespace(probs=SimpleNamespace(data=torch.tensor(values)))
    return SimpleNamespace(task="classify", names={0: "red", 1: "green", 2: "yellow"},
                           predict=Mock(return_value=[result]))


class TrafficLightTests(unittest.TestCase):
    def setUp(self):
        for name in ("logwarn_throttle", "logerr_throttle"):
            patcher = patch.object(rospy, name)
            patcher.start()
            self.addCleanup(patcher.stop)

    def node(self):
        node = traffic.TrafficLightClassificationNode.__new__(traffic.TrafficLightClassificationNode)
        node.model = fake_model()
        node.names = traffic.validate_model(node.model)
        node.roi = (0, 0, 1, 0.5)
        node.confidence, node.device = 0.8, "cpu"
        node.stale_seconds, node.max_rate = 1.5, 5.0
        node.last_stamp_ns = 0
        node.image_pub, node.json_pub = Mock(), Mock()
        node._freshness = Mock(return_value="ok")
        return node

    def payload(self, node):
        return json.loads(node.json_pub.publish.call_args[0][0].data)

    def test_roi_exact_upper_half_and_no_mutation(self):
        original = traffic.decode_bgr(message())
        before = original.copy()
        roi, bounds = traffic.crop_roi(original, [0, 0, 1, 0.5])
        self.assertEqual(bounds, [0, 0, 8, 2])
        np.testing.assert_array_equal(roi, original[:5 // 2, :])
        roi[:] = 0
        np.testing.assert_array_equal(original, before)

    def test_roi_config_validation_and_empty_crop(self):
        for roi in ([0, 0, 1], [0, 0, 1, float("nan")], [-0.1, 0, 1, 1],
                    [1, 0, 0, 1], [0, 0, 1.1, 1]):
            with self.subTest(roi=roi), self.assertRaises(ValueError):
                traffic.validate_roi(roi)
        with self.assertRaises(ValueError):
            traffic.crop_roi(np.zeros((1, 1, 3), np.uint8), [0, 0, 1, 0.5])

    def test_mapping_uses_model_names(self):
        import itertools
        for labels in itertools.permutations(["red", "yellow", "green"]):
            model = fake_model()
            model.names = dict(enumerate(labels))
            names = traffic.validate_model(model)
            label, score, reason = traffic.classify(model, names, np.zeros((3, 4, 3)), 0.8)
            self.assertEqual(label, labels[1].upper())
            self.assertEqual(reason, "ok")
            self.assertAlmostEqual(score, 0.9)
            self.assertEqual(model.predict.call_args.kwargs["imgsz"], 224)
            self.assertEqual(model.predict.call_args.kwargs["device"], "cpu")

    def test_reject_wrong_model(self):
        for task, names in [("detect", {0: "red", 1: "green", 2: "yellow"}),
                            ("classify", {0: "red", 1: "green", 2: "unknown"}),
                            ("classify", {1: "red", 2: "green", 3: "yellow"})]:
            with self.assertRaises(ValueError):
                traffic.validate_model(SimpleNamespace(task=task, names=names))
        self.assertEqual(traffic.validate_model(SimpleNamespace(
            task="classify", names=["yellow", "red", "green"]))[0], "yellow")

    def test_low_confidence(self):
        node = self.node()
        node.model = fake_model([0.25, 0.5, 0.25])
        node._infer(message(), 0)
        payload = self.payload(node)
        self.assertEqual(payload["reason"], "low_confidence")
        self.assertEqual(payload["label"], "UNKNOWN")
        self.assertFalse(payload["frame_valid"])

    def test_invalid_probabilities(self):
        for values in ([float("nan"), 0, 1], [0, float("inf"), 0], [-1, 1, 1],
                       [0.1, 0.2], [0.1, 0.1, 0.1]):
            with self.subTest(values=values), self.assertRaises(ValueError):
                traffic.classify(fake_model(values), {0: "red", 1: "green", 2: "yellow"},
                                 np.zeros((3, 4, 3)), 0.8)

    def test_json_header_and_full_resolution_image(self):
        node, msg = self.node(), message()
        node._infer(msg, 0)
        payload = self.payload(node)
        self.assertEqual(payload["engine"], traffic.ENGINE)
        self.assertEqual(payload["schema_version"], 1)
        self.assertEqual(payload["header"], traffic.header_json(msg.header))
        self.assertEqual(payload["roi_xyxy"], [0, 0, 8, 2])
        self.assertEqual(payload["label"], "GREEN")
        self.assertTrue(payload["frame_valid"])
        self.assertFalse(payload["presence_verified"])
        self.assertGreaterEqual(payload["processing_ms"], 0)
        image = node.image_pub.publish.call_args[0][0]
        self.assertEqual((image.height, image.width), (msg.height, msg.width))
        serialize_message(BytesIO(), 99, image)
        self.assertEqual(msg.header.seq, 7)
        self.assertEqual(image.header.stamp, msg.header.stamp)
        self.assertEqual(image.header.frame_id, msg.header.frame_id)
        self.assertEqual(node.model.predict.call_args.kwargs["source"].shape, (2, 8, 3))

    def test_decode_failure_never_green(self):
        node, msg = self.node(), message()
        msg.data = b"x"
        node._infer(msg, 0)
        self.assertEqual(self.payload(node)["reason"], "decode_error")
        self.assertEqual(self.payload(node)["label"], "UNKNOWN")
        node.model.predict.assert_not_called()

    def test_inference_failure_never_green(self):
        node = self.node()
        node.model.predict.side_effect = RuntimeError("test inference failure")
        node._infer(message(), 0)
        self.assertEqual(self.payload(node)["reason"], "inference_error")
        self.assertFalse(self.payload(node)["frame_valid"])
        self.assertEqual(self.payload(node)["label"], "UNKNOWN")

    def test_stale_before_and_after_inference(self):
        for answers in (["camera_stale"], ["ok", "camera_stale"]):
            node = self.node()
            node._freshness.side_effect = answers
            node._infer(message(), 0)
            self.assertEqual(self.payload(node)["reason"], "camera_stale")
            self.assertEqual(self.payload(node)["label"], "UNKNOWN")
            self.assertFalse(self.payload(node)["frame_valid"])

    def test_both_clocks_and_future_or_zero_stamp(self):
        msg = message()
        stamp = msg.header.stamp.to_nsec()
        cases = [(0, 0.1, stamp, "ok"), (0, 2, stamp, "camera_stale"),
                 (0, 0.1, stamp + 2_000_000_000, "camera_stale"),
                 (0, 0.1, stamp - 1, "future_stamp"), (0, 0, 0, "invalid_stamp")]
        for arrival, wall, ros_ns, reason in cases:
            self.assertEqual(traffic.freshness_reason(msg, arrival, wall, ros_ns, 1.5), reason)
        msg.header.stamp = rospy.Time()
        self.assertEqual(traffic.freshness_reason(msg, 0, 0, stamp, 1.5), "invalid_stamp")

    def test_repeated_and_backwards_source_stamp(self):
        node, msg = self.node(), message()
        node._infer(msg, 0)
        for stamp in (rospy.Time(10, 123), rospy.Time(9, 0)):
            msg.header.stamp = stamp
            node._infer(msg, 0)
            self.assertEqual(self.payload(node)["reason"], "non_increasing_stamp")
            self.assertEqual(self.payload(node)["label"], "UNKNOWN")
        self.assertEqual(node.model.predict.call_count, 1)

    def test_latest_only_rate_limit_and_disconnect(self):
        node = self.node()
        node.lock = threading.Lock()
        node.latest = (None, 0, 1)
        node.processed_generation = 0
        node.last_process_wall = -float("inf")
        node.stale_reported = False
        node._infer = Mock()
        first, second = message(), message()
        with patch.object(traffic.time, "monotonic", return_value=1):
            node._receive(first)
            node._receive(second)
            node.step()
        node._infer.assert_called_once_with(second, 1)
        with patch.object(traffic.time, "monotonic", return_value=1.01):
            node._receive(first)
            node.step()
        self.assertEqual(node._infer.call_count, 1)
        with patch.object(traffic.time, "monotonic", return_value=1.3):
            node.step()
        self.assertEqual(node._infer.call_count, 2)
        with patch.object(traffic.time, "monotonic", return_value=4):
            node.step()
            node.step()
        self.assertEqual(node.json_pub.publish.call_count, 1)
        self.assertEqual(self.payload(node)["reason"], "camera_stale")

    def test_every_failure_forces_unknown(self):
        for reason in ("decode_error", "inference_error", "camera_stale", "low_confidence",
                       "invalid_stamp", "future_stamp", "non_increasing_stamp"):
            payload = traffic.make_payload(message().header, None, "GREEN", 0.99, reason, 0)
            self.assertEqual(payload["label"], "UNKNOWN")
            self.assertFalse(payload["frame_valid"])
            json.dumps(payload, allow_nan=False)


@unittest.skipUnless(os.environ.get("TRAFFIC_LIGHT_MODEL"), "Set TRAFFIC_LIGHT_MODEL for real CPU model smoke")
class RealModelSmoke(unittest.TestCase):
    def test_cpu_model_task_names_and_inference(self):
        # Synthetic input tests loading/API only, never traffic-light recognition accuracy.
        model_path = Path(os.environ["TRAFFIC_LIGHT_MODEL"])
        self.assertTrue(model_path.is_file(), str(model_path))
        model = traffic.YOLO(str(model_path))
        names = traffic.validate_model(model)
        roi, _ = traffic.crop_roi(np.zeros((480, 640, 3), np.uint8), [0, 0, 1, 0.5])
        label, score, reason = traffic.classify(model, names, roi, 0.8, "cpu")
        self.assertIn(label, {"RED", "YELLOW", "GREEN", "UNKNOWN"})
        self.assertTrue(0 <= score <= 1)
        self.assertIn(reason, {"ok", "low_confidence"})


if __name__ == "__main__":
    unittest.main(verbosity=2)
