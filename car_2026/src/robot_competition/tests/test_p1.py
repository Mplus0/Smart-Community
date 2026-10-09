#!/usr/bin/env python3
"""Run inside Docker with an isolated ROS master; never connects to real move_base."""
import math
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
import tf2_ros
import yaml
from std_msgs.msg import String
from test_p2c import frame

from main_controller import Controller
from waypoint_manager import RouteError, load_route, validate_route
from waypoint_recorder import append_point, capture_pose


def point(name="a", task="none", **kwargs):
    return dict(id=name, type="waypoint" if task == "none" else "task", task=task,
                x=0.0, y=0.0, yaw=1.2, **kwargs)


def route(points):
    return {"route": {"frame_id": "map", "points": points}}


class SchemaTests(unittest.TestCase):
    def test_valid_and_empty(self):
        for points in ([], [point(), point("b", "person", area="block_01")]):
            self.assertEqual(validate_route(route(points))["route"]["points"], points)

    def test_invalid(self):
        bad = [None, {}, {"route": []}, route([point(), point()])]
        for field in point():
            p = point()
            del p[field]
            bad.append(route([p]))
        for value in (None, True, "1", math.nan, math.inf, -math.inf):
            for field in ("x", "y", "yaw"):
                p = point()
                p[field] = value
                bad.append(route([p]))
        for extra in ({"type": "task"}, {"task": "fire"}, {"retries": -1},
                      {"navigation_timeout": 0}, {"stop_before_line": "true"}):
            p = point()
            p.update(extra)
            bad.append(route([p]))
        for data in bad:
            with self.subTest(data=data), self.assertRaises(RouteError):
                validate_route(data)

    def test_save_append_and_rejection_preserve_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "route.yaml")
            append_point(path, point())
            original = Path(path).read_bytes()
            with self.assertRaises(RouteError):
                append_point(path, point("b"))
            with self.assertRaises(RouteError):
                append_point(path, point(), True)
            with patch("waypoint_recorder.os.replace", side_effect=OSError("disk error")):
                with self.assertRaises(OSError):
                    append_point(path, point("b"), True)
            self.assertEqual(Path(path).read_bytes(), original)
            append_point(path, point("b", "plate"), True)
            self.assertEqual([p["id"] for p in load_route(path)["route"]["points"]], ["a", "b"])

    def test_unreadable_and_duplicate_yaml_key(self):
        with self.assertRaises(RouteError):
            load_route("/no/such/route.yaml")
        with tempfile.NamedTemporaryFile(mode="w") as stream:
            stream.write("route: {}\nroute: {}\n")
            stream.flush()
            with self.assertRaises(RouteError):
                load_route(stream.name)


class RosTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("p1_test", disable_signals=True)
        cls.behavior = []
        cls.goals = []
        cls.cancelled = 0
        cls.server = actionlib.SimpleActionServer("/p1_fake_move_base", MoveBaseAction,
                                                 execute_cb=cls.execute, auto_start=False)
        cls.server.start()

    @classmethod
    def execute(cls, goal):
        cls.goals.append(goal)
        behavior = cls.behavior.pop(0) if cls.behavior else "success"
        if behavior in ("wait", "ignore_cancel"):
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                if cls.server.is_preempt_requested() and behavior != "ignore_cancel":
                    cls.cancelled += 1
                    cls.server.set_preempted(MoveBaseResult())
                    return
                time.sleep(0.01)
        if behavior == "abort":
            cls.server.set_aborted(MoveBaseResult())
        elif behavior == "preempt":
            cls.server.set_preempted(MoveBaseResult())
        else:
            cls.server.set_succeeded(MoveBaseResult())

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.temp.name, "route.yaml")
        type(self).goals = []
        type(self).behavior = []
        type(self).cancelled = 0
        rospy.set_param("/use_sim_time", False)
        # Keep all original P1 assertions; perception has no publisher in this navigation regression.
        rospy.set_param('~results_root', os.path.join(self.temp.name, 'results'))
        rospy.set_param('~task_config', {'defaults': {
            'task_timeout_sec': 0.12, 'observation_sec': 0.05,
            'settle_sec': 0.0, 'task_retry_count': 0},
            'traffic_light': {'traffic_timeout_sec': 0.2, 'settle_sec': 0.0}})
        rospy.set_param('~traffic_light_json_topic', '/p1_test_traffic_json')
        rospy.set_param('~cmd_vel_topic', '/p1_test_cmd_vel')
        for name, value in dict(waypoints_file=self.path, move_base_action="/p1_fake_move_base",
                                navigation_timeout=1.0, server_timeout=2.0, cancel_timeout=0.5,
                                clock_timeout=0.3, retries=1, navigation_test_mode=False).items():
            rospy.set_param("~" + name, value)

    def tearDown(self):
        self.temp.cleanup()

    def run_route(self, points):
        Path(self.path).write_text(yaml.safe_dump(route(points)))
        self.controller = Controller()
        return self.controller.run()

    def test_order_tasks_and_quaternion(self):
        self.assertEqual(self.run_route([point(), point("b", "person"), point("c", "plate")]), 0)
        self.assertEqual(len(self.goals), 3)
        self.assertEqual(self.controller.state, "FINISH")
        self.assertAlmostEqual(self.goals[0].target_pose.pose.orientation.z, math.sin(0.6))
        self.assertEqual(self.goals[0].target_pose.header.frame_id, "map")

    def test_empty(self):
        self.assertEqual(self.run_route([]), 0)
        self.assertFalse(self.goals)

    def test_unconfirmed_traffic_refuses_entire_route(self):
        self.assertEqual(self.run_route([point(), point("light", "traffic_light")]), 1)
        self.assertFalse(self.goals)

    def test_confirmed_traffic_halts_before_next_goal(self):
        self.assertEqual(self.run_route([point("light", "traffic_light", stop_before_line=True), point("after")]), 1)
        self.assertEqual(len(self.goals), 1)

    def test_explicit_test_mode(self):
        rospy.set_param("~navigation_test_mode", True)
        self.assertEqual(self.run_route([point("light", "traffic_light"), point("after")]), 0)
        self.assertEqual(len(self.goals), 2)

    def test_traffic_green_release_and_invalid_data_stop_navigation(self):
        for invalid in (False, True):
            with self.subTest(invalid=invalid):
                type(self).goals = []
                rospy.set_param('~task_config', {'traffic_light': {
                    'traffic_timeout_sec': 2.5, 'settle_sec': 0.05}})
                publisher = rospy.Publisher('/p1_test_traffic_json', String, queue_size=10)
                Path(self.path).write_text(yaml.safe_dump(route([
                    point('light_01', 'traffic_light', stop_before_line=True), point('after')])))
                self.controller = Controller()
                finished = threading.Event()
                observed_states = []
                def publish():
                    while not finished.wait(0.08):
                        if self.controller.state == 'WAIT_TRAFFIC':
                            observed_states.append(self.controller.state)
                            publisher.publish(String(data='{' if invalid else frame(rospy.Time.now().to_sec())))
                worker = threading.Thread(target=publish)
                worker.start()
                try:
                    self.assertEqual(self.controller.run(), int(invalid))
                finally:
                    finished.set()
                    worker.join()
                    publisher.unregister()
                self.assertTrue(observed_states)
                self.assertEqual(len(self.goals), 1 if invalid else 2)
                result = self.controller.results.data['tasks'][-1]
                self.assertEqual(result['released'], not invalid)
                self.assertEqual(self.controller.state, 'ERROR' if invalid else 'FINISH')

    def test_failure_retry(self):
        type(self).behavior = ["abort", "success"]
        self.assertEqual(self.run_route([point()]), 0)
        self.assertEqual(len(self.goals), 2)

    def test_exhausted(self):
        type(self).behavior = ["abort", "abort"]
        self.assertEqual(self.run_route([point(), point("after")]), 1)
        self.assertEqual(len(self.goals), 2)

    def test_timeout_cancel_retry(self):
        type(self).behavior = ["wait", "success"]
        self.assertEqual(self.run_route([point(navigation_timeout=0.15)]), 0)
        self.assertEqual(self.cancelled, 1)
        self.assertEqual(len(self.goals), 2)

    def test_external_cancel_not_retried(self):
        type(self).behavior = ["preempt"]
        self.assertEqual(self.run_route([point()]), 1)
        self.assertEqual(len(self.goals), 1)

    def test_shutdown_callback(self):
        type(self).behavior = ["wait"]
        Path(self.path).write_text(yaml.safe_dump(route([point(), point("after")])))
        self.controller = Controller()
        def stop():
            deadline = time.monotonic() + 3
            while not self.goals and time.monotonic() < deadline:
                time.sleep(0.01)
            self.controller.shutdown()
        thread = threading.Thread(target=stop)
        thread.start()
        self.assertEqual(self.controller.run(), 1)
        thread.join()
        self.assertEqual(len(self.goals), 1)
        self.assertEqual(self.cancelled, 1)

    def test_server_timeout(self):
        rospy.set_param("~move_base_action", "/p1_nonexistent")
        rospy.set_param("~server_timeout", 0.2)
        start = time.monotonic()
        self.assertEqual(self.run_route([point()]), 1)
        self.assertLess(time.monotonic() - start, 1.5)

    def test_tf_capture_roundtrip_and_missing(self):
        buffer = tf2_ros.Buffer()
        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_footprint"
        transform.header.stamp = rospy.Time.now()
        transform.transform.translation.x = 1.25
        transform.transform.rotation.z = math.sin(0.4)
        transform.transform.rotation.w = math.cos(0.4)
        buffer.set_transform(transform, "test")
        p = point()
        p.update(capture_pose(buffer, "base_footprint", 0.1, 1.0))
        append_point(self.path, p)
        self.assertAlmostEqual(load_route(self.path)["route"]["points"][0]["yaw"], 0.8)
        with self.assertRaises(RouteError):
            capture_pose(buffer, "missing", 0.1, 1.0)
        transform.header.stamp = rospy.Time.now() - rospy.Duration(10)
        stale = tf2_ros.Buffer()
        stale.set_transform(transform, "test")
        with self.assertRaises(RouteError):
            capture_pose(stale, "base_footprint", 0.1, 1.0)

    def test_unacknowledged_cancel_no_retry(self):
        type(self).behavior = ["ignore_cancel"]
        self.assertEqual(self.run_route([point(navigation_timeout=0.1), point("after")]), 1)
        self.assertEqual(len(self.goals), 1)
        time.sleep(2.1)  # Let isolated fake server finish before next test.

    def test_z_paused_clock(self):
        rospy.set_param("/use_sim_time", True)
        controller = Controller()
        controller.clock_timeout = 0.1
        controller.use_sim_time = True
        frozen = rospy.Time(10)
        controller.last_clock = 10
        start = time.monotonic()
        with patch("main_controller.rospy.Time.now", return_value=frozen):
            with self.assertRaisesRegex(RuntimeError, "paused"):
                controller.wait(lambda: False, 5)
        self.assertLess(time.monotonic() - start, 0.5)
        rospy.set_param("/use_sim_time", False)

    def test_clock_uninitialized_no_goal(self):
        rospy.set_param("/use_sim_time", True)
        with patch("main_controller.rospy.Time.now", return_value=rospy.Time(0)):
            self.assertEqual(self.run_route([point()]), 1)
        self.assertFalse(self.goals)

    def test_clock_paused_during_goal_cancels(self):
        rospy.set_param("/use_sim_time", True)
        type(self).behavior = ["wait"]
        with patch("main_controller.rospy.Time.now", return_value=rospy.Time(10)):
            self.assertEqual(self.run_route([point(), point("after")]), 1)
        self.assertEqual(len(self.goals), 1)
        self.assertEqual(self.cancelled, 1)

    def test_recorder_refuses_paused_clock(self):
        rospy.set_param("/use_sim_time", True)
        with patch("waypoint_recorder.rospy.Time.now", return_value=rospy.Time(10)):
            with self.assertRaisesRegex(RouteError, "paused"):
                capture_pose(tf2_ros.Buffer(), "base_footprint", 0.1, 1.0)


if __name__ == "__main__":
    try:
        unittest.main(verbosity=2)
    finally:
        rospy.signal_shutdown("isolated tests complete")
