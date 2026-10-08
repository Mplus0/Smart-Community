#!/usr/bin/env python3
"""P1 mission state machine. All deadlines use monotonic wall time."""
import math
import os
import sys
import threading
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# catkin devel-space relays execute this source with a different sys.path[0].
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from waypoint_manager import RouteError, load_route, positive_number, retry_count


class MissionError(RuntimeError):
    pass


class Controller:
    def __init__(self):
        self.state = "INIT"
        self.points = []
        self.index = 0
        self.client = None
        self.active = False
        self.stopping = False
        self.lock = threading.RLock()
        self.done = threading.Event()
        self.result_status = None
        self.use_sim_time = rospy.get_param("/use_sim_time", False)
        self.last_clock = rospy.Time.now().to_sec()
        self.clock_changed = time.monotonic()
        rospy.on_shutdown(self.shutdown)

    def transition(self, state, reason):
        point = self.points[self.index] if self.index < len(self.points) else {}
        log = rospy.logerr if state == "ERROR" else rospy.loginfo
        log("%s -> %s [%d/%d id=%s x=%s y=%s yaw=%s task=%s] %s",
                      self.state, state, min(self.index + 1, len(self.points)), len(self.points), point.get("id", "-"),
                      point.get("x"), point.get("y"), point.get("yaw"), point.get("task"), reason)
        self.state = state

    def shutdown(self):
        with self.lock:
            self.stopping = True
            if self.active and self.client and not self.done.is_set():
                self.client.cancel_goal()
                rospy.logwarn("Shutdown: cancellation requested for active goal")

    def check_running(self):
        if self.stopping or rospy.is_shutdown():
            raise MissionError("ROS shutdown; no further goals")
        if self.use_sim_time:
            now = rospy.Time.now().to_sec()
            if now < self.last_clock:
                raise MissionError("simulation clock moved backwards")
            if now > self.last_clock:
                self.clock_changed = time.monotonic()
                self.last_clock = now
            if time.monotonic() - self.clock_changed > self.clock_timeout:
                raise MissionError("/clock is uninitialized or paused (wall-time watchdog)")

    def wait(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while True:
            self.check_running()
            if predicate():
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.05)

    def cancel(self):
        if not self.active:
            return
        if not self.done.is_set():
            self.client.cancel_goal()
        # Even when /clock is paused or shutdown begins, cancellation has a wall deadline.
        if not self.done.wait(self.cancel_timeout):
            raise MissionError("cancel acknowledgement timed out; refusing to send another goal")
        if self.result_status == GoalStatus.LOST:
            raise MissionError("goal LOST; cancellation cannot be confirmed")
        self.active = False

    def await_result(self):
        # Wait off the control thread: Noetic uses ROS time internally. Publish
        # completion only after SimpleActionClient has finished its DONE transition,
        # so a fast retry cannot race with the previous goal's done callback.
        if self.client.wait_for_result():
            self.result_status = self.client.get_state()
            self.done.set()

    def initialize(self):
        self.points = load_route(rospy.get_param("~waypoints_file"))["route"]["points"]
        self.test_mode = rospy.get_param("~navigation_test_mode", False)
        if type(self.test_mode) is not bool:
            raise RouteError("navigation_test_mode must be a boolean")
        self.nav_timeout = positive_number(rospy.get_param("~navigation_timeout", 90.0), "navigation_timeout")
        self.server_timeout = positive_number(rospy.get_param("~server_timeout", 15.0), "server_timeout")
        self.cancel_timeout = positive_number(rospy.get_param("~cancel_timeout", 3.0), "cancel_timeout")
        self.clock_timeout = positive_number(rospy.get_param("~clock_timeout", 5.0), "clock_timeout")
        self.retries = retry_count(rospy.get_param("~retries", 1))
        if self.test_mode:
            rospy.logwarn("NAVIGATION TEST MODE: no traffic-light control; not competition capable!")
        else:
            for point in self.points:
                if point["task"] == "traffic_light" and point.get("stop_before_line") is not True:
                    raise MissionError("{}: whole-body safe stop before line not confirmed; route refused".format(point["id"]))
        if not self.points:
            self.transition("FINISH", "EMPTY_ROUTE: no goals sent")
            return
        if self.use_sim_time and not self.wait(lambda: rospy.Time.now().to_sec() > 0, self.clock_timeout):
            raise MissionError("/clock did not initialize")
        self.client = actionlib.SimpleActionClient(rospy.get_param("~move_base_action", "/move_base"), MoveBaseAction)
        ready = threading.Event()
        # Noetic wait_for_server uses ROS time internally; bound it externally in wall time.
        def connect():
            if self.client.wait_for_server():
                ready.set()
        threading.Thread(target=connect, daemon=True).start()
        if not self.wait(ready.is_set, self.server_timeout):
            raise MissionError("move_base server timeout")
        self.transition("NAVIGATE", "action server ready")

    def navigate(self, point):
        timeout = point.get("navigation_timeout", self.nav_timeout)
        retries = point.get("retries", self.retries)
        for attempt in range(retries + 1):
            self.check_running()
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            pose = goal.target_pose.pose
            pose.position.x, pose.position.y = point["x"], point["y"]
            pose.orientation.z = math.sin(point["yaw"] / 2.0)
            pose.orientation.w = math.cos(point["yaw"] / 2.0)
            with self.lock:
                self.check_running()
                self.done.clear()
                self.result_status = None
                self.active = True
                self.client.send_goal(goal)
                threading.Thread(target=self.await_result, daemon=True).start()
            rospy.loginfo("Goal %s attempt %d/%d", point["id"], attempt + 1, retries + 1)
            completed = self.wait(self.done.is_set, timeout)
            status = self.result_status if completed else None
            rospy.loginfo("Goal %s result=%s", point["id"], "TIMEOUT" if status is None else GoalStatus.to_string(status))
            if status == GoalStatus.SUCCEEDED:
                self.active = False
                return
            self.cancel()
            if status in (GoalStatus.PREEMPTED, GoalStatus.RECALLED, GoalStatus.LOST):
                raise MissionError("goal cancelled externally or LOST; will not override cancellation")
            if attempt == retries:
                raise MissionError("navigation failed after {} attempts".format(attempt + 1))
            rospy.logwarn("Retrying %s after confirmed termination", point["id"])

    def task(self, point):
        if point["task"] == "traffic_light":
            # Future WAIT_TRAFFIC belongs here. P1 intentionally has no release condition.
            if not self.test_mode:
                raise MissionError("TRAFFIC_LIGHT_NOT_IMPLEMENTED: stopped at confirmed safe point; mission halted, no crossing")
            rospy.logwarn("TEST ONLY: bypassing traffic-light task %s", point["id"])
        else:
            rospy.logwarn("NOT_IMPLEMENTED task=%s id=%s area=%s", point["task"], point["id"], point.get("area", ""))

    def run(self):
        try:
            self.initialize()
            while self.state not in ("FINISH", "ERROR"):
                self.check_running()
                point = self.points[self.index]
                if self.state == "NAVIGATE":
                    self.navigate(point)
                    self.transition("TASK" if point["type"] == "task" else "NEXT_POINT", "navigation SUCCEEDED")
                elif self.state == "TASK":
                    self.task(point)
                    self.transition("NEXT_POINT", "placeholder handled")
                elif self.state == "NEXT_POINT":
                    self.index += 1
                    self.transition("FINISH" if self.index == len(self.points) else "NAVIGATE", "advance route")
            return 0
        except Exception as exc:
            self.transition("ERROR", str(exc))
            with self.lock:
                self.stopping = True
            try:
                self.cancel()
            except Exception as cancel_exc:
                rospy.logerr("Cancellation failed: %s. Check move_base/robot stop before restart.", cancel_exc)
            return 1


def main():
    rospy.init_node("main_controller")
    return Controller().run()


if __name__ == "__main__":
    sys.exit(main())
