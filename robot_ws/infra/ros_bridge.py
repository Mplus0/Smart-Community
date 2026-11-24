"""ROS bridge: one rospy node owning every subscription and publisher.

* Topic health (Hz / age) for the dashboard.
* Cached map / costmaps, laser scan, plans and pose for
  the navigation view.
* Publishers for 2D Pose Estimate, 2D Nav Goal, teleop ``/cmd_vel`` and
  ``/move_base/cancel``.
* Latest ``/mission/state`` JSON published by ``mission_node``.

``rospy.init_node`` blocks until the ROS master is reachable, so it runs
in a background thread: the web server starts even without roscore and
the bridge connects as soon as the master appears.
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

from . import config

_LOG = logging.getLogger("infra.ros_bridge")

try:
    import numpy as np
    import rosgraph
    import rosnode
    import rospy
    import tf2_ros
    from actionlib_msgs.msg import GoalID
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from map_msgs.msg import OccupancyGridUpdate
    from nav_msgs.msg import OccupancyGrid, Path
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    _HAS_ROSPY = True
except ImportError:
    _HAS_ROSPY = False
    _LOG.warning("rospy unavailable — ros_bridge will be inert")


# Topics shown on the dashboard. Names follow the real robot_ws graph:
# the IMU driver publishes /wit/imu, the chassis /odom_raw and the EKF /odom.
TOPIC_ODOM = "/odom"
TOPIC_ODOM_RAW = "/odom_raw"
TOPIC_IMU = "/wit/imu"
TOPIC_SCAN = "/scan"
TOPIC_AMCL = "/amcl_pose"
TOPIC_CAMERA = "/usb_cam/image_raw"
TOPIC_MAP = "/map"
# global_planner/GlobalPlanner publishes ~/plan; move_base.launch remaps
# both local planners' local_plan to /move_base/local_plan.
TOPIC_GLOBAL_PLAN = "/move_base/GlobalPlanner/plan"
TOPIC_LOCAL_PLAN = "/move_base/local_plan"
TOPIC_GLOBAL_COSTMAP = "/move_base/global_costmap/costmap"
TOPIC_LOCAL_COSTMAP = "/move_base/local_costmap/costmap"
TOPIC_MISSION_STATE = "/mission/state"

MAP_FRAME = "map"
BASE_FRAMES = ("base_footprint", "base_link")
MISSION_NODE = "/community_mission"

#: Grid layers served by ``/api/nav/*.png``.
GRID_TOPICS = {
    "localization": TOPIC_MAP,
    "global_costmap": TOPIC_GLOBAL_COSTMAP,
    "local_costmap": TOPIC_LOCAL_COSTMAP,
}

_MAX_SCAN_POINTS = 360
_MAX_PATH_POINTS = 300
_ESTOP_TELEOP_HOLD_S = 1.0
_NODE_CACHE_S = 1.0


# ---------------------------------------------------------------------------
# Topic metadata tracker
# ---------------------------------------------------------------------------


@dataclass
class TopicMeta:
    hz: float = 0.0
    last_ts: float = 0.0
    msg_count: int = 0
    window_start: float = 0.0


class TopicTracker:
    """Tracks frequency and age for multiple topics."""

    def __init__(self) -> None:
        self._topics: Dict[str, TopicMeta] = {}
        self._lock = threading.Lock()

    def tick(self, topic: str) -> None:
        now = time.monotonic()
        with self._lock:
            meta = self._topics.get(topic)
            if meta is None:
                meta = self._topics[topic] = TopicMeta(window_start=now, last_ts=now)
            meta.last_ts = now
            meta.msg_count += 1
            elapsed = now - meta.window_start
            if elapsed >= 1.0:
                meta.hz = meta.msg_count / elapsed
                meta.msg_count = 0
                meta.window_start = now

    def snapshot(self) -> Dict[str, Dict[str, float]]:
        now = time.monotonic()
        with self._lock:
            result = {}
            for name, meta in self._topics.items():
                age = now - meta.last_ts
                # A topic silent for 2 s is reported as 0 Hz instead of its
                # last computed rate.
                result[name] = {"hz": round(meta.hz if age < 2.0 else 0.0, 1), "age_s": round(age, 1)}
            return result


# ---------------------------------------------------------------------------
# Small geometry helpers
# ---------------------------------------------------------------------------


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


@dataclass
class _Grid:
    """Occupancy grid kept as a numpy array (row 0 = map origin row)."""

    data: Any            # np.ndarray int8, shape (height, width)
    resolution: float
    origin: Tuple[float, float, float]
    frame: str
    version: int


# ---------------------------------------------------------------------------
# ROSBridge
# ---------------------------------------------------------------------------


class ROSBridge:
    """Single rospy node managing subscriptions, publishers, and caches."""

    def __init__(self, node_name: str = "infra") -> None:
        self.node_name = node_name
        self._tracker = TopicTracker()
        self._lock = threading.Lock()
        self._ready = threading.Event()
        self._shutdown = False

        self._amcl_pose: Optional[Dict[str, Any]] = None
        self._scan: Optional[Any] = None
        self._plans: Dict[str, Any] = {}
        self._grids: Dict[str, _Grid] = {}
        self._grid_version = 0
        self._mission: Optional[Dict[str, Any]] = None
        self._mission_ts = 0.0

        self._last_cmd_vel = 0.0
        self._teleop_hold_until = 0.0
        self._nodes: List[str] = []
        self._nodes_ts = 0.0

        self._pub_initialpose = None
        self._pub_goal = None
        self._pub_cmd_vel = None
        self._pub_cancel = None
        self._tf_buffer = None
        self._tf_listener = None
        self._subscribers: List[Any] = []

    # -- lifecycle -----------------------------------------------------------

    def initialize(self) -> None:
        """Start the rospy node in a background thread (non-blocking)."""
        if not _HAS_ROSPY:
            _LOG.error("Cannot initialize ROSBridge: rospy not available")
            return
        threading.Thread(target=self._init_ros, name="infra-ros-init", daemon=True).start()

    def _init_ros(self) -> None:
        try:
            # disable_signals: uvicorn owns SIGINT/SIGTERM; required off the main thread.
            rospy.init_node(self.node_name, anonymous=False, disable_signals=True)
            self._pub_initialpose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
            self._pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
            self._pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
            self._pub_cancel = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
            self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

            def tick_only(topic: str):
                return rospy.Subscriber(topic, rospy.AnyMsg, lambda _m: self._tracker.tick(topic), queue_size=1)

            self._subscribers = [
                tick_only(TOPIC_ODOM),
                tick_only(TOPIC_ODOM_RAW),
                tick_only(TOPIC_IMU),
                # AnyMsg: count camera frames without deserialising images.
                tick_only(TOPIC_CAMERA),
                rospy.Subscriber(TOPIC_SCAN, LaserScan, self._cb_scan, queue_size=1),
                rospy.Subscriber(TOPIC_AMCL, PoseWithCovarianceStamped, self._cb_amcl_pose, queue_size=1),
                rospy.Subscriber(TOPIC_GLOBAL_PLAN, Path, self._cb_plan, "global", queue_size=1),
                rospy.Subscriber(TOPIC_LOCAL_PLAN, Path, self._cb_plan, "local", queue_size=1),
                rospy.Subscriber(TOPIC_MISSION_STATE, String, self._cb_mission, queue_size=10),
            ]
            for layer, topic in GRID_TOPICS.items():
                self._subscribers.append(
                    rospy.Subscriber(topic, OccupancyGrid, self._cb_grid, layer, queue_size=1, buff_size=2 ** 24))
            for layer in ("global_costmap", "local_costmap"):
                self._subscribers.append(rospy.Subscriber(
                    GRID_TOPICS[layer] + "_updates", OccupancyGridUpdate, self._cb_grid_update, layer, queue_size=5))

            rospy.Timer(rospy.Duration(0.1), self._check_cmd_vel_timeout)
            self._ready.set()
            _LOG.info("ROS bridge ready")
        except Exception:
            _LOG.exception("Failed to initialize ROS bridge")

    def shutdown(self) -> None:
        self._shutdown = True
        if _HAS_ROSPY and self._ready.is_set() and not rospy.is_shutdown():
            self._publish_zero()
            rospy.signal_shutdown("infra shutdown")

    @property
    def ready(self) -> bool:
        return self._ready.is_set()

    # -- callbacks -----------------------------------------------------------

    def _cb_scan(self, msg) -> None:
        self._tracker.tick(TOPIC_SCAN)
        self._scan = msg

    def _cb_amcl_pose(self, msg) -> None:
        self._tracker.tick(TOPIC_AMCL)
        pose = msg.pose.pose
        self._amcl_pose = {"x": pose.position.x, "y": pose.position.y,
                           "yaw": _yaw(pose.orientation), "frame": msg.header.frame_id}

    def _cb_plan(self, msg, kind: str) -> None:
        self._tracker.tick(TOPIC_GLOBAL_PLAN if kind == "global" else TOPIC_LOCAL_PLAN)
        poses = msg.poses
        step = max(1, len(poses) // _MAX_PATH_POINTS)
        points = [(p.pose.position.x, p.pose.position.y) for p in poses[::step]]
        if poses and step > 1:
            last = poses[-1].pose.position
            points.append((last.x, last.y))
        self._plans[kind] = (msg.header.frame_id, points, time.monotonic())

    def _cb_grid(self, msg, layer: str) -> None:
        self._tracker.tick(GRID_TOPICS[layer])
        info = msg.info
        # rospy deserialises int8[] as a tuple of ints.
        data = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
        origin = (info.origin.position.x, info.origin.position.y, _yaw(info.origin.orientation))
        with self._lock:
            self._grid_version += 1
            self._grids[layer] = _Grid(data, info.resolution, origin, msg.header.frame_id, self._grid_version)

    def _cb_grid_update(self, msg, layer: str) -> None:
        with self._lock:
            grid = self._grids.get(layer)
            if grid is None:
                return
            h, w = grid.data.shape
            if msg.x + msg.width > w or msg.y + msg.height > h:
                return  # stale update for a resized grid; next full message fixes it
            patch = np.array(msg.data, dtype=np.int8).reshape(msg.height, msg.width)
            grid.data[msg.y:msg.y + msg.height, msg.x:msg.x + msg.width] = patch
            self._grid_version += 1
            grid.version = self._grid_version

    def _cb_mission(self, msg) -> None:
        self._tracker.tick(TOPIC_MISSION_STATE)
        try:
            state = json.loads(msg.data)
        except ValueError:
            state = None
        if not isinstance(state, dict):
            _LOG.warning("invalid /mission/state payload: %r", msg.data[:200])
            return
        with self._lock:
            self._mission = state
            self._mission_ts = time.time()

    # -- publishers ----------------------------------------------------------

    def _require(self, pub, what: str):
        if pub is None:
            raise RuntimeError(f"ROS 未连接，无法发布 {what}")
        return pub

    def publish_initial_pose(self, x: float, y: float, yaw: float, frame: str = MAP_FRAME) -> None:
        """Publish a 2D Pose Estimate to /initialpose."""
        pub = self._require(self._pub_initialpose, "/initialpose")
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Same covariance RViz uses for 2D Pose Estimate.
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942
        pub.publish(msg)
        _LOG.info("initial pose x=%.2f y=%.2f yaw=%.2f", x, y, yaw)

    def publish_nav_goal(self, x: float, y: float, yaw: float, frame: str = MAP_FRAME) -> None:
        """Publish a navigation goal to /move_base_simple/goal."""
        pub = self._require(self._pub_goal, "/move_base_simple/goal")
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        pub.publish(msg)
        _LOG.info("nav goal x=%.2f y=%.2f yaw=%.2f", x, y, yaw)

    def cancel_navigation(self) -> None:
        """Cancel every move_base goal (empty GoalID = cancel all)."""
        self._require(self._pub_cancel, "/move_base/cancel").publish(GoalID())

    def publish_cmd_vel(self, linear: float, angular: float, lateral: float = 0.0) -> None:
        """Publish a clamped velocity; used by teleop."""
        pub = self._require(self._pub_cmd_vel, "/cmd_vel")
        if time.monotonic() < self._teleop_hold_until:
            linear = angular = lateral = 0.0  # just after an e-stop: ignore teleop input
        speed = math.hypot(linear, lateral)
        if speed > config.MAX_LINEAR_SPEED:
            linear = linear / speed * config.MAX_LINEAR_SPEED
            lateral = lateral / speed * config.MAX_LINEAR_SPEED
        msg = Twist()
        msg.linear.x = linear
        msg.linear.y = lateral
        msg.angular.z = max(-config.MAX_ANGULAR_SPEED, min(config.MAX_ANGULAR_SPEED, angular))
        pub.publish(msg)
        self._last_cmd_vel = time.monotonic()

    def _publish_zero(self) -> None:
        if self._pub_cmd_vel is not None:
            self._pub_cmd_vel.publish(Twist())

    def _check_cmd_vel_timeout(self, _event: Any) -> None:
        """Send zero velocity once if teleop stopped sending for 500 ms."""
        if self._shutdown:
            return
        if self._last_cmd_vel > 0 and time.monotonic() - self._last_cmd_vel > config.TELEOP_HEARTBEAT_MS / 1000.0:
            self._last_cmd_vel = 0.0
            self._publish_zero()

    def kill_mission_node(self) -> bool:
        """Shut down /community_mission if it is running. Returns True if killed.

        mission.launch marks the node ``required``: when it exits, the
        roslaunch that contains it (field / finals / community with
        run_mission) shuts down as a whole, exactly as after a normal
        mission end.
        """
        if MISSION_NODE not in self.get_ros_nodes(max_age=0.0):
            return False
        success, _fail = rosnode.kill_nodes([MISSION_NODE])
        if MISSION_NODE not in success:
            raise RuntimeError(f"rosnode kill {MISSION_NODE} 失败")
        self._nodes_ts = 0.0
        return True

    def emergency_stop(self) -> List[str]:
        """Zero velocity → cancel navigation → stop mission → zero again.

        The chassis driver keeps running so it can hold the zero command.
        Returns the list of steps performed.
        """
        if not self.ready:
            raise RuntimeError("ROS 未连接，无法发送零速度")
        steps = []
        self._teleop_hold_until = time.monotonic() + _ESTOP_TELEOP_HOLD_S
        self._last_cmd_vel = 0.0
        for _ in range(3):
            self._publish_zero()
            time.sleep(0.02)
        steps.append("已发送零速度")
        self.cancel_navigation()
        steps.append("已取消导航目标")
        try:
            if self.kill_mission_node():
                steps.append("已停止巡检节点（mission 为 required，包含它的 launch 会整体退出，"
                             "底盘驱动退出时再次下发零速度）")
        except Exception as exc:  # keep going: the robot is already stopped
            _LOG.warning("failed to kill mission node: %s", exc)
            steps.append(f"停止巡检节点失败：{exc}")
        self._publish_zero()
        return steps

    # -- queries -------------------------------------------------------------

    def get_topics_snapshot(self) -> Dict[str, Dict[str, float]]:
        return self._tracker.snapshot()

    def _lookup(self, target: str, source: str, max_age: Optional[float] = None):
        """Return (x, y, yaw) of ``source`` frame in ``target`` frame, or None."""
        if self._tf_buffer is None or not source:
            return None
        if source.lstrip("/") == target:
            return 0.0, 0.0, 0.0
        try:
            tf = self._tf_buffer.lookup_transform(target, source.lstrip("/"), rospy.Time(0), rospy.Duration(0.05))
        except Exception:
            return None
        if max_age is not None:
            age = (rospy.Time.now() - tf.header.stamp).to_sec()
            if tf.header.stamp.is_zero() or not -0.1 <= age <= max_age:
                return None
        t = tf.transform.translation
        return t.x, t.y, _yaw(tf.transform.rotation)

    @staticmethod
    def _apply(tf, points):
        tx, ty, yaw = tf
        c, s = math.cos(yaw), math.sin(yaw)
        return [[round(tx + c * x - s * y, 3), round(ty + s * x + c * y, 3)] for x, y in points]

    def get_pose(self) -> Optional[Dict[str, Any]]:
        """Robot pose in the map frame: TF first, AMCL message as fallback."""
        for base in BASE_FRAMES:
            tf = self._lookup(MAP_FRAME, base)
            if tf is not None:
                return {"x": round(tf[0], 3), "y": round(tf[1], 3), "yaw": round(tf[2], 3), "source": "tf"}
        if self._amcl_pose is not None:
            p = self._amcl_pose
            return {"x": round(p["x"], 3), "y": round(p["y"], 3), "yaw": round(p["yaw"], 3), "source": "amcl"}
        return None

    def get_waypoint_pose(self) -> Optional[Dict[str, float]]:
        for base in BASE_FRAMES:
            pose = self._lookup(MAP_FRAME, base, max_age=1.0)
            if pose is not None and all(math.isfinite(value) for value in pose):
                return {key: round(value, 4) for key, value in zip(("x", "y", "yaw"), pose)}
        return None

    def get_scan_points(self) -> List[List[float]]:
        scan = self._scan
        if scan is None:
            return []
        tf = self._lookup(MAP_FRAME, scan.header.frame_id)
        if tf is None:
            return []
        step = max(1, len(scan.ranges) // _MAX_SCAN_POINTS)
        points = []
        for i in range(0, len(scan.ranges), step):
            r = scan.ranges[i]
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                a = scan.angle_min + i * scan.angle_increment
                points.append((r * math.cos(a), r * math.sin(a)))
        return self._apply(tf, points)

    def get_plan(self, kind: str) -> List[List[float]]:
        entry = self._plans.get(kind)
        if entry is None:
            return []
        frame, points, ts = entry
        if not points or time.monotonic() - ts > 5.0:
            return []  # plan not refreshed: navigation finished or stopped
        tf = self._lookup(MAP_FRAME, frame)
        return self._apply(tf, points) if tf is not None else []

    def get_overlay(self) -> Dict[str, Any]:
        """Everything the navigation canvas draws on top of the map."""
        with self._lock:
            versions = {k: g.version for k, g in self._grids.items()}
        return {
            "ready": self.ready,
            "pose": self.get_pose(),
            "scan": self.get_scan_points(),
            "global_plan": self.get_plan("global"),
            "local_plan": self.get_plan("local"),
            "grid_versions": versions,
        }

    def get_grid(self, layer: str) -> Optional[_Grid]:
        """Copy of a cached grid (safe to use outside the lock)."""
        with self._lock:
            grid = self._grids.get(layer)
            if grid is None:
                return None
            return _Grid(grid.data.copy(), grid.resolution, grid.origin, grid.frame, grid.version)

    def grid_origin_in_map(self, grid: _Grid) -> Optional[Tuple[float, float, float]]:
        """Grid origin expressed in the map frame (local costmap lives in odom)."""
        tf = self._lookup(MAP_FRAME, grid.frame or MAP_FRAME)
        if tf is None:
            return None
        ox, oy, oyaw = grid.origin
        (x, y), = self._apply(tf, [(ox, oy)])
        return x, y, tf[2] + oyaw

    def get_mission_state(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            if self._mission is None:
                return None
            state = dict(self._mission)
            state["received_age_s"] = round(time.time() - self._mission_ts, 1)
        state["node_alive"] = MISSION_NODE in self.get_ros_nodes()
        return state

    def get_ros_nodes(self, max_age: float = _NODE_CACHE_S) -> List[str]:
        """Active ROS nodes (cached briefly; master queries are XML-RPC)."""
        if not _HAS_ROSPY:
            return []
        now = time.monotonic()
        if now - self._nodes_ts > max_age:
            try:
                self._nodes = rosnode.get_node_names()
            except Exception:
                self._nodes = []
            self._nodes_ts = now
        return list(self._nodes)

    def is_master_alive(self) -> bool:
        if not _HAS_ROSPY:
            return False
        try:
            rosgraph.Master("/infra_probe").getPid()
            return True
        except Exception:
            return False


# ---------------------------------------------------------------------------
# Singleton instance
# ---------------------------------------------------------------------------

_bridge: Optional[ROSBridge] = None


def get_bridge() -> ROSBridge:
    """Return the singleton ROSBridge instance."""
    global _bridge
    if _bridge is None:
        _bridge = ROSBridge()
    return _bridge
