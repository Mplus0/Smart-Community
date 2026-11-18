"""ROS Bridge: Single rospy node managing all subscriptions and publishers.

Batch 2 scope:
- Subscribe to core topics: /odom, /scan, /imu, /amcl_pose, camera images
- Subscribe to map topics: /map, /navigation_map
- Subscribe to navigation topics: global/local plans, costmaps
- Publish: /initialpose (2D Pose Estimate), /move_base_simple/goal, /cmd_vel
- Maintain topic frequency & age dict for Dashboard
- Provide cmd_vel with heartbeat timeout (500ms → zero velocity)
- Expose simple publish interfaces for other modules
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

_LOG = logging.getLogger("infra.ros_bridge")

# ROS imports -- fail gracefully if rospy not available (for unit tests)
try:
    import rospy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from sensor_msgs.msg import Image, Imu, LaserScan
    from std_msgs.msg import String

    _HAS_ROSPY = True
except ImportError:
    _HAS_ROSPY = False
    _LOG.warning("rospy unavailable — ros_bridge will be inert")


# ---------------------------------------------------------------------------
# Topic metadata tracker
# ---------------------------------------------------------------------------


@dataclass
class TopicMeta:
    """Per-topic metadata: frequency and age."""

    hz: float = 0.0  # Computed frequency
    age_s: float = 999.0  # Seconds since last message
    last_ts: float = 0.0  # Internal: time.monotonic() of last message
    msg_count: int = 0  # Internal: message counter for Hz computation
    window_start: float = 0.0  # Internal: start of current 1s window


class TopicTracker:
    """Tracks frequency and age for multiple topics."""

    def __init__(self) -> None:
        self._topics: Dict[str, TopicMeta] = {}
        self._lock = threading.Lock()

    def tick(self, topic: str) -> None:
        """Record a new message arrival on ``topic``."""
        now = time.monotonic()
        with self._lock:
            if topic not in self._topics:
                self._topics[topic] = TopicMeta(
                    window_start=now, last_ts=now
                )
            meta = self._topics[topic]
            meta.last_ts = now
            meta.msg_count += 1
            # Recompute Hz every 1 second
            elapsed = now - meta.window_start
            if elapsed >= 1.0:
                meta.hz = meta.msg_count / elapsed
                meta.msg_count = 0
                meta.window_start = now

    def snapshot(self) -> Dict[str, Dict[str, float]]:
        """Return {topic: {"hz": ..., "age_s": ...}} dict for all topics."""
        now = time.monotonic()
        with self._lock:
            return {
                name: {"hz": meta.hz, "age_s": now - meta.last_ts}
                for name, meta in self._topics.items()
            }


# ---------------------------------------------------------------------------
# Main ROSBridge
# ---------------------------------------------------------------------------


class ROSBridge:
    """Single rospy node managing subscriptions, publishers, and state."""

    def __init__(self, node_name: str = "infra") -> None:
        self.node_name = node_name
        self._tracker = TopicTracker()
        self._lock = threading.Lock()

        # Cached state
        self._pose: Dict[str, Any] = {}
        self._ros_nodes: List[str] = []
        self._last_cmd_vel: float = 0.0  # Timestamp of last cmd_vel command
        self._cmd_vel_timeout: float = 0.5  # 500ms as per WORKBENCH.md §4.2

        # Publishers
        self._pub_initialpose: Optional[Any] = None
        self._pub_goal: Optional[Any] = None
        self._pub_cmd_vel: Optional[Any] = None

        # Subscribers
        self._subscribers: List[Any] = []

        # Flag for shutdown
        self._shutdown = False

    def initialize(self) -> None:
        """Initialize the ROS node and set up subscriptions/publishers."""
        if not _HAS_ROSPY:
            _LOG.error("Cannot initialize ROSBridge: rospy not available")
            return

        try:
            rospy.init_node(self.node_name, anonymous=False, log_level=rospy.INFO)
            _LOG.info(f"ROS node '{self.node_name}' initialized")

            # Publishers
            self._pub_initialpose = rospy.Publisher(
                "/initialpose", PoseWithCovarianceStamped, queue_size=1
            )
            self._pub_goal = rospy.Publisher(
                "/move_base_simple/goal", PoseStamped, queue_size=1
            )
            self._pub_cmd_vel = rospy.Publisher(
                "/cmd_vel", Twist, queue_size=1
            )

            # Subscribers
            self._subscribers = [
                rospy.Subscriber("/odom", Odometry, self._cb_odom),
                rospy.Subscriber("/scan", LaserScan, self._cb_scan),
                rospy.Subscriber("/imu", Imu, self._cb_imu),
                rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._cb_amcl_pose),
                rospy.Subscriber("/usb_cam/image_raw", Image, self._cb_camera_image),
                rospy.Subscriber("/map", OccupancyGrid, self._cb_map),
                rospy.Subscriber("/navigation_map", OccupancyGrid, self._cb_navigation_map),
                rospy.Subscriber("/move_base/global_plan", Path, self._cb_global_plan),
                rospy.Subscriber("/move_base/local_plan", Path, self._cb_local_plan),
            ]

            # Start heartbeat timer for cmd_vel timeout
            rospy.Timer(rospy.Duration(0.1), self._check_cmd_vel_timeout)

            _LOG.info("ROS subscriptions and publishers ready")
        except Exception as e:
            _LOG.exception(f"Failed to initialize ROS bridge: {e}")

    def shutdown(self) -> None:
        """Shutdown ROS node cleanly."""
        self._shutdown = True
        if _HAS_ROSPY and not rospy.is_shutdown():
            _LOG.info("Shutting down ROS node")
            # Send zero velocity before shutting down
            self.publish_cmd_vel(0.0, 0.0)
            rospy.signal_shutdown("infra shutdown")

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _cb_odom(self, msg: Odometry) -> None:
        self._tracker.tick("/odom")

    def _cb_scan(self, msg: LaserScan) -> None:
        self._tracker.tick("/scan")

    def _cb_imu(self, msg: Imu) -> None:
        self._tracker.tick("/imu")

    def _cb_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """Update pose from AMCL."""
        self._tracker.tick("/amcl_pose")
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        # Convert quaternion to yaw (simplified 2D case)
        import math
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        with self._lock:
            self._pose = {
                "x": round(pos.x, 2),
                "y": round(pos.y, 2),
                "yaw": round(yaw, 2),
                "frame": msg.header.frame_id,
            }

    def _cb_camera_image(self, msg: Image) -> None:
        self._tracker.tick("/usb_cam/image_raw")

    def _cb_map(self, msg: OccupancyGrid) -> None:
        self._tracker.tick("/map")
        # TODO: cache map for GET /api/nav/map.png

    def _cb_navigation_map(self, msg: OccupancyGrid) -> None:
        self._tracker.tick("/navigation_map")
        # TODO: cache navigation_map

    def _cb_global_plan(self, msg: Path) -> None:
        self._tracker.tick("/move_base/global_plan")
        # TODO: cache for visualization

    def _cb_local_plan(self, msg: Path) -> None:
        self._tracker.tick("/move_base/local_plan")
        # TODO: cache for visualization

    # -----------------------------------------------------------------------
    # Publish interfaces
    # -----------------------------------------------------------------------

    def publish_initial_pose(self, x: float, y: float, yaw: float, frame: str = "map") -> None:
        """Publish a 2D Pose Estimate to /initialpose."""
        if not self._pub_initialpose:
            _LOG.error("initialpose publisher not ready")
            return

        import math
        # Convert yaw to quaternion (2D rotation around Z axis)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # Set covariance (small values to indicate confidence)
        msg.pose.covariance[0] = 0.25  # x variance
        msg.pose.covariance[7] = 0.25  # y variance
        msg.pose.covariance[35] = 0.068  # yaw variance

        self._pub_initialpose.publish(msg)
        _LOG.info(f"Published initial pose: x={x}, y={y}, yaw={yaw}")

    def publish_nav_goal(self, x: float, y: float, yaw: float, frame: str = "map") -> None:
        """Publish a navigation goal to /move_base_simple/goal."""
        if not self._pub_goal:
            _LOG.error("goal publisher not ready")
            return

        import math
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self._pub_goal.publish(msg)
        _LOG.info(f"Published nav goal: x={x}, y={y}, yaw={yaw}")

    def publish_cmd_vel(self, linear: float, angular: float) -> None:
        """Publish velocity command to /cmd_vel with speed limits."""
        if not self._pub_cmd_vel:
            _LOG.error("cmd_vel publisher not ready")
            return

        from . import config
        # Clamp to limits
        linear = max(-config.MAX_LINEAR_SPEED, min(config.MAX_LINEAR_SPEED, linear))
        angular = max(-config.MAX_ANGULAR_SPEED, min(config.MAX_ANGULAR_SPEED, angular))

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        self._pub_cmd_vel.publish(msg)
        self._last_cmd_vel = time.monotonic()

    def _check_cmd_vel_timeout(self, event: Any) -> None:
        """Timer callback to send zero velocity if no recent commands."""
        if self._shutdown:
            return
        now = time.monotonic()
        if self._last_cmd_vel > 0 and (now - self._last_cmd_vel) > self._cmd_vel_timeout:
            # Timeout - send zero velocity
            self._last_cmd_vel = 0.0  # Reset to prevent repeated sends
            msg = Twist()  # All zeros
            if self._pub_cmd_vel:
                self._pub_cmd_vel.publish(msg)

    # -----------------------------------------------------------------------
    # State queries
    # -----------------------------------------------------------------------

    def get_topics_snapshot(self) -> Dict[str, Dict[str, float]]:
        """Return {topic: {"hz": ..., "age_s": ...}} dict."""
        return self._tracker.snapshot()

    def get_pose(self) -> Dict[str, Any]:
        """Return current robot pose from AMCL."""
        with self._lock:
            return dict(self._pose)

    def get_ros_nodes(self) -> List[str]:
        """Return list of active ROS nodes."""
        if not _HAS_ROSPY or rospy.is_shutdown():
            return []
        try:
            import rosnode
            return rosnode.get_node_names()
        except Exception as e:
            _LOG.warning(f"Failed to get ROS nodes: {e}")
            return []

    def is_master_alive(self) -> bool:
        """Check if ROS Master is reachable."""
        if not _HAS_ROSPY:
            return False
        try:
            rospy.get_master().getPid()
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
