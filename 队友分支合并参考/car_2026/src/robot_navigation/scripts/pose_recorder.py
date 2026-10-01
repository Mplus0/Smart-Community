#!/usr/bin/env python3
import csv
import math
import os
import select
import sys
import termios
import time
import tty
from datetime import datetime
from threading import Lock

import rospkg
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class PoseRecorder:
    def __init__(self):
        source_type = rospy.get_param("~source_type", "odom").strip().lower()
        topic_override = rospy.get_param("~topic", "").strip()
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")

        self.lock = Lock()
        self.latest_pose = None
        self.point_count = 0

        if source_type in ("odom", "filtered_odom"):
            self.source_type = "odom"
            self.topic = topic_override or "/odometry/filtered"
            self.subscriber = rospy.Subscriber(
                self.topic, Odometry, self._odom_callback, queue_size=1
            )
        elif source_type in ("amcl", "map"):
            self.source_type = "amcl"
            self.topic = topic_override or "/amcl_pose"
            self.subscriber = rospy.Subscriber(
                self.topic, PoseWithCovarianceStamped,
                self._amcl_callback, queue_size=1
            )
        else:
            raise ValueError("~source_type must be 'odom' or 'amcl'")

        try:
            package_dir = rospkg.RosPack().get_path("robot_navigation")
        except rospkg.ResourceNotFound as exc:
            raise RuntimeError("找不到 robot_navigation 功能包") from exc

        default_dir = os.path.join(package_dir, "pose_records")
        os.makedirs(default_dir, exist_ok=True)
        default_name = "pose_{}.csv".format(time.strftime("%Y%m%d_%H%M%S"))
        self.output_file = os.path.expanduser(
            rospy.get_param("~output_file", os.path.join(default_dir, default_name))
        )
        output_dir = os.path.dirname(os.path.abspath(self.output_file))
        os.makedirs(output_dir, exist_ok=True)

        with open(self.output_file, "a", newline="", encoding="utf-8") as csv_file:
            if os.path.getsize(self.output_file) == 0:
                csv.writer(csv_file).writerow([
                    "point_id", "saved_at", "source_topic", "frame_id",
                    "base_frame", "x", "y", "z", "qx", "qy", "qz", "qw",
                    "yaw_rad", "yaw_deg", "source_stamp_sec"
                ])

    @staticmethod
    def _yaw_from_quaternion(q):
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(sin_yaw, cos_yaw)

    def _cache_pose(self, msg, pose, frame_id, base_frame):
        q = pose.orientation
        yaw = self._yaw_from_quaternion(q)
        snapshot = {
            "frame_id": frame_id or "unknown",
            "base_frame": base_frame or self.base_frame,
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "qx": q.x,
            "qy": q.y,
            "qz": q.z,
            "qw": q.w,
            "yaw_rad": yaw,
            "yaw_deg": math.degrees(yaw),
            "source_stamp_sec": msg.header.stamp.to_sec(),
        }
        with self.lock:
            self.latest_pose = snapshot

    def _odom_callback(self, msg):
        self._cache_pose(
            msg, msg.pose.pose, msg.header.frame_id, msg.child_frame_id
        )

    def _amcl_callback(self, msg):
        self._cache_pose(
            msg, msg.pose.pose, msg.header.frame_id, self.base_frame
        )

    def save_current_pose(self):
        with self.lock:
            if self.latest_pose is None:
                rospy.logwarn("还没有从 %s 收到位姿，本次未保存。", self.topic)
                return
            snapshot = dict(self.latest_pose)

        next_count = self.point_count + 1
        point_id = "P{:03d}".format(next_count)
        row = [
            point_id,
            datetime.now().isoformat(timespec="milliseconds"),
            self.topic,
            snapshot["frame_id"],
            snapshot["base_frame"],
            snapshot["x"],
            snapshot["y"],
            snapshot["z"],
            snapshot["qx"],
            snapshot["qy"],
            snapshot["qz"],
            snapshot["qw"],
            snapshot["yaw_rad"],
            snapshot["yaw_deg"],
            snapshot["source_stamp_sec"],
        ]

        try:
            with open(self.output_file, "a", newline="", encoding="utf-8") as csv_file:
                csv.writer(csv_file).writerow(row)
                csv_file.flush()
        except OSError as exc:
            rospy.logerr("无法写入位姿 CSV %s：%s", self.output_file, exc)
            return

        self.point_count = next_count
        rospy.loginfo(
            "第 %d 次保存成功（%s）：坐标系=%s，基座=%s，"
            "位置 x=%.3f m，y=%.3f m，z=%.3f m，"
            "航向 yaw=%.3f rad（%.2f deg）",
            self.point_count,
            point_id,
            snapshot["frame_id"],
            snapshot["base_frame"],
            snapshot["x"],
            snapshot["y"],
            snapshot["z"],
            snapshot["yaw_rad"],
            snapshot["yaw_deg"],
        )

    def run_keyboard(self):
        if not sys.stdin.isatty():
            rospy.logerr(
                "请在交互式 WSL 终端运行此节点，才能使用按键控制。"
            )
            return

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            rospy.loginfo(
                "位姿记录器已启动：轻按 s 保存当前位置，"
                "按 q 退出。"
            )
            rospy.loginfo("位姿话题：%s", self.topic)
            rospy.loginfo("CSV 文件：%s", self.output_file)

            while not rospy.is_shutdown():
                readable, _, _ = select.select([sys.stdin], [], [], 0.2)
                if not readable:
                    continue
                key = sys.stdin.read(1).lower()
                if key == "s":
                    self.save_current_pose()
                elif key == "q":
                    rospy.loginfo("收到 q，退出位姿记录器。")
                    rospy.signal_shutdown("quit key pressed")
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main():
    rospy.init_node("pose_recorder")
    try:
        recorder = PoseRecorder()
    except (ValueError, OSError, RuntimeError) as exc:
        rospy.logerr("位姿记录器启动失败：%s", exc)
        return

    try:
        recorder.run_keyboard()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
