"""Sequential mission execution with a lightweight image callback."""
import math
from pathlib import Path
import threading
import time

import actionlib
import cv2
from cv_bridge import CvBridge
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image

from .rules import validate_route, detections
from .services import Services, SpeechQueue
from .inspection import Inspection


class Mission:
    def __init__(self):
        self.route = validate_route(rospy.get_param('~'))
        self.goal_timeout = float(rospy.get_param('~goal_timeout', 120.0))
        self.image_timeout = float(rospy.get_param('~image_timeout', 5.0))
        self.light_timeout = float(rospy.get_param('~traffic_light_timeout', 15.0))
        if any(not math.isfinite(v) or v <= 0 for v in
               (self.goal_timeout, self.image_timeout, self.light_timeout)):
            raise ValueError('Timeouts must be finite and positive')
        self.data_dir = Path(rospy.get_param('~data_dir', '~/.ros/community')).expanduser()
        for folder in ('photo', 'videos'):
            (self.data_dir/folder).mkdir(parents=True, exist_ok=True)
        self.planner = rospy.get_param('~planner_namespace', '/move_base/DWAPlannerROS')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.services = Services()
        self.bridge = CvBridge()
        self.condition = threading.Condition()
        self.latest = None
        self.sequence = 0
        self.recording = False
        self.closed = False
        self.speech = SpeechQueue(self.services)
        self.inspection = Inspection(self.route, self.services, self.speech)
        self.subscriber = rospy.Subscriber(rospy.get_param('~image_topic', '/usb_cam/image_raw'),
                                           Image, self.image_callback, queue_size=1, buff_size=2**24)
        rospy.on_shutdown(self.close)

    def image_callback(self, image):
        with self.condition:
            self.latest = image
            self.sequence += 1
            self.condition.notify_all()

    def fresh_image(self, timeout=None):
        deadline = time.monotonic()+(self.image_timeout if timeout is None else timeout)
        with self.condition:
            previous = self.sequence
            while not rospy.is_shutdown() and self.sequence == previous:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise RuntimeError('Timed out waiting for a fresh camera frame')
                self.condition.wait(min(remaining, 0.1))
            if rospy.is_shutdown():
                raise RuntimeError('Mission stopped')
            return self.latest

    def move_to(self, coordinates):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        x, y, yaw = coordinates
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = math.sin(yaw/2)
        goal.target_pose.pose.orientation.w = math.cos(yaw/2)
        self.client.send_goal(goal)
        deadline = time.monotonic()+self.goal_timeout
        while not rospy.is_shutdown():
            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                return
            if state not in (GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.PREEMPTING,
                             GoalStatus.RECALLING):
                raise RuntimeError(f'Navigation failed with state {state}')
            if time.monotonic() >= deadline:
                self.client.cancel_goal()
                raise RuntimeError('Navigation goal timed out')
            time.sleep(0.05)
        raise RuntimeError('Mission stopped')

    def wait_for_green(self):
        deadline = time.monotonic()+self.light_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            image = self.fresh_image(min(self.image_timeout, max(0.01, deadline-time.monotonic())))
            objects = detections(self.services.detect('traffic_light_detector', image))
            if time.monotonic() < deadline and any(obj.label == 'green_light' for obj in objects):
                return
        raise RuntimeError('Traffic light timeout; mission stopped')

    def run(self):
        if not self.client.wait_for_server(rospy.Duration(30)):
            raise RuntimeError('move_base is unavailable')
        self.services.set_lateral_speed(self.planner, 0.5)
        self.speech.put('我要验牌')
        for index, goal in enumerate(self.route['goals']):
            self.move_to(goal)
            rospy.sleep(1.2)
            if index in self.route['traffic_light_idx']:
                self.wait_for_green()
            if any(index in self.route[key] for key in
                   ('photo_idx', 'license_plate_idx', 'fire_idx', 'trash_can_idx', 'electrocar_idx')):
                image = self.fresh_image()
                frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
                if not cv2.imwrite(str(self.data_dir/'photo'/f'{index:03d}.jpg'), frame):
                    raise RuntimeError('Failed to save inspection image')
                self.inspection.run_at(index, image)
            if index == self.route['announcement_index']:
                self.speech.put('前方发现禁止直行标识')
            if index == self.route['record_start_index']:
                self.services.set_lateral_speed(self.planner, 0.15)
                self.services.start_recording(self.data_dir/'videos/video.mp4')
                self.recording = True
            if index == self.route['record_stop_index']:
                self.services.stop_recording()
                self.recording = False
                self.services.set_lateral_speed(self.planner, 0.5)
                self.inspection.inspect_video(self.data_dir/'videos/video.mp4')
        if not self.speech.drain():
            rospy.logwarn('Speech queue did not finish before its deadline')

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.client.cancel_all_goals()
        if self.recording:
            try:
                self.services.stop_recording()
            except Exception as error:
                rospy.logerr('Failed to stop recorder: %s', error)
            self.recording = False
        self.subscriber.unregister()
        self.speech.close()


def main():
    rospy.init_node('community_mission')
    mission = None
    try:
        mission = Mission()
        mission.run()
        return 0
    except Exception as error:
        rospy.logfatal('Mission stopped: %s', error)
        return 1
    finally:
        if mission is not None:
            mission.close()
