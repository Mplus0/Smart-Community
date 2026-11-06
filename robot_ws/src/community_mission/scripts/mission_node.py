#!/usr/bin/env python3
"""Explicit mission states with a lightweight image callback."""
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

from rules import validate_route, detections
from services import Services, SpeechQueue
from inspection import Inspection
from result_manager import ResultManager


class Mission:
    def __init__(self):
        self.state = "INIT"
        self.index = 0
        self.reason = "等待初始化"
        self.stopping = False
        self.route = validate_route(rospy.get_param('~'))
        self.runtime = rospy.get_param('~mission')
        self.goal_timeout = float(rospy.get_param('~goal_timeout', self.runtime['goal_timeout']))
        self.image_timeout = float(rospy.get_param('~image_timeout', self.runtime['image_timeout']))
        self.light_timeout = float(rospy.get_param('~traffic_light_timeout', self.runtime['traffic_light_timeout']))
        self.green_confirm_frames = rospy.get_param('~green_confirm_frames', self.runtime['green_confirm_frames'])
        self.navigation_retries = rospy.get_param('~navigation_retries', self.runtime['navigation_retries'])
        self.cancel_timeout = float(rospy.get_param('~navigation_cancel_timeout', self.runtime['navigation_cancel_timeout']))
        if type(self.green_confirm_frames) is not int or self.green_confirm_frames < 1:
            raise ValueError('green_confirm_frames must be a positive integer')
        if type(self.navigation_retries) is not int or self.navigation_retries < 0:
            raise ValueError('navigation_retries must be a nonnegative integer')
        if any(not math.isfinite(v) or v <= 0 for v in
               (self.goal_timeout, self.image_timeout, self.light_timeout, self.cancel_timeout)):
            raise ValueError('Timeouts must be finite and positive')
        if any(not math.isfinite(self.runtime[k]) or self.runtime[k] <= 0 for k in
               ('server_timeout', 'arrival_wait', 'ocr_interval', 'service_timeout',
                'planner_service_timeout', 'speech_drain_timeout', 'speech_join_timeout')):
            raise ValueError('Runtime intervals must be finite and positive')
        self.results = ResultManager(rospy.get_param('~data_dir', '~/.ros/community'),
                                     self.route['points'], rospy.loginfo)
        self.data_dir = self.results.path
        self.video_path = self.data_dir/'videos/video.mp4'
        self.planner = rospy.get_param('~planner_namespace', '/move_base/DWAPlannerROS')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.services = Services(self.runtime)
        self.bridge = CvBridge()
        self.condition = threading.Condition()
        self.latest = None
        self.sequence = 0
        self.recording = False
        self.closed = False
        self.speech = SpeechQueue(self.services, self.runtime)
        self.inspection = Inspection(self.route, self.services, self.speech, rospy.get_param('~recognition_rules'),
                                     self.runtime['ocr_interval'])
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

    def check_running(self):
        if self.stopping or rospy.is_shutdown():
            raise RuntimeError('CANCELLED: Mission stopped')

    def stop_navigation(self):
        # Reuse the existing lock so shutdown cannot race with sending a goal.
        with self.condition:
            self.stopping = True
            self.client.cancel_goal()

    def cancel_for_retry(self):
        self.client.cancel_goal()
        deadline = time.monotonic() + self.cancel_timeout
        while time.monotonic() < deadline:
            self.check_running()
            state = self.client.get_state()
            if state in (GoalStatus.PREEMPTED, GoalStatus.RECALLED, GoalStatus.SUCCEEDED,
                         GoalStatus.ABORTED, GoalStatus.REJECTED):
                return
            if state == GoalStatus.LOST:
                raise RuntimeError('NAV_LOST: cancellation cannot be confirmed')
            time.sleep(0.05)
        raise RuntimeError('CANCEL_TIMEOUT: no new navigation goal will be sent')

    def move_to(self, coordinates):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        x, y, yaw = coordinates
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = math.sin(yaw/2)
        goal.target_pose.pose.orientation.w = math.cos(yaw/2)
        try:
            for attempt in range(self.navigation_retries + 1):
                with self.condition:
                    self.check_running()
                    goal.target_pose.header.stamp = rospy.Time.now()
                    self.client.send_goal(goal)
                deadline = time.monotonic() + self.goal_timeout
                while True:
                    self.check_running()
                    state = self.client.get_state()
                    if state == GoalStatus.SUCCEEDED:
                        return {'outcome': 'SUCCESS', 'attempts': attempt + 1}
                    if state in (GoalStatus.PREEMPTED, GoalStatus.RECALLED,
                                 GoalStatus.PREEMPTING, GoalStatus.RECALLING):
                        raise RuntimeError('NAV_CANCELLED: navigation cancelled externally')
                    if state == GoalStatus.LOST:
                        raise RuntimeError('NAV_LOST: navigation goal state lost')
                    if state in (GoalStatus.ABORTED, GoalStatus.REJECTED):
                        reason = f'NAV_FAILED: Navigation failed with state {state}'
                        break
                    if state not in (GoalStatus.PENDING, GoalStatus.ACTIVE):
                        raise RuntimeError(f'NAV_FAILED: unexpected state {state}')
                    if time.monotonic() >= deadline:
                        reason = 'NAV_TIMEOUT: Navigation goal timed out'
                        break
                    time.sleep(0.05)
                if attempt == self.navigation_retries:
                    raise RuntimeError(reason)
                rospy.logwarn('[导航] %s；取消确认后重试 %s/%s', reason, attempt + 1, self.navigation_retries)
                self.cancel_for_retry()
        except Exception:
            self.stop_navigation()
            raise

    def wait_for_green(self):
        deadline = time.monotonic() + self.light_timeout
        streak, last_stamp = 0, 0
        rospy.loginfo('[红绿灯] 进入等待：连续 %s 帧绿灯，超时 %.1f 秒',
                      self.green_confirm_frames, self.light_timeout)
        while time.monotonic() < deadline:
            self.check_running()
            image = self.fresh_image(min(self.image_timeout, max(0.01, deadline-time.monotonic())))
            stamp = image.header.stamp.to_nsec()
            if stamp <= last_stamp:
                streak = 0
                continue
            last_stamp = stamp
            objects = detections(self.services.detect('traffic_light_detector', image))
            self.check_running()
            if time.monotonic() >= deadline:
                break
            # Empty/red/yellow/conflicting/invalid detections break the streak.
            green = bool(objects) and all(
                obj.label == 'green_light' and math.isfinite(obj.confidence) and 0 < obj.confidence <= 1
                and all(math.isfinite(v) for v in obj.box)
                and obj.box[2] > obj.box[0] and obj.box[3] > obj.box[1] for obj in objects)
            streak = streak + 1 if green else 0
            if streak >= self.green_confirm_frames:
                rospy.loginfo('[红绿灯] 连续 %s 帧有效绿灯，允许通行', streak)
                return {'released': True, 'green_streak': streak,
                        'detected_classes': [obj.label for obj in objects]}
        raise RuntimeError('TRAFFIC_TIMEOUT: Traffic light timeout; mission stopped')

    def transition(self, state, reason):
        previous = self.state
        self.state, self.reason = state, reason
        self.results.state(state, self.route['points'][self.index], reason)
        if state == 'ERROR':
            rospy.logerr('[状态] %s -> ERROR: %s', previous, reason)

    def initialize(self):
        if not self.client.wait_for_server(rospy.Duration(self.runtime['server_timeout'])):
            raise RuntimeError('move_base is unavailable')
        self.services.set_lateral_speed(self.planner, self.runtime['normal_lateral_speed'])
        self.speech.put('我要验牌')
        self.transition('NAVIGATE', '导航服务已就绪')

    def navigate(self):
        rospy.loginfo('[导航] 前往航点=%s', self.route['points'][self.index]['id'])
        point = self.route['points'][self.index]
        self.results.waypoint(point, 'RUNNING')
        self.results.execute('navigation', point, self.move_to,
                             (point['x'], point['y'], point['yaw']))
        rospy.sleep(self.runtime['arrival_wait'])
        self.transition('WAIT_TRAFFIC' if 'traffic_light' in point.get('tasks', [])
                        else 'TASK', '已到达航点')

    def wait_traffic(self):
        rospy.loginfo('[红绿灯] 航点=%s，等待绿灯', self.route['points'][self.index]['id'])
        self.results.execute('traffic_light', self.route['points'][self.index], self.wait_for_green)
        self.transition('TASK', '已确认绿灯，继续本航点任务')

    def task(self):
        index = self.index
        point = self.route['points'][index]
        tasks = point.get('tasks', [])
        rospy.loginfo('[任务] 执行航点=%s 的全部任务', point['id'])
        # 保留独立 if，按原顺序完成同一航点的多个任务。
        if any(task in tasks for task in ('person', 'plate', 'fire', 'trash', 'vehicle_photo')):
            image_path = self.data_dir/'photo'/f'{index:03d}.jpg'
            image = self.fresh_image()
            frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            if not cv2.imwrite(str(image_path), frame):
                raise RuntimeError('Failed to save inspection image')
            for task_type in ('person', 'plate', 'fire', 'trash', 'vehicle_photo'):
                if task_type in tasks:
                    self.run_inspection(point, task_type, image, image_path)
        if 'announcement' in tasks:
            self.results.execute('announcement', point, self.announce)
        if 'record_start' in tasks:
            self.results.execute('record_start', point, self.start_recording,
                                 paths=[self.video_path])
        if 'record_stop' in tasks:
            self.results.execute('record_stop', point, self.stop_recording,
                                 paths=[self.video_path])
            self.run_inspection(point, 'vehicle_video', self.video_path,
                                self.video_path)
        self.results.waypoint(point, 'SUCCESS')
        self.transition('NEXT_POINT', '本航点任务完成')

    def run_inspection(self, point, task_type, source, media_path):
        result = self.inspection.execute(point, task_type, source)
        self.results.record_task(result, paths=[media_path])
        if result['status'] == 'FAILED':
            raise RuntimeError(f"{point['id']} {task_type}: {result['reason']}")
        return result

    def announce(self):
        text = '前方发现禁止直行标识'
        self.speech.put(text)
        return {'queued_text': text}

    def start_recording(self):
        self.services.set_lateral_speed(self.planner, self.runtime['recording_lateral_speed'])
        self.services.start_recording(self.video_path)
        self.recording = True

    def stop_recording(self):
        self.services.stop_recording()
        self.recording = False
        self.services.set_lateral_speed(self.planner, self.runtime['normal_lateral_speed'])

    def next_point(self):
        if self.index + 1 == len(self.route['points']):
            if not self.speech.drain():
                rospy.logwarn('[任务] Speech queue did not finish before its deadline')
            self.transition('FINISH', '全部航点完成')
        else:
            self.index += 1
            self.transition('NAVIGATE', '前往下一航点')

    def run(self):
        handlers = {'INIT': self.initialize, 'NAVIGATE': self.navigate,
                    'TASK': self.task, 'WAIT_TRAFFIC': self.wait_traffic,
                    'NEXT_POINT': self.next_point}
        try:
            self.transition('INIT', '开始巡检')
            while self.state not in ('FINISH', 'ERROR'):
                self.check_running()
                handlers[self.state]()
        except Exception as error:
            self.stop_navigation()
            self.transition('ERROR', str(error))
            raise
        finally:
            self.results.finish(self.state, self.reason if self.state == 'ERROR' else None)

    def close(self):
        if self.closed:
            return
        self.closed = True
        with self.condition:
            self.stopping = True
            self.client.cancel_all_goals()
        if self.recording:
            try:
                self.services.stop_recording()
            except Exception as error:
                rospy.logerr('[任务] Failed to stop recorder: %s', error)
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
        rospy.logfatal('[状态] Mission stopped: %s', error)
        return 1
    finally:
        if mission is not None:
            mission.close()


if __name__ == '__main__':
    raise SystemExit(main())
