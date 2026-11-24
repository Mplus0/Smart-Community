#!/usr/bin/env python3
"""Explicit mission states with a lightweight image callback."""
import json
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
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from rules import validate_route, detections, obstacle_route
from services import Services, SpeechQueue
from inspection import Inspection
from result_manager import ResultManager


class Mission:
    def __init__(self):
        # 工作台订阅的任务状态（JSON），latch 保证新连接也能拿到最近状态。
        self.state_pub = rospy.Publisher('/mission/state', String, queue_size=10, latch=True)
        self.state = "INIT"
        self.index = 0
        self.reason = "等待初始化"
        self.stopping = False
        self.navigation_only = rospy.get_param('~navigation_only', False)
        if type(self.navigation_only) is not bool:
            raise ValueError('navigation_only must be a boolean')
        self.route = validate_route(rospy.get_param('~'))
        self.obstacle_mode = rospy.get_param('~obstacle_mode', 'blocked')
        if self.obstacle_mode not in ('blocked', 'clear'):
            raise ValueError('obstacle_mode must be blocked or clear')
        self.single_waypoint = rospy.get_param('~single_waypoint', '')
        if not isinstance(self.single_waypoint, str):
            raise ValueError('single_waypoint must be a waypoint ID string')
        if self.single_waypoint:
            self.index = next((i for i, p in enumerate(self.route['points'])
                               if p['id'] == self.single_waypoint), None)
            if self.index is None:
                raise ValueError('Unknown single_waypoint: ' + self.single_waypoint)
            point = self.route['points'][self.index]
            if not self.navigation_only and point['task'] in ('record_start', 'record_stop'):
                raise ValueError('Single-waypoint recording tasks require a paired route; '
                                 'use navigation_only:=true to navigate without recording')
        self.runtime = rospy.get_param('~mission')
        self.obstacle_clear_x_ratio = float(self.runtime['obstacle_clear_x_ratio'])
        if not 0 < self.obstacle_clear_x_ratio < 1:
            raise ValueError('obstacle_clear_x_ratio must be between 0 and 1')
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
               ('server_timeout', 'ocr_interval', 'service_timeout',
                'planner_service_timeout', 'speech_drain_timeout', 'speech_join_timeout')):
            raise ValueError('Runtime intervals must be finite and positive')
        self.results = ResultManager(rospy.get_param('~data_dir', '~/.ros/community'),
                                     self.route['points'], rospy.loginfo)
        self.data_dir = self.results.path
        self.video_path = self.data_dir/'videos/video.mp4'
        self.planner = rospy.get_param('~planner_namespace', '/move_base/DWAPlannerROS')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.services = None
        self.bridge = None
        self.condition = threading.Condition()
        self.latest = None
        self.sequence = 0
        self.recording = False
        self.closed = False
        self.speech = None
        self.inspection = None
        self.subscriber = None
        if not self.navigation_only:
            self.services = Services(self.runtime)
            self.bridge = CvBridge()
            self.speech = SpeechQueue(self.services, self.runtime)
            self.inspection = Inspection(self.route, self.services, self.speech, rospy.get_param('~recognition_rules'),
                                         self.bridge, self.runtime['ocr_interval'])
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
        rospy.loginfo('[红绿灯] 进入等待：连续 %s 帧 go，超时 %.1f 秒',
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
            # stop、空结果、混合类别或无效检测打断连续 go 计数。
            green = bool(objects) and all(
                obj.label == 'go' and math.isfinite(obj.confidence) and 0 < obj.confidence <= 1
                and all(math.isfinite(v) for v in obj.box)
                and obj.box[2] > obj.box[0] and obj.box[3] > obj.box[1] for obj in objects)
            streak = streak + 1 if green else 0
            if streak >= self.green_confirm_frames:
                rospy.loginfo('[红绿灯] 连续 %s 帧有效 go，允许通行', streak)
                return {'released': True, 'green_streak': streak,
                        'detected_classes': [obj.label for obj in objects]}
        raise RuntimeError('TRAFFIC_TIMEOUT: Traffic light timeout; mission stopped')

    def transition(self, state, reason):
        previous = self.state
        self.state, self.reason = state, reason
        self.results.state(state, self.route['points'][self.index], reason)
        if state == 'ERROR':
            rospy.logerr('[状态] %s -> ERROR: %s', previous, reason)
        self.publish_state()

    def publish_state(self):
        point = self.route['points'][self.index]
        operations = self.results.data['operations']
        last = operations[-1] if operations else None
        self.state_pub.publish(json.dumps(dict(
            state=self.state, reason=self.reason, index=self.index,
            total=len(self.route['points']), current_waypoint=point['id'], task=point['task'],
            navigation_only=self.navigation_only, obstacle_mode=self.obstacle_mode,
            single_waypoint=self.single_waypoint, run_id=self.results.run_id,
            last_operation=None if last is None else dict(
                waypoint_id=last.get('waypoint_id'), task_type=last.get('task_type'),
                status=last.get('status'), reason=last.get('reason', '')),
            stamp=time.time()), ensure_ascii=False))

    def initialize(self):
        if not self.client.wait_for_server(rospy.Duration(self.runtime['server_timeout'])):
            raise RuntimeError('move_base is unavailable')
        if self.route.get('navigation_closures'):
            self.update_navigation_map()
            if self.navigation_only:
                self.update_navigation_map(self.obstacle_mode)
        if not self.navigation_only:
            self.services.set_lateral_speed(self.planner, self.runtime['normal_lateral_speed'])
        else:
            rospy.loginfo('[导航验收] 只导航模式，跳过全部航点任务及红绿灯等待')
        self.transition('NAVIGATE', '导航服务已就绪')

    def update_navigation_map(self, mode=None):
        name = '/navigation_map/reset' if mode is None else '/navigation_map/set_blocked'
        rospy.wait_for_service(name, timeout=self.runtime['server_timeout'])
        service = rospy.ServiceProxy(name, Trigger if mode is None else SetBool)
        response = service() if mode is None else service(mode == 'blocked')
        if not response.success:
            raise RuntimeError('Navigation map update failed: ' + response.message)
        # 在停车点等待一个全局代价地图更新周期，再下发下一导航目标。
        frequency = rospy.get_param('/move_base/global_costmap/update_frequency', 1.0)
        rospy.sleep(1.0 / frequency + 0.1)
        rospy.loginfo('[导航封口] %s', response.message)

    def navigate(self):
        point = self.route['points'][self.index]
        rospy.loginfo('[导航] 前往航点=%s', point['id'])
        self.results.waypoint(point, 'RUNNING')
        self.results.execute('navigation', point, self.move_to,
                             (point['x'], point['y'], point['yaw']))
        self.transition('TASK', '已到达航点')

    def wait_traffic(self):
        rospy.loginfo('[红绿灯] 航点=%s，等待绿灯', self.route['points'][self.index]['id'])
        self.results.execute('traffic_light', self.route['points'][self.index], self.wait_for_green)

    def task(self):
        index = self.index
        point = self.route['points'][index]
        task_type = point['task']
        rospy.loginfo('[任务] 航点=%s，任务=%s', point['id'], task_type)
        self.check_running()
        if self.navigation_only or task_type == 'navigation':
            pass
        elif task_type == 'traffic_light':
            self.wait_traffic()
        elif task_type == 'meter':
            raw_path = self.data_dir/'photo'/f'{index:03d}_meter.jpg'
            overlay_path = raw_path.with_name(f'{raw_path.stem}_overlay.jpg')
            self.results.execute('meter', point, self.read_meter, raw_path, overlay_path,
                                 paths=[raw_path, overlay_path])
        elif task_type == 'temperature':
            result = self.results.execute('temperature', point, self.measure_temperature)
            if result['announcement']:
                self.check_running()
                self.speech.put(result['announcement'])
                if not self.speech.drain():
                    rospy.logwarn('[温度] 等待播报结束超时或巡检已停止')
        elif task_type == 'announcement':
            pass  # 暂停播报任务，保留 announce() 供后续接入。
        elif task_type == 'record_start':
            self.results.execute('record_start', point, self.start_recording,
                                 paths=[self.video_path])
        elif task_type == 'record_stop':
            self.finish_recording(point)
        else:
            image_path = self.data_dir/'photo'/f'{index:03d}.jpg'
            image = self.fresh_image()
            frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            if not cv2.imwrite(str(image_path), frame):
                raise RuntimeError('Failed to save inspection image')
            if task_type == 'obstacle':
                self.results.execute('obstacle', point, self.select_obstacle_route,
                                     image, paths=[image_path])
            else:
                self.run_inspection(point, task_type, image, image_path)
        self.results.waypoint(point, 'SUCCESS')
        self.transition('NEXT_POINT', '本航点任务完成')

    def read_meter(self, raw_path, overlay_path):
        import meter_reader

        profile = rospy.get_param('~meter_profile', '')
        profile_path = Path(profile).expanduser() if profile else (
            Path(meter_reader.__file__).with_name('meter_profile')/'profile.json')
        reader = meter_reader.Reader(profile_path)
        topic = rospy.get_param('~meter_image_topic', '/ugreen_4k/image_raw')
        last_stamp = rospy.Time.now()
        timeout = float(rospy.get_param('~meter_timeout', 60.0))
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('meter_timeout must be positive')
        deadline = time.monotonic() + timeout
        frames = threading.Condition()
        latest = None

        def receive(image):
            nonlocal latest
            with frames:
                if image.header.stamp > last_stamp and (latest is None or image.header.stamp > latest.header.stamp):
                    latest = image
                    frames.notify()

        subscriber = rospy.Subscriber(topic, Image, receive, queue_size=1, buff_size=2**25)
        frame = None
        result = None
        reason = f'{topic} 未收到新的仪表相机图像'
        previous_energy = None
        energy_frames = 0
        try:
            while time.monotonic() < deadline:
                self.check_running()
                with frames:
                    if latest is None:
                        frames.wait(min(0.1, max(0.0, deadline - time.monotonic())))
                        continue
                    image, latest = latest, None
                    last_stamp = image.header.stamp
                frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
                values, overlay, status = reader.read(frame)
                reading = self.services.read_electricity(image)
                self.check_running()
                energy_text = reading.plate_texts[0] if len(reading.plate_texts) == 1 else None
                if energy_text is not None and energy_text == previous_energy:
                    energy_frames += 1
                else:
                    energy_frames = 1 if energy_text is not None else 0
                previous_energy = energy_text
                if time.monotonic() >= deadline:
                    reason = '仪表读取超时，三项读数未完成确认'
                    break
                if status == 'alignment_failed':
                    reason = '仪表盘配准失败'
                elif values['temperature'] is None or values['humidity'] is None:
                    reason = '未可靠读出温度或湿度指针'
                elif energy_frames < 2:
                    reason = '电表读数未连续两帧确认：' + reading.status_message
                else:
                    result = dict(
                        temperature_c=meter_reader.quantize_reading('temperature', values['temperature']),
                        humidity_percent=meter_reader.quantize_reading('humidity', values['humidity']),
                        electricity_kwh=energy_text,
                        electricity_unit='kWh',
                        electricity_confidence=float(reading.confidences[0]),
                        camera_topic=topic)
                    cv2.putText(overlay, f'Energy: {energy_text} kWh', (30, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 180, 0), 3)
                    break
            self.check_running()
            if frame is not None:
                meter_reader.save_image(raw_path, frame)
                meter_reader.save_image(overlay_path, overlay)
            if result is None:
                raise RuntimeError(reason)
            return result
        finally:
            subscriber.unregister()

    def measure_temperature(self):
        reading = self.services.measure_building_temperature()
        self.check_running()
        floors = list(reading.abnormal_floors)
        announcement = ''
        if floors:
            floor_text = '、'.join(map(str, floors))
            announcement = f'D大楼在 {floor_text} 楼处发现高温情况'
        return dict(abnormal_floors=floors, max_temperatures_c=list(reading.max_temperatures_c),
                    announcement=announcement)

    def select_obstacle_route(self, image):
        objects = detections(self.services.detect('obstacle_detector', image))
        self.check_running()
        result = obstacle_route(objects, image.width, self.obstacle_clear_x_ratio)
        if self.route.get('navigation_closures'):
            self.update_navigation_map(result['mode'])
        self.obstacle_mode = result['mode']
        rospy.loginfo('[障碍物] 中心横坐标比例=%.3f，路线=%s',
                      result['center_x_ratio'], self.obstacle_mode)
        return result

    def finish_recording(self, point):
        self.results.execute('record_stop', point, self.stop_recording,
                             paths=[self.video_path])
        self.run_inspection(point, 'vehicle_video', self.video_path, self.video_path)

    def run_inspection(self, point, task_type, source, media_path):
        result = self.inspection.execute(point, task_type, source)
        paths = [media_path]
        if task_type == 'trash' and result['status'] != 'FAILED':
            paths.extend(self.save_trash_evidence(source, result['result'], media_path))
        self.results.record_task(result, paths=paths)
        if result['status'] == 'FAILED':
            raise RuntimeError(f"{point['id']} {task_type}: {result['reason']}")
        return result

    def save_trash_evidence(self, image, result, media_path):
        if result['roi'] is None:
            return []
        frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8').copy()
        x1, y1, x2, y2 = result['roi']
        path = Path(media_path)
        roi_path = path.with_name(path.stem + '_roi.jpg')
        if not cv2.imwrite(str(roi_path), frame[y1:y2, x1:x2]):
            raise RuntimeError('Failed to save trash ROI image')
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
        colors = {'CORRECT': (0, 200, 0), 'INCORRECT': (0, 0, 255), 'UNKNOWN': (0, 165, 255)}
        for match in result['matches']:
            bx1, by1, bx2, by2 = map(int, match['bin_bbox'])
            color = colors[match['status']]
            cv2.rectangle(frame, (bx1, by1), (bx2, by2), color, 2)
            cv2.putText(frame, match['bin_label'] + ' ' + match['status'],
                        (bx1, max(16, by1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            for obj in match['garbage']:
                gx1, gy1, gx2, gy2 = map(int, obj['bbox'])
                cv2.rectangle(frame, (gx1, gy1), (gx2, gy2), color, 2)
                cv2.putText(frame, f"{obj['label']} {obj['confidence']:.2f}",
                            (gx1, max(16, gy1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                cv2.line(frame, ((bx1 + bx2) // 2, by2),
                         ((gx1 + gx2) // 2, (gy1 + gy2) // 2), color, 1)
        for obj in result['unmatched']:
            gx1, gy1, gx2, gy2 = map(int, obj['bbox'])
            cv2.rectangle(frame, (gx1, gy1), (gx2, gy2), colors['UNKNOWN'], 2)
        result_path = path.with_name(path.stem + '_trash.jpg')
        if not cv2.imwrite(str(result_path), frame):
            raise RuntimeError('Failed to save trash result image')
        return [roi_path, result_path]

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
        if self.single_waypoint:
            self.transition('FINISH', '单航点执行完成')
            return
        point = self.route['points'][self.index]
        if point['task'] == 'obstacle' and 'navigation_branches' in self.route:
            order = self.route['navigation_branches'][self.obstacle_mode]
            by_id = {p['id']: p for p in self.route['points']}
            self.route['points'][self.index + 1:] = [by_id[name] for name in order]
            rospy.loginfo('[路线] %s 分支=%s，后续航点=%s',
                          point['id'], self.obstacle_mode, ' -> '.join(order))
        if self.index + 1 == len(self.route['points']):
            self.transition('FINISH', '全部航点完成')
        else:
            self.index += 1
            self.transition('NAVIGATE', '前往下一航点')

    def run(self):
        handlers = {'INIT': self.initialize, 'NAVIGATE': self.navigate,
                    'TASK': self.task,
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
        if self.subscriber is not None:
            self.subscriber.unregister()
        if self.speech is not None:
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
