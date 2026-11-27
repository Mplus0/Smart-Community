"""ROS service adapters and a serialized speech worker."""
import queue
import threading
import time
import math

import rospy
from community_mission.srv import Speech
from community_perception.srv import CountFallStand
from community_perception.srv import LicensePlateOCR
from community_mission.srv import StartRecording, StopRecording
from community_perception.srv import MeasureBuildingTemperature, ObjectDetection
from dynamic_reconfigure.srv import Reconfigure
from dynamic_reconfigure.msg import Config, DoubleParameter


class Services:
    def __init__(self, config):
        self.config = config

    def call(self, name, service_type, *args, timeout=None):
        if timeout is None:
            timeout = self.config['service_timeout']
        name = '/' + name.strip('/')
        rospy.wait_for_service(name, timeout=timeout)
        response = rospy.ServiceProxy(name, service_type)(*args)
        if hasattr(response, 'success') and not response.success:
            raise RuntimeError(f'{name}: {getattr(response, "message", "service returned failure")}')
        return response

    def detect(self, detector, image):
        return self.call(detector+'/detect', ObjectDetection, image)

    def measure_building_temperature(self):
        return self.call('building_d_temperature/measure', MeasureBuildingTemperature)

    def ocr(self, image):
        return self.call('license_plate_ocr', LicensePlateOCR, image)

    def building_ocr(self, image):
        response = self.call('building_name_ocr_preview', LicensePlateOCR, image)
        if response.status_message not in ('', '未识别到文字'):
            raise RuntimeError(response.status_message)
        return response

    def read_electricity(self, image):
        return self.call('electricity_meter_ocr', LicensePlateOCR, image)

    def say(self, text):
        return self.call('speech_say', Speech, text)

    def start_recording(self, path):
        self.call('video_recorder_node/start_recording', StartRecording, str(path))

    def stop_recording(self):
        self.call('video_recorder_node/stop_recording', StopRecording)

    def count_video(self, path):
        return self.call('count_fall_stand', CountFallStand, str(path))

    def set_lateral_speed(self, namespace, speed):
        config = Config(doubles=[DoubleParameter(name='max_vel_y', value=speed),
                                 DoubleParameter(name='min_vel_y', value=-speed)])
        try:
            self.call(namespace+'/set_parameters', Reconfigure, config, timeout=self.config['planner_service_timeout'])
            return True
        except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
            rospy.logwarn('[导航参数] 侧移速度更新失败：%s', error)
            return False

    def get_lateral_limits(self, namespace):
        response = self.call(namespace+'/set_parameters', Reconfigure, Config(),
                             timeout=self.config['planner_service_timeout'])
        values = {parameter.name: parameter.value for parameter in response.config.doubles}
        if 'max_vel_y' not in values:
            raise RuntimeError('Planner does not expose max_vel_y: ' + namespace)
        # TEB only exposes max_vel_y; DWA also exposes min_vel_y.
        return {name: values[name] for name in ('max_vel_y', 'min_vel_y') if name in values}

    def set_lateral_limits(self, namespace, limits):
        config = Config(doubles=[DoubleParameter(name=name, value=value)
                                 for name, value in limits.items()])
        response = self.call(namespace+'/set_parameters', Reconfigure, config,
                             timeout=self.config['planner_service_timeout'])
        actual = {parameter.name: parameter.value for parameter in response.config.doubles}
        if any(name not in actual or not math.isclose(actual[name], value, abs_tol=1e-6)
               for name, value in limits.items()):
            raise RuntimeError('Planner did not apply lateral limits: ' + namespace)


class SpeechQueue:
    def __init__(self, services, config):
        self.services = services
        self.config = config
        self.queue = queue.Queue()
        self.closed = False
        self.thread = threading.Thread(target=self._work, name='community_speech', daemon=True)
        self.thread.start()

    def put(self, text):
        if not self.closed:
            self.queue.put(text)

    def _work(self):
        while True:
            text = self.queue.get()
            try:
                if text is None:
                    return
                self.services.say(text)
            except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
                rospy.logerr('[语音失败] %s', error)
            finally:
                self.queue.task_done()

    def drain(self, timeout=None):
        if timeout is None:
            timeout = self.config['speech_drain_timeout']
        deadline = time.monotonic()+timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.queue.all_tasks_done:
                if self.queue.unfinished_tasks == 0:
                    return True
            time.sleep(0.05)
        return False

    def close(self):
        if self.closed:
            return
        self.closed = True
        while True:
            try:
                self.queue.get_nowait()
                self.queue.task_done()
            except queue.Empty:
                break
        self.queue.put(None)
        self.thread.join(timeout=self.config['speech_join_timeout'])
