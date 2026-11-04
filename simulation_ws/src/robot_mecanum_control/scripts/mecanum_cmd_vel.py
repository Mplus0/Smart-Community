#!/usr/bin/env python3
import os
import sys
import threading
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mecanum_math import MecanumGeometry, inverse_kinematics, scale_to_limit

class MecanumCmdVel:
    def __init__(self):
        self.geometry = MecanumGeometry(
            wheel_radius=rospy.get_param('~wheel_radius', 0.037230282),
            half_wheelbase=rospy.get_param('~half_wheelbase', 0.106000002),
            half_track=rospy.get_param('~half_track', 0.125000000),
        )
        self.max_wheel_speed = float(rospy.get_param('~max_wheel_speed', 16.115913385))
        self.timeout = float(rospy.get_param('~cmd_timeout', 0.5))
        self.publish_rate = float(rospy.get_param('~publish_rate', 50.0))
        self.command_topic = rospy.get_param('~wheel_command_topic', '/wheel_velocity_controller/command')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.wheel_signs = rospy.get_param('~wheel_signs', [1.0, 1.0, 1.0, 1.0])
        if len(self.wheel_signs) != 4:
            raise ValueError('~wheel_signs must contain four values [FL, FR, RL, RR]')
        self._lock = threading.Lock()
        self._last_cmd_time = rospy.Time(0)
        self._target = [0.0] * 4
        self.pub = rospy.Publisher(self.command_topic, Float64MultiArray, queue_size=1)
        self.sub = rospy.Subscriber(self.cmd_vel_topic, Twist, self._on_cmd, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._on_timer)

    def _on_cmd(self, msg):
        wheels = inverse_kinematics(msg.linear.x, msg.linear.y, msg.angular.z, self.geometry)
        wheels, scale = scale_to_limit(wheels, self.max_wheel_speed)
        wheels = [v * float(s) for v, s in zip(wheels, self.wheel_signs)]
        with self._lock:
            self._target = wheels
            self._last_cmd_time = rospy.Time.now()
        if scale < 0.999:
            rospy.logwarn_throttle(2.0, 'Wheel command uniformly scaled to respect max_wheel_speed')

    def _on_timer(self, _event):
        with self._lock:
            stale = (rospy.Time.now() - self._last_cmd_time).to_sec() > self.timeout
            values = [0.0] * 4 if stale else list(self._target)
        self.pub.publish(Float64MultiArray(data=values))

if __name__ == '__main__':
    rospy.init_node('mecanum_cmd_vel')
    MecanumCmdVel()
    rospy.spin()
