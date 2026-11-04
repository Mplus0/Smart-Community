#!/usr/bin/env python3
import math
import os
import sys
import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mecanum_math import MecanumGeometry, forward_kinematics

JOINTS = ['front_left_wheel_joint','front_right_wheel_joint','rear_left_wheel_joint','rear_right_wheel_joint']

def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q

class WheelOdometry:
    def __init__(self):
        self.geometry = MecanumGeometry(
            wheel_radius=rospy.get_param('~wheel_radius', 0.037230282),
            half_wheelbase=rospy.get_param('~half_wheelbase', 0.106000002),
            half_track=rospy.get_param('~half_track', 0.125000000),
        )
        self.wheel_signs = rospy.get_param('~wheel_signs', [1.0, 1.0, 1.0, 1.0])
        self.frame_id = rospy.get_param('~frame_id', 'odom')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'base_footprint')
        self.topic = rospy.get_param('~odom_topic', '/wheel/odom')
        self.x = self.y = self.yaw = 0.0
        self.last_stamp = None
        self.pub = rospy.Publisher(self.topic, Odometry, queue_size=10)
        self.sub = rospy.Subscriber('/joint_states', JointState, self._on_joint_state, queue_size=20)

    def _on_joint_state(self, msg):
        if not msg.velocity:
            return
        index = {name: i for i, name in enumerate(msg.name)}
        if any(j not in index for j in JOINTS):
            return
        wheels = [msg.velocity[index[j]] * float(s) for j, s in zip(JOINTS, self.wheel_signs)]
        vx, vy, wz = forward_kinematics(wheels, self.geometry)
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        if self.last_stamp is not None:
            dt = (stamp - self.last_stamp).to_sec()
            if 0.0 < dt < 0.5:
                c, s = math.cos(self.yaw), math.sin(self.yaw)
                self.x += (vx * c - vy * s) * dt
                self.y += (vx * s + vy * c) * dt
                self.yaw += wz * dt
        self.last_stamp = stamp
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quat(self.yaw)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.pub.publish(odom)

if __name__ == '__main__':
    rospy.init_node('mecanum_wheel_odometry')
    WheelOdometry()
    rospy.spin()
