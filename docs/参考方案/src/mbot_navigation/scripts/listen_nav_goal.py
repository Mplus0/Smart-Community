#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
import math

def goal_callback(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.orientation.z
    w = msg.pose.orientation.w
    # 计算偏航角
    yaw = 2 * math.atan2(z, w)
    if yaw > math.pi:
        yaw -= 2 * math.pi
    elif yaw < -math.pi:
        yaw += 2 * math.pi
    rospy.loginfo("- {x: %.4f, y: %.4f, yaw: %.3f}" % (x, y, yaw))

def listener():
    rospy.init_node('goal_listener', anonymous=True)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

