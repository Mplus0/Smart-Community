#!/usr/bin/env python3
"""Publish wheel odometry; robot_localization remains responsible for the odom TF."""
import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class OdometryIntegrator:
    def __init__(self, scale_x=1.0, scale_y=1.0, max_interval=1.0):
        self.scale_x, self.scale_y, self.max_interval = scale_x, scale_y, max_interval
        self.x = self.y = self.heading = 0.0
        self.previous_time = None

    def update(self, timestamp, vx, vy, yaw_rate):
        if not all(math.isfinite(v) for v in (timestamp, vx, vy, yaw_rate)):
            return None
        previous = self.previous_time
        self.previous_time = timestamp
        if previous is None or not 0 < timestamp-previous <= self.max_interval:
            return None
        dt = timestamp-previous
        vx, vy = vx*self.scale_x, vy*self.scale_y
        self.x += (vx*math.cos(self.heading)-vy*math.sin(self.heading))*dt
        self.y += (vx*math.sin(self.heading)+vy*math.cos(self.heading))*dt
        self.heading += yaw_rate*dt
        return self.x, self.y, self.heading, vx, vy, yaw_rate


class OdometryNode:
    def __init__(self):
        self.integrator = OdometryIntegrator(rospy.get_param('~linear_scale_x', 1.0),
                                             rospy.get_param('~linear_scale_y', 1.0))
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_footprint_frame', 'base_footprint')
        self.publisher = rospy.Publisher('/pub_odom', Odometry, queue_size=50)
        self.subscriber = rospy.Subscriber('/sub_vel', Twist, self.callback, queue_size=50)

    def callback(self, velocity):
        now = rospy.Time.now()
        state = self.integrator.update(now.to_sec(), velocity.linear.x, velocity.linear.y,
                                       velocity.angular.z)
        if state is None:
            return
        x, y, yaw, vx, vy, yaw_rate = state
        message = Odometry()
        message.header.stamp, message.header.frame_id = now, self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x, message.pose.pose.position.y = x, y
        message.pose.pose.orientation.z = math.sin(yaw/2)
        message.pose.pose.orientation.w = math.cos(yaw/2)
        message.twist.twist.linear.x, message.twist.twist.linear.y = vx, vy
        message.twist.twist.angular.z = yaw_rate
        for index in (0, 7, 35):
            message.pose.covariance[index] = 0.001
            message.twist.covariance[index] = 0.0001
        self.publisher.publish(message)


if __name__ == '__main__':
    rospy.init_node('odometry_publisher')
    node = OdometryNode()
    rospy.spin()
