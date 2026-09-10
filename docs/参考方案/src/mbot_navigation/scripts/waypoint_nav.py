#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, actionlib, math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Empty
import tf

class WaypointNav:
    def __init__(self):
        rospy.init_node("waypoint_nav")

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base.")

        # Load waypoints from YAML
        waypoints = rospy.get_param("~waypoints", [])
        self.waypoints = []
        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = wp["x"]
            pose.pose.position.y = wp["y"]
            yaw = wp.get("yaw", 0.0)
            q = tf.transformations.quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            self.waypoints.append(pose)

        rospy.loginfo("Loaded %d waypoints.", len(self.waypoints))
        self.current = 0

        # Publish Path for visualization
        self.path_pub = rospy.Publisher("/waypoints_path", Path, queue_size=1, latch=True)
        path = Path()
        path.header.frame_id = "map"
        path.poses = self.waypoints
        self.path_pub.publish(path)

        # Add control services
        rospy.Service("~start", Empty, self.start)
        rospy.Service("~stop",  Empty, self.stop)
        rospy.Service("~clear", Empty, self.clear)

        self.running = False
        rospy.loginfo("Ready. Call /waypoint_nav/start to begin.")
        rospy.spin()

    def start(self, req):
        self.running = True
        rospy.loginfo("Starting waypoint navigation...")
        self.send_next_goal()
        return []

    def stop(self, req):
        self.client.cancel_all_goals()
        self.running = False
        rospy.logwarn("Stopped current goal.")
        return []

    def clear(self, req):
        self.waypoints = []
        rospy.loginfo("Cleared waypoints list.")
        return []

    def send_next_goal(self):
        if not self.running:
            return
        if self.current >= len(self.waypoints):
            rospy.loginfo("All waypoints reached!")
            self.running = False
            return

        goal = MoveBaseGoal()
        goal.target_pose = self.waypoints[self.current]
        goal.target_pose.header.stamp = rospy.Time.now()
        rospy.loginfo("Sending goal #%d: (%.2f, %.2f)",
                      self.current + 1,
                      goal.target_pose.pose.position.x,
                      goal.target_pose.pose.position.y)
        self.client.send_goal(goal,
                              done_cb=self.done_cb,
                              active_cb=None,
                              feedback_cb=None)

    def done_cb(self, state, result):
        if state == 3:
            rospy.loginfo("Goal %d reached.", self.current + 1)
            self.current += 1
            self.send_next_goal()
        else:
            rospy.logwarn("Goal %d failed (state %d).", self.current + 1, state)
            self.client.cancel_all_goals()
            self.running = False

if __name__ == "__main__":
    try:
        WaypointNav()
    except rospy.ROSInterruptException:
        pass
