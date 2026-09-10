#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import yaml
import os
from math import cos, sin
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

def make_goal(x, y, yaw, frame_id="map"):
    q = quaternion_from_euler(0.0, 0.0, yaw)
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = frame_id
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]
    return goal

def load_waypoints(yaml_path):
    if not os.path.exists(yaml_path):
        rospy.logerr("Waypoints file not found: %s", yaml_path)
        return []
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    return data.get("waypoints", [])

def status_str(code):
    m = {
        GoalStatus.PENDING: "PENDING",
        GoalStatus.ACTIVE: "ACTIVE",
        GoalStatus.PREEMPTED: "PREEMPTED",
        GoalStatus.SUCCEEDED: "SUCCEEDED",
        GoalStatus.ABORTED: "ABORTED",
        GoalStatus.REJECTED: "REJECTED",
        GoalStatus.PREEMPTING: "PREEMPTING",
        GoalStatus.RECALLING: "RECALLING",
        GoalStatus.RECALLED: "RECALLED",
        GoalStatus.LOST: "LOST",
    }
    return m.get(code, str(code))

if __name__ == "__main__":
    rospy.init_node("multi_goal_nav")

    yaml_path = rospy.get_param("~waypoints_file", "")
    loop = rospy.get_param("~loop", False)              # 是否循环巡航
    goal_timeout = rospy.get_param("~goal_timeout", 180.0)  # 单个目标超时（秒）

    waypoints = load_waypoints(yaml_path)
    if not waypoints:
        rospy.logerr("No waypoints loaded. Exit.")
        exit(1)

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    if not client.wait_for_server(rospy.Duration(30.0)):
        rospy.logerr("move_base action server not available.")
        exit(1)
    rospy.loginfo("Connected to move_base.")

    idx = 0
    while not rospy.is_shutdown():
        wp = waypoints[idx]
        name = wp.get("name", "WP{}".format(idx+1))
        x, y, yaw = float(wp["x"]), float(wp["y"]), float(wp["yaw"])

        rospy.loginfo("Sending goal %d/%d: %s (x=%.3f, y=%.3f, yaw=%.3f)",
                      idx+1, len(waypoints), name, x, y, yaw)

        goal = make_goal(x, y, yaw, frame_id="map")
        client.send_goal(goal)

        finished = client.wait_for_result(rospy.Duration(goal_timeout))
        if not finished:
            rospy.logwarn("Goal %s timed out after %.1f s. Canceling...", name, goal_timeout)
            client.cancel_goal()
            client.wait_for_result(rospy.Duration(2.0))

        state = client.get_state()
        rospy.loginfo("Result for %s: %s", name, status_str(state))

        # 到达失败时：继续下一个（也可选择重试）
        idx += 1
        if idx >= len(waypoints):
            if loop:
                idx = 0
                rospy.loginfo("Loop enabled: restarting from first waypoint.")
            else:
                rospy.loginfo("All waypoints processed. Done.")
                break

        rospy.sleep(0.5)  # 给 move_base 一点空隙
