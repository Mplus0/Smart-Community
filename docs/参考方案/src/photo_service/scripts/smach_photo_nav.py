#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SMACH 状态机：多点自主巡航 + 定点拍照。

- 49 个导航点按顺序巡航，途经 3 个拍照点（ro1 / ro2 / chepai）时自动调用 /take_photo 拍照。
- GoToPose 通过 /move_base action 导航；TakePhotoState 调用 /take_photo 服务。
- 通过循环 + 模糊匹配动态生成状态机结构，避免硬编码每个状态。
"""
import rospy
import smach
import smach_ros
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from photo_service.srv import TakePhoto


def quaternion_from_yaw(yaw):
    return tf.transformations.quaternion_from_euler(0, 0, yaw)


class GoToPose(smach.State):
    def __init__(self, x, y, yaw):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.x = x
        self.y = y
        self.yaw = yaw
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("等待 move_base action 服务器...")
        self.client.wait_for_server()
        rospy.loginfo("连接 move_base 成功。")

    def execute(self, userdata):
        rospy.loginfo("导航到目标点: x=%.3f, y=%.3f, yaw=%.3f" % (self.x, self.y, self.yaw))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.x
        goal.target_pose.pose.position.y = self.y

        q = quaternion_from_yaw(self.yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        self.client.wait_for_result()
        state = self.client.get_state()

        if state == 3:  # GoalStatus.SUCCEEDED
            rospy.loginfo("到达目标点 (x=%.2f, y=%.2f)" % (self.x, self.y))
            return 'succeeded'
        else:
            rospy.logwarn("导航失败 (状态码: %s)" % str(state))
            return 'aborted'


class TakePhotoState(smach.State):
    def __init__(self, filename, folder, wait_time=2.0):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.filename = filename
        self.folder = folder
        self.wait_time = wait_time
        rospy.wait_for_service('/take_photo')
        self.take_photo = rospy.ServiceProxy('/take_photo', TakePhoto)

    def execute(self, userdata):
        rospy.loginfo("到达拍照点，等待 %.1f 秒刷新相机..." % self.wait_time)
        rospy.sleep(self.wait_time)

        try:
            rospy.loginfo("调用拍照服务 -> %s/%s.jpg" % (self.folder, self.filename))
            resp = self.take_photo(self.folder, self.filename)
            if resp.success:
                rospy.loginfo("拍照成功: %s/%s.jpg" % (self.folder, self.filename))
                return 'succeeded'
            else:
                rospy.logwarn("拍照失败: %s" % resp.message)
                return 'aborted'
        except rospy.ServiceException as e:
            rospy.logerr("拍照服务调用失败: %s" % e)
            return 'aborted'


def main():
    rospy.init_node('smach_photo_nav')

    # 全部导航点（共 49 个）
    waypoints = [
        {'x': 1.7497, 'y': -0.5104, 'yaw': 1.581},
        {'x': 1.7288, 'y': 0.0285, 'yaw': 1.616},
        {'x': 1.7078, 'y': 0.5752, 'yaw': 1.597},
        {'x': 1.7017, 'y': 0.9768, 'yaw': 1.551},
        {'x': 1.7210, 'y': 1.5026, 'yaw': 1.590},
        {'x': 1.7343, 'y': 1.7352, 'yaw': 1.551},
        {'x': 1.7426, 'y': 1.7178, 'yaw': 2.130},
        {'x': 1.7334, 'y': 1.6921, 'yaw': 2.571},
        {'x': 1.7236, 'y': 1.6319, 'yaw': 2.853},
        {'x': 1.7665, 'y': 1.6224, 'yaw': 3.136},
        {'x': 1.2661, 'y': 1.6153, 'yaw': 3.106},
        {'x': 0.7573, 'y': 1.6256, 'yaw': -3.138},
        {'x': 0.4981, 'y': 1.6050, 'yaw': -3.120},
        {'x': 0.5155, 'y': 1.6133, 'yaw': 2.658},
        {'x': 0.5074, 'y': 1.6393, 'yaw': 2.159},   # 拍照点 ro1
        {'x': 0.4810, 'y': 1.6140, 'yaw': 2.581},
        {'x': 0.4719, 'y': 1.5883, 'yaw': 3.101},
        {'x': 0.4726, 'y': 1.6228, 'yaw': -2.514},
        {'x': 0.2356, 'y': 1.4205, 'yaw': -2.376},
        {'x': 0.2358, 'y': 1.4291, 'yaw': -1.591},
        {'x': 0.2350, 'y': 0.9633, 'yaw': -1.563},
        {'x': 0.2464, 'y': 0.6785, 'yaw': -1.591},  # 拍照点 ro2
        {'x': 0.2170, 'y': 0.0752, 'yaw': -1.591},
        {'x': 0.2601, 'y': 0.0743, 'yaw': -2.172},
        {'x': 0.1621, 'y': -0.0789, 'yaw': -2.827},
        {'x': 0.1269, 'y': -0.1127, 'yaw': 3.058},
        {'x': -0.3558, 'y': -0.0943, 'yaw': 3.084},
        {'x': -0.7438, 'y': -0.0865, 'yaw': 3.121},
        {'x': -1.0287, 'y': -0.0980, 'yaw': 3.121},  # 拍照点 chepai
        {'x': -1.5031, 'y': -0.0970, 'yaw': 3.121},
        {'x': -1.4774, 'y': -0.1061, 'yaw': -2.774},
        {'x': -1.4521, 'y': -0.1325, 'yaw': -2.278},
        {'x': -1.6118, 'y': -0.3536, 'yaw': -2.179},
        {'x': -1.6461, 'y': -0.3443, 'yaw': -1.644},
        {'x': -1.6571, 'y': -0.8875, 'yaw': -1.591},
        {'x': -1.6457, 'y': -1.1724, 'yaw': -1.591},
        {'x': -1.6201, 'y': -1.1901, 'yaw': -0.872},
        {'x': -1.6548, 'y': -1.1981, 'yaw': -0.218},
        {'x': -1.5862, 'y': -1.2167, 'yaw': -0.020},
        {'x': -1.0521, 'y': -1.2534, 'yaw': -0.020},
        {'x': -0.4395, 'y': -1.2486, 'yaw': 0.030},
        {'x': 0.1558, 'y': -1.2434, 'yaw': 0.032},
        {'x': 0.7426, 'y': -1.2294, 'yaw': -0.020},
        {'x': 1.3021, 'y': -1.2924, 'yaw': 0.042},
        {'x': 1.6413, 'y': -1.2859, 'yaw': 0.032},
        {'x': 1.7788, 'y': -1.2874, 'yaw': -0.706},
        {'x': 1.8331, 'y': -1.3499, 'yaw': -1.546},
        {'x': 1.8285, 'y': -1.3455, 'yaw': -1.928},
        {'x': 1.6765, 'y': -1.7453, 'yaw': -1.849},
    ]

    # 拍照点（按坐标模糊匹配）
    photo_points = [
        {'x': 0.5074, 'y': 1.6393, 'folder': 'photo1', 'name': 'ro1'},
        {'x': 0.2464, 'y': 0.6785, 'folder': 'photo1', 'name': 'ro2'},
        {'x': -1.0287, 'y': -0.0980, 'folder': 'photo2', 'name': 'chepai'},
    ]

    def is_photo_point(wp):
        for p in photo_points:
            if abs(wp['x'] - p['x']) < 0.05 and abs(wp['y'] - p['y']) < 0.05:
                return p
        return None

    sm = smach.StateMachine(outcomes=['TASK_DONE', 'TASK_ABORTED'])
    with sm:
        for i, wp in enumerate(waypoints):
            is_last = (i == len(waypoints) - 1)
            p = is_photo_point(wp)

            goto_name = "GOTO_POINT_%02d" % (i + 1)
            next_state = "TASK_DONE" if is_last else "GOTO_POINT_%02d" % (i + 2)

            if p:
                transitions = {'succeeded': "PHOTO_POINT_%02d" % (i + 1),
                               'aborted': 'TASK_ABORTED'}
            else:
                transitions = {'succeeded': next_state, 'aborted': 'TASK_ABORTED'}

            smach.StateMachine.add(goto_name, GoToPose(wp['x'], wp['y'], wp['yaw']),
                                   transitions=transitions)

            if p:
                photo_name = "PHOTO_POINT_%02d" % (i + 1)
                smach.StateMachine.add(photo_name,
                                       TakePhotoState(p['name'], p['folder'], wait_time=2.0),
                                       transitions={'succeeded': next_state,
                                                    'aborted': 'TASK_ABORTED'})

    outcome = sm.execute()
    rospy.loginfo("任务执行结果: %s" % outcome)


if __name__ == '__main__':
    main()
