#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import select
import termios
import tty

import rospy
from geometry_msgs.msg import Twist


HELP = """
============================================================
 Smart Community - Mecanum Keyboard Teleop
============================================================

移动：

            W
            ↑
       A ←  停  → D
            ↓
            S

W : 前进
S : 后退
A : 左横移
D : 右横移

Q : 原地左转
E : 原地右转

空格 / X : 停止

速度调整：

+ / = : 增加平移速度
-     : 减小平移速度

]     : 增加旋转速度
[     : 减小旋转速度

H     : 显示帮助

Ctrl+C : 退出

============================================================
"""


class KeyboardTeleop:
    def __init__(self):
        rospy.init_node("keyboard_teleop")

        # --------------------------------------------------
        # ROS parameters
        # --------------------------------------------------

        self.cmd_vel_topic = rospy.get_param(
            "~cmd_vel_topic",
            "/cmd_vel"
        )

        self.linear_speed = rospy.get_param(
            "~linear_speed",
            0.25
        )

        self.angular_speed = rospy.get_param(
            "~angular_speed",
            0.60
        )

        self.linear_step = rospy.get_param(
            "~linear_step",
            0.05
        )

        self.angular_step = rospy.get_param(
            "~angular_step",
            0.10
        )

        self.max_linear_speed = rospy.get_param(
            "~max_linear_speed",
            1.0
        )

        self.max_angular_speed = rospy.get_param(
            "~max_angular_speed",
            2.0
        )

        # --------------------------------------------------
        # Publisher
        # --------------------------------------------------

        self.cmd_pub = rospy.Publisher(
            self.cmd_vel_topic,
            Twist,
            queue_size=10
        )

        self.current_twist = Twist()

        self.settings = termios.tcgetattr(sys.stdin)

        rospy.on_shutdown(self.stop_robot)

    def get_key(self, timeout=0.05):
        """Read one keyboard character without blocking forever."""
        tty.setraw(sys.stdin.fileno())

        ready, _, _ = select.select(
            [sys.stdin],
            [],
            [],
            timeout
        )

        if ready:
            key = sys.stdin.read(1)
        else:
            key = ""

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            self.settings
        )

        return key

    def publish_velocity(self, vx=0.0, vy=0.0, wz=0.0):
        twist = Twist()

        twist.linear.x = vx
        twist.linear.y = vy
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = wz

        self.current_twist = twist

    def stop_robot(self):
        twist = Twist()

        # Publish several zero commands to ensure Gazebo receives stop.
        for _ in range(3):
            try:
                self.cmd_pub.publish(twist)
                rospy.sleep(0.02)
            except Exception:
                pass

    def print_speed(self):
        print(
            "\rLinear: {:.2f} m/s    Angular: {:.2f} rad/s      ".format(
                self.linear_speed,
                self.angular_speed
            )
        )

    def run(self):
        print(HELP)
        self.print_speed()

        rate = rospy.Rate(20)

        try:
            while not rospy.is_shutdown():

                key = self.get_key()

                # ==========================================
                # Movement
                # ==========================================

                if key in ("w", "W"):
                    self.publish_velocity(
                        vx=self.linear_speed
                    )

                elif key in ("s", "S"):
                    self.publish_velocity(
                        vx=-self.linear_speed
                    )

                elif key in ("a", "A"):
                    self.publish_velocity(
                        vy=self.linear_speed
                    )

                elif key in ("d", "D"):
                    self.publish_velocity(
                        vy=-self.linear_speed
                    )

                elif key in ("q", "Q"):
                    self.publish_velocity(
                        wz=self.angular_speed
                    )

                elif key in ("e", "E"):
                    self.publish_velocity(
                        wz=-self.angular_speed
                    )

                # ==========================================
                # Stop
                # ==========================================

                elif key == " " or key in ("x", "X"):
                    self.publish_velocity()

                # ==========================================
                # Linear speed
                # ==========================================

                elif key in ("+", "="):
                    self.linear_speed += self.linear_step

                    self.linear_speed = min(
                        self.linear_speed,
                        self.max_linear_speed
                    )

                    self.print_speed()

                elif key == "-":
                    self.linear_speed -= self.linear_step

                    self.linear_speed = max(
                        self.linear_speed,
                        0.05
                    )

                    self.print_speed()

                # ==========================================
                # Angular speed
                # ==========================================

                elif key == "]":
                    self.angular_speed += self.angular_step

                    self.angular_speed = min(
                        self.angular_speed,
                        self.max_angular_speed
                    )

                    self.print_speed()

                elif key == "[":
                    self.angular_speed -= self.angular_step

                    self.angular_speed = max(
                        self.angular_speed,
                        0.10
                    )

                    self.print_speed()

                # ==========================================
                # Help
                # ==========================================

                elif key in ("h", "H"):
                    print(HELP)
                    self.print_speed()

                # Ctrl+C
                elif key == "\x03":
                    break

                # Continuously publish current command.
                self.cmd_pub.publish(
                    self.current_twist
                )

                rate.sleep()

        finally:
            self.stop_robot()

            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                self.settings
            )

            print("\nKeyboard teleop stopped.")


if __name__ == "__main__":
    try:
        teleop = KeyboardTeleop()
        teleop.run()

    except rospy.ROSInterruptException:
        pass