#!/usr/bin/env python3
"""Apply camera controls from ROS parameters using the external v4l2-ctl utility."""
import subprocess
import rospy

if __name__ == '__main__':
    rospy.init_node('configure_camera')
    device = rospy.get_param('~device', '/dev/video0')
    for name, value in rospy.get_param('~controls', {}).items():
        result = subprocess.run(['v4l2-ctl', '-d', device, '--set-ctrl', f'{name}={value}'],
                                capture_output=True, text=True, timeout=5)
        if result.returncode:
            rospy.logwarn('Camera control %s failed: %s', name, result.stderr)
