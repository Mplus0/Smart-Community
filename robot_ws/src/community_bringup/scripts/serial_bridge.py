#!/usr/bin/env python3
"""Single serial owner for the two protocols found in the original vehicle code."""
import struct
import threading
import time

import rospy
import serial
from geometry_msgs.msg import Twist


def encode_velocity(protocol, x, y, yaw):
    if protocol == 'aa_bb':
        return b'\xaa\xbb\x0a\x12\x02' + struct.pack('<hhh', int(x * 1000), int(y * 1000), int(yaw * 1000)) + b'\x00'
    if protocol == 'legacy_11':
        return b'\x11\x00\x00\x00' + struct.pack('>hh', int(x * 100), int(yaw * 100)) + b'\x00'
    raise ValueError('protocol must be aa_bb or legacy_11')


def extract_frames(buffer):
    """Retain partial frames and resynchronize after corrupt bytes."""
    frames = []
    while len(buffer) >= 2:
        index = buffer.find(b'\xaa\xbb')
        if index < 0:
            buffer[:] = buffer[-1:] if buffer[-1] == 0xaa else b''
            break
        del buffer[:index]
        if len(buffer) < 12:
            break
        frame = bytes(buffer[:12])
        if (sum(frame[2:10]) & 0xff) != frame[11]:
            del buffer[0]
            continue
        frames.append(struct.unpack('<hhh', frame[4:10]))
        del buffer[:12]
    return frames


class SerialBridge:
    def __init__(self):
        self.protocol = rospy.get_param('~protocol')
        encode_velocity(self.protocol, 0, 0, 0)  # validate before opening device
        self.timeout = rospy.get_param('~command_timeout', 0.5)
        self.yaw_scale = rospy.get_param('~feedback_yaw_scale', 0.95)
        self.lock = threading.Lock()
        self.last_command = time.monotonic()
        self.stopped = True
        self.port = serial.Serial(rospy.get_param('~port', '/dev/carserial'),
                                  rospy.get_param('~baud', 115200), timeout=rospy.get_param('~read_timeout', 0.05),
                                  write_timeout=rospy.get_param('~write_timeout', 0.5))
        self.publisher = rospy.Publisher('/vel_raw', Twist, queue_size=20)
        # Both original sender scripts emit this initialization sequence.
        self.send(b'\x11' + bytes(8), delay=0.001)
        self.subscriber = rospy.Subscriber('/cmd_vel', Twist, self.command, queue_size=1)
        rospy.on_shutdown(self.close)

    def send(self, frame, delay=0.0004):
        with self.lock:
            for byte in frame:
                self.port.write(bytes([byte]))
                time.sleep(delay)

    def command(self, msg):
        try:
            frame = encode_velocity(self.protocol, msg.linear.x, msg.linear.y, msg.angular.z)
            self.send(frame)
            self.last_command = time.monotonic()
            self.stopped = False
        except (ValueError, OverflowError, struct.error, serial.SerialException) as error:
            rospy.logerr('Velocity command rejected: %s', error)

    def run(self):
        buffer = bytearray()
        while not rospy.is_shutdown():
            buffer.extend(self.port.read(64))
            for x, y, yaw in extract_frames(buffer):
                msg = Twist()
                msg.linear.x, msg.linear.y = x / 1000.0, y / 1000.0
                msg.angular.z = yaw / 1000.0 * self.yaw_scale
                self.publisher.publish(msg)
            if not self.stopped and time.monotonic() - self.last_command > self.timeout:
                self.send(encode_velocity(self.protocol, 0, 0, 0))
                self.stopped = True

    def close(self):
        if self.port.is_open:
            try:
                self.send(encode_velocity(self.protocol, 0, 0, 0))
            finally:
                self.port.close()


if __name__ == '__main__':
    rospy.init_node('community_serial')
    bridge = SerialBridge()
    try:
        bridge.run()
    except serial.SerialException as error:
        rospy.logfatal('Serial connection failed: %s', error)
    finally:
        bridge.close()
