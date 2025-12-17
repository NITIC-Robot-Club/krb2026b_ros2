#!/usr/bin/env python3
import struct

import rclpy
from rclpy.node import Node

from natto_msgs.msg import Swerve, Can


class SwerveCanBridge(Node):
    def __init__(self):
        super().__init__('swerve_can_bridge')

        # parameters
        self.num_wheels = 4

        # subscribers
        self.sub_swerve = self.create_subscription(
            Swerve,
            '/swerve/command',
            self.swerve_callback,
            10
        )

        self.sub_can = self.create_subscription(
            Can,
            '/receive',
            self.can_callback,
            10
        )

        # publishers
        self.pub_can = self.create_publisher(
            Can,
            '/transmit',
            10
        )

        self.pub_swerve = self.create_publisher(
            Swerve,
            '/swerve/result',
            10
        )

        # buffer for CAN -> Swerve
        self.angles = [0.0] * self.num_wheels
        self.speeds = [0.0] * self.num_wheels
        self.received = [False] * self.num_wheels

    # ---------- Swerve -> CAN ----------
    def swerve_callback(self, msg: Swerve):
        n = min(len(msg.wheel_angle), len(msg.wheel_speed), self.num_wheels)

        for i in range(n):
            can = Can()
            can.header.stamp = self.get_clock().now().to_msg()
            can.id = 0x001 + i
            can.len = 8
            can.is_error = False
            can.is_extended = False
            can.is_rtr = False

            can.data = [0] * 64

            angle_bytes = struct.pack('<f', float(msg.wheel_angle[i]))
            speed_bytes = struct.pack('<f', float(msg.wheel_speed[i] * 60.0))

            for j in range(4):
                can.data[j] = angle_bytes[j]
                can.data[j + 4] = speed_bytes[j]

            self.pub_can.publish(can)

    # ---------- CAN -> Swerve ----------
    def can_callback(self, msg: Can):
        if msg.id < 0x101 or msg.id > 0x104:
            return

        if msg.len != 8:
            return

        i = msg.id - 0x101
        if i < 0 or i >= self.num_wheels:
            return

        data_bytes = bytes(msg.data[:8])

        angle = struct.unpack('<f', data_bytes[0:4])[0]
        speed = struct.unpack('<f', data_bytes[4:8])[0]

        self.angles[i] = float(angle)
        self.speeds[i] = float(speed) / 60.0
        self.received[i] = True

        if all(self.received):
            swerve = Swerve()
            swerve.wheel_angle = list(self.angles)
            swerve.wheel_speed = list(self.speeds)

            self.pub_swerve.publish(swerve)

            self.received = [False] * self.num_wheels


def main(args=None):
    rclpy.init(args=args)
    node = SwerveCanBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
