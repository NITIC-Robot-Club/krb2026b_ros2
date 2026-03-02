#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import time


class HzMonitor(Node):

    def __init__(self):
        super().__init__('duck_bbox_hz_monitor')

        self.subscription = self.create_subscription(
            Image,
            '/detection/duck_bbox_image',
            self.callback,
            10
        )

        self.last_time = None
        self.count = 0
        self.start_time = time.time()

        self.get_logger().info("Hz monitor started...")

    def callback(self, msg):
        now = time.time()

        if self.last_time is not None:
            dt = now - self.last_time
            if dt > 0:
                hz = 1.0 / dt
                self.get_logger().info(f"Instant Hz: {hz:.2f}")

        self.last_time = now

        # 平均Hzも出す
        self.count += 1
        elapsed = now - self.start_time
        if elapsed > 5.0:
            avg_hz = self.count / elapsed
            self.get_logger().info(f"Average Hz (5s): {avg_hz:.2f}")
            self.count = 0
            self.start_time = now


def main(args=None):
    rclpy.init(args=args)
    node = HzMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()