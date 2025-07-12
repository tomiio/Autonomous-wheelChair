#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan
import math

class RangeToLaserScan(Node):
    def __init__(self):
        super().__init__('range_to_laserscan')

        # Subscribe to ultrasonic sensors
        self.left_sub = self.create_subscription(Range, '/ultrasonic_left', self.left_callback, 10)
        self.right_sub = self.create_subscription(Range, '/ultrasonic_right', self.right_callback, 10)

        # Publishers for LaserScan
        self.left_pub = self.create_publisher(LaserScan, '/ultrasonic_left/scan', 10)
        self.right_pub = self.create_publisher(LaserScan, '/ultrasonic_right/scan', 10)

    def convert_to_laserscan(self, range_msg):
        scan = LaserScan()
        scan.header = range_msg.header
        scan.angle_min = -0.1  # Small FOV to simulate a beam
        scan.angle_max = 0.1
        scan.angle_increment = 0.1
        scan.range_min = range_msg.min_range
        scan.range_max = range_msg.max_range
        scan.ranges = [range_msg.range if range_msg.range >= range_msg.min_range else float('inf')]
        return scan

    def left_callback(self, msg):
        self.left_pub.publish(self.convert_to_laserscan(msg))

    def right_callback(self, msg):
        self.right_pub.publish(self.convert_to_laserscan(msg))

def main(args=None):
    rclpy.init(args=args)
    node = RangeToLaserScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
