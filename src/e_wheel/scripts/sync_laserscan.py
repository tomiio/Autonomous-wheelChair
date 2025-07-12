#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from collections import deque
import time


class LaserScanSync(Node):
    def __init__(self):
        super().__init__("laser_scan_sync")

        # QoS Profiles (to match publishers' QoS)
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,  # Ensure compatibility
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.sub_lidar = self.create_subscription(LaserScan, "/scan", self.lidar_callback, qos_profile)
        self.sub_camera = self.create_subscription(LaserScan, "/camera_scan", self.camera_callback, qos_profile)

        # Publisher
        self.pub_sync_scan = self.create_publisher(LaserScan, "/merged_scan", qos_profile)

        # Buffers to store latest messages
        self.last_lidar_scan = None
        self.last_camera_scan = None
        self.last_lidar_time = None
        self.last_camera_time = None

        # Maximum allowable time difference (adjust as needed)
        self.sync_tolerance = 0.1  # 100ms

    def lidar_callback(self, msg):
        """ Callback when LiDAR scan arrives. """
        self.last_lidar_scan = msg
        self.last_lidar_time = time.time()
        self.publish_synced_scan()

    def camera_callback(self, msg):
        """ Callback when Camera scan arrives. """
        self.last_camera_scan = msg
        self.last_camera_time = time.time()
        self.publish_synced_scan()

    def publish_synced_scan(self):
        """ Publish a synchronized LaserScan message. """
        now = time.time()

        # Use latest messages, or fallback to the last known scan
        lidar_scan = self.last_lidar_scan if self.last_lidar_scan else None
        camera_scan = self.last_camera_scan if self.last_camera_scan else None

        # If lidar is too old, use the last known
        if self.last_lidar_scan and (now - self.last_lidar_time) > self.sync_tolerance:
            lidar_scan = self.last_lidar_scan  # Keep using last known
            self.get_logger().warn("Using previous LiDAR scan for sync")

        # If camera is too old, use the last known
        if self.last_camera_scan and (now - self.last_camera_time) > self.sync_tolerance:
            camera_scan = self.last_camera_scan  # Keep using last known
            self.get_logger().warn("Using previous Camera scan for sync")

        # Ensure we have valid data for both scans before publishing
        if lidar_scan and camera_scan:
            merged_scan = self.merge_scans(lidar_scan, camera_scan)
            self.pub_sync_scan.publish(merged_scan)

    def merge_scans(self, lidar_scan, camera_scan):
        """ Merge two LaserScan messages by taking the minimum range value per angle. """
        merged_scan = LaserScan()
        merged_scan.header.stamp = self.get_clock().now().to_msg()
        merged_scan.header.frame_id = lidar_scan.header.frame_id  # Use LiDAR frame

        # Merge ranges (choose the closest measurement per index)
        merged_scan.ranges = [
            min(l, c) if l > 0 and c > 0 else (l if l > 0 else c)
            for l, c in zip(lidar_scan.ranges, camera_scan.ranges)
        ]

        # Use LiDAR scan properties for the merged scan
        merged_scan.angle_min = lidar_scan.angle_min
        merged_scan.angle_max = lidar_scan.angle_max
        merged_scan.angle_increment = lidar_scan.angle_increment
        merged_scan.time_increment = lidar_scan.time_increment
        merged_scan.scan_time = lidar_scan.scan_time
        merged_scan.range_min = min(lidar_scan.range_min, camera_scan.range_min)
        merged_scan.range_max = max(lidar_scan.range_max, camera_scan.range_max)
        return merged_scan


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanSync()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
