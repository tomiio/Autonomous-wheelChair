from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Convert Depth Camera PointCloud to LaserScan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[{
                "min_height": 0.2,
                "max_height": 2.0,
                "target_frame": "base_link",
                "range_min": 0.3,
                "range_max": 3.0,  # Adjust based on depth camera specs
                "angle_min": -0.785,  # -90 degrees
                "angle_max": 0.785,   # 90 degrees
                "angle_increment": 0.01,
                "scan_time": 0.1,
                "use_inf": True,
                "concurrency_level": 1,
            }],
            remappings=[
                ("/cloud_in", "/camera/points"),  # Input point cloud
                ("/scan", "/camera_scan")  # Output laser scan
            ],
            # Explicitly set sensor QoS
            arguments=["--ros-args", "--remap", "__node:=pointcloud_to_laserscan", "--ros-args", "--param", "qos_overrides./scan.publisher.reliability:=best_effort"]
        )
    ])
