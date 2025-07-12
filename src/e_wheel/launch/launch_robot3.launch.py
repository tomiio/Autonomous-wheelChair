import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'e_wheel'  # <--- CHANGE ME

    # Include the robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    robot_description = Command([
        'ros2 param get --hide-type /robot_state_publisher robot_description'
    ])
    controller_params_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'my_controllers.yaml'
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output='screen',
        parameters=[{'robot_description': robot_description}, controller_params_file],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    relay_node = Node(
        package="topic_tools",
        executable="relay",
        name="cmd_vel_relay",
        output="screen",
        arguments=["/cmd_vel", "/diff_cont/cmd_vel_unstamped"]
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')
        ])
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ])
    )

    depth_to_pointcloud_node = Node(
        package="depth_image_proc",
        executable="point_cloud_xyz_node",
        name="depth_to_pointcloud",
        output="screen",
        remappings=[
            ("image_rect", "/camera/camera/depth/image_rect_raw"),
            ("camera_info", "/camera/camera/depth/camera_info"),
            ("points", "/camera/points")
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'nav2_localization_launch.py')
        ]),
        launch_arguments={'map': './map20.yaml'}.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'nav2_navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'map_subscribe_transient_local': 'true'
        }.items()
    )

    return LaunchDescription([
        rsp,
        TimerAction(period=3.0, actions=[controller_manager]),
        TimerAction(period=6.0, actions=[diff_drive_spawner]),
        TimerAction(period=9.0, actions=[joint_broad_spawner]),
        TimerAction(period=12.0, actions=[relay_node]),
        TimerAction(period=15.0, actions=[rplidar_launch]),
        TimerAction(period=18.0, actions=[realsense_launch]),
        TimerAction(period=21.0, actions=[depth_to_pointcloud_node]),
        TimerAction(period=24.0, actions=[rviz_node]),
        TimerAction(period=27.0, actions=[localization_launch]),
        TimerAction(period=30.0, actions=[navigation_launch]),
    ])
