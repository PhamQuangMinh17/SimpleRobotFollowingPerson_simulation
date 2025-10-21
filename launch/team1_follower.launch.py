#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'team1_tb3_sim'
    pkg_share = get_package_share_directory(pkg_name)

    # Common setup
    setup_env = [f"source {os.path.expanduser('~/turtlebot3_ws/install/setup.bash')}"]

    #  1️Object Detection Node (YOLOv8) 
    yolo_node = ExecuteProcess(
        cmd=[
            '/bin/bash', '-c',
            ' && '.join([
                f"source {os.path.expanduser('~/turtlebot3_ws/install/setup.bash')}",
                f"source {os.path.expanduser('~/venvs/ultra/bin/activate')}",
                f"ros2 run {pkg_name} object_detection_node.py"
            ])
        ],
        output='screen'
    )

    # 2️ Angle-from-Pixel Node
    angle_node = TimerAction(
        period = 3.0,  # wait 3 seconds
        actions=[
            Node(
                package=pkg_name,
                executable='angle_from_pixel_node.py',
                name='angle_from_pixel_node',
                output='screen'
            )
        ]
    )

    #  3 Distance-from-YOLO Node 
    distance_node = TimerAction(
        period=6.0,  # wait 6 sec after launch start
        actions=[
            ExecuteProcess(
                cmd=[
                    '/bin/bash', '-c',
                    ' && '.join([
                        f"source {os.path.expanduser('~/turtlebot3_ws/install/setup.bash')}",
                        f"source {os.path.expanduser('~/venvs/ultra/bin/activate')}",
                        f"ros2 run {pkg_name} distance_from_yolo_node.py"
                    ])
                ],
                output='screen'
            )
        ]
    )

    # 4️ Control Node 
    control_node = TimerAction(
        period=9.0,  # wait 9 sec
        actions=[
            Node(
                package=pkg_name,
                executable='control_node.py',
                name='follow_controller',
                output='screen',
                parameters=[
                    {'reverse_cmd': True},
                    {'desired_distance': 2.0},
                    {'yaw_kp': 1.2}, {'yaw_kd': 0.15},
                    {'dist_kp': 0.6}, {'dist_kd': 0.05},
                ]
            )
        ]
    )

    return LaunchDescription([
        yolo_node,
        angle_node,
        distance_node,
        control_node
    ])
