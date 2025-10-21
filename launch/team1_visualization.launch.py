#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_team1 = get_package_share_directory('team1_tb3_sim')

    # allow overrides from CLI: rviz_config and use_sim_time
    rviz_config = LaunchConfiguration(
        'rviz_config',
        default=os.path.join(pkg_team1, 'rviz', 'team1_model.rviz')
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([rviz])
