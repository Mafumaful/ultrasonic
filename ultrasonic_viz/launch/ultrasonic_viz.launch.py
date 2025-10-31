#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for ultrasonic visualization node."""

    # Get package directory
    pkg_dir = get_package_share_directory('ultrasonic_viz')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'ultrasonic_params.yaml'),
        description='Path to the config file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Create ultrasonic visualization node
    ultrasonic_viz_node = Node(
        package='ultrasonic_viz',
        executable='ultrasonic_viz_node',
        name='ultrasonic_viz_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True,
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        ultrasonic_viz_node,
    ])
