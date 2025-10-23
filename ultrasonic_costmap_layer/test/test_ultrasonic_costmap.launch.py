#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch file to test ultrasonic costmap layer.

    This will:
    1. Start a local costmap with the ultrasonic layer plugin
    2. Subscribe to /ultrasonic topic for sensor data
    3. Publish costmap to /local_costmap/costmap for visualization
    """

    print("Starting ultrasonic costmap layer test launch file...")

    # Get package directory
    pkg_dir = get_package_share_directory('ultrasonic_costmap_layer')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'ultrasonic_layer_params.yaml'),
        description='Path to the config file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Create costmap node
    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/local_costmap/costmap', '/ultrasonic_costmap'),
            ('/local_costmap/costmap_updates', '/ultrasonic_costmap_updates'),
        ]
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        costmap_node,
    ])
