#!/usr/bin/env python3
# 仅启动CAN里程计节点

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('rc26_merge_odom')
    params_file = os.path.join(pkg_share, 'config', 'merge_odom_params.yaml')

    can_interface_arg = DeclareLaunchArgument(
        'can_interface', default_value='can0',
        description='CAN interface name')

    can_odom_node = Node(
        package='rc26_merge_odom',
        executable='can_odom_node',
        name='can_odom_node',
        output='screen',
        parameters=[params_file, {
            'can_interface': LaunchConfiguration('can_interface'),
        }]
    )

    return LaunchDescription([
        can_interface_arg,
        can_odom_node,
    ])
