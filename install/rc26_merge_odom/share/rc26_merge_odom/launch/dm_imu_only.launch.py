#!/usr/bin/env python3
# 仅启动达妙IMU节点

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('rc26_merge_odom')
    params_file = os.path.join(pkg_share, 'config', 'merge_odom_params.yaml')

    imu_port_arg = DeclareLaunchArgument(
        'imu_port', default_value='/dev/ttyACM0',
        description='IMU serial port')

    dm_imu_node = Node(
        package='rc26_merge_odom',
        executable='dm_imu_node',
        name='dm_imu_node',
        output='screen',
        parameters=[params_file, {
            'port': LaunchConfiguration('imu_port'),
        }]
    )

    return LaunchDescription([
        imu_port_arg,
        dm_imu_node,
    ])
