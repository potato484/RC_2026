#!/usr/bin/env python3
# RC2026 融合里程计完整启动文件
# 启动: CAN里程计节点 + 达妙IMU节点 + EKF融合节点 + 速度发送节点

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('rc26_merge_odom')

    params_file = os.path.join(pkg_share, 'config', 'merge_odom_params.yaml')
    ekf_params_file = os.path.join(pkg_share, 'config', 'ekf_params.yaml')

    # 声明参数
    can_interface_arg = DeclareLaunchArgument(
        'can_interface', default_value='can0',
        description='CAN interface name')

    imu_port_arg = DeclareLaunchArgument(
        'imu_port', default_value='/dev/ttyACM0',
        description='IMU serial port')

    feedback_serial_port_arg = DeclareLaunchArgument(
        'feedback_serial_port', default_value='/dev/ttyUSB0',
        description='MCU feedback serial port (velocity feedback)')

    target_serial_port_arg = DeclareLaunchArgument(
        'target_serial_port', default_value='/dev/ttyUSB1',
        description='MCU target serial port (velocity target)')

    # CAN里程计节点
    can_odom_node = Node(
        package='rc26_merge_odom',
        executable='can_odom_node',
        name='can_odom_node',
        output='screen',
        parameters=[params_file, {
            'can_interface': LaunchConfiguration('can_interface'),
        }]
    )

    # 达妙IMU节点
    dm_imu_node = Node(
        package='rc26_merge_odom',
        executable='dm_imu_node.py',
        name='dm_imu_node',
        output='screen',
        parameters=[params_file, {
            'port': LaunchConfiguration('imu_port'),
        }]
    )

    # robot_localization EKF节点
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file],
        remappings=[
            ('odometry/filtered', 'merge_odom'),
        ]
    )

    # 速度发送节点 (双串口)
    pose_sender_node = Node(
        package='rc26_merge_odom',
        executable='pose_sender_node',
        name='pose_sender_node',
        output='screen',
        parameters=[params_file, {
            'feedback_serial_port': LaunchConfiguration('feedback_serial_port'),
            'target_serial_port': LaunchConfiguration('target_serial_port'),
        }]
    )

    return LaunchDescription([
        can_interface_arg,
        imu_port_arg,
        feedback_serial_port_arg,
        target_serial_port_arg,
        can_odom_node,
        dm_imu_node,
        ekf_node,
        pose_sender_node,
    ])
