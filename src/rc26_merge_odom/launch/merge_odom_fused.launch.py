#!/usr/bin/env python3
# RC2026 融合里程计 fused 启动文件
# 启动: merge_odom_node(wheel) + can_odom_node + dm_imu_node + wheel_odom_fuser_node + EKF

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('rc26_merge_odom')
    params_file = os.path.join(pkg_share, 'config', 'merge_odom_params.yaml')
    ekf_params_file = os.path.join(pkg_share, 'config', 'ekf_params.yaml')

    with open(params_file, 'r', encoding='utf-8') as f:
        params_yaml = yaml.safe_load(f) or {}

    merge_params = (params_yaml.get('merge_odom_node') or {}).get('ros__parameters') or {}
    can_params = (params_yaml.get('can_odom_node') or {}).get('ros__parameters') or {}
    imu_params = (params_yaml.get('dm_imu_node') or {}).get('ros__parameters') or {}
    fuser_params = (params_yaml.get('wheel_odom_fuser_node') or {}).get('ros__parameters') or {}

    can_interface_default = str(
        merge_params.get('can_interface', can_params.get('can_interface', 'can0'))
    )
    imu_port_default = str(imu_params.get('port', '/dev/ttyACM0'))
    feedback_serial_port_default = str(merge_params.get('feedback_serial_port', '/dev/ttyUSB0'))
    target_serial_port_default = str(merge_params.get('target_serial_port', '/dev/ttyUSB1'))
    fused_odom_topic_default = str(fuser_params.get('fused_odom_topic', 'wheel_odom_fused'))

    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value=can_interface_default,
        description='CAN interface name',
    )

    imu_port_arg = DeclareLaunchArgument(
        'imu_port',
        default_value=imu_port_default,
        description='IMU serial port',
    )

    feedback_serial_port_arg = DeclareLaunchArgument(
        'feedback_serial_port',
        default_value=feedback_serial_port_default,
        description='MCU feedback serial port (ODOM_DATA + POSE_FEEDBACK)',
    )

    target_serial_port_arg = DeclareLaunchArgument(
        'target_serial_port',
        default_value=target_serial_port_default,
        description='MCU target serial port (POSE_TARGET)',
    )

    merge_odom_node = Node(
        package='rc26_merge_odom',
        executable='merge_odom_node',
        name='merge_odom_node',
        output='screen',
        parameters=[params_file, {
            'use_can_odom': ParameterValue(False, value_type=bool),
            'can_interface': LaunchConfiguration('can_interface'),
            'feedback_serial_port': LaunchConfiguration('feedback_serial_port'),
            'target_serial_port': LaunchConfiguration('target_serial_port'),
        }],
    )

    can_odom_node = Node(
        package='rc26_merge_odom',
        executable='can_odom_node',
        name='can_odom_node',
        output='screen',
        parameters=[params_file, {
            'can_interface': LaunchConfiguration('can_interface'),
        }],
    )

    dm_imu_node = Node(
        package='rc26_merge_odom',
        executable='dm_imu_node',
        name='dm_imu_node',
        output='screen',
        parameters=[params_file, {
            'port': LaunchConfiguration('imu_port'),
        }],
    )

    wheel_odom_fuser_node = Node(
        package='rc26_merge_odom',
        executable='wheel_odom_fuser_node',
        name='wheel_odom_fuser_node',
        output='screen',
        parameters=[params_file],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {
            'odom0': fused_odom_topic_default,
            'imu0': 'DM_IMU',
        }],
        remappings=[
            ('odometry/filtered', 'merge_odom'),
        ],
    )

    return LaunchDescription([
        can_interface_arg,
        imu_port_arg,
        feedback_serial_port_arg,
        target_serial_port_arg,
        merge_odom_node,
        can_odom_node,
        dm_imu_node,
        wheel_odom_fuser_node,
        ekf_node,
    ])
