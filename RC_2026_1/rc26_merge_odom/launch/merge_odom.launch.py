#!/usr/bin/env python3
# RC2026 融合里程计完整启动文件
# 启动: 融合里程计节点(可选CAN/Wheel) + 达妙IMU节点 + EKF融合节点

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import yaml


def generate_launch_description():
    pkg_share = get_package_share_directory('rc26_merge_odom')

    params_file = os.path.join(pkg_share, 'config', 'merge_odom_params.yaml')
    ekf_params_file = os.path.join(pkg_share, 'config', 'ekf_params.yaml')

    with open(params_file, 'r', encoding='utf-8') as f:
        params_yaml = yaml.safe_load(f) or {}
    merge_params = (params_yaml.get('merge_odom_node') or {}).get('ros__parameters') or {}

    use_can_odom_default = bool(merge_params.get('use_can_odom', True))
    can_interface_default = str(merge_params.get('can_interface', 'can0'))
    imu_port_default = str(((params_yaml.get('dm_imu_node') or {}).get('ros__parameters') or {}).get('port', '/dev/ttyACM0'))
    feedback_serial_port_default = str(merge_params.get('feedback_serial_port', '/dev/ttyUSB0'))
    target_serial_port_default = str(merge_params.get('target_serial_port', '/dev/ttyUSB1'))
    odom0_topic_override = str(merge_params.get('can_odom_topic', 'Can_Odom'))

    use_can_odom_arg = DeclareLaunchArgument(
        'use_can_odom',
        default_value=str(use_can_odom_default).lower(),
        description='Odometry source switch: true=can_odom, false=wheel_odom')

    can_interface_arg = DeclareLaunchArgument(
        'can_interface', default_value=can_interface_default,
        description='CAN interface name')

    imu_port_arg = DeclareLaunchArgument(
        'imu_port', default_value=imu_port_default,
        description='IMU serial port')

    feedback_serial_port_arg = DeclareLaunchArgument(
        'feedback_serial_port', default_value=feedback_serial_port_default,
        description='MCU feedback serial port (velocity feedback)')

    target_serial_port_arg = DeclareLaunchArgument(
        'target_serial_port', default_value=target_serial_port_default,
        description='MCU target serial port (velocity target + ODOM_DATA upstream)')

    # 融合里程计节点 (整合CAN/Wheel里程计 + 速度发送，双串口架构)
    merge_odom_node = Node(
        package='rc26_merge_odom',
        executable='merge_odom_node',
        name='merge_odom_node',
        output='screen',
        parameters=[params_file, {
            'use_can_odom': ParameterValue(LaunchConfiguration('use_can_odom'), value_type=bool),
            'can_interface': LaunchConfiguration('can_interface'),
            'feedback_serial_port': LaunchConfiguration('feedback_serial_port'),
            'target_serial_port': LaunchConfiguration('target_serial_port'),
        }]
    )

    # 达妙IMU节点
    dm_imu_node = Node(
        package='rc26_merge_odom',
        executable='dm_imu_node',
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
        parameters=[ekf_params_file, {
            # 用 merge_odom_node 的里程计 topic 覆盖 ekf_params.yaml 中的 odom0
            'odom0': odom0_topic_override,
        }],
        remappings=[
            ('odometry/filtered', 'merge_odom'),
        ]
    )

    return LaunchDescription([
        use_can_odom_arg,
        can_interface_arg,
        imu_port_arg,
        feedback_serial_port_arg,
        target_serial_port_arg,
        merge_odom_node,
        dm_imu_node,
        ekf_node,
    ])
