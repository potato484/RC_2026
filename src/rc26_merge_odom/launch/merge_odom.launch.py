#!/usr/bin/env python3
# RC2026 融合里程计完整启动文件
# 启动: 融合里程计节点(可选CAN/Wheel) + 达妙IMU节点 + (可选)EKF融合节点

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml


def parse_launch_bool(value):
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def drop_sensor_parameters(parameters, base_name):
    return {
        key: value for key, value in parameters.items()
        if key != base_name and not key.startswith(f'{base_name}_')
    }


def create_runtime_nodes(context, params_file, ekf_params_file, can_odom_topic, wheel_odom_topic,
                         imu_topic_default, slip_enable_default, imu_gate_enable_default,
                         latency_comp_enable_default, stats_log_enable_default):
    use_can_odom = parse_launch_bool(LaunchConfiguration('use_can_odom').perform(context))
    start_ekf = parse_launch_bool(LaunchConfiguration('start_ekf').perform(context))
    use_imu_for_ekf = parse_launch_bool(LaunchConfiguration('use_imu_for_ekf').perform(context))
    start_imu = parse_launch_bool(LaunchConfiguration('start_imu').perform(context))
    effective_use_imu_for_ekf = start_imu and use_imu_for_ekf
    odom0_topic_override = can_odom_topic if use_can_odom else wheel_odom_topic
    pose_feedback_topic = 'merge_odom' if start_ekf else odom0_topic_override
    chassis_model = LaunchConfiguration('chassis_model').perform(context).strip()
    imu_topic = imu_topic_default if start_imu else ''
    slip_enable = slip_enable_default if start_imu else False
    imu_gate_enable = imu_gate_enable_default if start_imu else False
    latency_comp_enable = latency_comp_enable_default if start_imu else False
    stats_log_enable = parse_launch_bool(LaunchConfiguration('stats_log_enable').perform(context))

    merge_odom_node = Node(
        package='rc26_merge_odom',
        executable='merge_odom_node',
        name='merge_odom_node',
        output='screen',
        parameters=[params_file, {
            'use_can_odom': use_can_odom,
            'can_interface': LaunchConfiguration('can_interface').perform(context),
            'feedback_serial_port': LaunchConfiguration('feedback_serial_port').perform(context),
            'target_serial_port': LaunchConfiguration('target_serial_port').perform(context),
            'merge_odom_topic': pose_feedback_topic,
            'imu_topic': imu_topic,
            'slip_enable': slip_enable,
            'imu_gate_enable': imu_gate_enable,
            'latency_comp_enable': latency_comp_enable,
            'stats_log_enable': stats_log_enable,
            'chassis_model': chassis_model,
        }]
    )

    nodes = [merge_odom_node]
    if start_imu:
        dm_imu_node = Node(
            package='rc26_merge_odom',
            executable='dm_imu_node',
            name='dm_imu_node',
            output='screen',
            parameters=[params_file, {
                'port': LaunchConfiguration('imu_port').perform(context),
            }]
        )
        nodes.append(dm_imu_node)

    if start_ekf:
        with open(ekf_params_file, 'r', encoding='utf-8') as f:
            ekf_yaml = yaml.safe_load(f) or {}
        ekf_parameters = dict(((ekf_yaml.get('ekf_filter_node') or {}).get('ros__parameters') or {}))
        ekf_parameters['odom0'] = odom0_topic_override
        if not effective_use_imu_for_ekf:
            ekf_parameters = drop_sensor_parameters(ekf_parameters, 'imu0')

        ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_parameters],
            remappings=[
                ('odometry/filtered', 'merge_odom'),
            ]
        )
        nodes.append(ekf_node)

    return nodes


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
    can_odom_topic_default = str(merge_params.get('can_odom_topic', 'Can_Odom'))
    wheel_odom_topic_default = str(merge_params.get('wheel_odom_topic', 'wheel_odom'))
    imu_topic_default = str(merge_params.get('imu_topic', 'DM_IMU'))
    slip_enable_default = bool(merge_params.get('slip_enable', True))
    imu_gate_enable_default = bool(merge_params.get('imu_gate_enable', True))
    latency_comp_enable_default = bool(merge_params.get('latency_comp_enable', True))
    stats_log_enable_default = bool(merge_params.get('stats_log_enable', False))
    chassis_model_default = str(merge_params.get('chassis_model', 'mecanum_4wheel'))

    use_can_odom_arg = DeclareLaunchArgument(
        'use_can_odom',
        default_value=str(use_can_odom_default).lower(),
        description='Odometry source switch: true=can_odom, false=wheel_odom')

    start_ekf_arg = DeclareLaunchArgument(
        'start_ekf',
        default_value='true',
        description='Whether to start robot_localization EKF. For teleop + SLAM, set false to avoid odom->base_link TF conflict.')
    use_imu_for_ekf_arg = DeclareLaunchArgument(
        'use_imu_for_ekf',
        default_value='true',
        description='Whether EKF fuses IMU input. false removes imu0/imu0_* from EKF only; dm_imu_node and merge_odom protections still run.')
    start_imu_arg = DeclareLaunchArgument(
        'start_imu',
        default_value='true',
        description='Whether to start/read DM IMU. false also disables IMU-dependent slip and PoseSender protections.')
    stats_log_enable_arg = DeclareLaunchArgument(
        'stats_log_enable',
        default_value=str(stats_log_enable_default).lower(),
        description='Enable PoseSender 1s stats logs')

    can_interface_arg = DeclareLaunchArgument(
        'can_interface', default_value=can_interface_default,
        description='CAN interface name')

    imu_port_arg = DeclareLaunchArgument(
        'imu_port', default_value=imu_port_default,
        description='IMU serial port')

    feedback_serial_port_arg = DeclareLaunchArgument(
        'feedback_serial_port', default_value=feedback_serial_port_default,
        description='MCU feedback serial port (ODOM_DATA receive + POSE_FEEDBACK send)')

    target_serial_port_arg = DeclareLaunchArgument(
        'target_serial_port', default_value=target_serial_port_default,
        description='MCU target serial port (POSE_TARGET + /mechanism/transport/*)')

    chassis_model_arg = DeclareLaunchArgument(
        'chassis_model', default_value=chassis_model_default,
        description='Chassis model: mecanum_4wheel | tracked_diff')

    return LaunchDescription([
        use_can_odom_arg,
        start_ekf_arg,
        use_imu_for_ekf_arg,
        start_imu_arg,
        stats_log_enable_arg,
        can_interface_arg,
        imu_port_arg,
        feedback_serial_port_arg,
        target_serial_port_arg,
        chassis_model_arg,
        OpaqueFunction(
            function=create_runtime_nodes,
            args=[
                params_file,
                ekf_params_file,
                can_odom_topic_default,
                wheel_odom_topic_default,
                imu_topic_default,
                slip_enable_default,
                imu_gate_enable_default,
                latency_comp_enable_default,
                stats_log_enable_default,
            ],
        ),
    ])
