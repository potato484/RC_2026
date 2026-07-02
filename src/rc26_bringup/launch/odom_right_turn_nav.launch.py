"""
右转导航独立入口。

启动:
  - 可选 odometry 链；当前树读取 /odom，默认启动
  - 可选 rc26_mcu_transport，消费 /cmd_vel 并下发底盘运动
  - rc26_decision decision_node，加载 odom_right_turn_nav_tree.xml

本入口不启动定位或遥控，确保 /cmd_vel 只有一个运动命令权威。
动作树内使用固定右转验收距离和通用 odom 相对导航参数，默认 odom 闭环 x+ 0.4m -> 右转90°闭环对齐 -> odom 闭环 x- 0.7m。
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _parse_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_bool(value):
    return str(bool(value)).lower()


def _launch_value(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def _select_str(context, name, default_value):
    value = _launch_value(context, name)
    return default_value if value == '' else value


def _select_bool(context, name, default_value):
    value = _launch_value(context, name)
    if value != '':
        return _parse_bool(value)
    return _parse_bool(default_value) if isinstance(default_value, str) else bool(default_value)


def _load_runtime_defaults(config_file):
    if not os.path.isabs(config_file):
        raise RuntimeError(f"runtime_config_file must be an absolute path: {config_file}")
    if not os.path.exists(config_file):
        raise RuntimeError(f"runtime_config_file does not exist: {config_file}")

    with open(config_file, 'r', encoding='utf-8') as f:
        yaml_data = yaml.safe_load(f) or {}

    runtime = yaml_data.get('r2_runtime') or {}
    decision = runtime.get('decision') or {}
    mcu_transport = runtime.get('mcu_transport') or {}
    mcu_params = mcu_transport.get('ros__parameters') or {}

    defaults = {
        'decision_params': dict(decision.get('ros__parameters') or {}),
        'mcu_transport': {
            'enabled': True,
            'target_serial_port': '/dev/ttyUSB0',
            'target_baudrate': 1000000,
            'open_retry_period_ms': 1000,
            'diagnostics_period_ms': 1000,
            'enable_chassis_cmd_vel_consumer': True,
            'chassis_cmd_vel_topic': 'cmd_vel',
            'chassis_target_send_rate_hz': 50,
            'chassis_cmd_vel_timeout_ms': 200,
            'chassis_v_max_mps': 2.0,
            'chassis_w_max_radps': 2.0,
            'chassis_stop_repeat_n': 10,
        },
    }

    enabled_value = mcu_transport.get('enabled', True)
    defaults['mcu_transport']['enabled'] = (
        _parse_bool(enabled_value) if isinstance(enabled_value, str) else bool(enabled_value)
    )
    for key in (
        'target_serial_port',
        'target_baudrate',
        'open_retry_period_ms',
        'diagnostics_period_ms',
        'enable_chassis_cmd_vel_consumer',
        'chassis_cmd_vel_topic',
        'chassis_target_send_rate_hz',
        'chassis_cmd_vel_timeout_ms',
        'chassis_v_max_mps',
        'chassis_w_max_radps',
        'chassis_stop_repeat_n',
    ):
        if key in mcu_params:
            defaults['mcu_transport'][key] = mcu_params[key]
    return defaults


def _create_actions(context, *, bringup_dir, decision_dir, sensor_extrinsics_dir, mcu_transport_dir):
    runtime_config_file = _launch_value(context, 'runtime_config_file')
    runtime_defaults = _load_runtime_defaults(runtime_config_file)
    decision_params = dict(runtime_defaults['decision_params'])
    decision_params.pop('tree_file', None)
    decision_params.pop('team', None)

    namespace = _launch_value(context, 'namespace')
    use_sim_time = _parse_bool(_launch_value(context, 'use_sim_time'))
    use_sim_time_text = _launch_bool(use_sim_time)
    team = _select_str(context, 'team', str(runtime_defaults['decision_params'].get('team', 'blue')))
    tree_file = os.path.join(decision_dir, 'behavior_trees', 'odom_right_turn_nav_tree.xml')

    sensor_extrinsics_file = _select_str(
        context,
        'sensor_extrinsics_file',
        os.path.join(sensor_extrinsics_dir, 'config', 'r2_sensor_extrinsics.yaml'))
    sensor_extrinsics_profile = _launch_value(context, 'sensor_extrinsics_profile')
    point_lio_config_file = _launch_value(context, 'point_lio_config_file')
    start_mid360_driver = _launch_value(context, 'start_mid360_driver') or 'true'
    recover_mid360_stream = _launch_value(context, 'recover_mid360_stream') or 'false'
    start_odometry = _parse_bool(_launch_value(context, 'start_odometry') or 'true')

    mcu_defaults = runtime_defaults['mcu_transport']
    start_mcu_transport = _select_bool(context, 'start_mcu_transport', mcu_defaults['enabled'])

    actions = []
    if start_odometry:
        odometry_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_dir, 'launch', 'odometry.launch.py'])
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time_text,
                'point_lio_config_file': point_lio_config_file,
                'sensor_extrinsics_file': sensor_extrinsics_file,
                'sensor_extrinsics_profile': sensor_extrinsics_profile,
                'point_lio_publish_odometry_without_downsample': 'false',
                'odom_interface_publish_bootstrap_pose': 'false',
                'start_sensor_scan': 'false',
                'start_mid360_driver': start_mid360_driver,
                'recover_mid360_stream': recover_mid360_stream,
            }.items()
        )
        actions.append(GroupAction(actions=[odometry_launch], scoped=True))

    if start_mcu_transport:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([mcu_transport_dir, 'launch', 'mcu_transport.launch.py'])
            ),
            launch_arguments={
                'target_serial_port': str(mcu_defaults['target_serial_port']),
                'target_baudrate': str(mcu_defaults['target_baudrate']),
                'open_retry_period_ms': str(mcu_defaults['open_retry_period_ms']),
                'diagnostics_period_ms': str(mcu_defaults['diagnostics_period_ms']),
                'enable_chassis_cmd_vel_consumer': _launch_bool(
                    mcu_defaults['enable_chassis_cmd_vel_consumer']),
                'chassis_cmd_vel_topic': str(mcu_defaults['chassis_cmd_vel_topic']),
                'chassis_target_send_rate_hz': str(mcu_defaults['chassis_target_send_rate_hz']),
                'chassis_cmd_vel_timeout_ms': str(mcu_defaults['chassis_cmd_vel_timeout_ms']),
                'chassis_v_max_mps': str(mcu_defaults['chassis_v_max_mps']),
                'chassis_w_max_radps': str(mcu_defaults['chassis_w_max_radps']),
                'chassis_stop_repeat_n': str(mcu_defaults['chassis_stop_repeat_n']),
            }.items(),
        ))

    actions.append(Node(
        package='rc26_decision',
        executable='decision_node',
        name='rc26_decision',
        namespace=namespace,
        output='screen',
        parameters=[
            decision_params,
            {
                'use_sim_time': use_sim_time,
                'team': team,
                'tree_file': tree_file,
                'startup_wait_for_odom': True,
                'startup_odom_topic': decision_params.get(
                    'startup_odom_topic',
                    decision_params.get('odom_relative_nav_odom_topic', 'odom')),
                'startup_odom_timeout_s': decision_params.get(
                    'startup_odom_timeout_s',
                    decision_params.get('odom_relative_nav_odom_timeout_s', 0.5)),
                'startup_odom_wait_timeout_s': decision_params.get(
                    'startup_odom_wait_timeout_s', 20.0),
                'startup_odom_min_wait_s': decision_params.get(
                    'startup_odom_min_wait_s', 1.0),
                'startup_odom_stable_samples': decision_params.get(
                    'startup_odom_stable_samples', 10),
                'startup_odom_max_linear_speed_mps': decision_params.get(
                    'startup_odom_max_linear_speed_mps', 0.03),
                'startup_odom_max_angular_speed_radps': decision_params.get(
                    'startup_odom_max_angular_speed_radps', 0.05),
            },
        ],
    ))
    return actions


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc26_bringup')
    decision_dir = get_package_share_directory('rc26_decision')
    sensor_extrinsics_dir = get_package_share_directory('rc26_sensor_extrinsics')
    mcu_transport_dir = get_package_share_directory('rc26_mcu_transport')

    return LaunchDescription([
        DeclareLaunchArgument(
            'runtime_config_file',
            default_value=PathJoinSubstitution([bringup_dir, 'config', 'r2_red.yaml']),
            description='R2 统一运行配置 YAML'),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='顶级命名空间'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用仿真时间'),
        DeclareLaunchArgument(
            'team',
            default_value='',
            description='Active competition side: blue | red；空字符串表示使用运行配置'),
        DeclareLaunchArgument(
            'start_odometry',
            default_value='true',
            description='是否启动 odometry 链；当前闭环右转树读取 /odom'),
        DeclareLaunchArgument(
            'start_mcu_transport',
            default_value='',
            description='是否启动 rc26_mcu_transport；空字符串表示使用运行配置'),
        DeclareLaunchArgument(
            'point_lio_config_file',
            default_value='',
            description='Point-LIO 参数文件路径；为空时使用 rc26_point_lio/config/mid360.yaml'),
        DeclareLaunchArgument(
            'sensor_extrinsics_file',
            default_value='',
            description='传感器安装外参 YAML 文件路径；为空时使用 rc26_sensor_extrinsics 默认配置'),
        DeclareLaunchArgument(
            'sensor_extrinsics_profile',
            default_value='',
            description='传感器安装外参 profile；空字符串表示使用 YAML defaults.active_profile'),
        DeclareLaunchArgument(
            'start_mid360_driver',
            default_value='true',
            description='是否由 odometry 链启动 MID-360 驱动'),
        DeclareLaunchArgument(
            'recover_mid360_stream',
            default_value='false',
            description='启动前先运行 Mid-360 恢复脚本'),
        OpaqueFunction(
            function=_create_actions,
            kwargs={
                'bringup_dir': bringup_dir,
                'decision_dir': decision_dir,
                'sensor_extrinsics_dir': sensor_extrinsics_dir,
                'mcu_transport_dir': mcu_transport_dir,
            },
        ),
    ])
