"""
KFS 阶梯等待独立测试入口。

启动:
  - 可选 odometry 链，提供 /odom yaw
  - 可选 RealSense D455，提供 KFS 彩色/深度图像
  - 可选 rc26_mcu_transport，消费 /cmd_vel 并提供 /mechanism/send_command
  - 可选 KFS OpenCV overlay UI，只观察相机图像与推理结果
  - rc26_decision decision_node，按 direction 加载 KFS 上/下台阶测试树

本入口不启动 Nav2、定位或遥控，确保 /cmd_vel 只有一个运动命令权威。
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _parse_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_bool(value):
    if isinstance(value, str):
        return str(_parse_bool(value)).lower()
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


def _tree_for_direction(decision_dir, direction):
    if direction == 'climb':
        return os.path.join(decision_dir, 'behavior_trees', 'kfs_stair_climb_test_tree.xml')
    if direction == 'descend':
        return os.path.join(decision_dir, 'behavior_trees', 'kfs_stair_descend_test_tree.xml')
    raise RuntimeError(f"direction must be climb or descend; got: {direction}")


def _resolve_vision_config_file(vision_dir, configured):
    configured = str(configured or '').strip()
    if configured and os.path.exists(configured):
        return os.path.normpath(configured)
    candidate = (
        os.path.join(vision_dir, 'config', 'vision_models.yaml')
        if not configured else os.path.join(vision_dir, configured)
    )
    if os.path.exists(candidate):
        return os.path.normpath(candidate)
    return configured


def _create_actions(context, *, bringup_dir, decision_dir, sensor_extrinsics_dir, mcu_transport_dir, vision_dir):
    runtime_config_file = _launch_value(context, 'runtime_config_file')
    runtime_defaults = _load_runtime_defaults(runtime_config_file)
    decision_params = dict(runtime_defaults['decision_params'])
    decision_params.pop('tree_file', None)
    decision_params.pop('team', None)

    namespace = _launch_value(context, 'namespace')
    use_sim_time = _parse_bool(_launch_value(context, 'use_sim_time'))
    use_sim_time_text = _launch_bool(use_sim_time)
    team = _select_str(context, 'team', str(runtime_defaults['decision_params'].get('team', 'blue')))
    direction = _launch_value(context, 'direction').lower()
    tree_file = _tree_for_direction(decision_dir, direction)

    sensor_extrinsics_file = _select_str(
        context,
        'sensor_extrinsics_file',
        os.path.join(sensor_extrinsics_dir, 'config', 'r2_sensor_extrinsics.yaml'))
    sensor_extrinsics_profile = _launch_value(context, 'sensor_extrinsics_profile')
    point_lio_config_file = _launch_value(context, 'point_lio_config_file')
    start_mid360_driver = _launch_value(context, 'start_mid360_driver') or 'true'
    recover_mid360_stream = _launch_value(context, 'recover_mid360_stream') or 'false'
    start_odometry = _parse_bool(_launch_value(context, 'start_odometry') or 'true')
    use_realsense = _parse_bool(_launch_value(context, 'use_realsense') or 'true')
    realsense_serial_no = _launch_value(context, 'realsense_serial_no') or "''"
    camera_namespace = _launch_value(context, 'camera_namespace')
    realsense_config_file = _select_str(
        context,
        'realsense_config_file',
        os.path.join(bringup_dir, 'config', 'realsense_d455.yaml'))
    use_kfs_vision_ui = _parse_bool(_launch_value(context, 'use_kfs_vision_ui') or 'false')
    kfs_vision_ui_window_name = _select_str(
        context,
        'kfs_vision_ui_window_name',
        'KFS Stair Test Vision')
    kfs_vision_ui_display_rate_ms = _select_str(
        context,
        'kfs_vision_ui_display_rate_ms',
        '33')

    mcu_defaults = runtime_defaults['mcu_transport']
    start_mcu_transport = _select_bool(context, 'start_mcu_transport', mcu_defaults['enabled'])

    actions = []
    if start_odometry:
        actions.append(GroupAction(actions=[IncludeLaunchDescription(
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
                'start_mid360_driver': start_mid360_driver,
                'recover_mid360_stream': recover_mid360_stream,
            }.items()
        )], scoped=True))

    if use_realsense:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_dir, 'launch', 'realsense_d455.launch.py'])
            ),
            launch_arguments={
                'serial_no': realsense_serial_no,
                'config_file': realsense_config_file,
                'camera_namespace': camera_namespace,
            }.items(),
        ))

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

    if use_kfs_vision_ui:
        kfs_vision_config_file = _resolve_vision_config_file(
            vision_dir,
            decision_params.get('kfs_vision_config_file'))
        kfs_model_id = str(decision_params.get('kfs_model_id') or 'kfs_default')
        kfs_depth_min_m = float(decision_params.get('kfs_depth_min_m', 0.6))
        kfs_depth_max_m = float(decision_params.get('kfs_depth_max_m', 1.2))
        actions.append(Node(
            package='rc26_vision',
            executable='kfs_vision_test_node',
            name='kfs_vision_ui',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'vision_config_file': kfs_vision_config_file,
                'model_id': kfs_model_id,
                'show_window': True,
                'window_name': kfs_vision_ui_window_name,
                'display_rate_ms': ParameterValue(kfs_vision_ui_display_rate_ms, value_type=int),
                'vision_depth_min_m': kfs_depth_min_m,
                'vision_depth_max_m': kfs_depth_max_m,
            }],
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
            },
        ],
    ))
    return actions


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc26_bringup')
    decision_dir = get_package_share_directory('rc26_decision')
    sensor_extrinsics_dir = get_package_share_directory('rc26_sensor_extrinsics')
    mcu_transport_dir = get_package_share_directory('rc26_mcu_transport')
    vision_dir = get_package_share_directory('rc26_vision')

    return LaunchDescription([
        DeclareLaunchArgument(
            'runtime_config_file',
            default_value=PathJoinSubstitution([bringup_dir, 'config', 'r2_runtime.yaml']),
            description='R2 统一运行配置 YAML'),
        DeclareLaunchArgument(
            'direction',
            default_value='climb',
            description='KFS 阶梯测试方向: climb | descend'),
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
            description='Active competition side: blue | red；空字符串表示使用 r2_runtime.yaml'),
        DeclareLaunchArgument(
            'start_odometry',
            default_value='true',
            description='是否启动 odometry 链提供 /odom'),
        DeclareLaunchArgument(
            'use_realsense',
            default_value='true',
            description='是否启动 RealSense D455'),
        DeclareLaunchArgument(
            'realsense_serial_no',
            default_value="''",
            description='RealSense serial number；空表示自动选择'),
        DeclareLaunchArgument(
            'camera_namespace',
            default_value='',
            description='RealSense camera namespace；默认空以发布 /camera/color/image_raw 等 KFS 默认话题'),
        DeclareLaunchArgument(
            'realsense_config_file',
            default_value='',
            description='RealSense YAML config file；为空时使用 bringup 默认配置'),
        DeclareLaunchArgument(
            'use_kfs_vision_ui',
            default_value='false',
            description='是否启动 KFS OpenCV overlay 摄像头 UI；默认关闭'),
        DeclareLaunchArgument(
            'kfs_vision_ui_window_name',
            default_value='KFS Stair Test Vision',
            description='KFS 摄像头 UI 的 OpenCV 窗口标题'),
        DeclareLaunchArgument(
            'kfs_vision_ui_display_rate_ms',
            default_value='33',
            description='KFS 摄像头 UI 刷新周期，单位毫秒'),
        DeclareLaunchArgument(
            'start_mcu_transport',
            default_value='',
            description='是否启动 rc26_mcu_transport；空字符串表示使用 r2_runtime.yaml'),
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
                'vision_dir': vision_dir,
            },
        ),
    ])
