"""
src R2导航系统 - 主启动文件

启动:
  - 里程计 (rc26_point_lio + rc26_odom_interface + rc26_sensor_scan)
  - 定位 (rc26_localization)
  - Nav2 基础导航栈 (map_server + planner/controller/BT navigator/velocity smoother)
  - 决策系统 (rc26_decision)

额外模式:
  - run_mode:=mapping 且 pure_mapping_mode:=true 时，保留纯建图最小运动链路

默认装配口径:
  - 车端 bringup 维持 headless
  - r2_runtime.yaml 是点云、地图、行为树入口与决策参数的运行配置真源
  - /cmd_vel 的硬件执行由外部运行时提供；机构指令共享串口由 rc26_mcu_transport 提供
  - 如需图形观察，请手工启动工作区外部可视化工具只读消费 ROS2 输出
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


def _launch_bool(value):
    return str(bool(value)).lower()


def _parse_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_value(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def _read_required_abs_path(mapping, key):
    if key not in mapping:
        raise RuntimeError(f"r2_runtime.paths.{key} is required in r2_runtime.yaml")
    value = str(mapping.get(key, '')).strip()
    if value == '':
        raise RuntimeError(f"r2_runtime.paths.{key} must not be empty")
    if not os.path.isabs(value):
        raise RuntimeError(f"r2_runtime.paths.{key} must be an absolute path: {value}")
    return value


def _select_str(context, name, default_value):
    value = _launch_value(context, name)
    return default_value if value == '' else value


def _select_bool(context, name, default_value):
    value = _launch_value(context, name)
    return bool(default_value) if value == '' else _parse_bool(value)


def _select_abs_path(context, name, default_value):
    value = _select_str(context, name, default_value).strip()
    if not os.path.isabs(value):
        raise RuntimeError(f"{name} must be an absolute path: {value}")
    return value


def _read_run_mode(context):
    if 'slam' in context.launch_configurations:
        raise RuntimeError(
            "slam launch argument was removed; use run_mode:=navigation or run_mode:=mapping"
        )
    run_mode = _launch_value(context, 'run_mode').lower()
    if run_mode not in ('navigation', 'mapping'):
        raise RuntimeError(
            f"run_mode must be one of: navigation | mapping; got: {run_mode}"
        )
    return run_mode


def _load_r2_runtime_defaults(config_file):
    if not os.path.isabs(config_file):
        raise RuntimeError(f"runtime_config_file must be an absolute path: {config_file}")
    if not os.path.exists(config_file):
        raise RuntimeError(f"runtime_config_file does not exist: {config_file}")

    defaults = {
        'paths': {},
        'decision_params': {},
        'mcu_transport': {
            'enabled': True,
            'target_serial_port': '/dev/ttyUSB0',
            'target_baudrate': 1000000,
            'open_retry_period_ms': 1000,
            'diagnostics_period_ms': 1000,
        },
    }

    with open(config_file, 'r', encoding='utf-8') as f:
        yaml_data = yaml.safe_load(f) or {}

    runtime = yaml_data.get('r2_runtime') or {}
    paths = runtime.get('paths') or {}
    decision = runtime.get('decision') or {}
    mcu_transport = runtime.get('mcu_transport') or {}
    mcu_transport_params = mcu_transport.get('ros__parameters') or {}

    defaults['paths']['prior_pcd_file'] = _read_required_abs_path(paths, 'prior_pcd_file')
    defaults['paths']['nav2_map_file'] = _read_required_abs_path(paths, 'nav2_map_file')
    defaults['paths']['behavior_tree_file'] = _read_required_abs_path(paths, 'behavior_tree_file')

    defaults['decision_params'] = dict((decision.get('ros__parameters') or {}))
    defaults['decision_params'].pop('tree_file', None)
    enabled_value = mcu_transport.get('enabled', True)
    defaults['mcu_transport']['enabled'] = (
        _parse_bool(enabled_value) if isinstance(enabled_value, str) else bool(enabled_value)
    )
    for key in ('target_serial_port', 'target_baudrate', 'open_retry_period_ms', 'diagnostics_period_ms'):
        if key in mcu_transport_params:
            defaults['mcu_transport'][key] = mcu_transport_params[key]
    return defaults


def _create_runtime_actions(context, *, bringup_dir, sensor_extrinsics_dir, nav2_bringup_dir, mcu_transport_dir):
    runtime_config_file = _launch_value(context, 'runtime_config_file')
    runtime_defaults = _load_r2_runtime_defaults(runtime_config_file)

    namespace = _launch_value(context, 'namespace')
    use_sim_time = _parse_bool(_launch_value(context, 'use_sim_time'))
    use_sim_time_text = _launch_bool(use_sim_time)
    run_mode = _read_run_mode(context)
    navigation_mode = run_mode == 'navigation'
    mapping_mode = run_mode == 'mapping'
    pure_mapping_mode = _parse_bool(_launch_value(context, 'pure_mapping_mode'))
    prior_pcd_file = _select_abs_path(context, 'prior_pcd_file', runtime_defaults['paths']['prior_pcd_file'])
    point_lio_config_file = _launch_value(context, 'point_lio_config_file')
    sensor_extrinsics_file = _select_str(
        context,
        'sensor_extrinsics_file',
        os.path.join(sensor_extrinsics_dir, 'config', 'r2_sensor_extrinsics.yaml'))
    sensor_extrinsics_profile = _launch_value(context, 'sensor_extrinsics_profile')
    start_mid360_driver = _launch_value(context, 'start_mid360_driver') or 'true'
    recover_mid360_stream = _launch_value(context, 'recover_mid360_stream') or 'false'
    localization_params_file = _select_str(
        context,
        'localization_params_file',
        os.path.join(bringup_dir, 'config', 'localization.yaml'))

    use_decision = _parse_bool(_launch_value(context, 'use_decision') or 'true')
    use_realsense = _parse_bool(_launch_value(context, 'use_realsense'))
    realsense_serial_no = _launch_value(context, 'realsense_serial_no') or "''"
    realsense_config_file = _select_str(
        context,
        'realsense_config_file',
        os.path.join(bringup_dir, 'config', 'realsense_d455.yaml'))
    team = _select_str(
        context,
        'team',
        str(runtime_defaults['decision_params'].get('team', 'blue')))
    nav2_params_file = _select_str(
        context,
        'nav2_params_file',
        os.path.join(bringup_dir, 'config', 'nav2_params.yaml'))
    nav2_map_file = _select_abs_path(context, 'nav2_map_file', runtime_defaults['paths']['nav2_map_file'])
    behavior_tree_file = runtime_defaults['paths']['behavior_tree_file']
    mcu_transport_defaults = runtime_defaults['mcu_transport']
    start_mcu_transport = _select_bool(context, 'start_mcu_transport', mcu_transport_defaults['enabled'])
    mcu_transport_target_serial_port = _select_str(
        context,
        'mcu_transport_target_serial_port',
        str(mcu_transport_defaults['target_serial_port']))
    mcu_transport_target_baudrate = _select_str(
        context,
        'mcu_transport_target_baudrate',
        str(mcu_transport_defaults['target_baudrate']))
    mcu_transport_open_retry_period_ms = _select_str(
        context,
        'mcu_transport_open_retry_period_ms',
        str(mcu_transport_defaults['open_retry_period_ms']))
    mcu_transport_diagnostics_period_ms = _select_str(
        context,
        'mcu_transport_diagnostics_period_ms',
        str(mcu_transport_defaults['diagnostics_period_ms']))

    actions = []

    if start_mcu_transport:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([mcu_transport_dir, 'launch', 'mcu_transport.launch.py'])
            ),
            launch_arguments={
                'target_serial_port': mcu_transport_target_serial_port,
                'target_baudrate': mcu_transport_target_baudrate,
                'open_retry_period_ms': mcu_transport_open_retry_period_ms,
                'diagnostics_period_ms': mcu_transport_diagnostics_period_ms,
            }.items(),
        ))

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
            'start_mid360_driver': start_mid360_driver,
            'recover_mid360_stream': recover_mid360_stream,
        }.items()
    )
    actions.append(GroupAction(actions=[odometry_launch], scoped=True))

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_dir, 'launch', 'localization.launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time_text,
            'run_mode': run_mode,
            'prior_pcd_file': prior_pcd_file,
            'localization_params_file': localization_params_file,
        }.items()
    )
    actions.append(localization_launch)

    if navigation_mode:
        actions.append(Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace=namespace,
            output='screen',
            parameters=[
                nav2_params_file,
                {
                    'use_sim_time': use_sim_time,
                    'yaml_filename': nav2_map_file,
                },
            ],
        ))

        actions.append(Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ))

        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time_text,
                'params_file': nav2_params_file,
                'autostart': 'true',
                'use_composition': 'False',
                'use_respawn': 'False',
            }.items(),
        ))

    if use_decision and not (mapping_mode and pure_mapping_mode):
        decision_params = dict(runtime_defaults['decision_params'])
        decision_params.pop('team', None)
        decision_params.pop('tree_file', None)
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
                    'tree_file': behavior_tree_file,
                },
            ],
        ))

    if use_realsense:
        realsense_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_dir, 'launch', 'realsense_d455.launch.py'])
            ),
            launch_arguments={
                'serial_no': realsense_serial_no,
                'config_file': realsense_config_file,
            }.items()
        )
        actions.append(GroupAction(
            actions=[
                PushRosNamespace(namespace),
                realsense_launch,
            ],
        ))

    return actions


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc26_bringup')
    sensor_extrinsics_dir = get_package_share_directory('rc26_sensor_extrinsics')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    mcu_transport_dir = get_package_share_directory('rc26_mcu_transport')

    return LaunchDescription([
        DeclareLaunchArgument(
            'runtime_config_file',
            default_value=PathJoinSubstitution([bringup_dir, 'config', 'r2_runtime.yaml']),
            description='R2 统一运行配置 YAML；点云、地图、行为树路径必须为绝对路径'),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='顶级命名空间'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用仿真时间'),
        DeclareLaunchArgument(
            'run_mode',
            default_value='navigation',
            description='运行模式: navigation | mapping；navigation=完整导航/决策链路, mapping=建图链路'),
        DeclareLaunchArgument(
            'pure_mapping_mode',
            default_value='false',
            description='纯建图最小模式；仅在 run_mode:=mapping 时生效，跳过 decision 等非必要模块'),
        DeclareLaunchArgument(
            'prior_pcd_file',
            default_value='',
            description='先验点云文件绝对路径；空字符串表示使用 r2_runtime.yaml'),
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
            description='启动前先运行 Mid-360 恢复脚本（必要时重写 host_ipcfg 并软件重启雷达）'),
        DeclareLaunchArgument(
            'localization_params_file',
            default_value='',
            description='基础定位参数文件路径；为空时使用 rc26_bringup/config/localization.yaml'),
        DeclareLaunchArgument(
            'start_mcu_transport',
            default_value='',
            description='是否启动 rc26_mcu_transport；空字符串表示使用 r2_runtime.yaml'),
        DeclareLaunchArgument(
            'mcu_transport_target_serial_port',
            default_value='',
            description='目标 MCU 串口设备；为空时使用 r2_runtime.yaml'),
        DeclareLaunchArgument(
            'mcu_transport_target_baudrate',
            default_value='',
            description='目标 MCU 串口波特率；为空时使用 r2_runtime.yaml'),
        DeclareLaunchArgument(
            'mcu_transport_open_retry_period_ms',
            default_value='',
            description='目标 MCU 串口初始打开重试周期；为空时使用 r2_runtime.yaml'),
        DeclareLaunchArgument(
            'mcu_transport_diagnostics_period_ms',
            default_value='',
            description='rc26_mcu_transport diagnostics 发布周期；为空时使用 r2_runtime.yaml'),
        DeclareLaunchArgument(
            'use_decision',
            default_value='true',
            description='启动决策系统'),
        DeclareLaunchArgument(
            'use_realsense',
            default_value='false',
            description='启动 RealSense D455 (realsense2_camera)'),
        DeclareLaunchArgument(
            'realsense_serial_no',
            default_value="''",
            description='RealSense serial number (empty to auto-select)'),
        DeclareLaunchArgument(
            'realsense_config_file',
            default_value='',
            description='RealSense YAML config file (realsense2_camera params)；为空时使用 bringup 默认配置'),
        DeclareLaunchArgument(
            'team',
            default_value='',
            description='Active competition side: blue | red；空字符串表示使用 r2_runtime.yaml'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value='',
            description='Nav2 参数文件；为空时使用 rc26_bringup/config/nav2_params.yaml'),
        DeclareLaunchArgument(
            'nav2_map_file',
            default_value='',
            description='Nav2 map_server 使用的 2D occupancy map YAML 绝对路径；空字符串表示使用 r2_runtime.yaml'),
        OpaqueFunction(
            function=_create_runtime_actions,
            kwargs={
                'bringup_dir': bringup_dir,
                'sensor_extrinsics_dir': sensor_extrinsics_dir,
                'nav2_bringup_dir': nav2_bringup_dir,
                'mcu_transport_dir': mcu_transport_dir,
            },
        ),
    ])
