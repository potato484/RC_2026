"""
src R2 odom 闭环导航系统 - 主启动文件

启动:
  - 里程计 (rc26_point_lio + rc26_odom_interface)
  - 决策系统 (rc26_decision)
  - 目标 MCU 共享串口 transport (rc26_mcu_transport)

额外模式:
  - run_mode:=mapping 且 pure_mapping_mode:=true 时，保留纯建图最小运动链路

默认装配口径:
  - 车端 bringup 默认维持 headless，可通过 use_rviz:=true 临时打开 RViz2
  - r2_active_side.yaml 选择 r2_red.yaml / r2_blue.yaml 作为运行配置
  - /cmd_vel 的默认底盘执行与机构指令共享串口由 rc26_mcu_transport 提供
  - RViz2 只作为现场观察工具，不接管运行时权威
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


R2_D455_SERIAL_NO = '239222303644'
FIRST_PRESELECTION_TREE = '/home/aidlux/RC_2026/src/rc26_decision/behavior_trees/mc_repeat_preselection_tree.xml'
SECOND_PRESELECTION_TREE = '/home/aidlux/RC_2026/src/rc26_decision/behavior_trees/second_preselection_combo_tree.xml'
SECOND_PRESELECTION_KFS_COMPAT_TREE = '/home/aidlux/RC_2026/src/rc26_decision/behavior_trees/second_preselection_kfs_compat_combo_tree.xml'


def _launch_bool(value):
    return str(bool(value)).lower()


def _parse_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_value(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def _read_required_abs_path(mapping, key):
    if key not in mapping:
        raise RuntimeError(f"r2_runtime.paths.{key} is required in runtime config")
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
    if value != '':
        return _parse_bool(value)
    return _parse_bool(default_value) if isinstance(default_value, str) else bool(default_value)


def _select_nonnegative_float(context, name, default_value):
    value = _launch_value(context, name)
    text = str(default_value) if value == '' else value
    try:
        parsed = float(text)
    except ValueError as exc:
        raise RuntimeError(f"{name} must be a number, got: {text}") from exc
    if parsed < 0.0:
        raise RuntimeError(f"{name} must be >= 0.0, got: {parsed}")
    return parsed


def _select_abs_path(context, name, default_value):
    value = _select_str(context, name, default_value).strip()
    if not os.path.isabs(value):
        raise RuntimeError(f"{name} must be an absolute path: {value}")
    return value


def _resolve_startup_wait_for_odom(runtime_defaults, navigation_mode):
    configured = runtime_defaults.get('decision_params', {}).get(
        'startup_wait_for_odom', navigation_mode)
    return _parse_bool(configured) if isinstance(configured, str) else bool(configured)


def _should_start_startup_ready_notify(navigation_mode, use_decision, enabled):
    return bool(navigation_mode and use_decision and enabled)


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


def _load_yaml_file(path, label):
    if not os.path.isabs(path):
        raise RuntimeError(f"{label} must be an absolute path: {path}")
    if not os.path.exists(path):
        raise RuntimeError(f"{label} does not exist: {path}")
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def _resolve_runtime_config_file(context, bringup_dir):
    explicit_config = _launch_value(context, 'runtime_config_file')
    if explicit_config:
        if not os.path.isabs(explicit_config):
            raise RuntimeError(
                f"runtime_config_file must be an absolute path when explicitly set: {explicit_config}")
        if not os.path.exists(explicit_config):
            raise RuntimeError(f"runtime_config_file does not exist: {explicit_config}")
        return explicit_config

    selector_file = _launch_value(context, 'side_config_file')
    if selector_file == '':
        selector_file = os.path.join(bringup_dir, 'config', 'r2_active_side.yaml')
    selector = _load_yaml_file(selector_file, 'side_config_file')
    active_side = str(selector.get('active_side', '')).strip().lower()
    runtime_configs = selector.get('runtime_configs') or {}
    if active_side not in ('red', 'blue'):
        raise RuntimeError(
            f"r2_active_side.yaml active_side must be red or blue, got: {active_side}")
    if active_side not in runtime_configs:
        raise RuntimeError(
            f"r2_active_side.yaml missing runtime_configs.{active_side}")

    selected = str(runtime_configs.get(active_side, '')).strip()
    if selected == '':
        raise RuntimeError(f"r2_active_side.yaml runtime_configs.{active_side} must not be empty")
    if not os.path.isabs(selected):
        selected = os.path.abspath(os.path.join(os.path.dirname(selector_file), selected))
    if not os.path.exists(selected):
        raise RuntimeError(f"selected runtime config does not exist: {selected}")
    return selected


def _resolve_preselection_mode(context, bringup_dir):
    if _launch_value(context, 'runtime_config_file'):
        return None

    selector_file = _launch_value(context, 'side_config_file')
    if selector_file == '':
        selector_file = os.path.join(bringup_dir, 'config', 'r2_active_side.yaml')
    selector = _load_yaml_file(selector_file, 'side_config_file')
    mode = str(selector.get('preselection_mode', 'first')).strip().lower()
    if mode not in ('first', 'second'):
        raise RuntimeError(
            f"r2_active_side.yaml preselection_mode must be first or second, got: {mode}")
    return mode


def _resolve_second_preselection_kfs_search_compat_enable(context, bringup_dir):
    if _launch_value(context, 'runtime_config_file'):
        return False

    selector_file = _launch_value(context, 'side_config_file')
    if selector_file == '':
        selector_file = os.path.join(bringup_dir, 'config', 'r2_active_side.yaml')
    selector = _load_yaml_file(selector_file, 'side_config_file')
    configured = selector.get(
        'second_preselection_kfs_search_compat_enable', False)
    return _parse_bool(configured) if isinstance(configured, str) else bool(configured)


def _managed_preselection_tree(default_tree, preselection_mode,
                               second_kfs_compat_enable):
    if preselection_mode == 'first':
        return FIRST_PRESELECTION_TREE
    if preselection_mode == 'second':
        return (
            SECOND_PRESELECTION_KFS_COMPAT_TREE
            if second_kfs_compat_enable
            else SECOND_PRESELECTION_TREE
        )
    return default_tree


def _resolve_first_preselection_repeat_overrides(context, bringup_dir):
    if _launch_value(context, 'runtime_config_file'):
        return {}

    selector_file = _launch_value(context, 'side_config_file')
    if selector_file == '':
        selector_file = os.path.join(bringup_dir, 'config', 'r2_active_side.yaml')
    selector = _load_yaml_file(selector_file, 'side_config_file')
    active_side = str(selector.get('active_side', '')).strip().lower()
    if active_side not in ('red', 'blue'):
        raise RuntimeError(
            f"r2_active_side.yaml active_side must be red or blue, got: {active_side}")
    overrides = {}
    mapping = {
        'first_preselection_mc_repeat_enable': bool,
        'first_preselection_mc_repeat_max_count': int,
    }
    for key, caster in mapping.items():
        if key not in selector:
            continue
        try:
            value = selector[key]
            if caster is bool:
                overrides[key] = _parse_bool(value) if isinstance(value, str) else bool(value)
            else:
                overrides[key] = caster(value)
        except (TypeError, ValueError) as exc:
            raise RuntimeError(f"r2_active_side.yaml {key} has invalid value: {selector[key]}") from exc
    step_key = 'first_preselection_mc_repeat_forward_x_step_m'
    if step_key not in selector:
        raise RuntimeError(f"r2_active_side.yaml {step_key} is required")
    step_by_side = selector.get(step_key) or {}
    if not isinstance(step_by_side, dict):
        raise RuntimeError(
            f"r2_active_side.yaml {step_key} must map red/blue to signed step values")
    for side in ('red', 'blue'):
        if side not in step_by_side:
            raise RuntimeError(
                f"r2_active_side.yaml {step_key}.{side} is required")
    if active_side not in step_by_side:
        raise RuntimeError(
            f"r2_active_side.yaml {step_key}.{active_side} is required")
    try:
        overrides[step_key] = float(step_by_side[active_side])
    except (TypeError, ValueError) as exc:
        raise RuntimeError(
            f"r2_active_side.yaml {step_key}.{active_side} has invalid value: "
            f"{step_by_side[active_side]}") from exc
    return overrides


def _load_r2_runtime_defaults(config_file):
    defaults = {
        'paths': {},
        'decision_params': {},
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

    yaml_data = _load_yaml_file(config_file, 'runtime_config_file')

    runtime = yaml_data.get('r2_runtime') or {}
    paths = runtime.get('paths') or {}
    decision = runtime.get('decision') or {}
    mcu_transport = runtime.get('mcu_transport') or {}
    mcu_transport_params = mcu_transport.get('ros__parameters') or {}

    defaults['paths']['prior_pcd_file'] = _read_required_abs_path(paths, 'prior_pcd_file')
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
    for key in (
        'enable_chassis_cmd_vel_consumer',
        'chassis_cmd_vel_topic',
        'chassis_target_send_rate_hz',
        'chassis_cmd_vel_timeout_ms',
        'chassis_v_max_mps',
        'chassis_w_max_radps',
        'chassis_stop_repeat_n',
    ):
        if key in mcu_transport_params:
            defaults['mcu_transport'][key] = mcu_transport_params[key]
    return defaults


def _after_delay(delay_sec, actions):
    if delay_sec <= 0.0:
        return list(actions)
    return [TimerAction(period=delay_sec, actions=list(actions))]


def _create_runtime_actions(context, *, bringup_dir, sensor_extrinsics_dir, mcu_transport_dir):
    runtime_config_file = _resolve_runtime_config_file(context, bringup_dir)
    preselection_mode = _resolve_preselection_mode(context, bringup_dir)
    second_kfs_compat_enable = (
        _resolve_second_preselection_kfs_search_compat_enable(context, bringup_dir)
    )
    first_repeat_overrides = _resolve_first_preselection_repeat_overrides(context, bringup_dir)
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
    startup_delay_localization_sec = _select_nonnegative_float(
        context, 'startup_delay_localization_sec', 4.0)
    startup_delay_decision_sec = _select_nonnegative_float(
        context, 'startup_delay_decision_sec', 8.0)
    startup_delay_realsense_sec = _select_nonnegative_float(
        context, 'startup_delay_realsense_sec', 12.0)

    use_decision = _parse_bool(_launch_value(context, 'use_decision') or 'true')
    use_realsense = _parse_bool(_launch_value(context, 'use_realsense'))
    startup_ready_notify_enable = _select_bool(
        context, 'startup_ready_notify_enable', navigation_mode and use_decision)
    startup_ready_notify_command_id = _select_str(
        context, 'startup_ready_notify_command_id', '0x20')
    startup_ready_notify_timeout_s = _select_nonnegative_float(
        context, 'startup_ready_notify_timeout_s', 60.0)
    startup_ready_notify_gate_state_topic = _select_str(
        context, 'startup_ready_notify_gate_state_topic',
        '/decision/preselection_gate_state')
    realsense_serial_no = _launch_value(context, 'realsense_serial_no') or R2_D455_SERIAL_NO
    realsense_config_file = _select_str(
        context,
        'realsense_config_file',
        os.path.join(bringup_dir, 'config', 'realsense_d455.yaml'))
    team = _select_str(
        context,
        'team',
        str(runtime_defaults['decision_params'].get('team', 'blue')))
    point_lio_full_map_publish_en = _select_str(
        context,
        'point_lio_full_map_publish_en',
        'false' if navigation_mode else 'true')
    behavior_tree_file = _managed_preselection_tree(
        runtime_defaults['paths']['behavior_tree_file'],
        preselection_mode,
        second_kfs_compat_enable,
    )
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
    mcu_transport_enable_chassis_cmd_vel_consumer = _select_bool(
        context,
        'mcu_transport_enable_chassis_cmd_vel_consumer',
        mcu_transport_defaults['enable_chassis_cmd_vel_consumer'])
    mcu_transport_chassis_cmd_vel_topic = _select_str(
        context,
        'mcu_transport_chassis_cmd_vel_topic',
        str(mcu_transport_defaults['chassis_cmd_vel_topic']))
    mcu_transport_chassis_target_send_rate_hz = _select_str(
        context,
        'mcu_transport_chassis_target_send_rate_hz',
        str(mcu_transport_defaults['chassis_target_send_rate_hz']))
    mcu_transport_chassis_cmd_vel_timeout_ms = _select_str(
        context,
        'mcu_transport_chassis_cmd_vel_timeout_ms',
        str(mcu_transport_defaults['chassis_cmd_vel_timeout_ms']))
    mcu_transport_chassis_v_max_mps = _select_str(
        context,
        'mcu_transport_chassis_v_max_mps',
        str(mcu_transport_defaults['chassis_v_max_mps']))
    mcu_transport_chassis_w_max_radps = _select_str(
        context,
        'mcu_transport_chassis_w_max_radps',
        str(mcu_transport_defaults['chassis_w_max_radps']))
    mcu_transport_chassis_stop_repeat_n = _select_str(
        context,
        'mcu_transport_chassis_stop_repeat_n',
        str(mcu_transport_defaults['chassis_stop_repeat_n']))

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
                'enable_chassis_cmd_vel_consumer': _launch_bool(
                    mcu_transport_enable_chassis_cmd_vel_consumer),
                'chassis_cmd_vel_topic': mcu_transport_chassis_cmd_vel_topic,
                'chassis_target_send_rate_hz': mcu_transport_chassis_target_send_rate_hz,
                'chassis_cmd_vel_timeout_ms': mcu_transport_chassis_cmd_vel_timeout_ms,
                'chassis_v_max_mps': mcu_transport_chassis_v_max_mps,
                'chassis_w_max_radps': mcu_transport_chassis_w_max_radps,
                'chassis_stop_repeat_n': mcu_transport_chassis_stop_repeat_n,
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
            'point_lio_full_map_publish_en': point_lio_full_map_publish_en,
            'odom_interface_publish_bootstrap_pose': 'false' if navigation_mode else 'true',
            'start_sensor_scan': 'false' if navigation_mode else 'true',
            'start_mid360_driver': start_mid360_driver,
            'recover_mid360_stream': recover_mid360_stream,
        }.items()
    )
    actions.append(GroupAction(actions=[odometry_launch], scoped=True))

    if mapping_mode:
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
        actions.extend(_after_delay(startup_delay_localization_sec, [localization_launch]))

    if use_decision and not (mapping_mode and pure_mapping_mode):
        decision_params = dict(runtime_defaults['decision_params'])
        if preselection_mode == 'first':
            decision_params.update(first_repeat_overrides)
        decision_params.pop('team', None)
        decision_params.pop('tree_file', None)
        decision_params.pop('startup_wait_for_odom', None)
        decision_params.pop('startup_odom_topic', None)
        decision_params.pop('startup_odom_timeout_s', None)
        decision_params.pop('startup_odom_wait_timeout_s', None)
        decision_params.pop('startup_odom_min_wait_s', None)
        decision_params.pop('startup_odom_stable_samples', None)
        decision_params.pop('startup_odom_max_linear_speed_mps', None)
        decision_params.pop('startup_odom_max_angular_speed_radps', None)
        startup_wait_for_odom = _resolve_startup_wait_for_odom(
            runtime_defaults, navigation_mode)
        startup_odom_topic = str(runtime_defaults['decision_params'].get(
            'startup_odom_topic', 'odom'))
        startup_odom_timeout_s = runtime_defaults['decision_params'].get(
            'startup_odom_timeout_s', 0.5)
        startup_odom_wait_timeout_s = runtime_defaults['decision_params'].get(
            'startup_odom_wait_timeout_s', 20.0)
        startup_odom_min_wait_s = runtime_defaults['decision_params'].get(
            'startup_odom_min_wait_s', 1.0)
        startup_odom_stable_samples = runtime_defaults['decision_params'].get(
            'startup_odom_stable_samples', 10)
        startup_odom_max_linear_speed_mps = runtime_defaults['decision_params'].get(
            'startup_odom_max_linear_speed_mps', 0.03)
        startup_odom_max_angular_speed_radps = runtime_defaults['decision_params'].get(
            'startup_odom_max_angular_speed_radps', 0.05)
        decision_node = Node(
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
                    'startup_wait_for_odom': startup_wait_for_odom,
                    'startup_odom_topic': startup_odom_topic,
                    'startup_odom_timeout_s': startup_odom_timeout_s,
                    'startup_odom_wait_timeout_s': startup_odom_wait_timeout_s,
                    'startup_odom_min_wait_s': startup_odom_min_wait_s,
                    'startup_odom_stable_samples': startup_odom_stable_samples,
                    'startup_odom_max_linear_speed_mps': startup_odom_max_linear_speed_mps,
                    'startup_odom_max_angular_speed_radps': startup_odom_max_angular_speed_radps,
                },
            ],
        )
        actions.extend(_after_delay(startup_delay_decision_sec, [decision_node]))

    if use_realsense:
        realsense_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_dir, 'launch', 'realsense_d455.launch.py'])
            ),
            launch_arguments={
                'serial_no': realsense_serial_no,
                'config_file': realsense_config_file,
                # 保持与 rc26_vision/test_kfs_vision.launch.py 一致：RealSense 发布
                # /camera/color/image_raw 等单层 topic，匹配 VisionInferenceManager 默认订阅。
                'camera_namespace': '',
            }.items()
        )
        realsense_group = GroupAction(
            actions=[
                PushRosNamespace(namespace),
                realsense_launch,
            ],
        )
        actions.extend(_after_delay(startup_delay_realsense_sec, [realsense_group]))

    if _should_start_startup_ready_notify(
            navigation_mode, use_decision, startup_ready_notify_enable):
        actions.append(Node(
            package='rc26_bringup',
            executable='startup_ready_notify_node.py',
            name='startup_ready_notify_node',
            namespace=namespace,
            output='screen',
            parameters=[{
                'command_id': startup_ready_notify_command_id,
                'timeout_s': startup_ready_notify_timeout_s,
                'send_command_service': '/mechanism/send_command',
                'feedback_topic': '/mechanism/command_feedback',
                'gate_state_topic': startup_ready_notify_gate_state_topic,
                'limit_feedback_ids': [0x06, 0x10],
            }],
        ))

    return actions


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc26_bringup')
    sensor_extrinsics_dir = get_package_share_directory('rc26_sensor_extrinsics')
    mcu_transport_dir = get_package_share_directory('rc26_mcu_transport')

    return LaunchDescription([
        DeclareLaunchArgument(
            'runtime_config_file',
            default_value='',
            description='显式覆盖 R2 运行配置 YAML；为空时由 r2_active_side.yaml 选择红/蓝配置'),
        DeclareLaunchArgument(
            'side_config_file',
            default_value=PathJoinSubstitution([bringup_dir, 'config', 'r2_active_side.yaml']),
            description='R2 红蓝方配置选择 YAML；runtime_config_file 为空时生效'),
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
            description='运行模式: navigation | mapping；navigation=odom 闭环导航/决策链路, mapping=建图链路'),
        DeclareLaunchArgument(
            'pure_mapping_mode',
            default_value='false',
            description='纯建图最小模式；仅在 run_mode:=mapping 时生效，跳过 decision 等非必要模块'),
        DeclareLaunchArgument(
            'prior_pcd_file',
            default_value='',
            description='先验点云文件绝对路径；空字符串表示使用当前红/蓝运行配置'),
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
            description='建图模式定位参数文件路径；为空时使用 rc26_bringup/config/localization.yaml'),
        DeclareLaunchArgument(
            'startup_delay_localization_sec',
            default_value='4.0',
            description='错峰启动延时：建图模式定位链路启动前等待秒数；0 表示不延时'),
        DeclareLaunchArgument(
            'startup_delay_decision_sec',
            default_value='8.0',
            description='错峰启动延时：decision_node 启动前等待秒数；0 表示不延时'),
        DeclareLaunchArgument(
            'startup_delay_realsense_sec',
            default_value='12.0',
            description='错峰启动延时：RealSense 启动前等待秒数；0 表示不延时'),
        DeclareLaunchArgument(
            'point_lio_full_map_publish_en',
            default_value='',
            description='Point-LIO 完整累计地图可视化发布开关；空字符串表示 navigation=false、mapping=true'),
        DeclareLaunchArgument(
            'start_mcu_transport',
            default_value='',
            description='是否启动 rc26_mcu_transport；空字符串表示使用当前红/蓝运行配置'),
        DeclareLaunchArgument(
            'mcu_transport_target_serial_port',
            default_value='',
            description='目标 MCU 串口设备；为空时使用当前红/蓝运行配置'),
        DeclareLaunchArgument(
            'mcu_transport_target_baudrate',
            default_value='',
            description='目标 MCU 串口波特率；为空时使用当前红/蓝运行配置'),
        DeclareLaunchArgument(
            'mcu_transport_open_retry_period_ms',
            default_value='',
            description='目标 MCU 串口初始打开重试周期；为空时使用当前红/蓝运行配置'),
        DeclareLaunchArgument(
            'mcu_transport_diagnostics_period_ms',
            default_value='',
            description='rc26_mcu_transport diagnostics 发布周期；为空时使用当前红/蓝运行配置'),
        DeclareLaunchArgument(
            'mcu_transport_enable_chassis_cmd_vel_consumer',
            default_value='',
            description='是否让 rc26_mcu_transport 订阅 /cmd_vel；为空时使用当前红/蓝运行配置'),
        DeclareLaunchArgument(
            'mcu_transport_chassis_cmd_vel_topic',
            default_value='',
            description='rc26_mcu_transport 消费的底盘速度话题；为空时使用当前红/蓝运行配置'),
        DeclareLaunchArgument(
            'mcu_transport_chassis_target_send_rate_hz',
            default_value='',
            description='rc26_mcu_transport 底盘目标速度发送频率；为空时使用当前红/蓝运行配置'),
        DeclareLaunchArgument(
            'mcu_transport_chassis_cmd_vel_timeout_ms',
            default_value='',
            description='rc26_mcu_transport 底盘速度超时；为空时使用当前红/蓝运行配置'),
        DeclareLaunchArgument(
            'mcu_transport_chassis_v_max_mps',
            default_value='',
            description='rc26_mcu_transport 底盘线速度上限；为空时使用当前红/蓝运行配置'),
        DeclareLaunchArgument(
            'mcu_transport_chassis_w_max_radps',
            default_value='',
            description='rc26_mcu_transport 底盘角速度上限；为空时使用当前红/蓝运行配置'),
        DeclareLaunchArgument(
            'mcu_transport_chassis_stop_repeat_n',
            default_value='',
            description='rc26_mcu_transport 底盘超时零速重复帧数；为空时使用当前红/蓝运行配置'),
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
            default_value=R2_D455_SERIAL_NO,
            description=f'RealSense serial number (default fixed R2 D455: {R2_D455_SERIAL_NO})'),
        DeclareLaunchArgument(
            'realsense_config_file',
            default_value='',
            description='RealSense YAML config file (realsense2_camera params)；为空时使用 bringup 默认配置'),
        DeclareLaunchArgument(
            'startup_ready_notify_enable',
            default_value='',
            description='启动就绪 0x20 no-ack 通知；空字符串表示 navigation+decision 时启用'),
        DeclareLaunchArgument(
            'startup_ready_notify_command_id',
            default_value='0x20',
            description='启动就绪通知下行命令 ID；默认 0x20'),
        DeclareLaunchArgument(
            'startup_ready_notify_timeout_s',
            default_value='60.0',
            description='启动就绪通知等待人工限位 gate 与机构 service 的超时时间；超时只告警'),
        DeclareLaunchArgument(
            'startup_ready_notify_gate_state_topic',
            default_value='/decision/preselection_gate_state',
            description='启动就绪通知等待的决策预选入口 gate 状态 topic'),
        DeclareLaunchArgument(
            'team',
            default_value='',
            description='Active competition side: blue | red；空字符串表示使用当前红/蓝运行配置'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='是否随正式 bringup 启动 RViz2；默认关闭'),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([bringup_dir, 'rviz', 'navigation_default.rviz']),
            description='RViz2 配置文件路径'),
        OpaqueFunction(
            function=_create_runtime_actions,
            kwargs={
                'bringup_dir': bringup_dir,
                'sensor_extrinsics_dir': sensor_extrinsics_dir,
                'mcu_transport_dir': mcu_transport_dir,
            },
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config_file')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
