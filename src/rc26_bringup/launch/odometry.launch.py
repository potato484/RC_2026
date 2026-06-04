"""
里程计模块启动文件

启动:
  - rc26_point_lio (LiDAR-IMU 里程计)
  - rc26_odom_interface (坐标变换: lidar_odom -> odom)
  - rc26_sensor_scan (发布 odom -> chassis 变换 + sensor_scan)
"""
import os
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
import yaml


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _load_yaml_file(path: str) -> dict:
    if not os.path.exists(path):
        raise RuntimeError(f'YAML 配置文件不存在: {path}')
    with open(path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise RuntimeError(f'YAML 配置文件格式非法，顶层必须是 mapping: {path}')
    return data


def _as_float_vector(values, *, name: str, length: int = 3) -> list[float]:
    if not isinstance(values, (list, tuple)) or len(values) != length:
        raise RuntimeError(f'{name} 必须是长度为 {length} 的数组')
    try:
        return [float(value) for value in values]
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f'{name} 必须全部是数字') from exc


def _as_frame_name(value, *, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise RuntimeError(f'{name} 必须是非空字符串')
    return value.strip()


def _mat_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3)]
        for row in range(3)
    ]


def _mat_vec_mul(a: list[list[float]], v: list[float]) -> list[float]:
    return [sum(a[row][col] * v[col] for col in range(3)) for row in range(3)]


def _mat_transpose(a: list[list[float]]) -> list[list[float]]:
    return [[a[col][row] for col in range(3)] for row in range(3)]


def _rot_from_rpy(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    rot_x = [
        [1.0, 0.0, 0.0],
        [0.0, cr, -sr],
        [0.0, sr, cr],
    ]
    rot_y = [
        [cp, 0.0, sp],
        [0.0, 1.0, 0.0],
        [-sp, 0.0, cp],
    ]
    rot_z = [
        [cy, -sy, 0.0],
        [sy, cy, 0.0],
        [0.0, 0.0, 1.0],
    ]
    return _mat_mul(_mat_mul(rot_z, rot_y), rot_x)


def _rpy_from_rot(rot: list[list[float]]) -> list[float]:
    if abs(rot[2][0]) < 1.0 - 1e-9:
        pitch = math.asin(-rot[2][0])
        roll = math.atan2(rot[2][1], rot[2][2])
        yaw = math.atan2(rot[1][0], rot[0][0])
        return [roll, pitch, yaw]

    pitch = math.pi / 2.0 if rot[2][0] <= -1.0 else -math.pi / 2.0
    roll = math.atan2(-rot[0][1], rot[1][1])
    return [roll, pitch, 0.0]


def _transform_compose(lhs: tuple[list[list[float]], list[float]],
                       rhs: tuple[list[list[float]], list[float]]) -> tuple[list[list[float]], list[float]]:
    lhs_rot, lhs_xyz = lhs
    rhs_rot, rhs_xyz = rhs
    rot = _mat_mul(lhs_rot, rhs_rot)
    rotated_rhs_xyz = _mat_vec_mul(lhs_rot, rhs_xyz)
    xyz = [lhs_xyz[i] + rotated_rhs_xyz[i] for i in range(3)]
    return rot, xyz


def _transform_inverse(transform: tuple[list[list[float]], list[float]]) -> tuple[list[list[float]], list[float]]:
    rot, xyz = transform
    inv_rot = _mat_transpose(rot)
    inv_xyz = _mat_vec_mul(inv_rot, [-xyz[0], -xyz[1], -xyz[2]])
    return inv_rot, inv_xyz


def _format_tf_value(value: float) -> str:
    if abs(value) < 1e-12:
        value = 0.0
    return f'{value:.12g}'


def _static_tf_node(*, name: str, parent_frame: str, child_frame: str,
                    xyz: list[float], rpy: list[float]) -> Node:
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', _format_tf_value(xyz[0]),
            '--y', _format_tf_value(xyz[1]),
            '--z', _format_tf_value(xyz[2]),
            '--roll', _format_tf_value(rpy[0]),
            '--pitch', _format_tf_value(rpy[1]),
            '--yaw', _format_tf_value(rpy[2]),
            '--frame-id', parent_frame,
            '--child-frame-id', child_frame,
        ],
    )


def _ros_parameters_from_yaml(data: dict, *, source: str) -> dict:
    if '/**' in data:
        wildcard_params = data.get('/**', {}).get('ros__parameters', {})
        if isinstance(wildcard_params, dict):
            return wildcard_params
    if 'ros__parameters' in data and isinstance(data['ros__parameters'], dict):
        return data['ros__parameters']
    raise RuntimeError(f'{source} 中未找到 ros__parameters')


def _mapping_extrinsics_from_point_lio_config(path: str) -> tuple[list[list[float]], list[float]]:
    data = _load_yaml_file(path)
    params = _ros_parameters_from_yaml(data, source=path)
    mapping = params.get('mapping')
    if not isinstance(mapping, dict):
        raise RuntimeError(f'{path} 中缺少 mapping 配置')

    extrinsic_t = _as_float_vector(mapping.get('extrinsic_T'), name='mapping.extrinsic_T')
    extrinsic_r_values = _as_float_vector(mapping.get('extrinsic_R'), name='mapping.extrinsic_R', length=9)
    extrinsic_r = [
        extrinsic_r_values[0:3],
        extrinsic_r_values[3:6],
        extrinsic_r_values[6:9],
    ]
    return extrinsic_r, extrinsic_t


def _select_sensor_extrinsics_profile(data: dict, requested_profile: str) -> tuple[str, dict]:
    root = data.get('sensor_extrinsics')
    if not isinstance(root, dict):
        raise RuntimeError('sensor_extrinsics_file 顶层缺少 sensor_extrinsics')

    profiles = root.get('profiles')
    if not isinstance(profiles, dict) or not profiles:
        raise RuntimeError('sensor_extrinsics.profiles 必须是非空 mapping')

    profile_name = requested_profile.strip()
    if not profile_name:
        defaults = root.get('defaults', {})
        if not isinstance(defaults, dict):
            raise RuntimeError('未指定 sensor_extrinsics_profile，且 sensor_extrinsics.defaults 非法')
        profile_name = _as_frame_name(defaults.get('active_profile'), name='sensor_extrinsics.defaults.active_profile')

    profile = profiles.get(profile_name)
    if not isinstance(profile, dict):
        available = ' | '.join(sorted(profiles.keys()))
        raise RuntimeError(f'不支持的 sensor_extrinsics_profile={profile_name}，可选: {available}')
    return profile_name, profile


def _create_sensor_extrinsics_actions(context, *, sensor_extrinsics_file, sensor_extrinsics_profile,
                                      point_lio_config_file, point_lio_dir):
    sensor_extrinsics_path = sensor_extrinsics_file.perform(context)
    requested_profile = sensor_extrinsics_profile.perform(context)
    sensor_data = _load_yaml_file(sensor_extrinsics_path)
    selected_profile, profile = _select_sensor_extrinsics_profile(sensor_data, requested_profile)

    lidar_mount = profile.get('lidar_mount')
    if not isinstance(lidar_mount, dict):
        raise RuntimeError(f'sensor_extrinsics profile {selected_profile} 缺少 lidar_mount')
    lidar_parent = _as_frame_name(lidar_mount.get('parent_frame'), name='lidar_mount.parent_frame')
    lidar_child = _as_frame_name(lidar_mount.get('child_frame'), name='lidar_mount.child_frame')
    lidar_xyz = _as_float_vector(lidar_mount.get('xyz_m'), name='lidar_mount.xyz_m')
    lidar_rpy = _as_float_vector(lidar_mount.get('rpy_rad'), name='lidar_mount.rpy_rad')

    control_mount = profile.get('control_mount', {})
    if not isinstance(control_mount, dict):
        raise RuntimeError(f'sensor_extrinsics profile {selected_profile} 的 control_mount 必须是 mapping')
    control_parent = _as_frame_name(control_mount.get('parent_frame'), name='control_mount.parent_frame')
    control_child = _as_frame_name(control_mount.get('child_frame'), name='control_mount.child_frame')
    if _as_bool(str(control_mount.get('mirror_lidar_mount', False))):
        control_xyz = lidar_xyz
        control_rpy = lidar_rpy
    else:
        control_xyz = _as_float_vector(control_mount.get('xyz_m'), name='control_mount.xyz_m')
        control_rpy = _as_float_vector(control_mount.get('rpy_rad'), name='control_mount.rpy_rad')

    point_lio = profile.get('point_lio')
    if not isinstance(point_lio, dict):
        raise RuntimeError(f'sensor_extrinsics profile {selected_profile} 缺少 point_lio')
    point_lio_body_frame = _as_frame_name(point_lio.get('body_frame'), name='point_lio.body_frame')
    if point_lio_body_frame != 'point_lio_body':
        raise RuntimeError(
            '当前 odom_interface.yaml 固定消费 point_lio_body，'
            f'sensor_extrinsics profile {selected_profile} 中 point_lio.body_frame={point_lio_body_frame} 不受支持'
        )

    explicit_point_lio_config = point_lio_config_file.perform(context).strip()
    resolved_point_lio_config = explicit_point_lio_config or os.path.join(point_lio_dir, 'config', 'mid360.yaml')
    point_lio_body_to_livox = _mapping_extrinsics_from_point_lio_config(resolved_point_lio_config)
    base_to_livox = (_rot_from_rpy(*lidar_rpy), lidar_xyz)
    base_to_point_lio_body = _transform_compose(base_to_livox, _transform_inverse(point_lio_body_to_livox))
    point_lio_body_rpy = _rpy_from_rot(base_to_point_lio_body[0])
    point_lio_body_xyz = base_to_point_lio_body[1]

    return [
        LogInfo(msg=(
            f'[odometry] 传感器安装外参使用 profile:{selected_profile}，'
            f'配置 {sensor_extrinsics_path}'
        )),
        _static_tf_node(
            name='static_tf_base_to_livox',
            parent_frame=lidar_parent,
            child_frame=lidar_child,
            xyz=lidar_xyz,
            rpy=lidar_rpy,
        ),
        _static_tf_node(
            name='static_tf_base_to_point_lio_body',
            parent_frame=lidar_parent,
            child_frame=point_lio_body_frame,
            xyz=point_lio_body_xyz,
            rpy=point_lio_body_rpy,
        ),
        _static_tf_node(
            name='static_tf_base_control_to_livox_control',
            parent_frame=control_parent,
            child_frame=control_child,
            xyz=control_xyz,
            rpy=control_rpy,
        ),
    ]


def _create_point_lio_actions(context, *, namespace, use_sim_time, point_lio_config_file, point_lio_dir,
                              point_lio_publish_odometry_without_downsample):
    namespace_value = namespace.perform(context)
    use_sim_time_value = _as_bool(use_sim_time.perform(context))
    explicit_config_file = point_lio_config_file.perform(context).strip()
    publish_odometry_without_downsample_value = _as_bool(
        point_lio_publish_odometry_without_downsample.perform(context)
    )
    base_config_file = os.path.join(point_lio_dir, 'config', 'mid360.yaml')

    if explicit_config_file:
        resolved_config_file = explicit_config_file
        selected_mode = f'custom:{explicit_config_file}'
    else:
        resolved_config_file = base_config_file
        selected_mode = 'default'

    if not os.path.exists(resolved_config_file):
        raise RuntimeError(f'Point-LIO 配置文件不存在: {resolved_config_file}')

    parameters = [
        resolved_config_file,
        {'use_sim_time': use_sim_time_value},
        {'frame.body_frame': 'point_lio_body'},
        {'odometry.publish_odometry_without_downsample': publish_odometry_without_downsample_value},
        {'publish.tf_send_en': False},
    ]

    point_lio_node = Node(
        package='rc26_point_lio',
        executable='pointlio_mapping',
        name='point_lio',
        namespace=namespace_value,
        output='screen',
        parameters=parameters,
    )

    return [
        LogInfo(msg=f'[odometry] Point-LIO 使用 {selected_mode} 配置 {resolved_config_file}'),
        LogInfo(
            msg='[odometry] 强制 odometry.publish_odometry_without_downsample='
                f'{str(publish_odometry_without_downsample_value).lower()}，'
                '确保 state_estimation 与 cloud_registered 时间戳保持同源'
        ),
        point_lio_node,
    ]


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc26_bringup')
    point_lio_dir = get_package_share_directory('rc26_point_lio')
    mid360_driver_dir = get_package_share_directory('rc26_mid360_driver')
    sensor_extrinsics_dir = get_package_share_directory('rc26_sensor_extrinsics')

    # 启动参数
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_mid360_driver = LaunchConfiguration('start_mid360_driver')
    point_lio_config_file = LaunchConfiguration('point_lio_config_file')
    point_lio_publish_odometry_without_downsample = LaunchConfiguration(
        'point_lio_publish_odometry_without_downsample')
    sensor_extrinsics_file = LaunchConfiguration('sensor_extrinsics_file')
    sensor_extrinsics_profile = LaunchConfiguration('sensor_extrinsics_profile')
    recover_mid360_stream = LaunchConfiguration('recover_mid360_stream')
    recover_mid360_lidar_ip = LaunchConfiguration('recover_mid360_lidar_ip')
    recover_mid360_host_ip = LaunchConfiguration('recover_mid360_host_ip')
    recover_mid360_timeout = LaunchConfiguration('recover_mid360_timeout')
    recover_mid360_warmup_before_reboot = LaunchConfiguration('recover_mid360_warmup_before_reboot')

    # 参数声明
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='顶级命名空间')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')

    declare_start_mid360_driver = DeclareLaunchArgument(
        'start_mid360_driver',
        default_value='true',
        description='启动 MID-360 驱动')

    declare_point_lio_config_file = DeclareLaunchArgument(
        'point_lio_config_file',
        default_value='',
        description='Point-LIO 参数文件路径；为空时使用 rc26_point_lio/config/mid360.yaml')

    declare_point_lio_publish_odometry_without_downsample = DeclareLaunchArgument(
        'point_lio_publish_odometry_without_downsample',
        default_value='false',
        description='是否允许 Point-LIO 在扫描内部提前发布 state_estimation；默认 false，保持与 cloud_registered 同戳')

    declare_sensor_extrinsics_file = DeclareLaunchArgument(
        'sensor_extrinsics_file',
        default_value=PathJoinSubstitution([sensor_extrinsics_dir, 'config', 'r2_sensor_extrinsics.yaml']),
        description='传感器安装外参 YAML 文件路径')

    declare_sensor_extrinsics_profile = DeclareLaunchArgument(
        'sensor_extrinsics_profile',
        default_value='',
        description='传感器安装外参 profile；空字符串表示使用 YAML defaults.active_profile')

    declare_recover_mid360_stream = DeclareLaunchArgument(
        'recover_mid360_stream',
        default_value='false',
        description='启动前先运行 Mid-360 恢复脚本（会在必要时重写 host_ipcfg 并软件重启雷达）')

    declare_recover_mid360_lidar_ip = DeclareLaunchArgument(
        'recover_mid360_lidar_ip',
        default_value='192.168.1.140',
        description='Mid-360 实机 IP（恢复脚本使用）')

    declare_recover_mid360_host_ip = DeclareLaunchArgument(
        'recover_mid360_host_ip',
        default_value='192.168.1.50',
        description='R2 主机接收口 IP（恢复脚本使用）')

    declare_recover_mid360_timeout = DeclareLaunchArgument(
        'recover_mid360_timeout',
        default_value='35.0',
        description='Mid-360 恢复脚本最长等待时间（秒）')

    declare_recover_mid360_warmup_before_reboot = DeclareLaunchArgument(
        'recover_mid360_warmup_before_reboot',
        default_value='5.0',
        description='发现雷达后等待多久再触发软件重启（秒）')

    # 配置文件路径
    mid360_driver_config = PathJoinSubstitution([mid360_driver_dir, 'config', 'param.yaml'])
    odom_interface_config = PathJoinSubstitution([bringup_dir, 'config', 'odom_interface.yaml'])
    sensor_scan_config = PathJoinSubstitution([bringup_dir, 'config', 'sensor_scan_generation.yaml'])

    start_mid360_driver_direct_condition = IfCondition(
        PythonExpression(["'", start_mid360_driver, "' == 'true' and '", recover_mid360_stream, "' != 'true'"])
    )
    recover_mid360_condition = IfCondition(
        PythonExpression(["'", start_mid360_driver, "' == 'true' and '", recover_mid360_stream, "' == 'true'"])
    )

    def create_mid360_driver_node(condition=None):
        kwargs = {
            'package': 'rc26_mid360_driver',
            'executable': 'rc26_mid360_driver_node',
            'name': 'mid360_driver',
            'namespace': namespace,
            'output': 'screen',
            'parameters': [
                mid360_driver_config,
                {'use_sim_time': use_sim_time},
            ],
        }
        if condition is not None:
            kwargs['condition'] = condition
        return Node(**kwargs)

    # MID-360 LiDAR 驱动节点
    mid360_driver_node = create_mid360_driver_node(condition=start_mid360_driver_direct_condition)
    mid360_driver_node_after_recover = create_mid360_driver_node()

    recover_mid360_process = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([mid360_driver_dir, 'scripts', 'recover_mid360_stream.py']),
            '--lidar-ip', recover_mid360_lidar_ip,
            '--host-ip', recover_mid360_host_ip,
            '--timeout', recover_mid360_timeout,
            '--warmup-before-reboot', recover_mid360_warmup_before_reboot,
        ],
        output='screen',
        condition=recover_mid360_condition,
    )

    start_mid360_after_recover = RegisterEventHandler(
        OnProcessExit(
            target_action=recover_mid360_process,
            on_exit=[mid360_driver_node_after_recover],
        )
    )

    # rc26_point_lio 里程计节点
    point_lio_actions = OpaqueFunction(
        function=lambda context: _create_point_lio_actions(
            context,
            namespace=namespace,
            use_sim_time=use_sim_time,
            point_lio_config_file=point_lio_config_file,
            point_lio_dir=point_lio_dir,
            point_lio_publish_odometry_without_downsample=point_lio_publish_odometry_without_downsample,
        )
    )

    # rc26_odom_interface: 将 rc26_point_lio 输出从 lidar_odom 转换到 odom 系
    odom_interface_node = Node(
        package='rc26_odom_interface',
        executable='rc26_odom_interface_node',
        name='odom_interface',
        namespace=namespace,
        output='screen',
        parameters=[
            odom_interface_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    # rc26_sensor_scan: 发布 odom -> chassis 变换和 sensor_scan
    sensor_scan_node = Node(
        package='rc26_sensor_scan',
        executable='rc26_sensor_scan_node',
        name='sensor_scan',
        namespace=namespace,
        output='screen',
        parameters=[
            sensor_scan_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    sensor_extrinsics_actions = OpaqueFunction(
        function=lambda context: _create_sensor_extrinsics_actions(
            context,
            sensor_extrinsics_file=sensor_extrinsics_file,
            sensor_extrinsics_profile=sensor_extrinsics_profile,
            point_lio_config_file=point_lio_config_file,
            point_lio_dir=point_lio_dir,
        )
    )

    return LaunchDescription([
        # 参数声明
        declare_namespace,
        declare_use_sim_time,
        declare_start_mid360_driver,
        declare_point_lio_config_file,
        declare_point_lio_publish_odometry_without_downsample,
        declare_sensor_extrinsics_file,
        declare_sensor_extrinsics_profile,
        declare_recover_mid360_stream,
        declare_recover_mid360_lidar_ip,
        declare_recover_mid360_host_ip,
        declare_recover_mid360_timeout,
        declare_recover_mid360_warmup_before_reboot,

        # 节点
        sensor_extrinsics_actions,
        recover_mid360_process,
        start_mid360_after_recover,
        mid360_driver_node,
        point_lio_actions,
        odom_interface_node,
        sensor_scan_node,
    ])
