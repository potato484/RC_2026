"""
里程计模块启动文件

启动:
  - rc26_point_lio (LiDAR-IMU 里程计)
  - rc26_odom_interface (坐标变换: lidar_odom -> odom)
  - rc26_sensor_scan (发布 odom -> chassis 变换 + sensor_scan)
  - rc26_lio_state_predictor (可选，提供控制态预测)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _resolve_point_lio_profile(requested_profile: str, *, slam_value: bool) -> tuple[str, dict]:
    profile_aliases = {
        'default': 'base',
    }
    profile_overrides = {
        'base': {},
        'cruise_light': {
            'publish.map_full_publish_en': False,
            'publish.map_full_publish_interval_sec': 1.5,
        },
        'mapping_dense': {
            'point_keep_ratio': 100.0,
            'filter_size_surf': 0.1,
            'filter_size_map': 0.1,
            'pcd_save.pcd_save_en': True,
        },
    }

    resolved_profile = requested_profile or 'auto'
    if resolved_profile == 'auto':
        resolved_profile = 'mapping_dense' if slam_value else 'cruise_light'

    resolved_profile = profile_aliases.get(resolved_profile, resolved_profile)
    if resolved_profile not in profile_overrides:
        supported_profiles = 'auto | base | cruise_light | mapping_dense'
        raise RuntimeError(
            f'不支持的 point_lio_profile={requested_profile}，可选: {supported_profiles}')

    return resolved_profile, profile_overrides[resolved_profile]


def _create_point_lio_actions(context, *, namespace, use_sim_time, prior_pcd_file, point_lio_config_file,
                              point_lio_profile, slam, param_overrides_file, point_lio_dir,
                              point_lio_publish_odometry_without_downsample):
    namespace_value = namespace.perform(context)
    use_sim_time_value = _as_bool(use_sim_time.perform(context))
    prior_pcd_file_value = prior_pcd_file.perform(context)
    explicit_config_file = point_lio_config_file.perform(context).strip()
    requested_profile = point_lio_profile.perform(context).strip().lower() or 'auto'
    slam_value = _as_bool(slam.perform(context))
    param_overrides_value = param_overrides_file.perform(context)
    publish_odometry_without_downsample_value = _as_bool(
        point_lio_publish_odometry_without_downsample.perform(context)
    )
    base_config_file = os.path.join(point_lio_dir, 'config', 'mid360.yaml')

    if explicit_config_file:
        resolved_config_file = explicit_config_file
        selected_mode = f'custom:{explicit_config_file}'
        profile_overrides = {}
    else:
        resolved_profile, profile_overrides = _resolve_point_lio_profile(
            requested_profile,
            slam_value=slam_value,
        )
        resolved_config_file = base_config_file
        selected_mode = f'profile:{resolved_profile}'

    if not os.path.exists(resolved_config_file):
        raise RuntimeError(f'Point-LIO 配置文件不存在: {resolved_config_file}')

    parameters = [resolved_config_file]
    if profile_overrides:
        parameters.append(profile_overrides)
    parameters.extend([
        param_overrides_value,
        {'use_sim_time': use_sim_time_value},
        {'prior_pcd.prior_pcd_map_path': prior_pcd_file_value},
        {'frame.body_frame': 'point_lio_body'},
        {'odometry.publish_odometry_without_downsample': publish_odometry_without_downsample_value},
        {'publish.tf_send_en': False},
    ])

    point_lio_node = Node(
        package='rc26_point_lio',
        executable='pointlio_mapping',
        name='point_lio',
        namespace=namespace_value,
        output='screen',
        parameters=parameters,
    )

    return [
        LogInfo(msg=f'[odometry] Point-LIO 使用 {selected_mode}，基础配置 {resolved_config_file}'),
        LogInfo(
            msg='[odometry] 强制 odometry.publish_odometry_without_downsample='
                f'{str(publish_odometry_without_downsample_value).lower()}，'
                '确保 state_estimation 与 cloud_registered 时间戳保持同源'
        ),
        point_lio_node,
    ]


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc26_bringup')
    _dir = get_package_share_directory('')
    point_lio_dir = get_package_share_directory('rc26_point_lio')
    mid360_driver_dir = get_package_share_directory('rc26_mid360_driver')

    # 启动参数
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    prior_pcd_file = LaunchConfiguration('prior_pcd_file')
    slam = LaunchConfiguration('slam')
    odometry_use_rviz = LaunchConfiguration('odometry_use_rviz')
    start_mid360_driver = LaunchConfiguration('start_mid360_driver')
    point_lio_config_file = LaunchConfiguration('point_lio_config_file')
    point_lio_profile = LaunchConfiguration('point_lio_profile')
    point_lio_publish_odometry_without_downsample = LaunchConfiguration(
        'point_lio_publish_odometry_without_downsample')
    enable_lio_state_predictor = LaunchConfiguration('enable_lio_state_predictor')
    recover_mid360_stream = LaunchConfiguration('recover_mid360_stream')
    recover_mid360_lidar_ip = LaunchConfiguration('recover_mid360_lidar_ip')
    recover_mid360_host_ip = LaunchConfiguration('recover_mid360_host_ip')
    recover_mid360_timeout = LaunchConfiguration('recover_mid360_timeout')
    recover_mid360_warmup_before_reboot = LaunchConfiguration('recover_mid360_warmup_before_reboot')
    param_overrides_file = PathJoinSubstitution([_dir, 'config', 'param_overrides.yaml'])

    # 参数声明
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='顶级命名空间')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')

    declare_prior_pcd_file = DeclareLaunchArgument(
        'prior_pcd_file',
        default_value='',
        description='先验点云文件路径 (建图模式下可为空)')

    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='是否处于建图模式（用于 auto 选择 Point-LIO profile）')

    declare_use_rviz = DeclareLaunchArgument(
        'odometry_use_rviz',
        default_value='true',
        description='启动 RViz')

    declare_start_mid360_driver = DeclareLaunchArgument(
        'start_mid360_driver',
        default_value='true',
        description='启动 MID-360 驱动')

    declare_point_lio_config_file = DeclareLaunchArgument(
        'point_lio_config_file',
        default_value='',
        description='Point-LIO 参数文件路径；非空时优先级高于 point_lio_profile')

    declare_point_lio_profile = DeclareLaunchArgument(
        'point_lio_profile',
        default_value='auto',
        description='Point-LIO 预设: auto | base | cruise_light | mapping_dense')

    declare_point_lio_publish_odometry_without_downsample = DeclareLaunchArgument(
        'point_lio_publish_odometry_without_downsample',
        default_value='false',
        description='是否允许 Point-LIO 在扫描内部提前发布 state_estimation；默认 false，保持与 cloud_registered 同戳')

    declare_enable_lio_state_predictor = DeclareLaunchArgument(
        'enable_lio_state_predictor',
        default_value='true',
        description='是否启动 lio_state_predictor；纯建图最小模式建议关闭以减少 stale 告警和额外负载')

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
    lio_state_predictor_yaml = PathJoinSubstitution([bringup_dir, 'config', 'lio_state_predictor.yaml'])

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
            prior_pcd_file=prior_pcd_file,
            point_lio_config_file=point_lio_config_file,
            point_lio_profile=point_lio_profile,
            slam=slam,
            param_overrides_file=param_overrides_file,
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

    lio_state_predictor_node = Node(
        package='rc26_lio_state_predictor',
        executable='rc26_lio_state_predictor_node',
        name='lio_state_predictor',
        namespace=namespace,
        output='screen',
        parameters=[
            lio_state_predictor_yaml,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(enable_lio_state_predictor)
    )

    # 静态TF: base_link -> livox_frame (与 Point-LIO 外参对齐)
    static_tf_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_livox',
        arguments=['--x', '0', '--y', '0', '--z', '0.13', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
    )

    # Point-LIO `body_frame` = IMU/body frame。
    # 依据当前 mid360.yaml: extrinsic_R=I, extrinsic_T=[-0.011, -0.02329, 0.04412]
    # 该参数表示 LiDAR 原点在 IMU 坐标系中的位置，因此 IMU 原点在 LiDAR 坐标系中为其相反数：
    #   p_imu_in_lidar = [0.011, 0.02329, -0.04412]
    # 进而 base_link -> point_lio_body = base_link -> livox_frame + livox_frame -> point_lio_body
    static_tf_point_lio_body = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_point_lio_body',
        arguments=['--x', '0.011', '--y', '0.02329', '--z', '0.08588', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'point_lio_body'],
    )

    static_tf_control_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_control_to_livox_control',
        arguments=['--x', '0', '--y', '0', '--z', '0.13', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link_control', '--child-frame-id', 'livox_frame_control'],
    )

    # RViz 可视化
    rviz_config = PathJoinSubstitution([bringup_dir, 'rviz', 'slam.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(odometry_use_rviz)
    )

    return LaunchDescription([
        # 参数声明
        declare_namespace,
        declare_use_sim_time,
        declare_prior_pcd_file,
        declare_slam,
        declare_use_rviz,
        declare_start_mid360_driver,
        declare_point_lio_config_file,
        declare_point_lio_profile,
        declare_point_lio_publish_odometry_without_downsample,
        declare_enable_lio_state_predictor,
        declare_recover_mid360_stream,
        declare_recover_mid360_lidar_ip,
        declare_recover_mid360_host_ip,
        declare_recover_mid360_timeout,
        declare_recover_mid360_warmup_before_reboot,

        # 节点
        static_tf_livox,
        static_tf_point_lio_body,
        static_tf_control_livox,
        recover_mid360_process,
        start_mid360_after_recover,
        mid360_driver_node,
        point_lio_actions,
        odom_interface_node,
        sensor_scan_node,
        lio_state_predictor_node,
        rviz_node,
    ])
