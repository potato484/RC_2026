import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import yaml


def _load_yaml_file(path: str) -> dict:
    if not os.path.exists(path):
        raise RuntimeError(f'YAML 配置文件不存在: {path}')
    with open(path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise RuntimeError(f'YAML 配置文件格式非法，顶层必须是 mapping: {path}')
    return data


def _as_frame_name(value, *, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise RuntimeError(f'{name} 必须是非空字符串')
    return value.strip()


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


def _create_mock_point_lio_action(context, *, bringup_dir, sensor_extrinsics_file, sensor_extrinsics_profile,
                                  mock_rate_hz, mock_obstacle_enable, mock_ground_enable,
                                  mock_stationary_warmup_sec):
    sensor_extrinsics_path = sensor_extrinsics_file.perform(context)
    requested_profile = sensor_extrinsics_profile.perform(context)
    sensor_data = _load_yaml_file(sensor_extrinsics_path)
    selected_profile, profile = _select_sensor_extrinsics_profile(sensor_data, requested_profile)

    point_lio = profile.get('point_lio')
    if not isinstance(point_lio, dict):
        raise RuntimeError(f'sensor_extrinsics profile {selected_profile} 缺少 point_lio')
    body_frame = _as_frame_name(point_lio.get('body_frame'), name='point_lio.body_frame')
    mock_script = os.path.join(bringup_dir, 'scripts', 'mock_point_lio.py')

    return [
        LogInfo(msg=(
            f'[odometry_mock] 使用 profile:{selected_profile} 的 Point-LIO 内部 body frame={body_frame}'
        )),
        ExecuteProcess(
            cmd=[
                'python3',
                mock_script,
                '--rate_hz',
                mock_rate_hz.perform(context),
                '--obstacle_enable',
                mock_obstacle_enable.perform(context),
                '--ground_enable',
                mock_ground_enable.perform(context),
                '--body_frame',
                body_frame,
                '--stationary_warmup_sec',
                mock_stationary_warmup_sec.perform(context),
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc26_bringup')
    sensor_extrinsics_dir = get_package_share_directory('rc26_sensor_extrinsics')

    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_map_to_odom = LaunchConfiguration('publish_map_to_odom')
    map_to_odom_x = LaunchConfiguration('map_to_odom_x')
    map_to_odom_y = LaunchConfiguration('map_to_odom_y')
    map_to_odom_z = LaunchConfiguration('map_to_odom_z')
    map_to_odom_roll = LaunchConfiguration('map_to_odom_roll')
    map_to_odom_pitch = LaunchConfiguration('map_to_odom_pitch')
    map_to_odom_yaw = LaunchConfiguration('map_to_odom_yaw')
    mock_rate_hz = LaunchConfiguration('mock_rate_hz')
    mock_obstacle_enable = LaunchConfiguration('mock_obstacle_enable')
    mock_ground_enable = LaunchConfiguration('mock_ground_enable')
    mock_stationary_warmup_sec = LaunchConfiguration('mock_stationary_warmup_sec')
    point_lio_config_file = LaunchConfiguration('point_lio_config_file')
    sensor_extrinsics_file = LaunchConfiguration('sensor_extrinsics_file')
    sensor_extrinsics_profile = LaunchConfiguration('sensor_extrinsics_profile')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间',
    )
    declare_publish_map_to_odom = DeclareLaunchArgument(
        'publish_map_to_odom',
        default_value='true',
        description='是否发布 map->odom 静态变换',
    )
    declare_map_to_odom_x = DeclareLaunchArgument(
        'map_to_odom_x',
        default_value='0.0',
        description='map->odom x',
    )
    declare_map_to_odom_y = DeclareLaunchArgument(
        'map_to_odom_y',
        default_value='0.0',
        description='map->odom y',
    )
    declare_map_to_odom_z = DeclareLaunchArgument(
        'map_to_odom_z',
        default_value='0.0',
        description='map->odom z',
    )
    declare_map_to_odom_roll = DeclareLaunchArgument(
        'map_to_odom_roll',
        default_value='0.0',
        description='map->odom roll',
    )
    declare_map_to_odom_pitch = DeclareLaunchArgument(
        'map_to_odom_pitch',
        default_value='0.0',
        description='map->odom pitch',
    )
    declare_map_to_odom_yaw = DeclareLaunchArgument(
        'map_to_odom_yaw',
        default_value='0.0',
        description='map->odom yaw',
    )
    declare_mock_rate_hz = DeclareLaunchArgument(
        'mock_rate_hz',
        default_value='10.0',
        description='mock_point_lio 发布频率',
    )
    declare_mock_obstacle_enable = DeclareLaunchArgument(
        'mock_obstacle_enable',
        default_value='true',
        description='是否发布障碍点',
    )
    declare_mock_ground_enable = DeclareLaunchArgument(
        'mock_ground_enable',
        default_value='true',
        description='是否发布地面点',
    )
    declare_mock_stationary_warmup_sec = DeclareLaunchArgument(
        'mock_stationary_warmup_sec',
        default_value='1.5',
        description='mock_point_lio 启动后保持静止的秒数，用于满足 odom_interface 首帧静止归零',
    )
    declare_point_lio_config_file = DeclareLaunchArgument(
        'point_lio_config_file',
        default_value='',
        description='Point-LIO 参数文件路径；用于推导 odom_interface 内部 body 外参',
    )
    declare_sensor_extrinsics_file = DeclareLaunchArgument(
        'sensor_extrinsics_file',
        default_value=PathJoinSubstitution([sensor_extrinsics_dir, 'config', 'r2_sensor_extrinsics.yaml']),
        description='传感器安装外参 YAML 文件路径',
    )
    declare_sensor_extrinsics_profile = DeclareLaunchArgument(
        'sensor_extrinsics_profile',
        default_value='',
        description='传感器安装外参 profile；空字符串表示使用 YAML defaults.active_profile',
    )

    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=[
            '--x',
            map_to_odom_x,
            '--y',
            map_to_odom_y,
            '--z',
            map_to_odom_z,
            '--roll',
            map_to_odom_roll,
            '--pitch',
            map_to_odom_pitch,
            '--yaw',
            map_to_odom_yaw,
            '--frame-id',
            'map',
            '--child-frame-id',
            'odom',
        ],
        condition=IfCondition(publish_map_to_odom),
    )

    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_dir, 'launch', 'odometry.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'start_mid360_driver': 'false',
            'start_point_lio': 'false',
            'point_lio_config_file': point_lio_config_file,
            'sensor_extrinsics_file': sensor_extrinsics_file,
            'sensor_extrinsics_profile': sensor_extrinsics_profile,
        }.items()
    )

    mock_point_lio = OpaqueFunction(
        function=lambda context: _create_mock_point_lio_action(
            context,
            bringup_dir=bringup_dir,
            sensor_extrinsics_file=sensor_extrinsics_file,
            sensor_extrinsics_profile=sensor_extrinsics_profile,
            mock_rate_hz=mock_rate_hz,
            mock_obstacle_enable=mock_obstacle_enable,
            mock_ground_enable=mock_ground_enable,
            mock_stationary_warmup_sec=mock_stationary_warmup_sec,
        )
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_publish_map_to_odom,
            declare_map_to_odom_x,
            declare_map_to_odom_y,
            declare_map_to_odom_z,
            declare_map_to_odom_roll,
            declare_map_to_odom_pitch,
            declare_map_to_odom_yaw,
            declare_mock_rate_hz,
            declare_mock_obstacle_enable,
            declare_mock_ground_enable,
            declare_mock_stationary_warmup_sec,
            declare_point_lio_config_file,
            declare_sensor_extrinsics_file,
            declare_sensor_extrinsics_profile,
            static_tf_map_to_odom,
            odometry_launch,
            mock_point_lio,
        ]
    )
