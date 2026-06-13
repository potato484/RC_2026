"""
定位模块启动文件

run_mode:=navigation: 启动 rc26_localization，作为 map -> odom 唯一权威
run_mode:=mapping: 发布静态 map -> odom 变换
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _launch_value(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def _parse_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


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


def _create_localization_actions(context):
    namespace = _launch_value(context, 'namespace')
    use_sim_time = _parse_bool(_launch_value(context, 'use_sim_time'))
    run_mode = _read_run_mode(context)
    prior_pcd_file = _launch_value(context, 'prior_pcd_file')
    localization_params_file = _launch_value(context, 'localization_params_file')

    if run_mode == 'navigation':
        return [Node(
            package='rc26_localization',
            executable='rc26_localization_node',
            name='localization',
            namespace=namespace,
            output='screen',
            parameters=[
                localization_params_file,
                {'use_sim_time': use_sim_time},
                {'prior_pcd_file': prior_pcd_file},
            ],
        )]

    return [Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static',
        namespace=namespace,
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )]


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc26_bringup')

    declare_namespace = DeclareLaunchArgument('namespace', default_value='')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_run_mode = DeclareLaunchArgument(
        'run_mode',
        default_value='navigation',
        description='运行模式: navigation | mapping；navigation=启动 rc26_localization, mapping=发布静态 map->odom')
    declare_prior_pcd_file = DeclareLaunchArgument(
        'prior_pcd_file',
        default_value=PathJoinSubstitution([bringup_dir, 'pcd', 'default.pcd']),
        description='先验点云文件路径')
    declare_localization_params_file = DeclareLaunchArgument(
        'localization_params_file',
        default_value=PathJoinSubstitution([bringup_dir, 'config', 'localization.yaml']),
        description='定位参数文件路径')

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_run_mode,
        declare_prior_pcd_file,
        declare_localization_params_file,
        OpaqueFunction(function=_create_localization_actions),
    ])
