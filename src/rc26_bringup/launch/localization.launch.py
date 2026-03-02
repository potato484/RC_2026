"""
定位模块启动文件

导航模式: 启动 rc26_localization (基于 small_gicp)
建图模式: 发布静态 map -> odom 变换
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc26_bringup')
    _dir = get_package_share_directory('')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    prior_pcd_file = LaunchConfiguration('prior_pcd_file')
    param_overrides_file = PathJoinSubstitution([_dir, 'config', 'param_overrides.yaml'])

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    declare_slam = DeclareLaunchArgument(
        'slam', default_value='false')

    declare_prior_pcd_file = DeclareLaunchArgument(
        'prior_pcd_file',
        default_value=PathJoinSubstitution([bringup_dir, 'pcd', 'default.pcd']),
        description='先验点云文件路径')

    # 导航模式: 启动重定位
    localization_node = Node(
        package='rc26_localization',
        executable='rc26_localization_node',
        name='localization',
        namespace=namespace,
        output='screen',
        parameters=[
            PathJoinSubstitution([bringup_dir, 'config', 'localization.yaml']),
            param_overrides_file,
            {'use_sim_time': use_sim_time},
            {'prior_pcd_file': prior_pcd_file},
        ],
        condition=UnlessCondition(slam)
    )

    # 建图模式: 静态变换 map -> odom
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static',
        namespace=namespace,
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(slam)
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_slam,
        declare_prior_pcd_file,
        localization_node,
        static_tf_node,
    ])
