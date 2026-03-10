"""
定位模块启动文件

导航模式: 启动 rc26_localization (基于 rc26_small_gicp)
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

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    prior_pcd_file = LaunchConfiguration('prior_pcd_file')
    competition_mode = LaunchConfiguration('competition_mode')
    enable_graph_backend = LaunchConfiguration('enable_graph_backend')
    p4_candidate_enable = LaunchConfiguration('p4_candidate_enable')
    min_inliers = LaunchConfiguration('min_inliers')

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

    declare_competition_mode = DeclareLaunchArgument(
        'competition_mode', default_value='true', description='比赛模式防呆开关')

    declare_enable_graph_backend = DeclareLaunchArgument(
        'enable_graph_backend', default_value='false', description='是否启用图后端')

    declare_p4_candidate_enable = DeclareLaunchArgument(
        'p4_candidate_enable', default_value='false', description='是否启用 P4 外部候选输入')

    declare_min_inliers = DeclareLaunchArgument(
        'min_inliers', default_value='200', description='局部配准质量门控最小内点数')

    # 导航模式: 启动重定位
    localization_node = Node(
        package='rc26_localization',
        executable='rc26_localization_node',
        name='localization',
        namespace=namespace,
        output='screen',
        parameters=[
            PathJoinSubstitution([bringup_dir, 'config', 'localization.yaml']),
            {'use_sim_time': use_sim_time},
            {'prior_pcd_file': prior_pcd_file},
            {'competition_mode': competition_mode},
            {'enable_graph_backend': enable_graph_backend},
            {'p4_candidate_enable': p4_candidate_enable},
            {'min_inliers': min_inliers},
        ],
        condition=UnlessCondition(slam)
    )

    # 建图模式: 静态变换 map -> odom
    static_tf_node = Node(
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
        condition=IfCondition(slam)
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_slam,
        declare_prior_pcd_file,
        declare_competition_mode,
        declare_enable_graph_backend,
        declare_p4_candidate_enable,
        declare_min_inliers,
        localization_node,
        static_tf_node,
    ])
