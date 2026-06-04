"""
R2 回环联调入口

功能:
  - 复用 odometry.launch.py + localization.launch.py
  - 固定默认开启 localization 图后端，验证闭环候选与后端状态

验证:
  - ros2 topic echo /localization/loop_closures
  - ros2 topic echo /localization/backend_status --once
  - ros2 topic echo /localization/route_observability --once
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc26_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    prior_pcd_file = LaunchConfiguration('prior_pcd_file')
    start_mid360_driver = LaunchConfiguration('start_mid360_driver')
    recover_mid360_stream = LaunchConfiguration('recover_mid360_stream')
    localization_params_file = LaunchConfiguration('localization_params_file')
    localization_overlay_file = LaunchConfiguration('localization_overlay_file')
    competition_mode = LaunchConfiguration('competition_mode')
    p4_candidate_enable = LaunchConfiguration('p4_candidate_enable')
    min_inliers = LaunchConfiguration('min_inliers')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')

    declare_prior_pcd_file = DeclareLaunchArgument(
        'prior_pcd_file',
        default_value=PathJoinSubstitution([bringup_dir, 'pcd', 'default.pcd']),
        description='回环所用先验点云文件路径')

    declare_start_mid360_driver = DeclareLaunchArgument(
        'start_mid360_driver',
        default_value='true',
        description='是否启动 MID-360 驱动')

    declare_recover_mid360_stream = DeclareLaunchArgument(
        'recover_mid360_stream',
        default_value='false',
        description='启动前先运行 Mid-360 恢复脚本')

    declare_localization_params_file = DeclareLaunchArgument(
        'localization_params_file',
        default_value=PathJoinSubstitution([bringup_dir, 'config', 'localization.yaml']),
        description='基础定位参数文件路径')

    declare_localization_overlay_file = DeclareLaunchArgument(
        'localization_overlay_file',
        default_value=PathJoinSubstitution([bringup_dir, 'config', 'localization_overlay_default.yaml']),
        description='定位 overlay 参数文件路径')

    declare_competition_mode = DeclareLaunchArgument(
        'competition_mode',
        default_value='false',
        description='回环联调默认关闭比赛防呆')

    declare_p4_candidate_enable = DeclareLaunchArgument(
        'p4_candidate_enable',
        default_value='false',
        description='是否启用 P4 外部候选输入')

    declare_min_inliers = DeclareLaunchArgument(
        'min_inliers',
        default_value='200',
        description='局部配准质量门控最小内点数')

    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_dir, 'launch', 'odometry.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'start_mid360_driver': start_mid360_driver,
            'recover_mid360_stream': recover_mid360_stream,
        }.items()
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_dir, 'launch', 'localization.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': 'false',
            'prior_pcd_file': prior_pcd_file,
            'localization_params_file': localization_params_file,
            'localization_overlay_file': localization_overlay_file,
            'competition_mode': competition_mode,
            'enable_graph_backend': 'true',
            'p4_candidate_enable': p4_candidate_enable,
            'min_inliers': min_inliers,
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_prior_pcd_file,
        declare_start_mid360_driver,
        declare_recover_mid360_stream,
        declare_localization_params_file,
        declare_localization_overlay_file,
        declare_competition_mode,
        declare_p4_candidate_enable,
        declare_min_inliers,
        odometry_launch,
        localization_launch,
    ])
