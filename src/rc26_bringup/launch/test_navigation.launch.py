"""
R2 导航联调入口

功能:
  - 复用 bringup.launch.py
  - 固定默认关闭 decision，保留定位 + 回环 + xhu_nav 主链

验证:
  - ros2 topic echo /xhu_nav/motion_mode_state
  - ros2 topic echo /xhu_nav/tracking_state
  - ros2 topic echo /cmd_vel
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
    point_lio_profile = LaunchConfiguration('point_lio_profile')
    recover_mid360_stream = LaunchConfiguration('recover_mid360_stream')
    team = LaunchConfiguration('team')
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
        description='导航所用先验点云文件路径')

    declare_point_lio_profile = DeclareLaunchArgument(
        'point_lio_profile',
        default_value='auto',
        description='导航阶段 Point-LIO 预设')

    declare_recover_mid360_stream = DeclareLaunchArgument(
        'recover_mid360_stream',
        default_value='false',
        description='启动前先运行 Mid-360 恢复脚本')

    declare_team = DeclareLaunchArgument(
        'team',
        default_value='blue',
        description='当前比赛侧别: blue | red')

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
        description='导航联调默认关闭比赛防呆')

    declare_p4_candidate_enable = DeclareLaunchArgument(
        'p4_candidate_enable',
        default_value='false',
        description='是否启用 P4 外部候选输入')

    declare_min_inliers = DeclareLaunchArgument(
        'min_inliers',
        default_value='200',
        description='局部配准质量门控最小内点数')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_dir, 'launch', 'bringup.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': 'false',
            'use_decision': 'false',
            'prior_pcd_file': prior_pcd_file,
            'point_lio_profile': point_lio_profile,
            'recover_mid360_stream': recover_mid360_stream,
            'team': team,
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
        declare_point_lio_profile,
        declare_recover_mid360_stream,
        declare_team,
        declare_localization_params_file,
        declare_localization_overlay_file,
        declare_competition_mode,
        declare_p4_candidate_enable,
        declare_min_inliers,
        bringup_launch,
    ])
