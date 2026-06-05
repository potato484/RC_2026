"""
R2 定位基础链联调入口

功能:
  - 复用 odometry.launch.py + localization.launch.py
  - 验证 registered_scan -> rc26_localization -> map->odom 基础链

验证:
  - ros2 run tf2_ros tf2_echo map odom
  - ros2 run tf2_ros tf2_echo odom base_footprint
  - ros2 topic echo /localization/pose_with_cov --once
  - ros2 topic echo /localization/diagnostics --once
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

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')
    declare_prior_pcd_file = DeclareLaunchArgument(
        'prior_pcd_file',
        default_value=PathJoinSubstitution([bringup_dir, 'pcd', 'default.pcd']),
        description='定位所用先验点云文件路径')
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
        description='定位参数文件路径')

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
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_prior_pcd_file,
        declare_start_mid360_driver,
        declare_recover_mid360_stream,
        declare_localization_params_file,
        odometry_launch,
        localization_launch,
    ])
