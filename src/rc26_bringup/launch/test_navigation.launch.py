"""
R2 导航联调入口

功能:
  - 复用 bringup.launch.py
  - 固定默认关闭 decision，保留定位 + Nav2 基础导航链

验证:
  - ros2 action info /navigate_to_pose
  - ros2 lifecycle get /controller_server
  - ros2 lifecycle get /bt_navigator
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
    recover_mid360_stream = LaunchConfiguration('recover_mid360_stream')
    team = LaunchConfiguration('team')
    localization_params_file = LaunchConfiguration('localization_params_file')
    nav2_map_file = LaunchConfiguration('nav2_map_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')
    declare_prior_pcd_file = DeclareLaunchArgument(
        'prior_pcd_file',
        default_value=PathJoinSubstitution([bringup_dir, 'pcd', 'default.pcd']),
        description='导航所用先验点云文件路径')
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
        description='定位参数文件路径')
    declare_nav2_map_file = DeclareLaunchArgument(
        'nav2_map_file',
        default_value=PathJoinSubstitution([bringup_dir, 'map', 'default.yaml']),
        description='Nav2 map_server 使用的 2D occupancy map YAML；实机导航应传入有效地图')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_dir, 'launch', 'bringup.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': 'false',
            'use_decision': 'false',
            'prior_pcd_file': prior_pcd_file,
            'recover_mid360_stream': recover_mid360_stream,
            'team': team,
            'localization_params_file': localization_params_file,
            'nav2_map_file': nav2_map_file,
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_prior_pcd_file,
        declare_recover_mid360_stream,
        declare_team,
        declare_localization_params_file,
        declare_nav2_map_file,
        bringup_launch,
    ])
