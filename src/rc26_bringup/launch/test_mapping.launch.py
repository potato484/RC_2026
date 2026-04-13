"""
R2 建图联调入口

功能:
  - 复用 bringup.launch.py 的纯建图最小链路
  - 固定默认开启 slam:=true 与 pure_mapping_mode:=true

验证:
  - ros2 topic hz /state_estimation
  - ros2 topic hz /registered_scan
  - ros2 topic echo /laser_map_full --once --field header.frame_id
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc26_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    point_lio_profile = LaunchConfiguration('point_lio_profile')
    recover_mid360_stream = LaunchConfiguration('recover_mid360_stream')
    enable_terrain_grid_map = LaunchConfiguration('enable_terrain_grid_map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')

    declare_point_lio_profile = DeclareLaunchArgument(
        'point_lio_profile',
        default_value='mapping_dense',
        description='建图阶段 Point-LIO 预设，默认 mapping_dense')

    declare_recover_mid360_stream = DeclareLaunchArgument(
        'recover_mid360_stream',
        default_value='false',
        description='启动前先运行 Mid-360 恢复脚本')

    declare_enable_terrain_grid_map = DeclareLaunchArgument(
        'enable_terrain_grid_map',
        default_value='false',
        description='纯建图链路是否额外发布 /terrain_grid_map')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_dir, 'launch', 'bringup.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': 'true',
            'pure_mapping_mode': 'true',
            'point_lio_profile': point_lio_profile,
            'use_decision': 'false',
            'recover_mid360_stream': recover_mid360_stream,
            'enable_terrain_grid_map': enable_terrain_grid_map,
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_point_lio_profile,
        declare_recover_mid360_stream,
        declare_enable_terrain_grid_map,
        bringup_launch,
    ])
