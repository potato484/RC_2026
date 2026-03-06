"""
rc26_localization 模块测试

功能: 验证基于 rc26_small_gicp 的点云配准定位
前置: 需要先验点云文件和 registered_scan 话题

测试指令:
    ros2 launch rc26_bringup test_localization.launch.py \
        prior_pcd_file:=${RC26_WS:-$HOME/RC_2026}/src/rc26_bringup/pcd/default.pcd

验证:
    ros2 run tf2_ros tf2_echo map odom
    ros2 topic echo /localization/status --once
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc26_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    prior_pcd_file = LaunchConfiguration('prior_pcd_file')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')
    
    declare_prior_pcd_file = DeclareLaunchArgument(
        'prior_pcd_file',
        default_value=PathJoinSubstitution([bringup_dir, 'pcd', 'default.pcd']),
        description='先验点云文件路径')
    
    localization_config = PathJoinSubstitution([
        bringup_dir, 'config', 'localization.yaml'
    ])
    
    localization_node = Node(
        package='rc26_localization',
        executable='rc26_localization_node',
        name='localization',
        output='screen',
        parameters=[
            localization_config,
            {
                'use_sim_time': use_sim_time,
                'prior_pcd_file': prior_pcd_file,
            },
        ],
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_prior_pcd_file,
        localization_node,
    ])
