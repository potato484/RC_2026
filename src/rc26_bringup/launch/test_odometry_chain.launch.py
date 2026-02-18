"""
完整里程计链测试 (point_lio + odom_interface + sensor_scan)

功能: 验证从 LiDAR 原始数据到 Nav2 可用里程计的完整数据流
前置: 需要 Livox MID-360 雷达连接或 rosbag 回放

测试指令:
    ros2 launch rc26_bringup test_odometry_chain.launch.py

验证:
    ros2 topic list | grep -E "(odom|scan|odometry)"
    ros2 topic hz /odometry
    ros2 run tf2_tools view_frames
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
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')
    
    declare_prior_pcd_file = DeclareLaunchArgument(
        'prior_pcd_file',
        default_value='',
        description='先验点云文件路径 (可选)')
    
    # 复用已有的 odometry.launch.py
    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_dir, 'launch', 'odometry.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'prior_pcd_file': prior_pcd_file,
        }.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_prior_pcd_file,
        odometry_launch,
    ])
