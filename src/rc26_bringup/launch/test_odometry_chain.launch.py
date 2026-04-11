"""
完整里程计链测试 (rc26_point_lio + odom_interface + sensor_scan)

功能: 验证从 LiDAR 原始数据到统一 odom/tf 的完整数据流
前置: 需要 Livox MID-360 雷达连接或 rosbag 回放

测试指令:
    ros2 launch rc26_bringup test_odometry_chain.launch.py
    ros2 launch rc26_bringup test_odometry_chain.launch.py recover_mid360_stream:=true

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
    start_mid360_driver = LaunchConfiguration('start_mid360_driver')
    recover_mid360_stream = LaunchConfiguration('recover_mid360_stream')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')

    declare_prior_pcd_file = DeclareLaunchArgument(
        'prior_pcd_file',
        default_value='',
        description='先验点云文件路径 (可选)')

    declare_start_mid360_driver = DeclareLaunchArgument(
        'start_mid360_driver',
        default_value='false',
        description='是否启动 MID-360 驱动（虚拟测试默认关闭）')

    declare_recover_mid360_stream = DeclareLaunchArgument(
        'recover_mid360_stream',
        default_value='false',
        description='启动 odometry 链前先运行 Mid-360 恢复脚本')

    # 复用已有的 odometry.launch.py
    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_dir, 'launch', 'odometry.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'prior_pcd_file': prior_pcd_file,
            'start_mid360_driver': start_mid360_driver,
            'recover_mid360_stream': recover_mid360_stream,
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_prior_pcd_file,
        declare_start_mid360_driver,
        declare_recover_mid360_stream,
        odometry_launch,
    ])
