"""
rc26_sensor_scan 模块测试

功能: 验证点云坐标转换和里程计速度发布
前置: 需要 odom_interface 运行或模拟数据

测试指令:
    ros2 launch rc26_bringup test_sensor_scan.launch.py

验证:
    ros2 topic echo /sensor_scan --once
    ros2 topic echo /odometry --once
    ros2 run tf2_ros tf2_echo base_link laser_link
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc26_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')
    
    sensor_scan_config = PathJoinSubstitution([
        bringup_dir, 'config', 'sensor_scan_generation.yaml'
    ])
    
    sensor_scan_node = Node(
        package='rc26_sensor_scan',
        executable='rc26_sensor_scan_node',
        name='sensor_scan',
        output='screen',
        parameters=[
            sensor_scan_config,
            {'use_sim_time': use_sim_time},
        ],
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        sensor_scan_node,
    ])
