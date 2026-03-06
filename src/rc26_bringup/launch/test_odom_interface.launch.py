"""
rc26_odom_interface 模块测试

功能: 验证 Point-LIO 到 Nav2 的坐标转换
前置: 需要 rc26_point_lio 运行或 rosbag 回放

测试指令:
    ros2 launch rc26_bringup test_odom_interface.launch.py

验证:
    ros2 topic echo /odom --once
    ros2 topic echo /registered_scan --once
    ros2 run tf2_ros tf2_echo odom base_link
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
    
    odom_interface_config = PathJoinSubstitution([
        bringup_dir, 'config', 'odom_interface.yaml'
    ])
    
    odom_interface_node = Node(
        package='rc26_odom_interface',
        executable='rc26_odom_interface_node',
        name='odom_interface',
        output='screen',
        parameters=[
            odom_interface_config,
            {'use_sim_time': use_sim_time},
        ],
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        odom_interface_node,
    ])
