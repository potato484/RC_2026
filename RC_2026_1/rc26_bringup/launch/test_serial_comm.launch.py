"""
rc26_decision 串口通信测试

功能: 验证与 MCU 的串口通信 (位姿发送、指令交互)
前置: 需要 MCU 设备连接或串口回环测试器

测试指令:
    # 连接真实 MCU，红方
    ros2 launch rc26_bringup test_serial_comm.launch.py \
        team:=red serial_port:=/dev/ttyUSB0

    # 连接真实 MCU，蓝方
    ros2 launch rc26_bringup test_serial_comm.launch.py \
        team:=blue serial_port:=/dev/ttyUSB0

    # 使用虚拟串口对 (用于调试)
    # 终端1: socat -d -d pty,raw,echo=0 pty,raw,echo=0
    # 终端2: ros2 launch rc26_bringup test_serial_comm.launch.py \
    #           team:=red serial_port:=/dev/pts/X

验证:
    # 检查串口连接状态
    ros2 topic echo /waypoint_patrol/serial_status --once
    
    # 发送测试位姿 (需要 odom 话题)
    ros2 topic pub /odometry nav_msgs/msg/Odometry "{pose: {pose: {position: {x: 1.0}}}}" --once
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('rc26_bringup')
    decision_dir = get_package_share_directory('rc26_decision')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    pose_publish_rate = LaunchConfiguration('pose_publish_rate')
    team = LaunchConfiguration('team').perform(context)

    # 根据队伍选择决策配置文件
    if team == 'blue':
        decision_config_path = os.path.join(bringup_dir, 'config', 'decision_blue.yaml')
    else:
        decision_config_path = os.path.join(bringup_dir, 'config', 'decision_red.yaml')

    # 回退到通用 decision.yaml，保持兼容
    if not os.path.exists(decision_config_path):
        decision_config_path = os.path.join(bringup_dir, 'config', 'decision.yaml')

    decision_config = decision_config_path
    tree_xml = PathJoinSubstitution([decision_dir, 'behavior_tree', 'waypoint_patrol.xml'])
    
    # 决策节点 (启用串口)
    decision_node = Node(
        package='rc26_decision',
        executable='waypoint_patrol_node',
        name='waypoint_patrol',
        output='screen',
        parameters=[
            decision_config,
            {
                'use_sim_time': use_sim_time,
                'enable_serial': True,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'pose_publish_rate': pose_publish_rate,
                'tree_xml_file': tree_xml,
            },
        ],
    )

    return [decision_node]


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')
    
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='串口设备路径')
    
    declare_serial_baudrate = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='串口波特率')
    
    declare_pose_publish_rate = DeclareLaunchArgument(
        'pose_publish_rate',
        default_value='50.0',
        description='位姿发送频率 (Hz)')

    declare_team = DeclareLaunchArgument(
        'team',
        default_value='red',
        description='队伍颜色: red 或 blue')

    decision_node = OpaqueFunction(function=launch_setup)
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_serial_port,
        declare_serial_baudrate,
        declare_pose_publish_rate,
        declare_team,
        decision_node,
    ])
