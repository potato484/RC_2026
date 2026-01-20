'''
Author: potato potato@potato.com
Date: 2025-12-03 23:54:50
LastEditors: potato potato@potato.com
LastEditTime: 2025-12-07 17:33:57
FilePath: /RC_2026/RC_2026_1/rc26_bringup/launch/test_decision.launch.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
"""
rc26_decision 模块测试

功能: 验证行为树决策逻辑
前置: 无强制依赖，可独立运行 (Nav2 相关动作会超时但不崩溃)

测试指令:
    ros2 launch rc26_bringup test_decision.launch.py

验证:
    ros2 node info /waypoint_patrol
    ros2 topic list | grep decision
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

    # LaunchConfiguration 对象用于参数传递
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_serial = LaunchConfiguration('enable_serial')
    decision_config_path = os.path.join(bringup_dir, 'config', 'decision.yaml')

    decision_config = decision_config_path
    tree_xml = PathJoinSubstitution([decision_dir, 'behavior_tree', 'waypoint_patrol.xml'])

    decision_node = Node(
        package='rc26_decision',
        executable='waypoint_patrol_node',
        name='waypoint_patrol',
        output='screen',
        parameters=[
            decision_config,
            {
                'use_sim_time': use_sim_time,
                'enable_serial': enable_serial,
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

    declare_enable_serial = DeclareLaunchArgument(
        'enable_serial',
        default_value='false',
        description='启用串口通信 (测试时通常关闭)')

    decision_node = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        declare_use_sim_time,
        declare_enable_serial,
        decision_node,
    ])
