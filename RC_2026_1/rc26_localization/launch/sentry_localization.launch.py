'''
Author: potato potato@potato.com
Date: 2025-12-01 21:04:58
LastEditors: potato potato@potato.com
LastEditTime: 2025-12-12 16:20:43
FilePath: /RC_2026/RC_2026_1/rc26_localization/launch/sentry_localization.launch.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
"""
rc26_localization 启动文件
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取配置文件路径
    bringup_dir = get_package_share_directory('rc26_bringup')
    config_file = os.path.join(bringup_dir, 'config', 'localization.yaml')
    
    # 参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    prior_pcd_file = LaunchConfiguration('prior_pcd_file')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')
    
    declare_prior_pcd_file = DeclareLaunchArgument(
        'prior_pcd_file',
        default_value='',
        description='先验点云文件路径')
    
    # 定位节点 (加载 YAML 配置，然后覆盖动态参数)
    localization_node = Node(
        package='rc26_localization',
        executable='rc26_localization_node',
        name='localization',
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'prior_pcd_file': prior_pcd_file,
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_prior_pcd_file,
        localization_node,
    ])
