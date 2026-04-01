import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('rc26_nav_mode_manager')
    default_profiles = os.path.join(pkg_dir, 'config', 'nav_profiles.yaml')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    profiles_file = LaunchConfiguration('profiles_file')
    odom_topic = LaunchConfiguration('odom_topic')
    default_mode = LaunchConfiguration('default_mode')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('profiles_file', default_value=default_profiles),
        DeclareLaunchArgument('odom_topic', default_value='control_state'),
        DeclareLaunchArgument('default_mode', default_value='hold'),
        Node(
            package='rc26_nav_mode_manager',
            executable='xhu_motion_mode_manager_node',
            name='xhu_motion_mode_manager',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'profiles_file': profiles_file,
                'odom_topic': odom_topic,
                'default_mode': default_mode,
            }],
        ),
    ])
