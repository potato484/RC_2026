from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('rc26_base_ground')

    use_sim_time = LaunchConfiguration('use_sim_time')
    parent_frame = LaunchConfiguration('parent_frame')
    config_file = LaunchConfiguration('config_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    declare_parent_frame = DeclareLaunchArgument(
        'parent_frame', default_value='odom')

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_dir, 'config', 'base_ground_estimator.yaml']))

    node = Node(
        package='rc26_base_ground',
        executable='base_ground_estimator_node',
        name='base_ground_estimator',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}, {'parent_frame': parent_frame}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_parent_frame,
        declare_config_file,
        node,
    ])
