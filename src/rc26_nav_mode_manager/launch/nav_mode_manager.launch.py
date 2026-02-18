import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('rc26_nav_mode_manager')
    default_params = os.path.join(pkg_dir, 'config', 'nav_mode_manager.yaml')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Parameter file path')

    node = Node(
        package='rc26_nav_mode_manager',
        executable='nav_mode_manager_node',
        name='nav_mode_manager',
        namespace=namespace,
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_params_file,
        node,
    ])
