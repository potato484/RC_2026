import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('rc26_terrain')
    default_params = os.path.join(pkg_dir, 'config', 'terrain_semantic.yaml')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    terrain_params_file = LaunchConfiguration('terrain_params_file')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='顶级命名空间（可为空）')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间（/clock）')

    declare_params_file = DeclareLaunchArgument(
        'terrain_params_file',
        default_value=default_params,
        description='rc26_terrain 参数文件路径')

    node = Node(
        package='rc26_terrain',
        executable='rc26_terrain_node',
        name='terrain_semantic',
        namespace=namespace,
        output='screen',
        parameters=[terrain_params_file, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_params_file,
        node,
    ])
