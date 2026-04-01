"""rc26_topo_nav launch file."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('rc26_topo_nav')

    team = LaunchConfiguration('team')
    graph_file = LaunchConfiguration('graph_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    execution_backend = LaunchConfiguration('execution_backend')
    xhu_exec_timeout_sec = LaunchConfiguration('xhu_exec_timeout_sec')
    xhu_hold_replan_timeout_sec = LaunchConfiguration('xhu_hold_replan_timeout_sec')

    return LaunchDescription([
        DeclareLaunchArgument('team', default_value='blue'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('execution_backend', default_value='nav2_follow_path'),
        DeclareLaunchArgument('xhu_exec_timeout_sec', default_value='45.0'),
        DeclareLaunchArgument('xhu_hold_replan_timeout_sec', default_value='2.0'),
        DeclareLaunchArgument(
            'graph_file',
            default_value='',
        ),

        Node(
            package='rc26_topo_nav',
            executable='topo_nav_node',
            name='topo_nav_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([pkg_dir, 'config', 'topo_nav.yaml']),
                {
                    'team': team,
                    'graph_file': graph_file,
                    'use_sim_time': use_sim_time,
                    'execution_backend': execution_backend,
                    'xhu.exec_timeout_sec': xhu_exec_timeout_sec,
                    'xhu.hold_replan_timeout_sec': xhu_hold_replan_timeout_sec,
                },
            ],
        ),
    ])
