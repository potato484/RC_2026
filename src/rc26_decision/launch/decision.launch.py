import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 获取包共享目录路径
    pkg_share = get_package_share_directory('rc26_decision')
    vision_share = get_package_share_directory('rc26_vision')
    
    # 默认配置文件路径
    default_config_file = os.path.join(pkg_share, 'config', 'decision_params.yaml')
    default_vision_config_file = os.path.join(vision_share, 'config', 'vision_models.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_config_file,
            description='ROS2 配置文件路径 (YAML)'
        ),
        DeclareLaunchArgument(
            'tree_file',
            default_value='main_tree.xml',
            description='行为树 XML 文件名 (如果 YAML 中未指定则使用此默认值)'
        ),
        DeclareLaunchArgument(
            'enable_vision',
            default_value='false',
            description='是否启用 rc26_vision 视觉推理（AidLite + ONNX/CPU，本地链路）'
        ),
        DeclareLaunchArgument(
            'vision_config_file',
            default_value=default_vision_config_file,
            description='rc26_vision 模型配置 YAML 路径（默认使用安装后的 vision_models.yaml）'
        ),
        Node(
            package='rc26_decision',
            executable='decision_node',
            name='rc26_decision',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'tree_file': LaunchConfiguration('tree_file'),
                    'enable_vision': LaunchConfiguration('enable_vision'),
                    'vision_config_file': LaunchConfiguration('vision_config_file'),
                }
            ]
        ),
    ])
