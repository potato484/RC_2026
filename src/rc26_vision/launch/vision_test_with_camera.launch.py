"""
rc26_vision 独立联调入口：启动 RealSense + vision_test_node。

用法示例：
- ros2 launch rc26_vision vision_test_with_camera.launch.py
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    vision_dir = get_package_share_directory("rc26_vision")

    vision_config_file = LaunchConfiguration("vision_config_file")

    declare_vision_config_file = DeclareLaunchArgument(
        "vision_config_file",
        default_value=PathJoinSubstitution([vision_dir, "config", "vision_models.yaml"]),
        description="rc26_vision 模型配置 YAML（vision.default_model / vision.models.*）",
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([vision_dir, "launch", "realsense_d455.launch.py"])
        ),
        launch_arguments={
            "config_file": PathJoinSubstitution([vision_dir, "config", "realsense_d455.yaml"]),
        }.items(),
    )

    vision_test_node = Node(
        package="rc26_vision",
        executable="vision_test_node",
        name="vision_test_node",
        output="screen",
        parameters=[
            {
                "vision_config_file": vision_config_file,
            }
        ],
    )

    return LaunchDescription(
        [
            declare_vision_config_file,
            realsense_launch,
            vision_test_node,
        ]
    )

