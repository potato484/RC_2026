from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("rc26_vision")
    default_params = PathJoinSubstitution(
        [pkg_share, "config", "tip_vision_params.yaml"]
    )
    default_vision_config = PathJoinSubstitution(
        [pkg_share, "config", "vision_models.yaml"]
    )

    params_arg = DeclareLaunchArgument("params_file", default_value=default_params)
    model_id_arg = DeclareLaunchArgument("model_id", default_value="tip_default")

    node = Node(
        package="rc26_vision",
        executable="tip_vision_test_node",
        name="tip_vision_test_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "vision_config_file": default_vision_config,
                "model_id": LaunchConfiguration("model_id"),
            },
        ],
    )

    return LaunchDescription([params_arg, model_id_arg, node])
