import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _create_point_lio_actions(context, *, namespace, point_lio_cfg_dir, remappings):
    namespace_value = namespace.perform(context)
    config_file = point_lio_cfg_dir.perform(context)

    if not os.path.exists(config_file):
        raise RuntimeError(f"Point-LIO 配置文件不存在: {config_file}")

    return [
        LogInfo(msg=f"[point_lio.launch] 使用配置 {config_file}"),
        Node(
            package="rc26_point_lio",
            executable="pointlio_mapping",
            namespace=namespace_value,
            parameters=[config_file],
            remappings=remappings,
            output="screen",
        ),
    ]


def generate_launch_description():
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    namespace = LaunchConfiguration("namespace")
    point_lio_cfg_dir = LaunchConfiguration("point_lio_cfg_dir")

    point_lio_dir = get_package_share_directory("rc26_point_lio")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the node",
    )

    declare_point_lio_cfg_dir = DeclareLaunchArgument(
        "point_lio_cfg_dir",
        default_value=PathJoinSubstitution([point_lio_dir, "config", "mid360.yaml"]),
        description="Path to the base Point-LIO config file",
    )

    start_point_lio_node = OpaqueFunction(
        function=lambda context: _create_point_lio_actions(
            context,
            namespace=namespace,
            point_lio_cfg_dir=point_lio_cfg_dir,
            remappings=remappings,
        )
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace)
    ld.add_action(declare_point_lio_cfg_dir)
    ld.add_action(start_point_lio_node)

    return ld
