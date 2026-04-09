import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _resolve_point_lio_profile(requested_profile):
    profile_aliases = {
        "default": "base",
    }
    profile_overrides = {
        "base": {},
        "cruise_light": {
            "publish.map_full_publish_en": False,
            "publish.map_full_publish_interval_sec": 1.5,
        },
        "mapping_dense": {
            "point_keep_ratio": 100.0,
            "filter_size_surf": 0.1,
            "filter_size_map": 0.1,
            "pcd_save.pcd_save_en": True,
        },
    }

    normalized_profile = requested_profile.strip().lower() or "base"
    resolved_profile = profile_aliases.get(normalized_profile, normalized_profile)
    if resolved_profile not in profile_overrides:
        supported_profiles = "base | cruise_light | mapping_dense"
        raise RuntimeError(
            f"不支持的 point_lio_profile={requested_profile}，可选: {supported_profiles}"
        )
    return resolved_profile, profile_overrides[resolved_profile]


def _create_point_lio_actions(context, *, namespace, point_lio_cfg_dir, point_lio_profile, remappings):
    namespace_value = namespace.perform(context)
    config_file = point_lio_cfg_dir.perform(context)
    resolved_profile, profile_overrides = _resolve_point_lio_profile(
        point_lio_profile.perform(context)
    )

    if not os.path.exists(config_file):
        raise RuntimeError(f"Point-LIO 配置文件不存在: {config_file}")

    parameters = [config_file]
    if profile_overrides:
        parameters.append(profile_overrides)

    return [
        LogInfo(msg=f"[point_lio.launch] 使用 profile:{resolved_profile}，基础配置 {config_file}"),
        Node(
            package="rc26_point_lio",
            executable="pointlio_mapping",
            namespace=namespace_value,
            parameters=parameters,
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
    use_rviz = LaunchConfiguration("rviz")
    point_lio_cfg_dir = LaunchConfiguration("point_lio_cfg_dir")
    point_lio_profile = LaunchConfiguration("point_lio_profile")

    point_lio_dir = get_package_share_directory("rc26_point_lio")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the node",
    )

    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="兼容参数；不再直接启动可视化。若需 GUI，请改用 rc26_bringup/odometry.launch.py 或直接运行 rviz2。",
    )

    declare_point_lio_cfg_dir = DeclareLaunchArgument(
        "point_lio_cfg_dir",
        default_value=PathJoinSubstitution([point_lio_dir, "config", "mid360.yaml"]),
        description="Path to the base Point-LIO config file",
    )

    declare_point_lio_profile = DeclareLaunchArgument(
        "point_lio_profile",
        default_value="base",
        description="Point-LIO 预设: base | cruise_light | mapping_dense",
    )

    start_point_lio_node = OpaqueFunction(
        function=lambda context: _create_point_lio_actions(
            context,
            namespace=namespace,
            point_lio_cfg_dir=point_lio_cfg_dir,
            point_lio_profile=point_lio_profile,
            remappings=remappings,
        )
    )

    rviz_deprecation_notice = LogInfo(
        condition=IfCondition(use_rviz),
        msg=(
            "[point_lio.launch] rviz:=true 已失效；该入口已改为纯 headless。"
            " 如需调试可视化，请使用 `ros2 launch rc26_bringup odometry.launch.py odometry_use_rviz:=true`"
            " 或直接运行 `ros2 run rviz2 rviz2 --rc26-mode navigation --rc26-layout diagnostic`。"
        ),
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace)
    ld.add_action(declare_rviz)
    ld.add_action(declare_point_lio_cfg_dir)
    ld.add_action(declare_point_lio_profile)
    ld.add_action(start_point_lio_node)
    ld.add_action(rviz_deprecation_notice)

    return ld
