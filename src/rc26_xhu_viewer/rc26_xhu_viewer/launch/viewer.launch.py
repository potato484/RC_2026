from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


VALID_LAYOUTS = {"operator", "engineering", "diagnostic"}


def _build_viewer(context, *args, **kwargs):
    layout = LaunchConfiguration("layout").perform(context).strip().lower()
    slam_enabled = LaunchConfiguration("slam").perform(context).strip().lower() == "true"
    mode = "slam" if slam_enabled else "navigation"
    if layout not in VALID_LAYOUTS:
        raise RuntimeError(
            f"invalid rc26_xhu_viewer layout '{layout}', expected one of {sorted(VALID_LAYOUTS)}"
        )

    share_dir = Path(get_package_share_directory("rc26_xhu_viewer"))
    config_path = share_dir / "config" / f"{mode}_{layout}.rviz"
    if not config_path.is_file():
        raise RuntimeError(f"viewer config does not exist: {config_path}")

    return [
        LogInfo(msg=f"[rc26_xhu_viewer] mode={mode} layout={layout} config={config_path}"),
        Node(
            package="rc26_xhu_viewer",
            executable="rc26_xhu_viewer",
            name="rc26_xhu_viewer",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            arguments=["-d", str(config_path)],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("slam", default_value="false"),
            DeclareLaunchArgument("layout", default_value="operator"),
            OpaqueFunction(function=_build_viewer),
        ]
    )
