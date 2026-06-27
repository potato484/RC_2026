"""
RealSense D455 启动封装（ROS2 Humble / realsense2_camera）。

目的：
- 把相机作为 rc26_vision 的可测试组件：vision 模块可单独启动，不依赖 rc26_decision/rc26_bringup。

说明：
- 默认读取本包 `config/realsense_d455.yaml`。
- 该 launch 仅负责启动 realsense2_camera，不包含任何视觉推理节点。
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node


<<<<<<< HEAD
R2_D455_SERIAL_NO = "239222303644"


def _normalize_serial_no(value: str) -> str:
    serial = str(value).strip()
    if serial in ("", "''", '""'):
        return ""
    if len(serial) >= 2 and serial[0] == serial[-1] and serial[0] in ("'", '"'):
        serial = serial[1:-1].strip()
    if serial.startswith("_"):
        serial = serial[1:].strip()
    return serial


=======
>>>>>>> e6cc0c8da891eafb82dd36f73c88c3e86e9b6c22
def generate_launch_description():
    vision_dir = get_package_share_directory("rc26_vision")
    realsense_dir = get_package_share_directory("realsense2_camera")

    config_file = LaunchConfiguration("config_file")
    serial_no = LaunchConfiguration("serial_no")
    camera_name = LaunchConfiguration("camera_name")
    camera_namespace = LaunchConfiguration("camera_namespace")
    device_type = LaunchConfiguration("device_type")
    suppress_startup_warnings = LaunchConfiguration("suppress_startup_warnings")
    log_level = LaunchConfiguration("log_level")
    librealsense_log_level = LaunchConfiguration("librealsense_log_level")

    declare_config_file = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([vision_dir, "config", "realsense_d455.yaml"]),
        description="realsense2_camera 的 YAML 配置文件路径",
    )
    declare_serial_no = DeclareLaunchArgument(
        "serial_no",
<<<<<<< HEAD
        default_value=R2_D455_SERIAL_NO,
        description=f"RealSense 序列号（默认固定 R2 D455: {R2_D455_SERIAL_NO}）",
=======
        default_value="''",
        description="RealSense 序列号（空表示自动选择）",
>>>>>>> e6cc0c8da891eafb82dd36f73c88c3e86e9b6c22
    )
    declare_camera_name = DeclareLaunchArgument(
        "camera_name",
        default_value="camera",
        description="realsense2_camera 节点名",
    )
    declare_camera_namespace = DeclareLaunchArgument(
        "camera_namespace",
        default_value="camera",
        description="realsense2_camera 的 namespace（相对）",
    )
    declare_device_type = DeclareLaunchArgument(
        "device_type",
        default_value="d455",
        description="设备型号选择（默认 d455）",
    )
    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="realsense2_camera 日志等级 [debug|info|warn|error|fatal]",
    )
    declare_librealsense_log_level = DeclareLaunchArgument(
        "librealsense_log_level",
        default_value="ERROR",
        description="librealsense 日志等级（环境变量 LRS_LOG_LEVEL）",
    )
    declare_suppress_startup_warnings = DeclareLaunchArgument(
        "suppress_startup_warnings",
        default_value="true",
        description="压制已知无害启动告警（将 logger 'camera.camera' 设为 ERROR）",
    )

    def _yaml_to_dict(path_to_yaml: str) -> dict:
        with open(path_to_yaml, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    def _launch_setup(context, *args, **kwargs):
        config_path = config_file.perform(context)
        params_from_file = {} if config_path == "''" else _yaml_to_dict(config_path)

        # 与 upstream 行为一致：根据 realsense2_camera 的 global_settings.yaml 决定是否使用 LifecycleNode
        lifecycle_param_file = os.path.join(realsense_dir, "config", "global_settings.yaml")
        try:
            lifecycle_params = _yaml_to_dict(lifecycle_param_file)
        except FileNotFoundError:
            lifecycle_params = {}
        use_lifecycle_node = bool(lifecycle_params.get("use_lifecycle_node", False))

<<<<<<< HEAD
        requested_serial_no = _normalize_serial_no(serial_no.perform(context))
        overrides = {
            "serial_no": requested_serial_no,
=======
        overrides = {
            "serial_no": serial_no.perform(context),
>>>>>>> e6cc0c8da891eafb82dd36f73c88c3e86e9b6c22
            "device_type": device_type.perform(context),
            "camera_name": camera_name.perform(context),
            "camera_namespace": camera_namespace.perform(context),
        }
<<<<<<< HEAD
        if not overrides["serial_no"]:
=======
        if overrides["serial_no"] == "''":
>>>>>>> e6cc0c8da891eafb82dd36f73c88c3e86e9b6c22
            overrides.pop("serial_no", None)
        if overrides["device_type"] == "''":
            overrides.pop("device_type", None)

        ros_args = ["--ros-args", "--log-level", log_level.perform(context)]
        if suppress_startup_warnings.perform(context).lower() in ("true", "1", "yes", "on"):
            ros_args += ["--log-level", "camera.camera:=error"]

        node_action = LifecycleNode if use_lifecycle_node else Node
        return [
            node_action(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                namespace=camera_namespace,
                name=camera_name,
                output="screen",
                parameters=[overrides, params_from_file],
                arguments=ros_args,
            )
        ]

    return LaunchDescription(
        [
            declare_config_file,
            declare_serial_no,
            declare_camera_name,
            declare_camera_namespace,
            declare_device_type,
            declare_log_level,
            declare_suppress_startup_warnings,
            declare_librealsense_log_level,
            SetEnvironmentVariable(name="LRS_LOG_LEVEL", value=librealsense_log_level),
            OpaqueFunction(function=_launch_setup),
        ]
    )
<<<<<<< HEAD
=======

>>>>>>> e6cc0c8da891eafb82dd36f73c88c3e86e9b6c22
