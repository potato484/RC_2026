"""
RealSense D455 bringup (ROS2 Humble).

This launch file wraps `realsense2_camera` so it can be treated as a project component.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node


R2_D455_SERIAL_NO = '239222303644'


def _normalize_serial_no(value: str) -> str:
    serial = str(value).strip()
    if serial in ('', "''", '""'):
        return R2_D455_SERIAL_NO
    if len(serial) >= 2 and serial[0] == serial[-1] and serial[0] in ("'", '"'):
        serial = serial[1:-1].strip()
    if serial.startswith('_'):
        serial = serial[1:].strip()
    return serial or R2_D455_SERIAL_NO


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc26_bringup')
    realsense_dir = get_package_share_directory('realsense2_camera')

    config_file = LaunchConfiguration('config_file')
    serial_no = LaunchConfiguration('serial_no')
    camera_name = LaunchConfiguration('camera_name')
    camera_namespace = LaunchConfiguration('camera_namespace')
    device_type = LaunchConfiguration('device_type')
    suppress_startup_warnings = LaunchConfiguration('suppress_startup_warnings')
    log_level = LaunchConfiguration('log_level')
    librealsense_log_level = LaunchConfiguration('librealsense_log_level')

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([bringup_dir, 'config', 'realsense_d455.yaml']),
        description='YAML config file for realsense2_camera',
    )
    declare_serial_no = DeclareLaunchArgument(
        'serial_no',
        default_value=R2_D455_SERIAL_NO,
        description=f"RealSense serial number (default fixed R2 D455: {R2_D455_SERIAL_NO})",
    )
    declare_camera_name = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='realsense2_camera node name',
    )
    declare_camera_namespace = DeclareLaunchArgument(
        'camera_namespace',
        default_value='camera',
        description='realsense2_camera namespace (relative)',
    )
    declare_device_type = DeclareLaunchArgument(
        'device_type',
        default_value='d455',
        description='Device type selector (default: d455)',
    )
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='realsense2_camera log level [debug|info|warn|error|fatal]',
    )
    declare_librealsense_log_level = DeclareLaunchArgument(
        'librealsense_log_level',
        default_value='ERROR',
        description='Librealsense log severity (env: LRS_LOG_LEVEL). Typical: NONE|DEBUG|INFO|WARN|ERROR|FATAL',
    )

    declare_suppress_startup_warnings = DeclareLaunchArgument(
        'suppress_startup_warnings',
        default_value='true',
        description="Suppress known harmless realsense2_camera startup warnings (sets logger 'camera.camera' to ERROR)",
    )

    def _yaml_to_dict(path_to_yaml: str) -> dict:
        with open(path_to_yaml, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f) or {}

    def _launch_setup(context, *args, **kwargs):
        config_path = config_file.perform(context)
        params_from_file = {} if config_path == "''" else _yaml_to_dict(config_path)

        # Keep behavior consistent with upstream rs_launch.py: switch to LifecycleNode if configured.
        lifecycle_param_file = os.path.join(
            realsense_dir,
            'config',
            'global_settings.yaml',
        )
        try:
            lifecycle_params = _yaml_to_dict(lifecycle_param_file)
        except FileNotFoundError:
            lifecycle_params = {}
        use_lifecycle_node = bool(lifecycle_params.get('use_lifecycle_node', False))

        requested_serial_no = _normalize_serial_no(serial_no.perform(context))
        overrides = {
            'serial_no': requested_serial_no,
            'device_type': device_type.perform(context),
            'camera_name': camera_name.perform(context),
            'camera_namespace': camera_namespace.perform(context),
        }
        if overrides['device_type'] == "''":
            overrides.pop('device_type', None)

        ros_args = ['--ros-args', '--log-level', log_level.perform(context)]
        if suppress_startup_warnings.perform(context).lower() in ('true', '1', 'yes', 'on'):
            # The "re-enable the stream for the change to take effect." message is emitted from a child logger.
            ros_args += ['--log-level', 'camera.camera:=error']

        node_action = LifecycleNode if use_lifecycle_node else Node
        return [
            node_action(
                package='realsense2_camera',
                executable='realsense2_camera_node',
                namespace=camera_namespace,
                name=camera_name,
                output='screen',
                parameters=[overrides, params_from_file],
                arguments=ros_args,
            )
        ]

    return LaunchDescription([
        declare_config_file,
        declare_serial_no,
        declare_camera_name,
        declare_camera_namespace,
        declare_device_type,
        declare_log_level,
        declare_suppress_startup_warnings,
        declare_librealsense_log_level,
        SetEnvironmentVariable(
            name='LRS_LOG_LEVEL',
            value=librealsense_log_level,
        ),
        OpaqueFunction(function=_launch_setup),
    ])
