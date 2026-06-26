"""
rc26_vision 独立联调入口：启动 RealSense D455 + kfs_vision_test_node（kfs.onnx 主链 + 实时 overlay）。

用法示例：
- ros2 launch rc26_vision test_kfs_vision.launch.py
- ros2 launch rc26_vision test_kfs_vision.launch.py show_window:=false   # 无显示环境
- ros2 launch rc26_vision test_kfs_vision.launch.py window_name:="my kfs"
- ros2 launch rc26_vision test_kfs_vision.launch.py action_enable:=true direction:=up
- ros2 launch rc26_vision test_kfs_vision.launch.py action_enable:=true direction:=down
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _parse_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _launch_value(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def _should_start_mcu_transport(context):
    override = _launch_value(context, "start_mcu_transport").lower()
    if override == "auto":
        return _parse_bool(_launch_value(context, "action_enable"))
    if override in ("1", "true", "yes", "on"):
        return True
    if override in ("0", "false", "no", "off"):
        return False
    raise RuntimeError("start_mcu_transport must be auto, true, or false")


def _validate_direction(context):
    direction = _launch_value(context, "direction").lower()
    if direction not in ("up", "down"):
        raise RuntimeError(f"direction must be up or down; got: {direction}")
    return direction


def _create_runtime_actions(context):
    direction = _validate_direction(context)
    action_enable = _parse_bool(_launch_value(context, "action_enable"))
    cmd_vel_topic = _launch_value(context, "cmd_vel_topic")

    actions = []

    if _should_start_mcu_transport(context):
        mcu_transport_dir = get_package_share_directory("rc26_mcu_transport")
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([mcu_transport_dir, "launch", "mcu_transport.launch.py"])
            ),
            launch_arguments={
                "target_serial_port": LaunchConfiguration("target_serial_port"),
                "target_baudrate": LaunchConfiguration("target_baudrate"),
                "enable_chassis_cmd_vel_consumer": "true",
                "chassis_cmd_vel_topic": cmd_vel_topic,
            }.items(),
        ))

    actions.append(Node(
        package="rc26_vision",
        executable="kfs_vision_test_node",
        name="kfs_vision_test_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "vision_config_file": LaunchConfiguration("vision_config_file"),
                "model_id": LaunchConfiguration("model_id"),
                # show_window 节点端声明为 bool，LaunchConfiguration 解析为字符串，
                # 必须用 ParameterValue 显式转 bool，否则参数类型不匹配。
                "show_window": ParameterValue(LaunchConfiguration("show_window"), value_type=bool),
                "window_name": LaunchConfiguration("window_name"),
                "kfs_action_enable": action_enable,
                "kfs_action_direction": direction,
                "kfs_action_cmd_vel_topic": cmd_vel_topic,
            },
        ],
    ))

    return actions


def generate_launch_description():
    vision_dir = get_package_share_directory("rc26_vision")

    declare_vision_config_file = DeclareLaunchArgument(
        "vision_config_file",
        default_value=PathJoinSubstitution([vision_dir, "config", "vision_models.yaml"]),
        description="rc26_vision 模型配置 YAML（vision.default_model / vision.models.*）",
    )
    declare_model_id = DeclareLaunchArgument(
        "model_id",
        default_value="kfs_default",
        description="KFS 模型 profile ID",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([vision_dir, "config", "kfs_vision_params.yaml"]),
        description="KFS 独立视觉/动作测试参数 YAML",
    )
    declare_show_window = DeclareLaunchArgument(
        "show_window",
        default_value="true",
        description="是否打开 OpenCV overlay 窗口（无显示环境设 false）",
    )
    declare_window_name = DeclareLaunchArgument(
        "window_name",
        default_value="KFS Vision - kfs.onnx",
        description="OpenCV 窗口标题",
    )
    declare_action_enable = DeclareLaunchArgument(
        "action_enable",
        default_value="false",
        description="是否启用 KFS 底盘/机构动作链，默认关闭",
    )
    declare_direction = DeclareLaunchArgument(
        "direction",
        default_value="up",
        description="KFS 夹取方向: up -> GRAB_KFS_UP(0x03), down -> GRAB_KFS_DOWN(0x02)",
    )
    declare_start_mcu_transport = DeclareLaunchArgument(
        "start_mcu_transport",
        default_value="auto",
        description="是否启动 rc26_mcu_transport: auto | true | false；auto 仅在 action_enable=true 时启动",
    )
    declare_cmd_vel_topic = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="cmd_vel",
        description="KFS 测试节点发布速度、mcu_transport 消费速度使用的话题",
    )
    declare_target_serial_port = DeclareLaunchArgument(
        "target_serial_port",
        default_value="/dev/ttyUSB0",
        description="自动启动 rc26_mcu_transport 时使用的目标 MCU 串口",
    )
    declare_target_baudrate = DeclareLaunchArgument(
        "target_baudrate",
        default_value="1000000",
        description="自动启动 rc26_mcu_transport 时使用的目标 MCU 波特率",
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([vision_dir, "launch", "realsense_d455.launch.py"])
        ),
        launch_arguments={
            "config_file": PathJoinSubstitution([vision_dir, "config", "realsense_d455.yaml"]),
            # realsense_d455.launch 默认 camera_namespace=camera + name=camera 会产生
            # 双层 /camera/camera/... topic；这里去掉 namespace，让相机发布单层
            # /camera/color/image_raw 等，匹配 VisionInferenceManager 的默认订阅。
            "camera_namespace": "",
        }.items(),
    )

    runtime_actions = OpaqueFunction(function=_create_runtime_actions)

    return LaunchDescription(
        [
            declare_vision_config_file,
            declare_model_id,
            declare_params_file,
            declare_show_window,
            declare_window_name,
            declare_action_enable,
            declare_direction,
            declare_start_mcu_transport,
            declare_cmd_vel_topic,
            declare_target_serial_port,
            declare_target_baudrate,
            realsense_launch,
            runtime_actions,
        ]
    )
