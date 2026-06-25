"""
rc26_vision 独立联调入口：启动 RealSense D455 + kfs_vision_test_node（kfs.onnx 主链 + 实时 overlay）。

用法示例：
- ros2 launch rc26_vision test_kfs_vision.launch.py
- ros2 launch rc26_vision test_kfs_vision.launch.py show_window:=false   # 无显示环境
- ros2 launch rc26_vision test_kfs_vision.launch.py window_name:="my kfs"
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    vision_dir = get_package_share_directory("rc26_vision")

    vision_config_file = LaunchConfiguration("vision_config_file")
    show_window = LaunchConfiguration("show_window")
    window_name = LaunchConfiguration("window_name")

    declare_vision_config_file = DeclareLaunchArgument(
        "vision_config_file",
        default_value=PathJoinSubstitution([vision_dir, "config", "vision_models.yaml"]),
        description="rc26_vision 模型配置 YAML（vision.default_model / vision.models.*）",
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

    kfs_vision_test_node = Node(
        package="rc26_vision",
        executable="kfs_vision_test_node",
        name="kfs_vision_test_node",
        output="screen",
        parameters=[
            {
                "vision_config_file": vision_config_file,
                # show_window 节点端声明为 bool，LaunchConfiguration 解析为字符串，
                # 必须用 ParameterValue 显式转 bool，否则参数类型不匹配。
                "show_window": ParameterValue(show_window, value_type=bool),
                "window_name": window_name,
            }
        ],
    )

    return LaunchDescription(
        [
            declare_vision_config_file,
            declare_show_window,
            declare_window_name,
            realsense_launch,
            kfs_vision_test_node,
        ]
    )
