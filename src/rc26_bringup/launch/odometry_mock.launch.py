from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("rc26_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_map_to_odom = LaunchConfiguration("publish_map_to_odom")
    map_to_odom_x = LaunchConfiguration("map_to_odom_x")
    map_to_odom_y = LaunchConfiguration("map_to_odom_y")
    map_to_odom_z = LaunchConfiguration("map_to_odom_z")
    map_to_odom_roll = LaunchConfiguration("map_to_odom_roll")
    map_to_odom_pitch = LaunchConfiguration("map_to_odom_pitch")
    map_to_odom_yaw = LaunchConfiguration("map_to_odom_yaw")
    mock_rate_hz = LaunchConfiguration("mock_rate_hz")
    mock_obstacle_enable = LaunchConfiguration("mock_obstacle_enable")
    mock_ground_enable = LaunchConfiguration("mock_ground_enable")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="使用仿真时间",
    )
    declare_publish_map_to_odom = DeclareLaunchArgument(
        "publish_map_to_odom",
        default_value="true",
        description="是否发布 map->odom 静态变换",
    )
    declare_map_to_odom_x = DeclareLaunchArgument(
        "map_to_odom_x",
        default_value="0.0",
        description="map->odom x",
    )
    declare_map_to_odom_y = DeclareLaunchArgument(
        "map_to_odom_y",
        default_value="0.0",
        description="map->odom y",
    )
    declare_map_to_odom_z = DeclareLaunchArgument(
        "map_to_odom_z",
        default_value="0.0",
        description="map->odom z",
    )
    declare_map_to_odom_roll = DeclareLaunchArgument(
        "map_to_odom_roll",
        default_value="0.0",
        description="map->odom roll",
    )
    declare_map_to_odom_pitch = DeclareLaunchArgument(
        "map_to_odom_pitch",
        default_value="0.0",
        description="map->odom pitch",
    )
    declare_map_to_odom_yaw = DeclareLaunchArgument(
        "map_to_odom_yaw",
        default_value="0.0",
        description="map->odom yaw",
    )
    declare_mock_rate_hz = DeclareLaunchArgument(
        "mock_rate_hz",
        default_value="10.0",
        description="mock_point_lio 发布频率",
    )
    declare_mock_obstacle_enable = DeclareLaunchArgument(
        "mock_obstacle_enable",
        default_value="true",
        description="是否发布障碍点",
    )
    declare_mock_ground_enable = DeclareLaunchArgument(
        "mock_ground_enable",
        default_value="true",
        description="是否发布地面点",
    )

    odom_interface_config = PathJoinSubstitution([bringup_dir, "config", "odom_interface.yaml"])
    sensor_scan_config = PathJoinSubstitution([bringup_dir, "config", "sensor_scan_generation.yaml"])
    mock_script = PathJoinSubstitution([bringup_dir, "scripts", "mock_point_lio.py"])

    static_tf_livox = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_livox",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0.13",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "livox_frame",
        ],
    )

    static_tf_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_to_odom",
        arguments=[
            "--x",
            map_to_odom_x,
            "--y",
            map_to_odom_y,
            "--z",
            map_to_odom_z,
            "--roll",
            map_to_odom_roll,
            "--pitch",
            map_to_odom_pitch,
            "--yaw",
            map_to_odom_yaw,
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
        condition=IfCondition(publish_map_to_odom),
    )

    odom_interface_node = Node(
        package="rc26_odom_interface",
        executable="rc26_odom_interface_node",
        name="odom_interface",
        output="screen",
        parameters=[
            odom_interface_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    sensor_scan_node = Node(
        package="rc26_sensor_scan",
        executable="rc26_sensor_scan_node",
        name="sensor_scan",
        output="screen",
        parameters=[
            sensor_scan_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    mock_point_lio = ExecuteProcess(
        cmd=[
            "python3",
            mock_script,
            "--rate_hz",
            mock_rate_hz,
            "--obstacle_enable",
            mock_obstacle_enable,
            "--ground_enable",
            mock_ground_enable,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_publish_map_to_odom,
            declare_map_to_odom_x,
            declare_map_to_odom_y,
            declare_map_to_odom_z,
            declare_map_to_odom_roll,
            declare_map_to_odom_pitch,
            declare_map_to_odom_yaw,
            declare_mock_rate_hz,
            declare_mock_obstacle_enable,
            declare_mock_ground_enable,
            static_tf_map_to_odom,
            static_tf_livox,
            odom_interface_node,
            sensor_scan_node,
            mock_point_lio,
        ]
    )
