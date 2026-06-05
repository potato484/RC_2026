"""
rc26_odom_interface 模块测试

功能: 复用 odometry.launch.py 的装配逻辑，仅启动 odom_interface
前置: 需要外部 Point-LIO 数据源、rosbag 回放或 mock publisher

测试指令:
    ros2 launch rc26_bringup test_odom_interface.launch.py

验证:
    ros2 topic echo /odom --once
    ros2 topic echo /registered_scan --once
    ros2 run tf2_ros tf2_echo odom base_link
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc26_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    point_lio_config_file = LaunchConfiguration('point_lio_config_file')
    sensor_extrinsics_file = LaunchConfiguration('sensor_extrinsics_file')
    sensor_extrinsics_profile = LaunchConfiguration('sensor_extrinsics_profile')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')
    declare_point_lio_config_file = DeclareLaunchArgument(
        'point_lio_config_file',
        default_value='',
        description='Point-LIO 参数文件路径；用于推导 odom_interface 内部 body 外参')
    declare_sensor_extrinsics_file = DeclareLaunchArgument(
        'sensor_extrinsics_file',
        default_value=PathJoinSubstitution([get_package_share_directory('rc26_sensor_extrinsics'), 'config', 'r2_sensor_extrinsics.yaml']),
        description='传感器安装外参 YAML 文件路径')
    declare_sensor_extrinsics_profile = DeclareLaunchArgument(
        'sensor_extrinsics_profile',
        default_value='',
        description='传感器安装外参 profile；空字符串表示使用 YAML defaults.active_profile')

    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_dir, 'launch', 'odometry.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'start_mid360_driver': 'false',
            'start_point_lio': 'false',
            'start_sensor_scan': 'false',
            'point_lio_config_file': point_lio_config_file,
            'sensor_extrinsics_file': sensor_extrinsics_file,
            'sensor_extrinsics_profile': sensor_extrinsics_profile,
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_point_lio_config_file,
        declare_sensor_extrinsics_file,
        declare_sensor_extrinsics_profile,
        odometry_launch,
    ])
