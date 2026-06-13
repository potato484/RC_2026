"""
R2 导航联调入口

功能:
  - 复用 bringup.launch.py
  - 固定默认关闭 decision，保留定位 + Nav2 基础导航链
  - 默认加载 navigation_default.rviz 打开 RViz2，便于现场观察定位、代价地图和路径

验证:
  - ros2 node list | grep rviz2
  - ros2 topic echo /sensor_scan --once
  - ros2 action info /navigate_to_pose
  - ros2 lifecycle get /controller_server
  - ros2 lifecycle get /bt_navigator
  - ros2 topic echo /cmd_vel
  - ros2 topic echo /merge_odom
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc26_bringup')

    runtime_config_file = LaunchConfiguration('runtime_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    prior_pcd_file = LaunchConfiguration('prior_pcd_file')
    recover_mid360_stream = LaunchConfiguration('recover_mid360_stream')
    team = LaunchConfiguration('team')
    localization_params_file = LaunchConfiguration('localization_params_file')
    nav2_map_file = LaunchConfiguration('nav2_map_file')
    start_pose_sender = LaunchConfiguration('start_pose_sender')
    pose_sender_feedback_serial_port = LaunchConfiguration('pose_sender_feedback_serial_port')
    pose_sender_target_serial_port = LaunchConfiguration('pose_sender_target_serial_port')
    pose_sender_baudrate = LaunchConfiguration('pose_sender_baudrate')
    merge_odom_use_can_odom = LaunchConfiguration('merge_odom_use_can_odom')
    merge_odom_require_output = LaunchConfiguration('merge_odom_require_output')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    declare_runtime_config_file = DeclareLaunchArgument(
        'runtime_config_file',
        default_value=PathJoinSubstitution([bringup_dir, 'config', 'r2_runtime.yaml']),
        description='R2 统一运行配置 YAML；点云、地图、行为树路径必须为绝对路径')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')
    declare_prior_pcd_file = DeclareLaunchArgument(
        'prior_pcd_file',
        default_value='',
        description='导航所用先验点云绝对路径；空字符串表示使用 r2_runtime.yaml')
    declare_recover_mid360_stream = DeclareLaunchArgument(
        'recover_mid360_stream',
        default_value='false',
        description='启动前先运行 Mid-360 恢复脚本')
    declare_team = DeclareLaunchArgument(
        'team',
        default_value='blue',
        description='当前比赛侧别: blue | red')
    declare_localization_params_file = DeclareLaunchArgument(
        'localization_params_file',
        default_value=PathJoinSubstitution([bringup_dir, 'config', 'localization.yaml']),
        description='定位参数文件路径')
    declare_nav2_map_file = DeclareLaunchArgument(
        'nav2_map_file',
        default_value='',
        description='Nav2 map_server 使用的 2D occupancy map YAML 绝对路径；空字符串表示使用 r2_runtime.yaml')
    declare_start_pose_sender = DeclareLaunchArgument(
        'start_pose_sender',
        default_value='true',
        description='是否启动 merge_odom 底盘执行链，把 /cmd_vel 接到底盘执行桥')
    declare_pose_sender_feedback_serial_port = DeclareLaunchArgument(
        'pose_sender_feedback_serial_port',
        default_value='__disabled__',
        description='保留的独立反馈串口参数；当前默认停用，WheelOdom 走 target 单口')
    declare_pose_sender_target_serial_port = DeclareLaunchArgument(
        'pose_sender_target_serial_port',
        default_value='/dev/ttyUSB0',
        description='merge_odom 目标串口')
    declare_pose_sender_baudrate = DeclareLaunchArgument(
        'pose_sender_baudrate',
        default_value='1000000',
        description='merge_odom 目标串口波特率')
    declare_merge_odom_use_can_odom = DeclareLaunchArgument(
        'merge_odom_use_can_odom',
        default_value='',
        description='覆盖 r2_runtime.yaml: 是否选择 CAN 里程计作为 /merge_odom 源')
    declare_merge_odom_require_output = DeclareLaunchArgument(
        'merge_odom_require_output',
        default_value='',
        description='覆盖 r2_runtime.yaml: 是否要求真实 odom 源发布稳定 /merge_odom')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='是否随导航联调入口启动 RViz2')
    declare_rviz_config_file = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([bringup_dir, 'rviz', 'navigation_default.rviz']),
        description='RViz2 配置文件路径')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_dir, 'launch', 'bringup.launch.py'])
        ),
        launch_arguments={
            'runtime_config_file': runtime_config_file,
            'use_sim_time': use_sim_time,
            'run_mode': 'navigation',
            'use_decision': 'false',
            'prior_pcd_file': prior_pcd_file,
            'recover_mid360_stream': recover_mid360_stream,
            'team': team,
            'localization_params_file': localization_params_file,
            'nav2_map_file': nav2_map_file,
            'start_pose_sender': start_pose_sender,
            'pose_sender_feedback_serial_port': pose_sender_feedback_serial_port,
            'pose_sender_target_serial_port': pose_sender_target_serial_port,
            'pose_sender_baudrate': pose_sender_baudrate,
            'merge_odom_use_can_odom': merge_odom_use_can_odom,
            'merge_odom_require_output': merge_odom_require_output,
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_runtime_config_file,
        declare_use_sim_time,
        declare_prior_pcd_file,
        declare_recover_mid360_stream,
        declare_team,
        declare_localization_params_file,
        declare_nav2_map_file,
        declare_start_pose_sender,
        declare_pose_sender_feedback_serial_port,
        declare_pose_sender_target_serial_port,
        declare_pose_sender_baudrate,
        declare_merge_odom_use_can_odom,
        declare_merge_odom_require_output,
        declare_use_rviz,
        declare_rviz_config_file,
        bringup_launch,
        rviz_node,
    ])
