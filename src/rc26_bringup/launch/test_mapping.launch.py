"""
R2 建图联调入口

功能:
  - 复用 bringup.launch.py 的纯建图最小链路
  - 固定默认开启 run_mode:=mapping 与 pure_mapping_mode:=true
  - 默认加载 slam.rviz 打开 RViz2，便于现场直接观察建图结果

验证:
  - ros2 topic hz /state_estimation
  - ros2 topic hz /registered_scan
  - ros2 topic echo /Laser_map --once --field header.frame_id
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
    sensor_extrinsics_dir = get_package_share_directory('rc26_sensor_extrinsics')

    use_sim_time = LaunchConfiguration('use_sim_time')
    sensor_extrinsics_file = LaunchConfiguration('sensor_extrinsics_file')
    sensor_extrinsics_profile = LaunchConfiguration('sensor_extrinsics_profile')
    point_lio_config_file = LaunchConfiguration('point_lio_config_file')
    recover_mid360_stream = LaunchConfiguration('recover_mid360_stream')
    run_mode = LaunchConfiguration('run_mode')
    pure_mapping_mode = LaunchConfiguration('pure_mapping_mode')
    use_decision = LaunchConfiguration('use_decision')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')

    declare_sensor_extrinsics_file = DeclareLaunchArgument(
        'sensor_extrinsics_file',
        default_value=PathJoinSubstitution([sensor_extrinsics_dir, 'config', 'r2_sensor_extrinsics.yaml']),
        description='传感器安装外参 YAML 文件路径')

    declare_sensor_extrinsics_profile = DeclareLaunchArgument(
        'sensor_extrinsics_profile',
        default_value='',
        description='传感器安装外参 profile；空字符串表示使用 YAML defaults.active_profile')

    declare_point_lio_config_file = DeclareLaunchArgument(
        'point_lio_config_file',
        default_value='',
        description='Point-LIO 参数文件路径；为空时使用 rc26_point_lio/config/mid360.yaml')

    declare_recover_mid360_stream = DeclareLaunchArgument(
        'recover_mid360_stream',
        default_value='false',
        description='启动前先运行 Mid-360 恢复脚本')

    declare_run_mode = DeclareLaunchArgument(
        'run_mode',
        default_value='mapping',
        description='运行模式: navigation | mapping；建图联调入口默认 mapping')

    declare_pure_mapping_mode = DeclareLaunchArgument(
        'pure_mapping_mode',
        default_value='true',
        description='纯建图最小模式；建图联调入口默认开启')

    declare_use_decision = DeclareLaunchArgument(
        'use_decision',
        default_value='false',
        description='是否启动决策系统；建图联调入口默认关闭')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='是否随建图调试入口启动 RViz2')

    declare_rviz_config_file = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([bringup_dir, 'rviz', 'slam.rviz']),
        description='RViz2 配置文件路径')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_dir, 'launch', 'bringup.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'run_mode': run_mode,
            'pure_mapping_mode': pure_mapping_mode,
            'sensor_extrinsics_file': sensor_extrinsics_file,
            'sensor_extrinsics_profile': sensor_extrinsics_profile,
            'point_lio_config_file': point_lio_config_file,
            'use_decision': use_decision,
            'recover_mid360_stream': recover_mid360_stream,
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
        declare_use_sim_time,
        declare_sensor_extrinsics_file,
        declare_sensor_extrinsics_profile,
        declare_point_lio_config_file,
        declare_recover_mid360_stream,
        declare_run_mode,
        declare_pure_mapping_mode,
        declare_use_decision,
        declare_use_rviz,
        declare_rviz_config_file,
        bringup_launch,
        rviz_node,
    ])
