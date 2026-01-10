"""
RC_2026_1 R2导航系统 - 主启动文件

启动:
  - 里程计 (point_lio + rc26_odom_interface + rc26_sensor_scan)
  - 定位 (rc26_localization)
  - Nav2 导航栈 (含自定义控制器)
  - 决策系统 (rc26_decision)
  - 感知模块 (rc26_perception) [可选]
  
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    bringup_dir = get_package_share_directory('rc26_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 启动参数
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    world = LaunchConfiguration('world')
    map_file = LaunchConfiguration('map')
    prior_pcd_file = LaunchConfiguration('prior_pcd_file')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_decision = LaunchConfiguration('use_decision')
    use_perception = LaunchConfiguration('use_perception')
    model_path = LaunchConfiguration('model_path')
    team = LaunchConfiguration('team')

    # 参数声明
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='顶级命名空间')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')

    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='建图模式 (True) 或导航模式 (False)')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='default',
        description='地图名称')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([bringup_dir, 'map', 'default.yaml']),
        description='地图文件路径')

    declare_prior_pcd_file = DeclareLaunchArgument(
        'prior_pcd_file',
        default_value=PathJoinSubstitution([bringup_dir, 'pcd', 'default.pcd']),
        description='先验点云文件路径')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([bringup_dir, 'config', 'nav2_params.yaml']),
        description='Nav2 参数文件')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='启动 RViz')

    declare_use_decision = DeclareLaunchArgument(
        'use_decision',
        default_value='true',
        description='启动决策系统')

    declare_use_perception = DeclareLaunchArgument(
        'use_perception',
        default_value='false',
        description='启动感知模块 (D455 + YOLO)')

    declare_model_path = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='YOLO模型路径 (空字符串=pass-through模式)')

    declare_team = DeclareLaunchArgument(
        'team',
        default_value='red',
        description='队伍颜色: red 或 blue')

    # 里程计模块 (point_lio + rc26_odom_interface + rc26_sensor_scan)
    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_dir, 'launch', 'odometry.launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'slam': slam,
            'prior_pcd_file': prior_pcd_file,
        }.items()
    )

    # 定位模块 (仅导航模式)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_dir, 'launch', 'localization.launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'slam': slam,
            'world': world,
            'prior_pcd_file': prior_pcd_file,
        }.items(),
        condition=UnlessCondition(slam)
    )

    # Nav2 导航栈：使用统一的参数文件 nav2_params.yaml，包含控制器 / costmap / BT 配置
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
        }.items()
    )

    # 地图服务 (非建图模式)
    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'localization_launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'map': map_file,
        }.items(),
        condition=UnlessCondition(slam)
    )

    # 决策系统：行为树节点（rc26_decision），默认启用，可通过 use_decision 控制
    decision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('rc26_decision'),
                'launch', 'waypoint_patrol.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'team': team,
        }.items(),
        condition=IfCondition(use_decision)
    )

    # 感知模块 (rc26_perception)
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('rc26_perception'),
                'launch', 'perception.launch.py'
            ])
        ),
        launch_arguments={
            'model_path': model_path,
        }.items(),
        condition=IfCondition(use_perception)
    )

    # RViz：提供默认配置 nav2_default.rviz，可通过 use_rviz 控制是否启动
    rviz_config = PathJoinSubstitution([bringup_dir, 'rviz', 'nav2_default.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        # 参数声明
        declare_namespace,
        declare_use_sim_time,
        declare_slam,
        declare_world,
        declare_map,
        declare_prior_pcd_file,
        declare_params_file,
        declare_use_rviz,
        declare_use_decision,
        declare_use_perception,
        declare_model_path,
        declare_team,

        # 启动模块
        odometry_launch,
        localization_launch,
        nav2_launch,
        map_server_launch,
        decision_launch,
        perception_launch,
        rviz_node,
    ])
