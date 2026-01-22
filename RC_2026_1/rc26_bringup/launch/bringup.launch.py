"""
RC_2026_1 R2导航系统 - 主启动文件

启动:
  - 里程计 (point_lio + rc26_odom_interface + rc26_sensor_scan)
  - 定位 (rc26_localization)
  - 地面高度估计 (rc26_base_ground)
  - 地形分析 (rc26_terrain)
  - Nav2 导航栈 (含自定义控制器)
  - 决策系统 (rc26_decision)
  - 地图服务 (nav2_map_server)

"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    bringup_dir = get_package_share_directory('rc26_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    decision_dir = get_package_share_directory('rc26_decision')
    base_ground_dir = get_package_share_directory('rc26_base_ground')

    # 启动参数
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_file = LaunchConfiguration('map')
    prior_pcd_file = LaunchConfiguration('prior_pcd_file')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_decision = LaunchConfiguration('use_decision')

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

    # 里程计模块 (point_lio + rc26_odom_interface + rc26_sensor_scan)
    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_dir, 'launch', 'odometry.launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'prior_pcd_file': prior_pcd_file,
        }.items()
    )

    # 定位模块
    # - 导航模式: 启动 rc26_localization
    # - 建图模式: 发布静态 map -> odom 变换
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_dir, 'launch', 'localization.launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'slam': slam,
            'prior_pcd_file': prior_pcd_file,
        }.items()
    )

    # 地形分析模块 (rc26_terrain)
    terrain_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('rc26_terrain'),
                'launch', 'terrain_semantic.launch.py'
            ])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # 地面高度估计 (rc26_base_ground)
    base_ground_config = PathJoinSubstitution([base_ground_dir, 'config', 'base_ground_estimator.yaml'])
    base_ground_node = Node(
        package='rc26_base_ground',
        executable='base_ground_estimator_node',
        name='base_ground_estimator',
        namespace=namespace,
        output='screen',
        parameters=[
            base_ground_config,
            {'use_sim_time': use_sim_time},
        ],
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
        }.items(),
        condition=UnlessCondition(slam)
    )

    # 导航模式管理器 (rc26_nav_mode_manager)
    nav_mode_manager_node = Node(
        package='rc26_nav_mode_manager',
        executable='nav_mode_manager_node',
        name='nav_mode_manager',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'costmap_node_name': 'local_costmap/local_costmap',
            'odom_topic': 'odom',
            'obstacles_topic': 'terrain_obstacles',
            'default_timeout_sec': 5.0,
        }],
        condition=UnlessCondition(slam)
    )

    # 地图服务：仅启动 map_server（不启动 AMCL，避免与 rc26_localization 的 map->odom 冲突）
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_file},
        ],
        condition=UnlessCondition(slam)
    )
    map_server_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']},
        ],
        condition=UnlessCondition(slam)
    )

    # 决策系统：行为树节点（rc26_decision/decision_node），默认启用，可通过 use_decision 控制
    decision_params = PathJoinSubstitution([decision_dir, 'config', 'decision_params.yaml'])
    decision_node = Node(
        package='rc26_decision',
        executable='decision_node',
        name='rc26_decision',
        namespace=namespace,
        output='screen',
        parameters=[
            decision_params,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(use_decision)
    )

    # RViz：导航模式使用 nav2_default.rviz，建图模式使用 slam.rviz
    rviz_nav_config = PathJoinSubstitution([bringup_dir, 'rviz', 'nav2_default.rviz'])
    rviz_slam_config = PathJoinSubstitution([bringup_dir, 'rviz', 'slam.rviz'])

    rviz_nav_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_nav_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(slam)
    )

    rviz_slam_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_slam_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(slam)
    )

    rviz_group = GroupAction(
        actions=[rviz_nav_node, rviz_slam_node],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        # 参数声明
        declare_namespace,
        declare_use_sim_time,
        declare_slam,
        declare_map,
        declare_prior_pcd_file,
        declare_params_file,
        declare_use_rviz,
        declare_use_decision,

        # 启动模块
        odometry_launch,
        localization_launch,
        base_ground_node,
        terrain_launch,
        nav2_launch,
        nav_mode_manager_node,
        map_server_node,
        map_server_lifecycle_manager,
        decision_node,
        rviz_group,
    ])
