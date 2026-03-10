"""
src R2导航系统 - 主启动文件

启动:
  - 里程计 (rc26_point_lio + rc26_odom_interface + rc26_sensor_scan)
  - 定位 (rc26_localization)
  - 地面高度估计 (rc26_base_ground)
  - 地形分析 (rc26_terrain)
  - Nav2 导航栈 (含自定义控制器)
  - 决策系统 (rc26_decision)
  - 地图服务 (nav2_map_server)

额外模式:
  - slam:=true 且 pure_mapping_mode:=true 时，保留纯建图最小运动链路，同时继续发布可视化诊断总线
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # 获取包路径
    bringup_dir = get_package_share_directory('rc26_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    decision_dir = get_package_share_directory('rc26_decision')
    base_ground_dir = get_package_share_directory('rc26_base_ground')
    kfs_keepout_dir = get_package_share_directory('rc26_kfs_keepout')
    point_lio_dir = get_package_share_directory('rc26_point_lio')
    visualization_dir = get_package_share_directory('rc26_visualization')
    display_available = 'true' if (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')) else 'false'

    # 启动参数
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    pure_mapping_mode = LaunchConfiguration('pure_mapping_mode')
    map_file = LaunchConfiguration('map')
    prior_pcd_file = LaunchConfiguration('prior_pcd_file')
    point_lio_config_file = LaunchConfiguration('point_lio_config_file')
    point_lio_profile = LaunchConfiguration('point_lio_profile')
    params_file = LaunchConfiguration('params_file')
    terrain_params_file = LaunchConfiguration('terrain_params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    visualization_backend = LaunchConfiguration('visualization_backend')
    visualization_status_enable = LaunchConfiguration('visualization_status_enable')
    foxglove_port = LaunchConfiguration('foxglove_port')
    recover_mid360_stream = LaunchConfiguration('recover_mid360_stream')
    use_decision = LaunchConfiguration('use_decision')
    use_realsense = LaunchConfiguration('use_realsense')
    realsense_serial_no = LaunchConfiguration('realsense_serial_no')
    realsense_config_file = LaunchConfiguration('realsense_config_file')
    kfs_heartbeat_topic = LaunchConfiguration('kfs_heartbeat_topic')

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

    declare_pure_mapping_mode = DeclareLaunchArgument(
        'pure_mapping_mode',
        default_value='false',
        description='纯建图最小模式；仅在 slam:=true 时生效，跳过 terrain/decision，但保留 visualization_status 供前端显示')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([bringup_dir, 'map', 'default.yaml']),
        description='地图文件路径')

    declare_prior_pcd_file = DeclareLaunchArgument(
        'prior_pcd_file',
        default_value=PathJoinSubstitution([bringup_dir, 'pcd', 'default.pcd']),
        description='先验点云文件路径')

    declare_point_lio_config_file = DeclareLaunchArgument(
        'point_lio_config_file',
        default_value='',
        description='Point-LIO 参数文件路径；非空时优先级高于 point_lio_profile')

    declare_point_lio_profile = DeclareLaunchArgument(
        'point_lio_profile',
        default_value='auto',
        description='Point-LIO 预设: auto | base | cruise_light | mapping_dense；auto 会按 slam 自动选择')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([bringup_dir, 'config', 'nav2_params.yaml']),
        description='Nav2 参数文件')

    declare_terrain_params_file = DeclareLaunchArgument(
        'terrain_params_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('rc26_terrain'), 'config', 'terrain_semantic.yaml'
        ]),
        description='rc26_terrain 参数文件')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='启动 RViz（兼容参数；当 visualization_backend!=rviz 时忽略）')

    declare_visualization_backend = DeclareLaunchArgument(
        'visualization_backend',
        default_value='rviz',
        description='可视化后端: rviz | foxglove | none')

    declare_visualization_status_enable = DeclareLaunchArgument(
        'visualization_status_enable',
        default_value='true',
        description='是否启动可视化状态/告警聚合节点')

    declare_foxglove_port = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='foxglove_bridge WebSocket 监听端口')

    declare_recover_mid360_stream = DeclareLaunchArgument(
        'recover_mid360_stream',
        default_value='false',
        description='启动前先运行 Mid-360 恢复脚本（必要时重写 host_ipcfg 并软件重启雷达）')

    declare_use_decision = DeclareLaunchArgument(
        'use_decision',
        default_value='true',
        description='启动决策系统')

    declare_use_realsense = DeclareLaunchArgument(
        'use_realsense',
        default_value='false',
        description='启动 RealSense D455 (realsense2_camera)')

    declare_realsense_serial_no = DeclareLaunchArgument(
        'realsense_serial_no',
        default_value="''",
        description='RealSense serial number (empty to auto-select)')

    declare_realsense_config_file = DeclareLaunchArgument(
        'realsense_config_file',
        default_value=PathJoinSubstitution([bringup_dir, 'config', 'realsense_d455.yaml']),
        description='RealSense YAML config file (realsense2_camera params)')

    declare_kfs_heartbeat_topic = DeclareLaunchArgument(
        'kfs_heartbeat_topic',
        default_value='/kfs_keepout_heartbeat',
        description='KFS keepout heartbeat topic shared by keepout producer and decision gate')

    pure_mapping_runtime = PythonExpression([
        "'", slam, "'.lower() == 'true' and '", pure_mapping_mode, "'.lower() == 'true'"
    ])

    # RealSense D455（可选）
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_dir, 'launch', 'realsense_d455.launch.py'])
        ),
        launch_arguments={
            'serial_no': realsense_serial_no,
            'config_file': realsense_config_file,
        }.items()
    )
    realsense_group = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            realsense_launch,
        ],
        condition=IfCondition(use_realsense),
    )

    # 里程计模块 (rc26_point_lio + rc26_odom_interface + rc26_sensor_scan)
    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_dir, 'launch', 'odometry.launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'slam': slam,
            'prior_pcd_file': prior_pcd_file,
            'point_lio_config_file': point_lio_config_file,
            'point_lio_profile': point_lio_profile,
            'point_lio_publish_odometry_without_downsample': 'false',
            'enable_lio_state_predictor': PythonExpression([
                "not ('", slam, "'.lower() == 'true' and '", pure_mapping_mode, "'.lower() == 'true')"
            ]),
            'odometry_use_rviz': 'false',
            'recover_mid360_stream': recover_mid360_stream,
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
            'terrain_params_file': terrain_params_file,
        }.items(),
        condition=UnlessCondition(pure_mapping_runtime)
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

    terrain_mode_adapter_node = Node(
        package='rc26_nav_mode_manager',
        executable='terrain_mode_adapter_node',
        name='terrain_mode_adapter',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'terrain_node_name': 'terrain_semantic',
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
            {'node_names': ['map_server', 'costmap_filter_info_server']},
        ],
        condition=UnlessCondition(slam)
    )

    # KFS keepout 融合节点
    kfs_grid_layout = PathJoinSubstitution([kfs_keepout_dir, 'config', 'mf_grid_layout.yaml'])
    kfs_block_fuser_node = Node(
        package='rc26_kfs_keepout',
        executable='kfs_block_fuser_node',
        name='kfs_block_fuser',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'kfs_state_topic': 'mf_kfs_state',
            'mask_topic': '/kfs_filter_mask',
            'heartbeat_topic': kfs_heartbeat_topic,
            'grid_layout_file': kfs_grid_layout,
            'map_resolution': 0.10,
            'keepout_shape': 'square',
            'block_half_size_m': 0.60,
            'keepout_margin_m': 0.03,
            'lo_hit_block': 1.099,
            'lo_hit_fake': 0.693,
            'lo_miss': -0.693,
            'decay_target_prob': 0.05,
            'decay_rate': 2.0,
            'ttl_sec': 10.0,
            'ttl_mode': 'hard',
            'dwell_cycles': 3,
        }],
        condition=UnlessCondition(slam)
    )

    # costmap_filter_info_server：向 Nav2 KeepoutFilter 广播掩码元数据
    costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'type': 0,
            'filter_info_topic': '/costmap_filter_info',
            'mask_topic': '/kfs_filter_mask',
            'base': 0.0,
            'multiplier': 1.0,
        }],
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
            {
                'use_sim_time': use_sim_time,
                'keepout_gate.heartbeat_topic': kfs_heartbeat_topic,
            },
        ],
        condition=IfCondition(PythonExpression([
            "'", use_decision, "'.lower() == 'true' and not ('", slam, "'.lower() == 'true' and '",
            pure_mapping_mode, "'.lower() == 'true')"
        ]))
    )

    visualization_status_config = PathJoinSubstitution([
        visualization_dir, 'config', 'visualization_status.yaml'
    ])
    visualization_status_node = Node(
        package='rc26_visualization',
        executable='rc26_visualization_status_node',
        name='rc26_visualization_status_node',
        namespace=namespace,
        output='screen',
        parameters=[
            visualization_status_config,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(visualization_status_enable)
    )

    pure_mapping_notice = LogInfo(
        msg='[bringup] pure_mapping_mode 已启用：建图时仅保留 Point-LIO/odom_interface/localization 等最小运动链路，跳过 lio_state_predictor、rc26_terrain 和 rc26_decision，但继续发布 visualization_status 供前端显示。',
        condition=IfCondition(pure_mapping_runtime)
    )

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        namespace=namespace,
        output='screen',
        parameters=[{
            'port': foxglove_port,
            'address': '0.0.0.0',
            'use_sim_time': use_sim_time,
        }]
    )

    foxglove_group = GroupAction(
        actions=[foxglove_bridge_node],
        condition=IfCondition(PythonExpression(["'", visualization_backend, "' == 'foxglove'"]))
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
        actions=[
            GroupAction(
                actions=[rviz_nav_node, rviz_slam_node],
                condition=IfCondition(use_rviz)
            )
        ],
        condition=IfCondition(PythonExpression([
            "'", visualization_backend, "' == 'rviz' and '", display_available, "' == 'true'"
        ]))
    )

    rviz_headless_notice = LogInfo(
        msg='[bringup] 未检测到 DISPLAY/WAYLAND_DISPLAY，跳过 RViz；可改用 visualization_backend:=foxglove 或 none。',
        condition=IfCondition(PythonExpression([
            "'", visualization_backend, "' == 'rviz' and '", use_rviz, "' == 'true' and '",
            display_available, "' != 'true'"
        ]))
    )

    return LaunchDescription([
        # 参数声明
        declare_namespace,
        declare_use_sim_time,
        declare_slam,
        declare_pure_mapping_mode,
        declare_map,
        declare_prior_pcd_file,
        declare_point_lio_config_file,
        declare_point_lio_profile,
        declare_params_file,
        declare_terrain_params_file,
        declare_use_rviz,
        declare_visualization_backend,
        declare_visualization_status_enable,
        declare_foxglove_port,
        declare_recover_mid360_stream,
        declare_use_decision,
        declare_use_realsense,
        declare_realsense_serial_no,
        declare_realsense_config_file,
        declare_kfs_heartbeat_topic,

        # 启动模块
        pure_mapping_notice,
        odometry_launch,
        localization_launch,
        base_ground_node,
        terrain_launch,

        map_server_node,
        costmap_filter_info_server,
        map_server_lifecycle_manager,
        kfs_block_fuser_node,
        nav_mode_manager_node,
        terrain_mode_adapter_node,
        nav2_launch,
        decision_node,
        visualization_status_node,
        realsense_group,
        rviz_headless_notice,
        foxglove_group,
        rviz_group,
    ])
