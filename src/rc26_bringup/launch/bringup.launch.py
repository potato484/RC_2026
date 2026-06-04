"""
src R2导航系统 - 主启动文件

启动:
  - 里程计 (rc26_point_lio + rc26_odom_interface + rc26_sensor_scan)
  - 定位 (rc26_localization)
  - 地面高度估计 (rc26_base_ground)
  - 地形分析 (rc26_terrain)
  - xhu 自研导航链 (rc26_xhu_nav: topo_nav_node + xhu_motion_mode_manager_node + xhu_motion_runtime_node)
  - 决策系统 (rc26_decision)

额外模式:
  - slam:=true 且 pure_mapping_mode:=true 时，保留纯建图最小运动链路

默认装配口径:
  - 车端 bringup 维持 headless
  - 如需图形观察，请手工启动工作区外部可视化工具只读消费 ROS2 输出
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace


def generate_launch_description():
    # 获取包路径
    bringup_dir = get_package_share_directory('rc26_bringup')
    decision_dir = get_package_share_directory('rc26_decision')
    base_ground_dir = get_package_share_directory('rc26_base_ground')
    kfs_keepout_dir = get_package_share_directory('rc26_kfs_keepout')
    point_lio_dir = get_package_share_directory('rc26_point_lio')
    robot_geometry_dir = get_package_share_directory('rc26_robot_geometry')
    sensor_extrinsics_dir = get_package_share_directory('rc26_sensor_extrinsics')
    xhu_nav_dir = get_package_share_directory('rc26_xhu_nav')

    # 启动参数
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    pure_mapping_mode = LaunchConfiguration('pure_mapping_mode')
    prior_pcd_file = LaunchConfiguration('prior_pcd_file')
    point_lio_config_file = LaunchConfiguration('point_lio_config_file')
    sensor_extrinsics_file = LaunchConfiguration('sensor_extrinsics_file')
    sensor_extrinsics_profile = LaunchConfiguration('sensor_extrinsics_profile')
    start_mid360_driver = LaunchConfiguration('start_mid360_driver')
    terrain_params_file = LaunchConfiguration('terrain_params_file')
    terrain_grid_map_params_file = LaunchConfiguration('terrain_grid_map_params_file')
    terrain_filter_chain_params_file = LaunchConfiguration('terrain_filter_chain_params_file')
    enable_terrain_grid_map = LaunchConfiguration('enable_terrain_grid_map')
    recover_mid360_stream = LaunchConfiguration('recover_mid360_stream')
    localization_params_file = LaunchConfiguration('localization_params_file')
    localization_overlay_file = LaunchConfiguration('localization_overlay_file')
    competition_mode = LaunchConfiguration('competition_mode')
    enable_graph_backend = LaunchConfiguration('enable_graph_backend')
    p4_candidate_enable = LaunchConfiguration('p4_candidate_enable')
    min_inliers = LaunchConfiguration('min_inliers')
    use_decision = LaunchConfiguration('use_decision')
    use_realsense = LaunchConfiguration('use_realsense')
    realsense_serial_no = LaunchConfiguration('realsense_serial_no')
    realsense_config_file = LaunchConfiguration('realsense_config_file')
    kfs_heartbeat_topic = LaunchConfiguration('kfs_heartbeat_topic')
    team = LaunchConfiguration('team')
    robot_geometry_file = LaunchConfiguration('robot_geometry_file')
    robot_geometry_profile = LaunchConfiguration('robot_geometry_profile')
    local_3d_planner_config_file = LaunchConfiguration('local_3d_planner_config_file')
    xhu_motion_runtime_config_file = LaunchConfiguration('xhu_motion_runtime_config_file')

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
        description='纯建图最小模式；仅在 slam:=true 时生效，跳过 terrain/decision 等非必要模块')

    declare_prior_pcd_file = DeclareLaunchArgument(
        'prior_pcd_file',
        default_value=PathJoinSubstitution([bringup_dir, 'pcd', 'default.pcd']),
        description='先验点云文件路径')

    declare_point_lio_config_file = DeclareLaunchArgument(
        'point_lio_config_file',
        default_value='',
        description='Point-LIO 参数文件路径；为空时使用 rc26_point_lio/config/mid360.yaml')

    declare_sensor_extrinsics_file = DeclareLaunchArgument(
        'sensor_extrinsics_file',
        default_value=PathJoinSubstitution([sensor_extrinsics_dir, 'config', 'r2_sensor_extrinsics.yaml']),
        description='传感器安装外参 YAML 文件路径')

    declare_sensor_extrinsics_profile = DeclareLaunchArgument(
        'sensor_extrinsics_profile',
        default_value='',
        description='传感器安装外参 profile；空字符串表示使用 YAML defaults.active_profile')

    declare_start_mid360_driver = DeclareLaunchArgument(
        'start_mid360_driver',
        default_value='true',
        description='是否由 odometry 链启动 MID-360 驱动')

    declare_terrain_params_file = DeclareLaunchArgument(
        'terrain_params_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('rc26_terrain'), 'config', 'terrain_semantic.yaml'
        ]),
        description='rc26_terrain 参数文件')

    declare_terrain_grid_map_params_file = DeclareLaunchArgument(
        'terrain_grid_map_params_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('rc26_terrain'), 'config', 'terrain_grid_map_bridge.yaml'
        ]),
        description='terrain_grid_map_bridge 参数文件')

    declare_terrain_filter_chain_params_file = DeclareLaunchArgument(
        'terrain_filter_chain_params_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('rc26_terrain'), 'config', 'terrain_filter_chain.yaml'
        ]),
        description='terrain_grid_map_bridge filter chain 参数文件')

    declare_enable_terrain_grid_map = DeclareLaunchArgument(
        'enable_terrain_grid_map',
        default_value='false',
        description='是否额外启用 terrain_semantic + terrain_grid_map_bridge；可在 pure_mapping_mode 或独立建图时打开 2.5D 栅格地图显示')

    declare_recover_mid360_stream = DeclareLaunchArgument(
        'recover_mid360_stream',
        default_value='false',
        description='启动前先运行 Mid-360 恢复脚本（必要时重写 host_ipcfg 并软件重启雷达）')

    declare_localization_params_file = DeclareLaunchArgument(
        'localization_params_file',
        default_value=PathJoinSubstitution([bringup_dir, 'config', 'localization.yaml']),
        description='基础定位参数文件路径')

    declare_localization_overlay_file = DeclareLaunchArgument(
        'localization_overlay_file',
        default_value=PathJoinSubstitution([bringup_dir, 'config', 'localization_overlay_default.yaml']),
        description='定位参数 overlay 文件路径')

    declare_competition_mode = DeclareLaunchArgument(
        'competition_mode',
        default_value='true',
        description='定位 competition_mode 参数')

    declare_enable_graph_backend = DeclareLaunchArgument(
        'enable_graph_backend',
        default_value='false',
        description='是否为 localization 启用图后端')

    declare_p4_candidate_enable = DeclareLaunchArgument(
        'p4_candidate_enable',
        default_value='false',
        description='是否为 localization 启用 P4 外部候选输入')

    declare_min_inliers = DeclareLaunchArgument(
        'min_inliers',
        default_value='200',
        description='localization 局部配准质量门控最小内点数')

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

    declare_team = DeclareLaunchArgument(
        'team',
        default_value='blue',
        description='Active competition side: blue | red')

    declare_robot_geometry_file = DeclareLaunchArgument(
        'robot_geometry_file',
        default_value=PathJoinSubstitution([robot_geometry_dir, 'config', 'r2_body_geometry.yaml']),
        description='统一机器人几何配置文件路径')

    declare_robot_geometry_profile = DeclareLaunchArgument(
        'robot_geometry_profile',
        default_value='compact',
        description='机器人几何 profile 名称')

    declare_local_3d_planner_config_file = DeclareLaunchArgument(
        'local_3d_planner_config_file',
        default_value=PathJoinSubstitution([xhu_nav_dir, 'config', 'local_3d_planner.yaml']),
        description='local_3d_planner 参数文件')

    declare_xhu_motion_runtime_config_file = DeclareLaunchArgument(
        'xhu_motion_runtime_config_file',
        default_value=PathJoinSubstitution([xhu_nav_dir, 'config', 'xhu_motion_runtime.yaml']),
        description='xhu_motion_runtime 参数文件')

    topo_graph_blue_file = PathJoinSubstitution([xhu_nav_dir, 'config', 'r2_field_graph_blue.yaml'])
    topo_graph_red_file = PathJoinSubstitution([xhu_nav_dir, 'config', 'r2_field_graph_red.yaml'])
    topo_graph_file = PythonExpression([
        "'", topo_graph_red_file, "' if '", team, "'.lower() == 'red' else '", topo_graph_blue_file, "'"
    ])

    pure_mapping_runtime = PythonExpression([
        "'", slam, "'.lower() == 'true' and '", pure_mapping_mode, "'.lower() == 'true'"
    ])
    terrain_grid_map_runtime = PythonExpression([
        "not ('", slam, "'.lower() == 'true' and '", pure_mapping_mode, "'.lower() == 'true') "
        "or '", enable_terrain_grid_map, "'.lower() == 'true'"
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
            'point_lio_config_file': point_lio_config_file,
            'sensor_extrinsics_file': sensor_extrinsics_file,
            'sensor_extrinsics_profile': sensor_extrinsics_profile,
            'point_lio_publish_odometry_without_downsample': 'false',
            'start_mid360_driver': start_mid360_driver,
            'enable_terrain_grid_map': 'false',
            'recover_mid360_stream': recover_mid360_stream,
        }.items()
    )
    # Scope child launch arguments so odometry's internal overrides
    # (for example enable_terrain_grid_map:=false) do not leak back
    # into the parent bringup context and suppress top-level terrain nodes.
    odometry_group = GroupAction(
        actions=[odometry_launch],
        scoped=True,
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
            'localization_params_file': localization_params_file,
            'localization_overlay_file': localization_overlay_file,
            'competition_mode': competition_mode,
            'enable_graph_backend': enable_graph_backend,
            'p4_candidate_enable': p4_candidate_enable,
            'min_inliers': min_inliers,
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
        condition=IfCondition(terrain_grid_map_runtime)
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

    # xhu 自研运动模式管理器
    xhu_profiles_file = PathJoinSubstitution([xhu_nav_dir, 'config', 'nav_profiles.yaml'])
    xhu_motion_mode_manager_node = Node(
        package='rc26_xhu_nav',
        executable='xhu_motion_mode_manager_node',
        name='xhu_motion_mode_manager',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': 'odom',
            'profiles_file': xhu_profiles_file,
            'default_mode': 'hold',
        }],
        condition=UnlessCondition(slam)
    )

    xhu_motion_runtime_node = Node(
        package='rc26_xhu_nav',
        executable='xhu_motion_runtime_node',
        name='xhu_motion_runtime',
        namespace=namespace,
        output='screen',
        parameters=[
            xhu_motion_runtime_config_file,
            local_3d_planner_config_file,
            {
                'use_sim_time': use_sim_time,
                'robot_geometry_file': robot_geometry_file,
                'robot_geometry_profile': robot_geometry_profile,
            },
        ],
        condition=UnlessCondition(slam)
    )

    # rc26_xhu_nav topo runtime
    topo_nav_node = Node(
        package='rc26_xhu_nav',
        executable='topo_nav_node',
        name='topo_nav_node',
        namespace=namespace,
        output='screen',
        parameters=[
            PathJoinSubstitution([xhu_nav_dir, 'config', 'topo_nav.yaml']),
            {
                'use_sim_time': use_sim_time,
                'team': team,
                'graph_file': topo_graph_file,
                'robot_geometry_file': robot_geometry_file,
                'robot_geometry_profile': robot_geometry_profile,
            },
        ],
        condition=UnlessCondition(slam)
    )

    # KFS keepout 运行时：空组件容器 + 常驻 runtime manager
    kfs_grid_layout = PathJoinSubstitution([kfs_keepout_dir, 'config', 'r2_mf_world.yaml'])
    kfs_keepout_container = ComposableNodeContainer(
        name='kfs_keepout_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[],
        output='screen',
        condition=UnlessCondition(slam)
    )
    kfs_keepout_runtime_manager_node = Node(
        package='rc26_kfs_keepout',
        executable='kfs_keepout_runtime_manager_node',
        name='kfs_keepout_runtime_manager',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'runtime_service_name': '/kfs_keepout/set_runtime',
            'component_container_name': 'kfs_keepout_container',
            'component_node_name': 'kfs_block_fuser',
            'component_runtime_control_service': 'set_runtime',
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

    terrain_grid_map_bridge_node = Node(
        package='rc26_terrain',
        executable='terrain_grid_map_bridge_node',
        name='terrain_grid_map_bridge',
        namespace=namespace,
        output='screen',
        parameters=[
            terrain_grid_map_params_file,
            terrain_filter_chain_params_file,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(terrain_grid_map_runtime)
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
                'team': team,
                'tree_file': 'main_tree.xml',
                'keepout_gate.heartbeat_topic': kfs_heartbeat_topic,
            },
        ],
        condition=IfCondition(PythonExpression([
            "'", use_decision, "'.lower() == 'true' and not ('", slam, "'.lower() == 'true' and '",
            pure_mapping_mode, "'.lower() == 'true')"
        ]))
    )

    pure_mapping_notice = LogInfo(
        msg='[bringup] pure_mapping_mode 已启用：建图时仅保留 Point-LIO/odom_interface/localization 等最小运动链路，跳过 rc26_terrain 和 rc26_decision。',
        condition=IfCondition(PythonExpression([
            "'", slam, "'.lower() == 'true' and '", pure_mapping_mode, "'.lower() == 'true' and '",
            enable_terrain_grid_map, "'.lower() != 'true'"
        ]))
    )

    pure_mapping_terrain_grid_notice = LogInfo(
        msg='[bringup] pure_mapping_mode 已启用，同时 enable_terrain_grid_map=true：额外启动 rc26_terrain 与 terrain_grid_map_bridge，用于发布 /terrain_grid_map 2.5D 栅格地图。',
        condition=IfCondition(PythonExpression([
            "'", slam, "'.lower() == 'true' and '", pure_mapping_mode, "'.lower() == 'true' and '",
            enable_terrain_grid_map, "'.lower() == 'true'"
        ]))
    )

    return LaunchDescription([
        # 参数声明
        declare_namespace,
        declare_use_sim_time,
        declare_slam,
        declare_pure_mapping_mode,
        declare_prior_pcd_file,
        declare_point_lio_config_file,
        declare_sensor_extrinsics_file,
        declare_sensor_extrinsics_profile,
        declare_start_mid360_driver,
        declare_terrain_params_file,
        declare_terrain_grid_map_params_file,
        declare_terrain_filter_chain_params_file,
        declare_enable_terrain_grid_map,
        declare_recover_mid360_stream,
        declare_localization_params_file,
        declare_localization_overlay_file,
        declare_competition_mode,
        declare_enable_graph_backend,
        declare_p4_candidate_enable,
        declare_min_inliers,
        declare_use_decision,
        declare_use_realsense,
        declare_realsense_serial_no,
        declare_realsense_config_file,
        declare_kfs_heartbeat_topic,
        declare_team,
        declare_robot_geometry_file,
        declare_robot_geometry_profile,
        declare_local_3d_planner_config_file,
        declare_xhu_motion_runtime_config_file,

        # 启动模块
        pure_mapping_notice,
        pure_mapping_terrain_grid_notice,
        odometry_group,
        localization_launch,
        base_ground_node,
        terrain_launch,

        kfs_keepout_container,
        kfs_keepout_runtime_manager_node,
        terrain_grid_map_bridge_node,
        xhu_motion_mode_manager_node,
        xhu_motion_runtime_node,
        topo_nav_node,
        decision_node,
        realsense_group,
    ])
