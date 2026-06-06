"""
src R2导航系统 - 主启动文件

启动:
  - 里程计 (rc26_point_lio + rc26_odom_interface + rc26_sensor_scan)
  - 定位 (rc26_localization)
  - Nav2 基础导航栈 (map_server + planner/controller/BT navigator/velocity smoother)
  - 底盘执行桥 (rc26_merge_odom/pose_sender_node)
  - 决策系统 (rc26_decision)

额外模式:
  - slam:=true 且 pure_mapping_mode:=true 时，保留纯建图最小运动链路

默认装配口径:
  - 车端 bringup 维持 headless
  - 如需图形观察，请手工启动工作区外部可视化工具只读消费 ROS2 输出
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # 获取包路径
    bringup_dir = get_package_share_directory('rc26_bringup')
    decision_dir = get_package_share_directory('rc26_decision')
    merge_odom_dir = get_package_share_directory('rc26_merge_odom')
    sensor_extrinsics_dir = get_package_share_directory('rc26_sensor_extrinsics')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

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
    recover_mid360_stream = LaunchConfiguration('recover_mid360_stream')
    localization_params_file = LaunchConfiguration('localization_params_file')
    start_pose_sender = LaunchConfiguration('start_pose_sender')
    pose_sender_feedback_serial_port = LaunchConfiguration('pose_sender_feedback_serial_port')
    pose_sender_target_serial_port = LaunchConfiguration('pose_sender_target_serial_port')
    pose_sender_baudrate = LaunchConfiguration('pose_sender_baudrate')
    use_decision = LaunchConfiguration('use_decision')
    use_realsense = LaunchConfiguration('use_realsense')
    realsense_serial_no = LaunchConfiguration('realsense_serial_no')
    realsense_config_file = LaunchConfiguration('realsense_config_file')
    team = LaunchConfiguration('team')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    nav2_map_file = LaunchConfiguration('nav2_map_file')

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
        description='纯建图最小模式；仅在 slam:=true 时生效，跳过 decision 等非必要模块')

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

    declare_recover_mid360_stream = DeclareLaunchArgument(
        'recover_mid360_stream',
        default_value='false',
        description='启动前先运行 Mid-360 恢复脚本（必要时重写 host_ipcfg 并软件重启雷达）')

    declare_localization_params_file = DeclareLaunchArgument(
        'localization_params_file',
        default_value=PathJoinSubstitution([bringup_dir, 'config', 'localization.yaml']),
        description='基础定位参数文件路径')

    declare_start_pose_sender = DeclareLaunchArgument(
        'start_pose_sender',
        default_value='true',
        description='是否启动 pose_sender_node，把导航 /cmd_vel 接到底盘目标 MCU')

    declare_pose_sender_feedback_serial_port = DeclareLaunchArgument(
        'pose_sender_feedback_serial_port',
        default_value='__disabled__',
        description='pose_sender_node 反馈串口；当前默认停用反馈链路')

    declare_pose_sender_target_serial_port = DeclareLaunchArgument(
        'pose_sender_target_serial_port',
        default_value='/dev/ttyUSB0',
        description='pose_sender_node 目标串口；用于 POSE_TARGET 与 mechanism transport')

    declare_pose_sender_baudrate = DeclareLaunchArgument(
        'pose_sender_baudrate',
        default_value='1000000',
        description='pose_sender_node 串口波特率')

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

    declare_team = DeclareLaunchArgument(
        'team',
        default_value='blue',
        description='Active competition side: blue | red')

    declare_nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([bringup_dir, 'config', 'nav2_params.yaml']),
        description='Nav2 参数文件；rc26_localization 仍负责 map->odom，Nav2 只负责规划/控制')

    declare_nav2_map_file = DeclareLaunchArgument(
        'nav2_map_file',
        default_value=PathJoinSubstitution([bringup_dir, 'map', 'default.yaml']),
        description='Nav2 map_server 使用的 2D occupancy map YAML；实机导航应传入有效地图')

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
            'recover_mid360_stream': recover_mid360_stream,
        }.items()
    )
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
        }.items()
    )

    # Nav2 基础导航栈。
    # rc26_localization 继续作为 map->odom 权威；这里只启动 map_server 和 Nav2 navigation 节点。
    nav2_map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace=namespace,
        output='screen',
        parameters=[
            nav2_params_file,
            {
                'use_sim_time': use_sim_time,
                'yaml_filename': nav2_map_file,
            },
        ],
        condition=UnlessCondition(slam)
    )

    nav2_map_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }],
        condition=UnlessCondition(slam)
    )

    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
        condition=UnlessCondition(slam)
    )

    pose_sender_config_file = PathJoinSubstitution([merge_odom_dir, 'config', 'merge_odom_params.yaml'])
    pose_sender_node = Node(
        package='rc26_merge_odom',
        executable='pose_sender_node',
        name='pose_sender_node',
        namespace=namespace,
        output='screen',
        parameters=[
            pose_sender_config_file,
            {
                'use_sim_time': use_sim_time,
                'feedback_serial_port': pose_sender_feedback_serial_port,
                'target_serial_port': pose_sender_target_serial_port,
                'baudrate': pose_sender_baudrate,
                'cmd_vel_topic': 'cmd_vel',
                'odom_topic': 'odom',
                'imu_topic': '',
                'imu_gate_enable': False,
                'latency_comp_enable': False,
            },
        ],
        condition=IfCondition(
            PythonExpression([
                "'", start_pose_sender, "'.lower() == 'true' and '", slam, "'.lower() != 'true'"
            ])
        ),
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
            },
        ],
        condition=IfCondition(PythonExpression([
            "'", use_decision, "'.lower() == 'true' and not ('", slam, "'.lower() == 'true' and '",
            pure_mapping_mode, "'.lower() == 'true')"
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
        declare_recover_mid360_stream,
        declare_localization_params_file,
        declare_start_pose_sender,
        declare_pose_sender_feedback_serial_port,
        declare_pose_sender_target_serial_port,
        declare_pose_sender_baudrate,
        declare_use_decision,
        declare_use_realsense,
        declare_realsense_serial_no,
        declare_realsense_config_file,
        declare_team,
        declare_nav2_params_file,
        declare_nav2_map_file,

        # 启动模块
        odometry_group,
        localization_launch,

        nav2_map_server_node,
        nav2_map_lifecycle_node,
        nav2_navigation_launch,
        pose_sender_node,
        decision_node,
        realsense_group,
    ])
