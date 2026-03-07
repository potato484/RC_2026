"""
里程计模块启动文件

启动:
  - rc26_point_lio (LiDAR-IMU 里程计)
  - rc26_odom_interface (坐标变换: lidar_odom -> odom)
  - rc26_sensor_scan (发布 odom -> chassis 变换 + sensor_scan)
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc26_bringup')
    _dir = get_package_share_directory('')
    point_lio_dir = get_package_share_directory('rc26_point_lio')
    mid360_driver_dir = get_package_share_directory('rc26_mid360_driver')

    # 启动参数
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    prior_pcd_file = LaunchConfiguration('prior_pcd_file')
    odometry_use_rviz = LaunchConfiguration('odometry_use_rviz')
    start_mid360_driver = LaunchConfiguration('start_mid360_driver')
    point_lio_config_file = LaunchConfiguration('point_lio_config_file')
    recover_mid360_stream = LaunchConfiguration('recover_mid360_stream')
    recover_mid360_lidar_ip = LaunchConfiguration('recover_mid360_lidar_ip')
    recover_mid360_host_ip = LaunchConfiguration('recover_mid360_host_ip')
    recover_mid360_timeout = LaunchConfiguration('recover_mid360_timeout')
    recover_mid360_warmup_before_reboot = LaunchConfiguration('recover_mid360_warmup_before_reboot')
    param_overrides_file = PathJoinSubstitution([_dir, 'config', 'param_overrides.yaml'])

    # 参数声明
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='顶级命名空间')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')

    declare_prior_pcd_file = DeclareLaunchArgument(
        'prior_pcd_file',
        default_value='',
        description='先验点云文件路径 (建图模式下可为空)')

    declare_use_rviz = DeclareLaunchArgument(
        'odometry_use_rviz',
        default_value='true',
        description='启动 RViz')

    declare_start_mid360_driver = DeclareLaunchArgument(
        'start_mid360_driver',
        default_value='true',
        description='启动 MID-360 驱动')

    declare_point_lio_config_file = DeclareLaunchArgument(
        'point_lio_config_file',
        default_value=PathJoinSubstitution([point_lio_dir, 'config', 'mid360.yaml']),
        description='Point-LIO 参数文件路径（建图/调试时可切换不同配置）')

    declare_recover_mid360_stream = DeclareLaunchArgument(
        'recover_mid360_stream',
        default_value='false',
        description='启动前先运行 Mid-360 恢复脚本（会在必要时重写 host_ipcfg 并软件重启雷达）')

    declare_recover_mid360_lidar_ip = DeclareLaunchArgument(
        'recover_mid360_lidar_ip',
        default_value='192.168.1.140',
        description='Mid-360 实机 IP（恢复脚本使用）')

    declare_recover_mid360_host_ip = DeclareLaunchArgument(
        'recover_mid360_host_ip',
        default_value='192.168.1.50',
        description='R2 主机接收口 IP（恢复脚本使用）')

    declare_recover_mid360_timeout = DeclareLaunchArgument(
        'recover_mid360_timeout',
        default_value='35.0',
        description='Mid-360 恢复脚本最长等待时间（秒）')

    declare_recover_mid360_warmup_before_reboot = DeclareLaunchArgument(
        'recover_mid360_warmup_before_reboot',
        default_value='5.0',
        description='发现雷达后等待多久再触发软件重启（秒）')

    # 配置文件路径
    mid360_driver_config = PathJoinSubstitution([mid360_driver_dir, 'config', 'param.yaml'])
    odom_interface_config = PathJoinSubstitution([bringup_dir, 'config', 'odom_interface.yaml'])
    sensor_scan_config = PathJoinSubstitution([bringup_dir, 'config', 'sensor_scan_generation.yaml'])
    lio_state_predictor_yaml = PathJoinSubstitution([bringup_dir, 'config', 'lio_state_predictor.yaml'])

    start_mid360_driver_direct_condition = IfCondition(
        PythonExpression(["'", start_mid360_driver, "' == 'true' and '", recover_mid360_stream, "' != 'true'"])
    )
    recover_mid360_condition = IfCondition(
        PythonExpression(["'", start_mid360_driver, "' == 'true' and '", recover_mid360_stream, "' == 'true'"])
    )

    def create_mid360_driver_node(condition=None):
        kwargs = {
            'package': 'rc26_mid360_driver',
            'executable': 'rc26_mid360_driver_node',
            'name': 'mid360_driver',
            'namespace': namespace,
            'output': 'screen',
            'parameters': [
                mid360_driver_config,
                {'use_sim_time': use_sim_time},
            ],
        }
        if condition is not None:
            kwargs['condition'] = condition
        return Node(**kwargs)

    # MID-360 LiDAR 驱动节点
    mid360_driver_node = create_mid360_driver_node(condition=start_mid360_driver_direct_condition)
    mid360_driver_node_after_recover = create_mid360_driver_node()

    recover_mid360_process = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([mid360_driver_dir, 'scripts', 'recover_mid360_stream.py']),
            '--lidar-ip', recover_mid360_lidar_ip,
            '--host-ip', recover_mid360_host_ip,
            '--timeout', recover_mid360_timeout,
            '--warmup-before-reboot', recover_mid360_warmup_before_reboot,
        ],
        output='screen',
        condition=recover_mid360_condition,
    )

    start_mid360_after_recover = RegisterEventHandler(
        OnProcessExit(
            target_action=recover_mid360_process,
            on_exit=[mid360_driver_node_after_recover],
        )
    )

    # rc26_point_lio 里程计节点
    point_lio_node = Node(
        package='rc26_point_lio',
        executable='pointlio_mapping',
        name='point_lio',
        namespace=namespace,
        output='screen',
        parameters=[
            point_lio_config_file,
            param_overrides_file,
            {'use_sim_time': use_sim_time},
            {'prior_pcd.prior_pcd_map_path': prior_pcd_file},
            # 集成里程计链要求 /state_estimation 与 /cloud_registered 共享同一 LiDAR 时戳。
            # 若开启无下采样高频里程计，Point-LIO 会以 time_current 给里程计打戳，
            # 下游 odom_interface / sensor_scan 会因为与点云时戳偏差过大而持续丢帧。
            {'odometry.publish_odometry_without_downsample': False},
            # 对齐 Point-LIO body_frame 与 lidar_frame 语义
            # 并关闭 Point-LIO 自身 TF 发布，避免与 rc26_odom_interface / rc26_sensor_scan 重复发布 odom->base_link
            {'frame.body_frame': 'livox_frame'},
            {'publish.tf_send_en': False},
        ],
    )

    # rc26_odom_interface: 将 rc26_point_lio 输出从 lidar_odom 转换到 odom 系
    odom_interface_node = Node(
        package='rc26_odom_interface',
        executable='rc26_odom_interface_node',
        name='odom_interface',
        namespace=namespace,
        output='screen',
        parameters=[
            odom_interface_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    # rc26_sensor_scan: 发布 odom -> chassis 变换和 sensor_scan
    sensor_scan_node = Node(
        package='rc26_sensor_scan',
        executable='rc26_sensor_scan_node',
        name='sensor_scan',
        namespace=namespace,
        output='screen',
        parameters=[
            sensor_scan_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    lio_state_predictor_node = Node(
        package='rc26_lio_state_predictor',
        executable='rc26_lio_state_predictor_node',
        name='lio_state_predictor',
        namespace=namespace,
        output='screen',
        parameters=[
            lio_state_predictor_yaml,
            {'use_sim_time': use_sim_time},
        ],
    )

    # 静态TF: base_link -> livox_frame (与 Point-LIO 外参对齐)
    static_tf_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_livox',
        arguments=['--x', '0', '--y', '0', '--z', '0.13', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
    )

    # RViz 可视化
    rviz_config = PathJoinSubstitution([bringup_dir, 'rviz', 'slam.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(odometry_use_rviz)
    )

    return LaunchDescription([
        # 参数声明
        declare_namespace,
        declare_use_sim_time,
        declare_prior_pcd_file,
        declare_use_rviz,
        declare_start_mid360_driver,
        declare_point_lio_config_file,
        declare_recover_mid360_stream,
        declare_recover_mid360_lidar_ip,
        declare_recover_mid360_host_ip,
        declare_recover_mid360_timeout,
        declare_recover_mid360_warmup_before_reboot,

        # 节点
        static_tf_livox,
        recover_mid360_process,
        start_mid360_after_recover,
        mid360_driver_node,
        point_lio_node,
        odom_interface_node,
        sensor_scan_node,
        lio_state_predictor_node,
        rviz_node,
    ])
