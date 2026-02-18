"""
里程计模块启动文件

启动:
  - point_lio (LiDAR-IMU 里程计)
  - rc26_odom_interface (坐标变换: lidar_odom -> odom)
  - rc26_sensor_scan (发布 odom -> chassis 变换 + sensor_scan)
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc26_bringup')
    point_lio_dir = get_package_share_directory('point_lio')
    mid360_driver_dir = get_package_share_directory('mid360_driver')

    # 启动参数
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    prior_pcd_file = LaunchConfiguration('prior_pcd_file')
    use_rviz = LaunchConfiguration('use_rviz')

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
        'use_rviz',
        default_value='true',
        description='启动 RViz')

    # 配置文件路径
    mid360_driver_config = PathJoinSubstitution([mid360_driver_dir, 'config', 'param.yaml'])
    point_lio_config = PathJoinSubstitution([point_lio_dir, 'config', 'mid360.yaml'])
    odom_interface_config = PathJoinSubstitution([bringup_dir, 'config', 'odom_interface.yaml'])
    sensor_scan_config = PathJoinSubstitution([bringup_dir, 'config', 'sensor_scan_generation.yaml'])

    # MID-360 LiDAR 驱动节点
    mid360_driver_node = Node(
        package='mid360_driver',
        executable='mid360_driver_node',
        name='mid360_driver',
        namespace=namespace,
        output='screen',
        parameters=[
            mid360_driver_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Point-LIO 里程计节点
    point_lio_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='point_lio',
        namespace=namespace,
        output='screen',
        parameters=[
            point_lio_config,
            {'use_sim_time': use_sim_time},
            {'prior_pcd.prior_pcd_map_path': prior_pcd_file},
            # 统一坐标系命名：项目内使用 base_link 作为车体坐标系
            # 并关闭 Point-LIO 自身 TF 发布，避免与 rc26_odom_interface / rc26_sensor_scan 重复发布 odom->base_link
            {'frame.body_frame': 'base_link'},
            {'publish.tf_send_en': False},
        ],
    )

    # rc26_odom_interface: 将 point_lio 输出从 lidar_odom 转换到 odom 系
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
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        # 参数声明
        declare_namespace,
        declare_use_sim_time,
        declare_prior_pcd_file,
        declare_use_rviz,

        # 节点
        static_tf_livox,
        mid360_driver_node,
        point_lio_node,
        odom_interface_node,
        sensor_scan_node,
        rviz_node,
    ])
