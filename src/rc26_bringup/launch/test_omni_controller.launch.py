"""
rc26_omni_controller 模块测试

功能: 验证全向运动 PID 追踪控制器
前置: 需要最小化 Nav2 环境 + TF 树

测试指令:
    ros2 launch rc26_bringup test_omni_controller.launch.py

验证:
    # 观察隔离后的测试速度输出（不会直接打到在线底盘）
    ros2 topic echo /cmd_vel_test --once

说明:
    控制器作为 Nav2 插件运行，需要完整的 Nav2 controller_server。
    此测试启动最小化 Nav2 栈用于验证控制器逻辑。
    默认使用 test_map -> test_odom -> test_base_link，避免污染在线 TF。
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc26_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([bringup_dir, 'config', 'nav2_test_omni_controller.yaml']),
        description='Nav2 参数文件 (包含控制器配置)')
    
    # 静态 TF: test_map -> test_odom (测试用，默认与在线系统隔离)
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'test_map', '--child-frame-id', 'test_odom'],
    )

    # 静态 TF: test_odom -> test_base_link (测试用，默认与在线系统隔离)
    static_tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'test_odom', '--child-frame-id', 'test_base_link'],
    )
    
    # Nav2 controller_server (仅控制器)
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_test')],
    )
    
    # 生命周期管理
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['controller_server'],
        }],
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        static_tf_map_odom,
        static_tf_odom_base,
        controller_server,
        lifecycle_manager,
    ])
