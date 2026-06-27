from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    mcu_transport_dir = get_package_share_directory('rc26_mcu_transport')
    hal_type = LaunchConfiguration('hal_type')
    start_mcu_transport = LaunchConfiguration('start_mcu_transport')
    mcu_transport_target_serial_port = LaunchConfiguration('mcu_transport_target_serial_port')
    mcu_transport_target_baudrate = LaunchConfiguration('mcu_transport_target_baudrate')
    mcu_transport_open_retry_period_ms = LaunchConfiguration('mcu_transport_open_retry_period_ms')
    mcu_transport_diagnostics_period_ms = LaunchConfiguration('mcu_transport_diagnostics_period_ms')

    hal_type_arg = DeclareLaunchArgument(
        'hal_type',
        default_value='shared_serial',
        description='Mechanism HAL type: shared_serial',
    )
    start_mcu_transport_arg = DeclareLaunchArgument(
        'start_mcu_transport',
        default_value='true',
        description='Whether to start rc26_mcu_transport alongside mechanism server',
    )
    mcu_transport_target_serial_port_arg = DeclareLaunchArgument(
        'mcu_transport_target_serial_port',
        default_value='/dev/ttyUSB0',
        description='Target MCU serial device owned by rc26_mcu_transport',
    )
    mcu_transport_target_baudrate_arg = DeclareLaunchArgument(
        'mcu_transport_target_baudrate',
        default_value='1000000',
        description='Target MCU serial baudrate',
    )
    mcu_transport_open_retry_period_ms_arg = DeclareLaunchArgument(
        'mcu_transport_open_retry_period_ms',
        default_value='1000',
        description='Target MCU serial initial open retry period in milliseconds',
    )
    mcu_transport_diagnostics_period_ms_arg = DeclareLaunchArgument(
        'mcu_transport_diagnostics_period_ms',
        default_value='1000',
        description='rc26_mcu_transport diagnostics publish period in milliseconds',
    )

    mcu_transport_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([mcu_transport_dir, 'launch', 'mcu_transport.launch.py'])
        ),
        launch_arguments={
            'target_serial_port': mcu_transport_target_serial_port,
            'target_baudrate': mcu_transport_target_baudrate,
            'open_retry_period_ms': mcu_transport_open_retry_period_ms,
            'diagnostics_period_ms': mcu_transport_diagnostics_period_ms,
        }.items(),
        condition=IfCondition(start_mcu_transport),
    )

    container = ComposableNodeContainer(
        name='mechanism_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='rc26_mechanism',
                plugin='rc26_mechanism::MechanismLifecycleServer',
                name='mechanism_server',
                parameters=[{'hal_type': hal_type}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )
    return LaunchDescription([
        hal_type_arg,
        start_mcu_transport_arg,
        mcu_transport_target_serial_port_arg,
        mcu_transport_target_baudrate_arg,
        mcu_transport_open_retry_period_ms_arg,
        mcu_transport_diagnostics_period_ms_arg,
        mcu_transport_launch,
        container,
    ])
