from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    serial_baud = LaunchConfiguration('serial_baud')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB1',
        description='Mechanism serial port path',
    )
    serial_baud_arg = DeclareLaunchArgument(
        'serial_baud',
        default_value='1000000',
        description='Mechanism serial baudrate',
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
                parameters=[{
                    'serial_port': serial_port,
                    'serial_baud': ParameterValue(serial_baud, value_type=int),
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )
    return LaunchDescription([serial_port_arg, serial_baud_arg, container])
