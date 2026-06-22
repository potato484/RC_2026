from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    target_serial_port = LaunchConfiguration('target_serial_port')
    target_baudrate = LaunchConfiguration('target_baudrate')
    open_retry_period_ms = LaunchConfiguration('open_retry_period_ms')
    diagnostics_period_ms = LaunchConfiguration('diagnostics_period_ms')
    send_command_service = LaunchConfiguration('send_command_service')
    command_feedback_topic = LaunchConfiguration('command_feedback_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'target_serial_port',
            default_value='/dev/ttyUSB0',
            description='Target MCU serial device owned by rc26_mcu_transport'),
        DeclareLaunchArgument(
            'target_baudrate',
            default_value='1000000',
            description='Target MCU serial baudrate'),
        DeclareLaunchArgument(
            'open_retry_period_ms',
            default_value='1000',
            description='Initial serial open retry period in milliseconds'),
        DeclareLaunchArgument(
            'diagnostics_period_ms',
            default_value='1000',
            description='Diagnostics publish period in milliseconds'),
        DeclareLaunchArgument(
            'send_command_service',
            default_value='/mechanism/send_command',
            description='Mechanism transport send service name'),
        DeclareLaunchArgument(
            'command_feedback_topic',
            default_value='/mechanism/command_feedback',
            description='Mechanism transport feedback topic name'),
        Node(
            package='rc26_mcu_transport',
            executable='mcu_transport_node',
            name='mcu_transport',
            output='screen',
            parameters=[{
                'target_serial_port': target_serial_port,
                'target_baudrate': ParameterValue(target_baudrate, value_type=int),
                'open_retry_period_ms': ParameterValue(open_retry_period_ms, value_type=int),
                'diagnostics_period_ms': ParameterValue(diagnostics_period_ms, value_type=int),
                'send_command_service': send_command_service,
                'command_feedback_topic': command_feedback_topic,
            }],
        ),
    ])
