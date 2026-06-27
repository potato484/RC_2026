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
    enable_chassis_cmd_vel_consumer = LaunchConfiguration('enable_chassis_cmd_vel_consumer')
    chassis_cmd_vel_topic = LaunchConfiguration('chassis_cmd_vel_topic')
    chassis_target_send_rate_hz = LaunchConfiguration('chassis_target_send_rate_hz')
    chassis_cmd_vel_timeout_ms = LaunchConfiguration('chassis_cmd_vel_timeout_ms')
    chassis_v_max_mps = LaunchConfiguration('chassis_v_max_mps')
    chassis_w_max_radps = LaunchConfiguration('chassis_w_max_radps')
    chassis_stop_repeat_n = LaunchConfiguration('chassis_stop_repeat_n')

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
        DeclareLaunchArgument(
            'enable_chassis_cmd_vel_consumer',
            default_value='true',
            description='Whether rc26_mcu_transport subscribes cmd_vel and sends POSE_TARGET to chassis MCU'),
        DeclareLaunchArgument(
            'chassis_cmd_vel_topic',
            default_value='cmd_vel',
            description='Chassis velocity command topic consumed by rc26_mcu_transport'),
        DeclareLaunchArgument(
            'chassis_target_send_rate_hz',
            default_value='50',
            description='POSE_TARGET send rate in Hz'),
        DeclareLaunchArgument(
            'chassis_cmd_vel_timeout_ms',
            default_value='200',
            description='Timeout before sending repeated zero POSE_TARGET frames'),
        DeclareLaunchArgument(
            'chassis_v_max_mps',
            default_value='2.0',
            description='Planar linear speed limit in m/s for chassis cmd_vel consumer'),
        DeclareLaunchArgument(
            'chassis_w_max_radps',
            default_value='2.0',
            description='Angular speed limit in rad/s for chassis cmd_vel consumer'),
        DeclareLaunchArgument(
            'chassis_stop_repeat_n',
            default_value='10',
            description='Number of zero POSE_TARGET frames sent once cmd_vel times out'),
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
                'enable_chassis_cmd_vel_consumer': ParameterValue(
                    enable_chassis_cmd_vel_consumer, value_type=bool),
                'chassis_cmd_vel_topic': chassis_cmd_vel_topic,
                'chassis_target_send_rate_hz': ParameterValue(chassis_target_send_rate_hz, value_type=int),
                'chassis_cmd_vel_timeout_ms': ParameterValue(chassis_cmd_vel_timeout_ms, value_type=int),
                'chassis_v_max_mps': ParameterValue(chassis_v_max_mps, value_type=float),
                'chassis_w_max_radps': ParameterValue(chassis_w_max_radps, value_type=float),
                'chassis_stop_repeat_n': ParameterValue(chassis_stop_repeat_n, value_type=int),
            }],
        ),
    ])
