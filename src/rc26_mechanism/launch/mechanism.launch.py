from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    serial_baud = LaunchConfiguration('serial_baud')
    hal_type = LaunchConfiguration('hal_type')
    sim_action_latency_ms = LaunchConfiguration('sim_action_latency_ms')
    sim_fail_rate = LaunchConfiguration('sim_fail_rate')
    sim_fail_error_code = LaunchConfiguration('sim_fail_error_code')
    fault_mode = LaunchConfiguration('fault_mode')
    fault_action_latency_ms = LaunchConfiguration('fault_action_latency_ms')
    fault_error_code = LaunchConfiguration('fault_error_code')
    replay_file = LaunchConfiguration('replay_file')
    replay_action_latency_ms = LaunchConfiguration('replay_action_latency_ms')
    replay_loop = LaunchConfiguration('replay_loop')

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
    hal_type_arg = DeclareLaunchArgument(
        'hal_type',
        default_value='serial',
        description='Mechanism HAL type: serial|shared_serial|sim|fault|replay',
    )
    sim_action_latency_arg = DeclareLaunchArgument(
        'sim_action_latency_ms',
        default_value='500',
        description='Sim HAL action latency in milliseconds',
    )
    sim_fail_rate_arg = DeclareLaunchArgument(
        'sim_fail_rate',
        default_value='0.0',
        description='Sim HAL fail rate (0.0-1.0)',
    )
    sim_fail_error_code_arg = DeclareLaunchArgument(
        'sim_fail_error_code',
        default_value='1',
        description='Sim HAL ACTION_FAIL error code',
    )
    fault_mode_arg = DeclareLaunchArgument(
        'fault_mode',
        default_value='action_fail_payload',
        description='Fault HAL mode: none|action_fail_payload|action_fail_empty|no_feedback|out_of_order|duplicate_done|late_success',
    )
    fault_action_latency_arg = DeclareLaunchArgument(
        'fault_action_latency_ms',
        default_value='200',
        description='Fault HAL action latency in milliseconds',
    )
    fault_error_code_arg = DeclareLaunchArgument(
        'fault_error_code',
        default_value='1',
        description='Fault HAL ACTION_FAIL error code',
    )
    replay_file_arg = DeclareLaunchArgument(
        'replay_file',
        default_value='',
        description='Replay HAL csv file path (seq,fb_id,payload_hex)',
    )
    replay_action_latency_arg = DeclareLaunchArgument(
        'replay_action_latency_ms',
        default_value='100',
        description='Replay HAL dispatch latency in milliseconds',
    )
    replay_loop_arg = DeclareLaunchArgument(
        'replay_loop',
        default_value='false',
        description='Replay HAL loop events when file reaches end',
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
                    'hal_type': hal_type,
                    'sim_action_latency_ms': ParameterValue(sim_action_latency_ms, value_type=int),
                    'sim_fail_rate': ParameterValue(sim_fail_rate, value_type=float),
                    'sim_fail_error_code': ParameterValue(sim_fail_error_code, value_type=int),
                    'fault_mode': fault_mode,
                    'fault_action_latency_ms': ParameterValue(fault_action_latency_ms, value_type=int),
                    'fault_error_code': ParameterValue(fault_error_code, value_type=int),
                    'replay_file': replay_file,
                    'replay_action_latency_ms': ParameterValue(replay_action_latency_ms, value_type=int),
                    'replay_loop': ParameterValue(replay_loop, value_type=bool),
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )
    return LaunchDescription([
        serial_port_arg,
        serial_baud_arg,
        hal_type_arg,
        sim_action_latency_arg,
        sim_fail_rate_arg,
        sim_fail_error_code_arg,
        fault_mode_arg,
        fault_action_latency_arg,
        fault_error_code_arg,
        replay_file_arg,
        replay_action_latency_arg,
        replay_loop_arg,
        container,
    ])
