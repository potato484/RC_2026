# ros2 launch rc26_telecontrol wheeltec_joy.launch.py control_mode:=dpad
# ros2 launch rc26_telecontrol wheeltec_joy.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('rc26_telecontrol')
    config_stick = os.path.join(pkg_share, 'config', 'joy_params.yaml')
    config_dpad = os.path.join(pkg_share, 'config', 'joy_params_dpad.yaml')

    control_mode_is_stick = IfCondition(
        PythonExpression(["'", LaunchConfiguration('control_mode'), "' == 'stick'"])
    )
    control_mode_is_dpad = IfCondition(
        PythonExpression(["'", LaunchConfiguration('control_mode'), "' == 'dpad'"])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'control_mode',
            default_value='stick',
            description='Control mode: stick(dual joystick) or dpad(directional pad + buttons)',
            choices=['stick', 'dpad']
        ),
        DeclareLaunchArgument(
            'v_linear',
            default_value='0.2',
            description='Maximum linear velocity (m/s)'
        ),
        DeclareLaunchArgument(
            'chassis_model',
            default_value='tracked_diff',
            description='Chassis model: mecanum_4wheel | tracked_diff'
        ),
        DeclareLaunchArgument(
            'v_angular',
            default_value='0.5',
            description='Maximum angular velocity (rad/s)'
        ),
        DeclareLaunchArgument(
            'joy_deadzone',
            default_value='0.15',
            description='Joystick deadzone threshold (only for stick mode)'
        ),
        DeclareLaunchArgument(
            'deadzone_hyst',
            default_value='0.02',
            description='Deadzone hysteresis width (only for stick mode)'
        ),
        DeclareLaunchArgument(
            'joy_timeout_s',
            default_value='0.3',
            description='Joystick message timeout in seconds (<=0 to disable watchdog)'
        ),
        DeclareLaunchArgument(
            'max_accel',
            default_value='1.5',
            description='Maximum linear acceleration (m/s^2)'
        ),
        DeclareLaunchArgument(
            'max_alpha',
            default_value='3.0',
            description='Maximum angular acceleration (rad/s^2)'
        ),
        DeclareLaunchArgument(
            'stop_repeat_n',
            default_value='10',
            description='Number of repeated zero-velocity stop frames'
        ),
        DeclareLaunchArgument(
            'require_deadman',
            default_value='false',
            description='Whether deadman safety button is required'
        ),
        DeclareLaunchArgument(
            'deadman_button',
            default_value='4',
            description='Deadman button index'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[config_stick],
            condition=control_mode_is_stick
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[config_dpad],
            condition=control_mode_is_dpad
        ),
        Node(
            package='rc26_telecontrol',
            executable='rc26_telecontrol',
            name='rc26_telecontrol',
            output='screen',
            parameters=[
                config_stick,
                {
                    'chassis_model': LaunchConfiguration('chassis_model'),
                    'v_linear': LaunchConfiguration('v_linear'),
                    'v_angular': LaunchConfiguration('v_angular'),
                    'joy_deadzone': LaunchConfiguration('joy_deadzone'),
                    'deadzone_hyst': LaunchConfiguration('deadzone_hyst'),
                    'joy_timeout_s': LaunchConfiguration('joy_timeout_s'),
                    'max_accel': LaunchConfiguration('max_accel'),
                    'max_alpha': LaunchConfiguration('max_alpha'),
                    'stop_repeat_n': LaunchConfiguration('stop_repeat_n'),
                    'require_deadman': LaunchConfiguration('require_deadman'),
                    'deadman_button': LaunchConfiguration('deadman_button')
                }
            ],
            condition=control_mode_is_stick
        ),
        Node(
            package='rc26_telecontrol',
            executable='rc26_telecontrol_dpad',
            name='rc26_telecontrol_dpad',
            output='screen',
            parameters=[
                config_dpad,
                {
                    'chassis_model': LaunchConfiguration('chassis_model'),
                    'v_linear': LaunchConfiguration('v_linear'),
                    'v_angular': LaunchConfiguration('v_angular'),
                    'joy_timeout_s': LaunchConfiguration('joy_timeout_s'),
                    'max_accel': LaunchConfiguration('max_accel'),
                    'max_alpha': LaunchConfiguration('max_alpha'),
                    'stop_repeat_n': LaunchConfiguration('stop_repeat_n'),
                    'require_deadman': LaunchConfiguration('require_deadman'),
                    'deadman_button': LaunchConfiguration('deadman_button')
                }
            ],
            condition=control_mode_is_dpad
        ),
    ])
