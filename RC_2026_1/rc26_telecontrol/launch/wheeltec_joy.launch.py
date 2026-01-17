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
    # 获取包共享目录
    pkg_share = get_package_share_directory('rc26_telecontrol')
    
    # 配置文件路径
    config_stick = os.path.join(pkg_share, 'config', 'joy_params.yaml')
    config_dpad = os.path.join(pkg_share, 'config', 'joy_params_dpad.yaml')
    
    return LaunchDescription([
        # ========== 模式选择参数 ==========
        DeclareLaunchArgument(
            'control_mode',
            default_value='stick',
            description='Control mode: stick(dual joystick) or dpad(directional pad + buttons)',
            choices=['stick', 'dpad']
        ),
        
        # ========== 通用参数 ==========
        DeclareLaunchArgument(
            'v_linear',
            default_value='0.2',
            description='Maximum linear velocity (m/s)'
        ),
        DeclareLaunchArgument(
            'v_angular',
            default_value='0.5',
            description='Maximum angular velocity (rad/s)'
        ),
        DeclareLaunchArgument(
            'smoothing_alpha',
            default_value='0.2',
            description='Speed smoothing coefficient (0.0~1.0)'
        ),
        # ========== Stick模式专用参数 ==========
        DeclareLaunchArgument(
            'joy_deadzone',
            default_value='0.15',
            description='Joystick deadzone threshold (only for stick mode)'
        ),
        
        # ========== Joy节点（手柄驱动） ==========
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[config_stick]
        ),
        
        # ========== Stick模式控制节点 ==========
        Node(
            package='rc26_telecontrol',
            executable='rc26_telecontrol',
            name='rc26_telecontrol',
            output='screen',
            parameters=[
                {
                    'v_linear': LaunchConfiguration('v_linear'),
                    'v_angular': LaunchConfiguration('v_angular'),
                    'joy_deadzone': LaunchConfiguration('joy_deadzone'),
                    'smoothing_alpha': LaunchConfiguration('smoothing_alpha')
                },
                config_stick
            ],
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('control_mode'), "' == 'stick'"])
            )
        ),

        # ========== D-Pad模式控制节点 ==========
        Node(
            package='rc26_telecontrol',
            executable='rc26_telecontrol_dpad',
            name='rc26_telecontrol_dpad',
            output='screen',
            parameters=[
                {
                    'v_linear': LaunchConfiguration('v_linear'),
                    'v_angular': LaunchConfiguration('v_angular'),
                    'smoothing_alpha': LaunchConfiguration('smoothing_alpha')
                },
                config_dpad
            ],
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('control_mode'), "' == 'dpad'"])
            )
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node_dpad',
            parameters=[config_dpad],
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('control_mode'), "' == 'dpad'"])
            )
        ),
    ])
