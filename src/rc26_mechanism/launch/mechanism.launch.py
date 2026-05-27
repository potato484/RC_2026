from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    hal_type = LaunchConfiguration('hal_type')

    hal_type_arg = DeclareLaunchArgument(
        'hal_type',
        default_value='shared_serial',
        description='Mechanism HAL type: shared_serial',
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
        container,
    ])
