from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='Serial port for DM-IMU'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='921600',
            description='Baudrate for DM-IMU'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='imu_link',
            description='Frame ID for IMU messages'
        ),
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='Enable verbose logging'
        ),
        Node(
            package='rc26_merge_odom',
            executable='dm_imu_node',
            name='dm_imu_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'verbose': LaunchConfiguration('verbose'),
            }],
        ),
    ])
