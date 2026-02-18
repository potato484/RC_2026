from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mid360_driver_node = Node(
        package="mid360_driver",
        executable="mid360_driver_node",
        name="mid360_driver",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("mid360_driver"),
                    "config",
                    "param.yaml",
                ]
            )
        ],
    )

    return LaunchDescription([mid360_driver_node])
