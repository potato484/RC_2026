#!/usr/bin/env python3
# 仅启动串口轮式里程计节点

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("rc26_merge_odom")
    params_file = os.path.join(pkg_share, "config", "merge_odom_params.yaml")

    serial_port_arg = DeclareLaunchArgument(
        "serial_port", default_value="/dev/ttyUSB0", description="MCU serial port for ODOM_DATA"
    )

    wheel_odom_node = Node(
        package="rc26_merge_odom",
        executable="wheel_odom_node",
        name="wheel_odom_node",
        output="screen",
        parameters=[
            params_file,
            {
                "serial_port": LaunchConfiguration("serial_port"),
            },
        ],
    )

    return LaunchDescription(
        [
            serial_port_arg,
            wheel_odom_node,
        ]
    )

