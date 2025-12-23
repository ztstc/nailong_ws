#!/usr/bin/env python3
"""Launch base_driver_node only."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="Serial port device for the base driver (e.g. /dev/ttyACM0)"
        ),
        DeclareLaunchArgument(
            "baudrate",
            default_value="115200",
            description="Serial port baudrate"
        ),
        Node(
            package="base_driver",
            executable="base_driver_node",
            name="uart_protocol_node",
            output="screen",
            parameters=[{
                "serial_port": serial_port,
                "baudrate": baudrate,
            }],
        ),
    ])
