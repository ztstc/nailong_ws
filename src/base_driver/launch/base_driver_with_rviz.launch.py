#!/usr/bin/env python3
"""Launch base_driver_node with robot_state_publisher and RViz2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    use_rviz = LaunchConfiguration("use_rviz")

    urdf_file = PathJoinSubstitution([
        FindPackageShare("base_driver"),
        "urdf",
        "nailong_base_model.urdf",
    ])

    robot_description_content = ParameterValue(Command(["cat ", urdf_file]), value_type=str)

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
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to launch RViz2"
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
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description_content}],
        ),
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[],
            output="screen",
        ),
    ])
