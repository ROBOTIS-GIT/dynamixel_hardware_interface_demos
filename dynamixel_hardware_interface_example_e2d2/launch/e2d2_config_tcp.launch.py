#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = FindPackageShare('dynamixel_hardware_interface_example_e2d2')

    service_port_name = LaunchConfiguration('service_port_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'service_port_name',
            default_value='e2d2svc:192.168.0.1:5008',
            description='E2D2 UDP configuration service port.',
        ),
        ExecuteProcess(
            cmd=[
                'python3',
                PathJoinSubstitution([
                    package,
                    'scripts',
                    'configure_e2d2_bridge_protocol.py',
                ]),
                '--service-port-name',
                service_port_name,
                '--bridge-protocol',
                '0',
            ],
            output='screen',
        ),
    ])
