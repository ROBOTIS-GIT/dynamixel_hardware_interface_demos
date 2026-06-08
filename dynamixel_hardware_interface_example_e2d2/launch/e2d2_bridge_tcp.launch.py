#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = FindPackageShare('dynamixel_hardware_interface_example_e2d2')

    description_file = LaunchConfiguration('description_file')
    prefix = LaunchConfiguration('prefix')
    bridge_port_name = LaunchConfiguration('bridge_port_name')
    baud_rate = LaunchConfiguration('baud_rate')
    dxl_id = LaunchConfiguration('dxl_id')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([package, 'config', description_file]),
        ' ',
        'prefix:=', prefix,
        ' ',
        'enable_bridge:=true',
        ' ',
        'bridge_port_name:=', bridge_port_name,
        ' ',
        'baud_rate:=', baud_rate,
        ' ',
        'dxl_id:=', dxl_id,
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    controller_config = PathJoinSubstitution([package, 'config', 'ros2_controllers.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument('description_file', default_value='e2d2_system.urdf.xacro'),
        DeclareLaunchArgument(
            'bridge_port_name',
            default_value='e2d2tcp:192.168.0.1:5001',
            description='E2D2 TCP bridge port for one DXL bus.',
        ),
        DeclareLaunchArgument('baud_rate', default_value='6000000'),
        DeclareLaunchArgument('dxl_id', default_value='1'),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controller_config],
            output='both',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='both',
        ),
    ])
