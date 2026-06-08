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
    comm_port_name_1 = LaunchConfiguration('comm_port_name_1')
    comm_port_name_2 = LaunchConfiguration('comm_port_name_2')
    comm_port_name_3 = LaunchConfiguration('comm_port_name_3')
    comm_port_name_4 = LaunchConfiguration('comm_port_name_4')
    comm_port_name_5 = LaunchConfiguration('comm_port_name_5')
    baud_rate = LaunchConfiguration('baud_rate')
    dxl_id_1 = LaunchConfiguration('dxl_id_1')
    dxl_id_2 = LaunchConfiguration('dxl_id_2')
    dxl_id_3 = LaunchConfiguration('dxl_id_3')
    dxl_id_4 = LaunchConfiguration('dxl_id_4')
    dxl_id_5 = LaunchConfiguration('dxl_id_5')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([package, 'config', description_file]),
        ' ',
        'prefix:=', prefix,
        ' ',
        'enable_bridge:=false',
        ' ',
        'enable_comm_test:=true',
        ' ',
        'comm_port_name_1:=', comm_port_name_1,
        ' ',
        'comm_port_name_2:=', comm_port_name_2,
        ' ',
        'comm_port_name_3:=', comm_port_name_3,
        ' ',
        'comm_port_name_4:=', comm_port_name_4,
        ' ',
        'comm_port_name_5:=', comm_port_name_5,
        ' ',
        'baud_rate:=', baud_rate,
        ' ',
        'dxl_id_1:=', dxl_id_1,
        ' ',
        'dxl_id_2:=', dxl_id_2,
        ' ',
        'dxl_id_3:=', dxl_id_3,
        ' ',
        'dxl_id_4:=', dxl_id_4,
        ' ',
        'dxl_id_5:=', dxl_id_5,
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    controller_config = PathJoinSubstitution([package, 'config', 'ros2_controllers.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument('description_file', default_value='e2d2_system.urdf.xacro'),
        DeclareLaunchArgument(
            'comm_port_name_1',
            default_value='e2d2udp:192.168.0.1:5001',
            description='E2D2 UDP bridge port for DXL channel 1.',
        ),
        DeclareLaunchArgument(
            'comm_port_name_2',
            default_value='e2d2udp:192.168.0.1:5002',
            description='E2D2 UDP bridge port for DXL channel 2.',
        ),
        DeclareLaunchArgument(
            'comm_port_name_3',
            default_value='e2d2udp:192.168.0.1:5003',
            description='E2D2 UDP bridge port for DXL channel 3.',
        ),
        DeclareLaunchArgument(
            'comm_port_name_4',
            default_value='e2d2udp:192.168.0.1:5004',
            description='E2D2 UDP bridge port for DXL channel 4.',
        ),
        DeclareLaunchArgument(
            'comm_port_name_5',
            default_value='e2d2udp:192.168.0.1:5005',
            description='E2D2 UDP bridge port for DXL channel 5.',
        ),
        DeclareLaunchArgument('baud_rate', default_value='6000000'),
        DeclareLaunchArgument('dxl_id_1', default_value='1'),
        DeclareLaunchArgument('dxl_id_2', default_value='1'),
        DeclareLaunchArgument('dxl_id_3', default_value='1'),
        DeclareLaunchArgument('dxl_id_4', default_value='1'),
        DeclareLaunchArgument('dxl_id_5', default_value='1'),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controller_config],
            output='both',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='both',
        ),
    ])
