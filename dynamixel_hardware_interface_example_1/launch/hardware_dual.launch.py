#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Woojin Wie


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint names'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='dynamixel_dual_system.urdf.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'port_name_1',
            default_value='/dev/ttyUSB0',
            description='Port name for the first Dynamixel device.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'port_name_2',
            default_value='/dev/ttyUSB1',
            description='Port name for the second Dynamixel device.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'baud_rate_1',
            default_value='4000000',
            description='Baudrate for the first Dynamixel port.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'baud_rate_2',
            default_value='4000000',
            description='Baudrate for the second Dynamixel port.'
        )
    )

    description_file = LaunchConfiguration('description_file')
    prefix = LaunchConfiguration('prefix')
    port_name_1 = LaunchConfiguration('port_name_1')
    port_name_2 = LaunchConfiguration('port_name_2')
    baud_rate_1 = LaunchConfiguration('baud_rate_1')
    baud_rate_2 = LaunchConfiguration('baud_rate_2')

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('dynamixel_hardware_interface_example_1'),
            'config',
            'ros2_controllers.yaml',
        ]
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='both',
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('dynamixel_hardware_interface_example_1'),
                    'config',
                    description_file,
                ]
            ),
            ' ',
            'prefix:=',
            prefix,
            ' ',
            'port_name_1:=',
            port_name_1,
            ' ',
            'port_name_2:=',
            port_name_2,
            ' ',
            'baud_rate_1:=',
            baud_rate_1,
            ' ',
            'baud_rate_2:=',
            baud_rate_2,
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
        ],
        parameters=[robot_description],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    nodes = [
        control_node,
        robot_controller_spawner,
        robot_state_publisher_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
