#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
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
# Bringup launch file for the dynamixel_tb3_system example (example_5).
# Loads a single ros2_control system that drives the two TurtleBot3 wheel
# Dynamixels and reads the OpenCR (custom Dynamixel-protocol firmware) IMU
# and battery voltage. Spawns standard broadcasters/controllers and a small
# Python node that publishes the OpenCR battery voltage as
# sensor_msgs/BatteryState.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of joint/frame names.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='dynamixel_tb3_system.urdf.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyACM0',
            description='Serial port for the OpenCR (Dynamixel-protocol firmware).',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'baud_rate',
            default_value='1000000',
            description='Baudrate for the OpenCR Dynamixel bus.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'left_wheel_id',
            default_value='1',
            description='Dynamixel ID of the left wheel motor.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'right_wheel_id',
            default_value='2',
            description='Dynamixel ID of the right wheel motor.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'opencr_id',
            default_value='100',
            description='Dynamixel ID emulated by the OpenCR for IMU and battery.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_diff_drive',
            default_value='true',
            description='Spawn diff_drive_controller (set false to use raw '
                        'velocity commands or your own controller).',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'battery_publish_rate',
            default_value='1.0',
            description='Battery state publish rate [Hz].',
        )
    )

    prefix = LaunchConfiguration('prefix')
    description_file = LaunchConfiguration('description_file')
    port_name = LaunchConfiguration('port_name')
    baud_rate = LaunchConfiguration('baud_rate')
    left_wheel_id = LaunchConfiguration('left_wheel_id')
    right_wheel_id = LaunchConfiguration('right_wheel_id')
    opencr_id = LaunchConfiguration('opencr_id')
    use_diff_drive = LaunchConfiguration('use_diff_drive')
    battery_publish_rate = LaunchConfiguration('battery_publish_rate')

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('dynamixel_hardware_interface_example_5'),
            'config',
            'ros2_controllers.yaml',
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('dynamixel_hardware_interface_example_5'),
                    'config',
                    description_file,
                ]
            ),
            ' ',
            'prefix:=', prefix,
            ' ',
            'port_name:=', port_name,
            ' ',
            'baud_rate:=', baud_rate,
            ' ',
            'left_wheel_id:=', left_wheel_id,
            ' ',
            'right_wheel_id:=', right_wheel_id,
            ' ',
            'opencr_id:=', opencr_id,
        ]
    )

    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content, value_type=str)
    }

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='both',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    imu_sensor_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster'],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        condition=IfCondition(use_diff_drive),
    )

    opencr_battery_publisher_node = Node(
        package='dynamixel_hardware_interface_example_5',
        executable='opencr_battery_publisher',
        name='opencr_battery_publisher',
        output='both',
        parameters=[{
            'opencr_id': ParameterValue(opencr_id, value_type=int),
            'publish_rate': ParameterValue(
                battery_publish_rate, value_type=float),
        }],
    )

    nodes = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        diff_drive_controller_spawner,
        # opencr_battery_publisher_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
