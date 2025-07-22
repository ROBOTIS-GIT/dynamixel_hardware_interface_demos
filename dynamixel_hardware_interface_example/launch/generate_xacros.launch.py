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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "num_joints",
            default_value="1",
            description="Number of joints to generate in the xacro files."
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value=os.path.join(
                get_package_share_directory('dynamixel_hardware_interface_example'),
                'config'
            ),
            description="Directory to save generated xacro files."
        ),
        DeclareLaunchArgument(
            "baudrate",
            default_value="4000000",
            description="Baudrate for the Dynamixel port."
        ),
        DeclareLaunchArgument(
            "port_name",
            default_value="/dev/ttyUSB0",
            description="Port name for the Dynamixel device."
        ),
    ]

    num_joints = LaunchConfiguration("num_joints")
    output_dir = LaunchConfiguration("output_dir")
    baudrate = LaunchConfiguration("baudrate")
    port_name = LaunchConfiguration("port_name")

    generate_xacros_node = Node(
        package="dynamixel_hardware_interface_example",
        executable="generate_xacros",
        name="generate_xacros",
        output="screen",
        arguments=[num_joints, output_dir, baudrate, port_name],
    )

    return LaunchDescription(declared_arguments + [generate_xacros_node])
