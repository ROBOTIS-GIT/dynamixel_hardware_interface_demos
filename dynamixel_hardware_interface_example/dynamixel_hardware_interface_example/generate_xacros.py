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


import sys
import os

# Generate the .ros2_control.xacro file
def generate_ros2_control_xacro(num_joints, filename, baudrate, port_name):
    matrix = ""
    for i in range(num_joints):
        row = ["1" if i == j else "0" for j in range(num_joints)]
        line = ", ".join(row)
        if i < num_joints - 1:
            matrix += f"        {line},\n"
        else:
            matrix += f"        {line}\n"

    with open(filename, "w") as f:
        f.write('<?xml version="1.0"?>\n')
        f.write('<robot xmlns:xacro="http://www.ros.org/wiki/xacro">\n')
        f.write('  <xacro:macro name="dynamixel_system_ros2_control" params="name">\n')
        f.write('    <ros2_control name="${name}" type="system">\n')
        f.write('      <hardware>\n')
        f.write('        <plugin>dynamixel_hardware_interface/DynamixelHardware</plugin>\n')
        f.write(f'        <param name="port_name">{port_name}</param>\n')
        f.write(f'        <param name="baud_rate">{baudrate}</param>\n')
        f.write('        <param name="dynamixel_model_folder">/param/dxl_model</param>\n')
        f.write(f'        <param name="number_of_joints">{num_joints}</param>\n')
        f.write(f'        <param name="number_of_transmissions">{num_joints}</param>\n')
        f.write('        <param name="disable_torque_at_init">true</param>\n')
        f.write('        <param name="transmission_to_joint_matrix">\n')
        f.write(matrix)
        f.write('        </param>\n')
        f.write('        <param name="joint_to_transmission_matrix">\n')
        f.write(matrix)
        f.write('        </param>\n')
        f.write('        <param name="dynamixel_state_pub_msg_name">dynamixel_hardware_interface/dxl_state</param>\n')
        f.write('        <param name="get_dynamixel_data_srv_name">dynamixel_hardware_interface/get_dxl_data</param>\n')
        f.write('        <param name="set_dynamixel_data_srv_name">dynamixel_hardware_interface/set_dxl_data</param>\n')
        f.write('        <param name="reboot_dxl_srv_name">dynamixel_hardware_interface/reboot_dxl</param>\n')
        f.write('        <param name="set_dxl_torque_srv_name">dynamixel_hardware_interface/set_dxl_torque</param>\n')
        f.write('      </hardware>\n')
        f.write('\n')
        for i in range(1, num_joints+1):
            f.write(f'      <joint name="joint{i}">\n')
            f.write('        <command_interface name="position"/>\n')
            f.write('        <state_interface name="position"/>\n')
            f.write('        <state_interface name="velocity"/>\n')
            f.write('      </joint>\n')
        f.write('\n')
        for i in range(1, num_joints+1):
            f.write(f'      <gpio name="dxl{i}">\n')
            f.write('        <param name="type">dxl</param>\n')
            f.write(f'        <param name="ID">{i}</param>\n')
            f.write('        <command_interface name="Goal Position"/>\n')
            f.write('        <state_interface name="Present Position"/>\n')
            f.write('        <state_interface name="Present Velocity"/>\n')
            f.write('        <param name="Return Delay Time">0</param>\n')
            f.write('      </gpio>\n')
        f.write('    </ros2_control>\n')
        f.write('  </xacro:macro>\n')
        f.write('</robot>\n')

# Generate the .urdf.xacro file
def generate_urdf_xacro(num_joints, filename):
    with open(filename, "w") as f:
        f.write('<?xml version="1.0"?>\n')
        f.write('<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="dynamixel_system">\n')
        f.write('  <xacro:arg name="prefix" default="" />\n')
        f.write('  <xacro:include filename="dynamixel_system.ros2_control.xacro" />\n')
        f.write('\n')
        f.write('  <xacro:dynamixel_system_ros2_control name="dynamixel_system"/>\n')
        f.write('\n')
        f.write('  <link name="$(arg prefix)base_link"/>\n')
        f.write('\n')
        for i in range(1, num_joints+1):
            f.write(f'  <joint name="$(arg prefix)joint{i}" type="revolute">\n')
            parent = "base_link" if i == 1 else f"link{i-1}"
            f.write(f'    <parent link="$(arg prefix){parent}"/>\n')
            f.write(f'    <child link="$(arg prefix)link{i}"/>\n')
            f.write('    <origin xyz="0 0 0" rpy="0 0 0"/>\n')
            f.write('    <axis xyz="1 0 0" />\n')
            f.write('    <limit velocity="4.8" effort="1" lower="${-pi*100.0}" upper="${pi*100.0}" />\n')
            f.write('  </joint>\n')
            f.write(f'  <link name="$(arg prefix)link{i}"/>\n')
        f.write('</robot>\n')

if __name__ == "__main__":
    main()

def main():
    if len(sys.argv) < 2:
        print("Usage: python generate_dynamixel_xacros.py <number_of_joints> [output_dir] [baudrate] [port_name]")
        sys.exit(1)
    try:
        num_joints = int(sys.argv[1])
    except ValueError:
        print("First argument must be an integer (number of joints)")
        sys.exit(1)
    config_dir = os.path.join(os.path.dirname(__file__), "config")
    baudrate = "4500000"
    port_name = "/dev/ttyUSB0"
    if len(sys.argv) >= 3:
        config_dir = sys.argv[2]
    if len(sys.argv) >= 4:
        baudrate = sys.argv[3]
    if len(sys.argv) >= 5:
        port_name = sys.argv[4]
    os.makedirs(config_dir, exist_ok=True)
    ros2_control_path = os.path.join(config_dir, "dynamixel_system.ros2_control.xacro")
    urdf_xacro_path = os.path.join(config_dir, "dynamixel_system.urdf.xacro")
    generate_ros2_control_xacro(num_joints, ros2_control_path, baudrate, port_name)
    generate_urdf_xacro(num_joints, urdf_xacro_path)
    print(f"Generated xacro files for {num_joints} joints in {config_dir}. Baudrate: {baudrate}, Port: {port_name}")
