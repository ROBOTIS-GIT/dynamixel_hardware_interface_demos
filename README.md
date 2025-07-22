# dynamixel_hardware_interface_demos

This repository collects example packages and resources for working with Dynamixel hardware using the ros2_control framework. 

## Overview
This repository is intended to help users get started with Dynamixel hardware integration in ROS 2. It provides example configurations, launch files, and scripts to demonstrate how to use the `dynamixel_hardware_interface` with ros2_control and controller_manager.

## Included Packages

- [dynamixel_hardware_interface_example](dynamixel_hardware_interface_example/README.md)
  - Example package with configuration files, launch files, and scripts for setting up and running a Dynamixel-based robot system using ros2_control.

## Getting Started

### Prerequisites
- ROS 2 (Jazzy or later recommended)
- Dynamixel hardware (e.g., motors, U2D2, etc.)
- The `dynamixel_hardware_interface` package and its dependencies

### Installation
Clone this repository into your ROS 2 workspace and install dependencies:

```bash
cd <your_ros2_ws>/src
git clone https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface_demos.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```
