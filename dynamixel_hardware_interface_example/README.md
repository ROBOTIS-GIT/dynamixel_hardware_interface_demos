# Dynamixel Hardware Interface Example

This package provides example configuration and launch files for using the `dynamixel_hardware_interface` with ROS 2. It demonstrates how to set up a Dynamixel-based robot system using ROS 2 control, including hardware interface configuration, URDF/XACRO generation, and controller setup.

## Features
- Example configuration for Dynamixel hardware interface
- Automatic generation of XACRO files for custom joint counts
- Launch files for hardware interface and controller manager
- Example controller configuration (joint state broadcaster)

## Directory Structure
```
dynamixel_hardware_interface_example/
├── config/
│   ├── dynamixel_system.ros2_control.xacro   # XACRO for hardware interface
│   ├── dynamixel_system.urdf.xacro           # XACRO for robot description
│   └── ros2_controllers.yaml                 # Controller manager config
├── launch/
│   ├── generate_xacros.launch.py             # Launch to generate XACROs
│   └── hardware.launch.py                    # Launch hardware interface
├── dynamixel_hardware_interface_example/
│   ├── __init__.py
│   └── generate_xacros.py                    # Script to generate XACROs
├── package.xml
├── setup.py
├── setup.cfg
└── CHANGELOG.rst
```

## Usage

### 1. Generate XACRO Files
You can generate custom XACRO files for your robot using the provided script and launch file:

```bash
ros2 launch dynamixel_hardware_interface_example generate_xacros.launch.py \
  num_joints:=4 \
  baudrate:=4000000 \
  port_name:=/dev/ttyUSB0
```
- `num_joints`: Number of joints/motors in your robot
- `baudrate`: Baudrate for the Dynamixel port
- `port_name`: Serial port for the Dynamixel device

This will generate new XACRO files in the `config/` directory.

### 2. Launch the Hardware Interface
After generating the XACRO files, launch the hardware interface and controller manager:

```bash
ros2 launch dynamixel_hardware_interface_example hardware.launch.py \
  description_file:=dynamixel_system.urdf.xacro \
  prefix:=""
```
- `description_file`: URDF/XACRO file to use (default: `dynamixel_system.urdf.xacro`)
- `prefix`: Optional prefix for joint names

## Configuration Files
- **dynamixel_system.ros2_control.xacro**: Defines the hardware interface and parameters for Dynamixel devices.
- **dynamixel_system.urdf.xacro**: Robot description including joints and links.
- **ros2_controllers.yaml**: Controller manager configuration (e.g., joint state broadcaster).

## Nodes and Scripts
- **generate_xacros.py**: Python script to generate XACRO files for custom joint counts and hardware settings.
- **hardware.launch.py**: Launches the controller manager and robot state publisher with the generated robot description.
- **generate_xacros.launch.py**: Launch file to run the XACRO generation script with arguments.
