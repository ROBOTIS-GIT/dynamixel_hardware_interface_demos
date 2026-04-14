# Dynamixel Hardware Interface Example 2 (IMU ros2_control)

This example shows a single `ros2_control` system that uses `dynamixel_hardware_interface` to read an IMU (ID 1) and publish its data through an `imu_sensor_broadcaster`. The robot description is built from `config/dynamixel_system.urdf.xacro`, which wraps the ros2_control configuration defined in `config/dynamixel_system.ros2_control.xacro`.

## Usage

Build and source your workspace, then launch:

```bash
ros2 launch dynamixel_hardware_interface_example_2 hardware.launch.py \
  port_name:=/dev/ttyUSB0 \
  baud_rate:=4000000
```

### Launch arguments
- `port_name` (default `/dev/ttyUSB0`): Serial port connected to the Dynamixel IMU.
- `baud_rate` (default `4000000`): Baud rate for the port.
- `description_file` (default `dynamixel_system.urdf.xacro`): Xacro to generate the robot description.
- `prefix` (default empty): Optional prefix for joint/frame names.
