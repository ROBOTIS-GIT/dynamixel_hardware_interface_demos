# Dynamixel Hardware Interface Example 1 (Dual ros2_control)

This package demonstrates a setup with two ros2_control interfaces using `dynamixel_hardware_interface`, each marked with `is_async="true"`.

## Usage

Build and source your workspace, then run:

```bash
ros2 launch dynamixel_hardware_interface_example_1 hardware_dual.launch.py \
  port_name_1:=/dev/ttyUSB0 \
  port_name_2:=/dev/ttyUSB1 \
  baud_rate_1:=4000000 \
  baud_rate_2:=4000000
```

The robot description is defined in `config/dynamixel_dual_system.urdf.xacro` and includes two ros2_control system blocks, each on a different port.
