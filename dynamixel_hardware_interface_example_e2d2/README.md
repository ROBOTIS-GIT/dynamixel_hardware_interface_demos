# E2D2 ROS 2 Control Example

This package configures an E2D2 H5 Ethernet-to-Dynamixel bridge and then
controls one Dynamixel bus through an E2D2 bridge channel.

## Why There Are Separate Launches

E2D2 board configuration uses the UDP service port:

```text
e2d2svc:192.168.0.1:5008
```

Dynamixel traffic uses bridge ports:

```text
e2d2tcp:192.168.0.1:5001
e2d2udp:192.168.0.1:5001
```

The config launch files send raw Dynamixel Protocol 2.0 write packets to the
E2D2 service port. The bridge launch files use `ros2_control` only for the
actual Dynamixel bus behind an E2D2 bridge port.

## Launch Files

Set E2D2 bridge channels to TCP:

```bash
ros2 launch dynamixel_hardware_interface_example_e2d2 e2d2_config_tcp.launch.py
```

Set E2D2 bridge channels to UDP:

```bash
ros2 launch dynamixel_hardware_interface_example_e2d2 e2d2_config_udp.launch.py
```

These config launches write E2D2 control-table addresses 34..39 directly and
then exit. They do not start `ros2_control_node` and do not need an E2D2 model
file.

After changing TCP/UDP mode, reset or power-cycle the E2D2. The firmware latches
bridge protocol mode during bridge initialization.

Run bridge channel 1 as TCP:

```bash
ros2 launch dynamixel_hardware_interface_example_e2d2 e2d2_bridge_tcp.launch.py
```

Run bridge channel 1 as UDP:

```bash
ros2 launch dynamixel_hardware_interface_example_e2d2 e2d2_bridge_udp.launch.py
```

Run a UDP bridge communication-only check for five DXL PCBs, one PCB on each of
E2D2 channels 1..5:

```bash
ros2 launch dynamixel_hardware_interface_example_e2d2 e2d2_bridge_udp_comm_test.launch.py
```

Useful overrides:

```bash
ros2 launch dynamixel_hardware_interface_example_e2d2 e2d2_bridge_udp.launch.py \
  bridge_port_name:=e2d2udp:192.168.0.1:5002 \
  dxl_id:=1 \
  baud_rate:=6000000
```

For the communication-only launch, the default ports are
`e2d2udp:192.168.0.1:5001` through `5005`, and each channel expects DXL ID `1`
by default. Override the per-channel IDs or ports if needed:

```bash
ros2 launch dynamixel_hardware_interface_example_e2d2 e2d2_bridge_udp_comm_test.launch.py \
  dxl_id_1:=1 dxl_id_2:=1 dxl_id_3:=1 dxl_id_4:=1 dxl_id_5:=1
```

## Required SDK Support

The host ROS package links against `DynamixelSDK/ros/dynamixel_sdk`. This repo's
local SDK overlay includes E2D2 dispatch for these prefixes:

```text
e2d2svc:
e2d2tcp:
e2d2udp:
e2d2:
```

The Docker compose file maps `../libs/DynamixelSDK` to
`/root/ros2_ws/src/DynamixelSDK`, so rebuilding or colcon-building inside the
container uses this patched local SDK.

## Model Files

No E2D2 board model file is required. E2D2 is treated as an Ethernet port
handler for bridge traffic, like U2D2/OpenRB/OpenCR are treated as transport
interfaces in other examples.

Model files are still needed for the actual Dynamixel devices connected behind
the E2D2 bridge. For example, a YM080-230-M001 board reports model number 4120,
which is already present in `dynamixel_hardware_interface/param/dxl_model`.

If package files changed, rebuild the package as usual:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-up-to dynamixel_hardware_interface_example_e2d2
```
