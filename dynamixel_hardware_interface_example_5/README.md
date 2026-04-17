# Dynamixel Hardware Interface Example 5 (TurtleBot3 / OpenCR `dynamixel_tb3_system`)

This example brings up a TurtleBot3 platform whose **OpenCR** runs a custom
Dynamixel-Protocol firmware (model: **OPENCR-TB3**, ModelNumber **6040**,
Type `Dynamixel2`). From the host's point of view, the OpenCR USB port looks
like a single Dynamixel bus on which three Dynamixel-protocol devices live:

| Bus ID  | Role                | Notes                                                      |
|--------:|---------------------|------------------------------------------------------------|
| **1**   | Left wheel motor    | Standard X-series wheel Dynamixel, velocity mode           |
| **2**   | Right wheel motor   | Standard X-series wheel Dynamixel, velocity mode           |
| **100** | OpenCR (sensor)     | OpenCR firmware exposing on-board IMU + Battery Voltage    |

A single `ros2_control` system named **`dynamixel_tb3_system`** is configured
to talk to all three through `dynamixel_hardware_interface`. The OpenCR
firmware exposes the IMU registers under the standard names that
`imu_sensor_broadcaster` expects (`orientation.{w,x,y,z}`,
`angular_velocity.{x,y,z}`, `linear_acceleration.{x,y,z}`), so the IMU is
published with no extra translation.

Loaded controllers / broadcasters:

- `joint_state_broadcaster` – wheel joint states on `/joint_states`
- `imu_sensor_broadcaster`  – `/imu` (`sensor_msgs/Imu`)
- `diff_drive_controller`   – `/cmd_vel` → wheel velocities, publishes `/odom`

A small Python node, **`opencr_battery_publisher`**, additionally polls the
OpenCR's `Battery Voltage (mV)` register through the
`dynamixel_hardware_interface/get_dxl_data` service and publishes it as
`sensor_msgs/BatteryState` on `/battery_state` (no standard ros2_controllers
broadcaster exists for voltage).

## OpenCR control table mapping

The OpenCR firmware ships with the model file `opencr_tb3.model`. The items
relevant to this example are:

| Item name                       | Addr | Len | Sign  | Scale       | Unit   | Used by                   |
|---------------------------------|-----:|----:|-------|-------------|--------|---------------------------|
| `Battery Voltage (mV)`          |   40 |   2 | uint  | × 0.001     | V      | `opencr_battery_publisher`|
| `orientation.{w,x,y,z}`         |80–92 |   4 | int32 | × 1e-6      | (none) | `imu_sensor_broadcaster`  |
| `angular_velocity.{x,y,z}`      |96–100|   2 | int16 | × 0.001064  | rad/s  | `imu_sensor_broadcaster`  |
| `linear_acceleration.{x,y,z}`   |102–106|  2 | int16 | × 0.004789  | m/s²   | `imu_sensor_broadcaster`  |

## Deploying the model file

`dynamixel_hardware_interface` reads its register map from
`dynamixel_model_folder` (default in the xacro: `/param/dxl_model`). Copy the
firmware's model file there before bringing up the system:

```bash
sudo mkdir -p /param/dxl_model
sudo cp /home/hc/playground/ai_sapiens_mcu_firmware/projects/opencr_tb3/opencr_tb3.model \
        /param/dxl_model/
```

The X-series wheel models (for ID 1, 2) are usually already present in the
folder via the existing dynamixel model bundle.

## Layout

```
dynamixel_hardware_interface_example_5/
├── config/
│   ├── dynamixel_tb3_system.urdf.xacro           # Top-level URDF
│   ├── dynamixel_tb3_system.ros2_control.xacro   # Macro defining the system
│   └── ros2_controllers.yaml                     # Controller params
├── dynamixel_hardware_interface_example_5/
│   └── opencr_battery_publisher.py               # Battery publisher
├── launch/
│   └── bringup.launch.py                         # Bringup launch
├── package.xml
├── setup.py / setup.cfg
└── README.md
```

## Usage

Build and source the workspace, then launch:

```bash
ros2 launch dynamixel_hardware_interface_example_5 bringup.launch.py \
    port_name:=/dev/ttyACM0 \
    baud_rate:=1000000
```

### Useful published topics

| Topic            | Type                       | Source                           |
|------------------|----------------------------|----------------------------------|
| `/joint_states`  | `sensor_msgs/JointState`   | `joint_state_broadcaster`        |
| `/imu`           | `sensor_msgs/Imu`          | `imu_sensor_broadcaster`         |
| `/odom`          | `nav_msgs/Odometry`        | `diff_drive_controller`          |
| `/battery_state` | `sensor_msgs/BatteryState` | `opencr_battery_publisher`       |

### Useful subscribed topics

| Topic       | Type                  | Sink                     |
|-------------|-----------------------|--------------------------|
| `/cmd_vel`  | `geometry_msgs/Twist` | `diff_drive_controller`  |

### Launch arguments

| Argument               | Default                                  | Description |
|------------------------|------------------------------------------|-------------|
| `port_name`            | `/dev/ttyACM0`                           | Serial port for the OpenCR. |
| `baud_rate`            | `1000000`                                | Baudrate for the bus. |
| `description_file`     | `dynamixel_tb3_system.urdf.xacro`        | Top-level xacro. |
| `prefix`               | `""`                                     | Prefix for joint/frame names. |
| `left_wheel_id`        | `1`                                      | Dynamixel ID for the left wheel. |
| `right_wheel_id`       | `2`                                      | Dynamixel ID for the right wheel. |
| `opencr_id`            | `100`                                    | Dynamixel ID emulated by the OpenCR. |
| `use_diff_drive`       | `true`                                   | Spawn `diff_drive_controller`. |
| `battery_publish_rate` | `1.0`                                    | Battery publish rate [Hz]. |
