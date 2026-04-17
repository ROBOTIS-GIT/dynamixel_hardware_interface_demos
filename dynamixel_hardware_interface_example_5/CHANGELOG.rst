^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dynamixel_hardware_interface_example_5
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2026-04-17)
------------------
* Introduced dynamixel_hardware_interface_example_5 package
* Single ros2_control system "dynamixel_tb3_system" using a Dynamixel-protocol
  enabled OpenCR firmware to drive two wheel Dynamixels (ID 1, 2) and read the
  OpenCR on-board IMU and battery voltage
* Bringup launch with diff_drive_controller, joint_state_broadcaster,
  imu_sensor_broadcaster and a custom opencr_battery_publisher node
