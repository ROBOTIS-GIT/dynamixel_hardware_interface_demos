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
# Periodically queries the OpenCR's "Battery Voltage (mV)" register through the
# dynamixel_hardware_interface get_dxl_data service and republishes it as a
# sensor_msgs/BatteryState message. The IMU side is handled directly by
# imu_sensor_broadcaster, so it is not duplicated here.

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import BatteryState

try:
    from dynamixel_interfaces.srv import GetDataFromDxl
except ImportError:
    GetDataFromDxl = None


class OpenCRBatteryPublisher(Node):

    def __init__(self):
        super().__init__('opencr_battery_publisher')

        self.declare_parameter('opencr_id', 100)
        self.declare_parameter('item_name', 'Battery Voltage (mV)')
        # Conversion from raw register value to volts. The opencr_tb3.model
        # file already declares "Battery Voltage (mV)" with scale 0.001 V/mV,
        # so the service may already return a scaled value depending on the
        # dynamixel_hardware_interface version. Override this parameter if
        # your stack returns raw counts instead.
        self.declare_parameter('voltage_scale', 0.001)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('service_name',
                               'dynamixel_hardware_interface/get_dxl_data')
        # Generous timeout: the OpenCR bus is shared with the controller
        # manager's read/write cycle, so a 0.2 s read pass can briefly delay
        # service replies. 1 s leaves room without making 1 Hz polls feel slow.
        self.declare_parameter('service_timeout_sec', 1.0)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('topic', 'battery_state')

        self._opencr_id = int(self.get_parameter('opencr_id').value)
        self._item_name = str(self.get_parameter('item_name').value)
        self._voltage_scale = float(self.get_parameter('voltage_scale').value)
        self._publish_rate = float(self.get_parameter('publish_rate').value)
        self._service_name = str(self.get_parameter('service_name').value)
        self._service_timeout = float(
            self.get_parameter('service_timeout_sec').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        topic = str(self.get_parameter('topic').value)

        self._publisher = self.create_publisher(BatteryState, topic, 10)

        if GetDataFromDxl is None:
            self.get_logger().error(
                'dynamixel_interfaces.srv.GetDataFromDxl is not available; '
                'install dynamixel_interfaces.')
            self._client = None
            return

        # Reentrant group for the service so the synchronous Client.call()
        # inside the timer callback never deadlocks against the executor.
        self._service_cb_group = ReentrantCallbackGroup()
        self._timer_cb_group = MutuallyExclusiveCallbackGroup()

        self._client = self.create_client(
            GetDataFromDxl,
            self._service_name,
            callback_group=self._service_cb_group,
        )
        self._timer = self.create_timer(
            1.0 / max(self._publish_rate, 0.01),
            self._on_timer,
            callback_group=self._timer_cb_group,
        )

        self.get_logger().info(
            f'OpenCRBatteryPublisher: id={self._opencr_id}, '
            f'item="{self._item_name}", scale={self._voltage_scale}, '
            f'rate={self._publish_rate} Hz, service="{self._service_name}"')

    def _on_timer(self):
        if self._client is None:
            return
        if not self._client.service_is_ready():
            if not self._client.wait_for_service(timeout_sec=self._service_timeout):
                self.get_logger().warn(
                    f'Service {self._service_name} not available yet.',
                    throttle_duration_sec=5.0)
                return

        request = GetDataFromDxl.Request()
        request.id = self._opencr_id
        request.item_name = self._item_name
        request.timeout_sec = self._service_timeout

        try:
            response = self._client.call(request)
        except Exception as exc:
            self.get_logger().warn(
                f'Voltage service call raised: {exc}',
                throttle_duration_sec=5.0)
            return
        if response is None or not response.result:
            self.get_logger().warn(
                'Voltage service returned no/failed response.',
                throttle_duration_sec=5.0)
            return

        voltage = float(response.item_data) * self._voltage_scale

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.voltage = voltage
        msg.current = float('nan')
        msg.charge = float('nan')
        msg.capacity = float('nan')
        msg.design_capacity = float('nan')
        msg.percentage = float('nan')
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present = True
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpenCRBatteryPublisher()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        # Jazzy's default SIGINT handler may already have shut down the
        # context; calling rclpy.shutdown() again would raise RCLError.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
