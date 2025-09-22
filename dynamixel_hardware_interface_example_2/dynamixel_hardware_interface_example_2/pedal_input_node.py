#!/usr/bin/env python3

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import JointState


class PedalInputNode(Node):
    def __init__(self) -> None:
        super().__init__('pedal_input')

        # Parameters
        self.declare_parameter('desired_position', 0.5)
        self.declare_parameter('joint_name', 'pedal_joint')
        self.declare_parameter('pressed_center', 0.0)
        self.declare_parameter('pressed_window', 0.1)
        self.declare_parameter('long_press_seconds', 1.0)
        self.declare_parameter('command_publish_rate_hz', 10.0)

        self.desired_position: float = float(self.get_parameter('desired_position').value)
        self.joint_name: str = str(self.get_parameter('joint_name').value)
        self.pressed_center: float = float(self.get_parameter('pressed_center').value)
        self.pressed_window: float = float(self.get_parameter('pressed_window').value)
        self.long_press_seconds: float = float(self.get_parameter('long_press_seconds').value)
        self.command_publish_rate_hz: float = float(self.get_parameter('command_publish_rate_hz').value)

        # Publishers
        # Commands publisher (remap '~/commands' to the controller input topic in launch)
        self.commands_pub = self.create_publisher(Float64MultiArray, 'position_controller/commands', 10)
        # Pedal state publisher: 0 or 1
        self.state_pub = self.create_publisher(Bool, 'pedal_state', 10)

        # Subscriber
        self.joint_states_sub = self.create_subscription(JointState, 'joint_states', self._on_joint_states, 10)

        # Timers
        period = 1.0 / max(self.command_publish_rate_hz, 0.1)
        self.command_timer = self.create_timer(period, self._publish_command)

        # Press/toggle state
        self.current_state = False  # 0 or 1
        self.pressed_start_time: Optional[rclpy.time.Time] = None
        self.toggle_done_for_current_press: bool = False

        # Publish initial state
        self._publish_state()

    def _publish_command(self) -> None:
        msg = Float64MultiArray()
        # Single joint command
        msg.data = [self.desired_position]
        self.commands_pub.publish(msg)

    def _publish_state(self) -> None:
        state_msg = Bool()
        state_msg.data = self.current_state
        self.state_pub.publish(state_msg)

    def _is_pressed(self, position_value: float) -> bool:
        return abs(position_value - self.pressed_center) < self.pressed_window

    def _on_joint_states(self, msg: JointState) -> None:
        try:
            idx = msg.name.index(self.joint_name)
        except ValueError:
            return

        if idx >= len(msg.position):
            return

        pedal_pos = float(msg.position[idx])
        now = self.get_clock().now()

        if self._is_pressed(pedal_pos):
            if self.pressed_start_time is None:
                self.pressed_start_time = now
                self.toggle_done_for_current_press = False
                return

            if not self.toggle_done_for_current_press:
                held_duration = now - self.pressed_start_time
                if held_duration >= Duration(seconds=self.long_press_seconds):
                    # Toggle state once per press-and-hold
                    self.current_state = False if self.current_state == True else True
                    self._publish_state()
                    self.toggle_done_for_current_press = True
        else:
            # Released: reset tracking for next press
            self.pressed_start_time = None
            self.toggle_done_for_current_press = False


def main() -> None:
    rclpy.init()
    node = PedalInputNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


