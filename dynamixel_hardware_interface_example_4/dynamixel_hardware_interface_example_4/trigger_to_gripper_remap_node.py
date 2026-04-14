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
# Reads joint_states, maps trigger_joint position to gripper_position_controller
# so the gripper follows the trigger (joint name remapping).

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class TriggerToGripperRemapNode(Node):
    """Maps trigger_joint position from joint_states to gripper_position_controller commands."""

    def __init__(self):
        super().__init__('trigger_to_gripper_remap_node')
        self.declare_parameter('trigger_joint_name', 'trigger_joint')
        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('gripper_commands_topic', 'gripper_position_controller/commands')
        self.declare_parameter('scaling_factor', 3.0)
        self.declare_parameter('clip_min', 0.0)
        self.declare_parameter('clip_max', 1.14)
        self.declare_parameter('trigger_commands_topic', 'trigger_position_controller/commands')
        self.declare_parameter('trigger_command_value', -0.02)

        trigger_joint = self.get_parameter('trigger_joint_name').get_parameter_value().string_value
        joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        gripper_topic = self.get_parameter('gripper_commands_topic').get_parameter_value().string_value
        scaling_factor = self.get_parameter('scaling_factor').get_parameter_value().double_value
        clip_min = self.get_parameter('clip_min').get_parameter_value().double_value
        clip_max = self.get_parameter('clip_max').get_parameter_value().double_value
        trigger_commands_topic = self.get_parameter('trigger_commands_topic').get_parameter_value().string_value
        trigger_command_value = self.get_parameter('trigger_command_value').get_parameter_value().double_value

        self._trigger_joint_name = trigger_joint
        self._trigger_command_value = trigger_command_value
        self._scaling_factor = scaling_factor
        self._clip_min = clip_min
        self._clip_max = clip_max
        self._last_trigger_position = 0.0

        self._sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self._joint_states_cb,
            10,
        )
        self._pub = self.create_publisher(
            Float64MultiArray,
            gripper_topic,
            10,
        )
        self._trigger_pub = self.create_publisher(
            Float64MultiArray,
            trigger_commands_topic,
            10,
        )
        self._trigger_timer = self.create_timer(
            1.0 / 50.0,
            self._trigger_command_cb,
        )

        self.get_logger().info(
            f'Remapping {trigger_joint} -> {gripper_topic} (scaling_factor={scaling_factor})'
        )

    def _joint_states_cb(self, msg: JointState):
        try:
            idx = msg.name.index(self._trigger_joint_name)
        except ValueError:
            return
        if idx >= len(msg.position):
            return
        pos = msg.position[idx]
        self._last_trigger_position = pos
        scaled_pos = pos * self._scaling_factor
        clipped_pos = max(self._clip_min, min(self._clip_max, scaled_pos))
        cmd = Float64MultiArray()
        cmd.data = [clipped_pos]
        self._pub.publish(cmd)

    def _trigger_command_cb(self):
        cmd = Float64MultiArray()
        cmd.data = [self._trigger_command_value]
        self._trigger_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TriggerToGripperRemapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
