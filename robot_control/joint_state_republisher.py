#!/usr/bin/env python3
"""
joint_state_republisher.py
==========================
ROS2 Humble port of parse_and_publish.cpp (originally ROS1 C++).

Subscribes to /joint_states and republishes joint angles in degrees as a
UInt8MultiArray on /joint_array, in hardware order:
  [0] base_joint        (180 - angle)
  [1] shoulder_joint
  [2] elbow_joint
  [3] wrist_pitch_joint
  [4] wrist_roll_joint
  [5] gripper_joint

Usage:
  ros2 run robot_control joint_state_republisher
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension, MultiArrayLayout

JOINT_ORDER = [
    'base_joint',
    'shoulder_joint',
    'elbow_joint',
    'wrist_pitch_joint',
    'wrist_roll_joint',
    'gripper_joint',
]


class JointStateRepublisher(Node):

    def __init__(self):
        super().__init__('joint_state_republisher')

        self._angles: dict = {j: 0 for j in JOINT_ORDER}

        self._sub = self.create_subscription(
            JointState, 'joint_states', self._joint_state_cb, 10)

        self._pub = self.create_publisher(UInt8MultiArray, 'joint_array', 10)

        # Publish at 10 Hz
        self._timer = self.create_timer(0.1, self._publish_cb)

        self.get_logger().info('JointStateRepublisher started.')

    def _joint_state_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self._angles:
                deg = int(round(math.degrees(pos)))
                self._angles[name] = max(0, min(255, deg))

    def _publish_cb(self):
        arr = UInt8MultiArray()
        arr.layout = MultiArrayLayout()
        arr.layout.dim = [MultiArrayDimension(
            label='joints', size=len(JOINT_ORDER), stride=len(JOINT_ORDER))]
        arr.layout.data_offset = 0

        data = [
            max(0, min(255, 180 - self._angles['base_joint'])),  # base inverted
            self._angles['shoulder_joint'],
            self._angles['elbow_joint'],
            self._angles['wrist_pitch_joint'],
            self._angles['wrist_roll_joint'],
            self._angles['gripper_joint'],
        ]
        arr.data = [max(0, min(255, v)) for v in data]
        self._pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
