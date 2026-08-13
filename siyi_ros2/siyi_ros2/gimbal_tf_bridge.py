"""
gimbal_tf_bridge  –  SIYI attitude → JointState bridge.

Subscribes to  /siyi/attitude  (siyi_msgs/GimbalAttitude) which is
published by siyi_node at attitude_stream_hz (default 50 Hz).

Republishes the three gimbal angles as  sensor_msgs/JointState  on
/joint_states  so that  robot_state_publisher  can update the dynamic
TF transforms for the three revolute joints declared in the URDF:

    gimbal_yaw   (Z axis – pan)
    gimbal_pitch (Y axis – tilt)
    gimbal_roll  (X axis – roll)

This node is launched alongside robot_state_publisher by
siyi_a8_tf.launch.py, making the TF description self-contained:
the only external dependency is siyi_node being alive and publishing
/siyi/attitude.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from siyi_msgs.msg import GimbalAttitude


# Joint names must exactly match the <joint name="..."> entries in the URDF.
_JOINT_NAMES = ["gimbal_yaw", "gimbal_pitch", "gimbal_roll"]


class GimbalTFBridge(Node):
    """Converts /siyi/attitude → /joint_states for robot_state_publisher."""

    def __init__(self) -> None:
        super().__init__("gimbal_tf_bridge")

        # Publisher – robot_state_publisher subscribes here
        self._js_pub = self.create_publisher(
            JointState, "/joint_states", qos_profile_sensor_data
        )

        # Subscriber – receives attitude from siyi_node
        self._att_sub = self.create_subscription(
            GimbalAttitude,
            "/siyi/attitude",
            self._on_attitude,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "gimbal_tf_bridge ready: /siyi/attitude → /joint_states "
            f"({_JOINT_NAMES})"
        )

    def _on_attitude(self, msg: GimbalAttitude) -> None:
        """Convert GimbalAttitude (degrees) to JointState (radians)."""
        js = JointState()
        js.header.stamp = msg.header.stamp  # keep the original timestamp
        js.name = _JOINT_NAMES
        js.position = [
            math.radians(msg.yaw_deg),
            math.radians(msg.pitch_deg),
            math.radians(msg.roll_deg),
        ]
        js.velocity = [
            math.radians(msg.yaw_rate_dps),
            math.radians(msg.pitch_rate_dps),
            math.radians(msg.roll_rate_dps),
        ]
        js.effort = []  # SIYI SDK does not provide motor effort
        self._js_pub.publish(js)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GimbalTFBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
