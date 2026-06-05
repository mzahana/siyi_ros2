"""Publishes GimbalAttitude messages from the siyi_sdk attitude stream."""

from __future__ import annotations

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from siyi_msgs.msg import GimbalAttitude as GimbalAttitudeMsg
from siyi_sdk.models import GimbalAttitude
from siyi_sdk.transport.base import Unsubscribe

from siyi_ros2.state import GimbalState


class AttitudePublisher:
    """Subscribes to siyi_sdk attitude push and publishes siyi_msgs/GimbalAttitude.

    Uses SensorDataQoS (best-effort, depth 1) — consumers of a high-rate
    feedback stream care about the freshest sample, not history.
    """

    TOPIC = "/siyi/attitude"

    def __init__(self, node: Node, client, state: GimbalState | None = None) -> None:
        self._node = node
        self._state = state
        self._pub = node.create_publisher(
            GimbalAttitudeMsg, self.TOPIC, qos_profile_sensor_data
        )
        self._unsub: Unsubscribe = client.on_attitude(self._on_attitude)

    def _on_attitude(self, att: GimbalAttitude) -> None:
        now = self._node.get_clock().now()
        msg = GimbalAttitudeMsg()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "siyi_gimbal"
        msg.yaw_deg = att.yaw_deg
        msg.pitch_deg = att.pitch_deg
        msg.roll_deg = att.roll_deg
        msg.yaw_rate_dps = att.yaw_rate_dps
        msg.pitch_rate_dps = att.pitch_rate_dps
        msg.roll_rate_dps = att.roll_rate_dps
        self._pub.publish(msg)

        if self._state is not None:
            self._state.update(att.yaw_deg, att.pitch_deg, att.roll_deg, now.nanoseconds)

    def destroy(self) -> None:
        self._unsub()
        self._node.destroy_publisher(self._pub)
