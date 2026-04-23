"""Publishes GimbalAttitude messages from the siyi_sdk attitude stream."""

from rclpy.node import Node
from siyi_msgs.msg import GimbalAttitude as GimbalAttitudeMsg
from siyi_sdk.models import GimbalAttitude
from siyi_sdk.transport.base import Unsubscribe


class AttitudePublisher:
    """Subscribes to siyi_sdk attitude push and publishes siyi_msgs/GimbalAttitude."""

    TOPIC = "/siyi/attitude"
    QOS_DEPTH = 10

    def __init__(self, node: Node, client) -> None:
        self._node = node
        self._pub = node.create_publisher(GimbalAttitudeMsg, self.TOPIC, self.QOS_DEPTH)
        self._unsub: Unsubscribe = client.on_attitude(self._on_attitude)

    def _on_attitude(self, att: GimbalAttitude) -> None:
        msg = GimbalAttitudeMsg()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "siyi_gimbal"
        msg.yaw_deg = att.yaw_deg
        msg.pitch_deg = att.pitch_deg
        msg.roll_deg = att.roll_deg
        msg.yaw_rate_dps = att.yaw_rate_dps
        msg.pitch_rate_dps = att.pitch_rate_dps
        msg.roll_rate_dps = att.roll_rate_dps
        self._pub.publish(msg)

    def destroy(self) -> None:
        self._unsub()
        self._node.destroy_publisher(self._pub)
