"""Publishes LaserDistance messages from the siyi_sdk laser push stream."""

from rclpy.node import Node
from siyi_msgs.msg import LaserDistance as LaserDistanceMsg
from siyi_sdk.models import LaserDistance
from siyi_sdk.transport.base import Unsubscribe


class LaserPublisher:
    """Subscribes to siyi_sdk laser distance push and publishes siyi_msgs/LaserDistance."""

    TOPIC = "/siyi/laser_distance"
    QOS_DEPTH = 10

    def __init__(self, node: Node, client) -> None:
        self._node = node
        self._pub = node.create_publisher(LaserDistanceMsg, self.TOPIC, self.QOS_DEPTH)
        self._unsub: Unsubscribe = client.on_laser_distance(self._on_laser)

    def _on_laser(self, laser: LaserDistance) -> None:
        msg = LaserDistanceMsg()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "siyi_gimbal"
        msg.valid = laser.distance_m is not None
        msg.distance_m = laser.distance_m if laser.distance_m is not None else 0.0
        self._pub.publish(msg)

    def destroy(self) -> None:
        self._unsub()
        self._node.destroy_publisher(self._pub)
