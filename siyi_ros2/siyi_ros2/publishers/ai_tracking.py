"""Publishes AITrackingTarget messages from the siyi_sdk AI tracking push stream."""

from rclpy.node import Node
from siyi_msgs.msg import AITrackingTarget as AITrackingMsg
from siyi_sdk.transport.base import Unsubscribe


class AITrackingPublisher:
    """Subscribes to siyi_sdk AI tracking push and publishes siyi_msgs/AITrackingTarget."""

    TOPIC = "/siyi/ai_tracking"
    QOS_DEPTH = 10

    def __init__(self, node: Node, client) -> None:
        self._node = node
        self._pub = node.create_publisher(AITrackingMsg, self.TOPIC, self.QOS_DEPTH)
        self._unsub: Unsubscribe = client.on_ai_tracking(self._on_tracking)

    def _on_tracking(self, target) -> None:
        # target is siyi_sdk.models.AITrackingTarget with fields:
        # x, y, w, h (pixel coordinates in 1280x720 resolution)
        # target_id, status
        msg = AITrackingMsg()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "siyi_gimbal"
        # Determine if actively tracking: status indicates active tracking state
        msg.tracking = target.status is not None and target.status.value > 0
        # Convert pixel coordinates to float (normalized or as-is)
        msg.target_x = float(target.x)
        msg.target_y = float(target.y)
        msg.target_w = float(target.w)
        msg.target_h = float(target.h)
        self._pub.publish(msg)

    def destroy(self) -> None:
        self._unsub()
        self._node.destroy_publisher(self._pub)
