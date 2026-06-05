"""Publishes GimbalAttitude messages and optionally a dynamic TF transform."""

from __future__ import annotations

import math

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from siyi_msgs.msg import GimbalAttitude as GimbalAttitudeMsg
from siyi_sdk.models import GimbalAttitude
from siyi_sdk.transport.base import Unsubscribe
from tf2_ros import TransformBroadcaster

from siyi_ros2.state import GimbalState


def _euler_zyx_to_quaternion(
    yaw_rad: float, pitch_rad: float, roll_rad: float
) -> tuple[float, float, float, float]:
    """Convert ZYX Euler angles (yaw→pitch→roll) to quaternion (x, y, z, w).

    ZYX is the aerospace convention used by SIYI: yaw rotates about Z first,
    then pitch about the new Y, then roll about the new X.
    """
    cy, sy = math.cos(yaw_rad * 0.5), math.sin(yaw_rad * 0.5)
    cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
    cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


class AttitudePublisher:
    """Subscribes to siyi_sdk attitude push and publishes siyi_msgs/GimbalAttitude.

    Optionally broadcasts a dynamic TF transform from *parent_frame* to
    *child_frame* on every attitude update.  Enable with the ``publish_tf``
    node parameter (default: ``false``).

    Uses SensorDataQoS (best-effort, depth 1) — consumers of a high-rate
    feedback stream care about the freshest sample, not history.
    """

    TOPIC = "/siyi/attitude"

    def __init__(
        self,
        node: Node,
        client,
        state: GimbalState | None = None,
        publish_tf: bool = False,
        tf_parent_frame: str = "base_link",
        tf_child_frame: str = "siyi_gimbal",
    ) -> None:
        self._node = node
        self._state = state
        self._publish_tf = publish_tf
        self._tf_parent_frame = tf_parent_frame
        self._tf_child_frame = tf_child_frame

        self._pub = node.create_publisher(
            GimbalAttitudeMsg, self.TOPIC, qos_profile_sensor_data
        )

        if self._publish_tf:
            self._tf_broadcaster = TransformBroadcaster(node)
            node.get_logger().info(
                f"Gimbal TF enabled: '{tf_parent_frame}' → '{tf_child_frame}'"
            )

        self._unsub: Unsubscribe = client.on_attitude(self._on_attitude)

    def _on_attitude(self, att: GimbalAttitude) -> None:
        now = self._node.get_clock().now()

        msg = GimbalAttitudeMsg()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._tf_child_frame
        msg.yaw_deg = att.yaw_deg
        msg.pitch_deg = att.pitch_deg
        msg.roll_deg = att.roll_deg
        msg.yaw_rate_dps = att.yaw_rate_dps
        msg.pitch_rate_dps = att.pitch_rate_dps
        msg.roll_rate_dps = att.roll_rate_dps
        self._pub.publish(msg)

        if self._state is not None:
            self._state.update(att.yaw_deg, att.pitch_deg, att.roll_deg, now.nanoseconds)

        if self._publish_tf:
            self._broadcast_tf(att, now.to_msg())

    def _broadcast_tf(self, att: GimbalAttitude, stamp) -> None:
        yaw_rad = math.radians(att.yaw_deg)
        pitch_rad = math.radians(att.pitch_deg)
        roll_rad = math.radians(att.roll_deg)
        qx, qy, qz, qw = _euler_zyx_to_quaternion(yaw_rad, pitch_rad, roll_rad)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self._tf_parent_frame
        t.child_frame_id = self._tf_child_frame
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        # Translation is zero: the gimbal rotates about its mount point.
        # Use a static_transform_publisher to express the mount offset on the vehicle.
        self._tf_broadcaster.sendTransform(t)

    def destroy(self) -> None:
        self._unsub()
        self._node.destroy_publisher(self._pub)
