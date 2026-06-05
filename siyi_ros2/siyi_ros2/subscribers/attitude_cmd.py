"""Absolute-attitude command subscriber (fire-and-forget).

Useful for "look at this bearing" use cases where rate control is not
needed. Setpoints are clamped to the configured mechanical limits and
dispatched without waiting for ACK. Honours the same enable flag as the
rate-command subscriber.
"""

from __future__ import annotations

import asyncio

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from siyi_msgs.msg import GimbalAttitudeCmd

from siyi_ros2._async_bridge import AsyncBridge


class AttitudeCommandSubscriber:
    """Receives absolute attitude setpoints and dispatches them."""

    TOPIC = "/siyi/cmd/attitude"

    def __init__(
        self,
        node: Node,
        bridge: AsyncBridge,
        client,
        rate_subscriber,
    ) -> None:
        self._node = node
        self._bridge = bridge
        self._client = client
        # Borrow the rate subscriber's limit parameters + enable flag.
        # Keeping them in one place avoids divergence.
        self._rate_sub = rate_subscriber

        self._sub = node.create_subscription(
            GimbalAttitudeCmd,
            self.TOPIC,
            self._on_attitude_cmd,
            qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup(),
        )

    def _on_attitude_cmd(self, msg: GimbalAttitudeCmd) -> None:
        if not self._rate_sub._enabled:  # noqa: SLF001 — intentional shared gate
            return

        yaw = max(
            self._rate_sub._yaw_min_deg,
            min(self._rate_sub._yaw_max_deg, float(msg.yaw_deg)),
        )
        pitch = max(
            self._rate_sub._pitch_min_deg,
            min(self._rate_sub._pitch_max_deg, float(msg.pitch_deg)),
        )

        fut = asyncio.run_coroutine_threadsafe(
            self._client.set_attitude_nowait(yaw, pitch), self._bridge.loop
        )
        fut.add_done_callback(self._on_done)

    def _on_done(self, fut) -> None:
        exc = fut.exception()
        if exc is not None:
            self._node.get_logger().warning(
                f"Fire-and-forget set_attitude failed: {exc!r}"
            )

    def destroy(self) -> None:
        self._node.destroy_subscription(self._sub)
