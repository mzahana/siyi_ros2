"""Aircraft body-rate ingestion for feedforward.

Subscribes to ``/siyi/aircraft_body_rate`` (geometry_msgs/Vector3Stamped,
units: deg/s, expressed in the gimbal frame) and updates the shared
``BodyRate`` state. The rate-command subscriber reads this and adds a
scaled version to the tracker's setpoint so the visual loop only
corrects residual error.

Note: this is *not* the same as 0x22 ``send_aircraft_attitude``, which
forwards full aircraft Euler angles to the gimbal's internal
stabiliser. That can be added later if you want to offload more work
to the gimbal firmware.
"""

from __future__ import annotations

from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from siyi_ros2.state import BodyRate


class AircraftBodyRateSubscriber:
    """Ingests aircraft body rates and writes them to shared state."""

    TOPIC = "/siyi/aircraft_body_rate"

    def __init__(self, node: Node, body_rate: BodyRate) -> None:
        self._node = node
        self._state = body_rate
        self._sub = node.create_subscription(
            Vector3Stamped, self.TOPIC, self._on_body_rate, qos_profile_sensor_data
        )

    def _on_body_rate(self, msg: Vector3Stamped) -> None:
        # x = roll, y = pitch, z = yaw — standard convention.
        self._state.update(
            yaw_rate_dps=float(msg.vector.z),
            pitch_rate_dps=float(msg.vector.y),
            ts_ns=self._node.get_clock().now().nanoseconds,
        )

    def destroy(self) -> None:
        self._node.destroy_subscription(self._sub)
