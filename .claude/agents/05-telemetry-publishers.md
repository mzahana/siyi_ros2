---
name: 05-telemetry-publishers
description: Phase 5 — implements attitude, laser distance, and AI tracking ROS2 publishers that subscribe to siyi_sdk callbacks and publish stamped messages
model: claude-haiku-4-5-20251001
color: "#1ABC9C"
---

# Phase 5 — Telemetry Publishers

## Context

Working directory: `~/ros2_ws/src/siyi_ros2/`
siyi_sdk location: `~/src/siyi_sdk`
Phases 1–4 must be complete.

You are implementing three publisher modules and wiring them into `SIYINode._setup_publishers()`.

---

## How siyi_sdk push subscriptions work

The SDK pushes data asynchronously via callbacks registered with:
- `client.on_attitude(cb)` → called with `GimbalAttitude` on each push
- `client.on_laser_distance(cb)` → called with `LaserDistance` on each push
- `client.on_ai_tracking(cb)` → called with `AITrackingTarget` on each push

These callbacks are invoked from the **asyncio thread** (inside the SDK's reader task).
Calling `publisher.publish()` from any thread is thread-safe in ROS2.

Return value of `on_*()` is an `Unsubscribe` callable — store it so publishers can
clean up on shutdown.

---

## File 1: `siyi_ros2/siyi_ros2/publishers/attitude.py`

```python
"""Publishes GimbalAttitude messages from the siyi_sdk attitude stream."""

from rclpy.node import Node
from rclpy.time import Time
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
```

---

## File 2: `siyi_ros2/siyi_ros2/publishers/laser.py`

```python
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
```

---

## File 3: `siyi_ros2/siyi_ros2/publishers/ai_tracking.py`

```python
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
        # target is siyi_sdk.models.AITrackingTarget
        # Read ~/src/siyi_sdk/siyi_sdk/models.py for exact field names
        msg = AITrackingMsg()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "siyi_gimbal"
        # Map AITrackingTarget fields to msg fields
        # Check models.py for the exact dataclass fields and map accordingly
        # Common fields: tracking (bool), target position/size
        self._pub.publish(msg)

    def destroy(self) -> None:
        self._unsub()
        self._node.destroy_publisher(self._pub)
```

**Important**: Before writing `_on_tracking`, read `~/src/siyi_sdk/siyi_sdk/models.py`
and find the `AITrackingTarget` dataclass to get exact field names. Map them to the
`AITrackingTarget.msg` fields: `tracking`, `target_x`, `target_y`, `target_w`, `target_h`.

---

## Wire into SIYINode

Edit `siyi_ros2/siyi_ros2/siyi_node.py` to replace the `_setup_publishers` stub:

```python
def _setup_publishers(self) -> None:
    from siyi_ros2.publishers.attitude import AttitudePublisher
    from siyi_ros2.publishers.laser import LaserPublisher
    from siyi_ros2.publishers.ai_tracking import AITrackingPublisher
    self._att_pub = AttitudePublisher(self, self._client)
    self._laser_pub = LaserPublisher(self, self._client)
    self._ai_pub = AITrackingPublisher(self, self._client)
```

Also update `destroy_node()` to call `.destroy()` on each publisher before closing the client:

```python
def destroy_node(self) -> None:
    if hasattr(self, '_att_pub'):
        self._att_pub.destroy()
    if hasattr(self, '_laser_pub'):
        self._laser_pub.destroy()
    if hasattr(self, '_ai_pub'):
        self._ai_pub.destroy()
    if self._client is not None:
        self._bridge.run_async(self._client.close())
    self._bridge.shutdown()
    super().destroy_node()
```

---

## Acceptance Criteria

- Three publisher files created in `siyi_ros2/publishers/`
- Each has a class with `__init__(node, client)` and `destroy()` method
- AI tracking publisher reads the actual `AITrackingTarget` dataclass from models.py
- `SIYINode._setup_publishers` instantiates all three
- `destroy_node` calls `.destroy()` on all publishers
- `colcon build --packages-select siyi_ros2` succeeds
- `ros2 topic list` after launch shows `/siyi/attitude`, `/siyi/laser_distance`, `/siyi/ai_tracking`
