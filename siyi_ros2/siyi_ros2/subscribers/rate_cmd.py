"""Rate-command subscriber: primary tracking control path.

Subscribes to ``/siyi/cmd/rate`` (siyi_msgs/GimbalRateCmd) at any rate
(typically 30–100 Hz from a visual tracking node) and dispatches
fire-and-forget velocity commands to the gimbal.

Key properties:

- Latest-wins coalescing. Only the most recent setpoint is sent; never
  queue.
- Watchdog. If no command arrives within ``watchdog_timeout_ms``, a zero
  rate is sent and re-sent at the watchdog tick until a fresh command
  arrives. Protects against a dead tracker leaving the gimbal slewing.
- Soft position clamping. If the current attitude has reached a
  mechanical limit and the commanded rate would push further in that
  direction, the offending axis is zeroed before sending.
- Optional body-rate feedforward. If enabled, the latest aircraft body
  rate is added (scaled) to the tracker's setpoint so the visual loop
  only corrects the residual error.
- Saturation publication. Whenever clamping zeroes an axis, a
  ``/siyi/saturation`` message is published so the upstream stack can
  command the airframe to yaw and recover the limit.
- Enable/disable. ``/siyi/tracking/enable`` (std_srvs/SetBool) gates
  dispatch; on disable, a zero rate is sent immediately.
"""

from __future__ import annotations

import asyncio
import threading
import time

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from siyi_msgs.msg import GimbalRateCmd, GimbalSaturation
from std_srvs.srv import SetBool

from siyi_sdk.constants import (
    A8MINI_PITCH_MAX_DEG,
    A8MINI_PITCH_MIN_DEG,
    A8MINI_YAW_MAX_DEG,
    A8MINI_YAW_MIN_DEG,
    GIMBAL_RATE_CMD_MAX,
    GIMBAL_RATE_CMD_MIN,
)

from siyi_ros2._async_bridge import AsyncBridge
from siyi_ros2.state import BodyRate, GimbalState

_LIMIT_MARGIN_DEG = 0.5  # how close to the wall before we start blocking


class RateCommandSubscriber:
    """Receives tracking rate setpoints and dispatches them to the gimbal."""

    TOPIC = "/siyi/cmd/rate"
    SATURATION_TOPIC = "/siyi/saturation"
    ENABLE_SERVICE = "/siyi/tracking/enable"

    def __init__(
        self,
        node: Node,
        bridge: AsyncBridge,
        client,
        gimbal_state: GimbalState,
        body_rate: BodyRate,
    ) -> None:
        self._node = node
        self._bridge = bridge
        self._client = client
        self._gimbal_state = gimbal_state
        self._body_rate = body_rate

        self._declare_params()

        # State guarded by _lock.
        self._lock = threading.Lock()
        self._last_cmd_time_ns: int = 0
        self._last_sent_yaw_norm: int = 0
        self._last_sent_pitch_norm: int = 0
        self._enabled: bool = bool(
            node.get_parameter("tracking_enabled_at_start").value
        )
        self._in_flight: int = 0  # diagnostics: dispatched but not yet completed
        self._dropped: int = 0  # diagnostics: callbacks that arrived while saturated

        # Saturation publisher.
        self._sat_pub = node.create_publisher(
            GimbalSaturation, self.SATURATION_TOPIC, 10
        )

        cmd_group = ReentrantCallbackGroup()
        timer_group = MutuallyExclusiveCallbackGroup()

        self._sub = node.create_subscription(
            GimbalRateCmd,
            self.TOPIC,
            self._on_rate_cmd,
            qos_profile_sensor_data,
            callback_group=cmd_group,
        )

        watchdog_hz = max(5.0, 1000.0 / float(self._watchdog_timeout_ms) * 2.0)
        self._watchdog_period_s = 1.0 / watchdog_hz
        self._timer = node.create_timer(
            self._watchdog_period_s, self._watchdog_tick, callback_group=timer_group
        )

        self._srv = node.create_service(
            SetBool, self.ENABLE_SERVICE, self._on_enable, callback_group=timer_group
        )

        node.get_logger().info(
            "Rate-command subscriber ready "
            f"(max_yaw={self._max_yaw_rate_dps} dps, "
            f"max_pitch={self._max_pitch_rate_dps} dps, "
            f"watchdog={self._watchdog_timeout_ms} ms, "
            f"ff_gain={self._feedforward_gain}, "
            f"enabled={self._enabled})"
        )

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def _declare_params(self) -> None:
        n = self._node
        # The maximum yaw/pitch rate the gimbal will be asked to slew at
        # when the normalised command is at ±100. Tune to the gimbal's
        # actual max slew rate. A8 mini: ~90 dps yaw, ~90 dps pitch.
        n.declare_parameter("max_yaw_rate_dps", 90.0)
        n.declare_parameter("max_pitch_rate_dps", 90.0)

        # 0.0 disables feedforward. 1.0 fully cancels measured body rate
        # (use only when /siyi/aircraft_body_rate is in the gimbal frame).
        n.declare_parameter("feedforward_gain", 0.0)

        # If no rate cmd is received within this window, zero is sent.
        n.declare_parameter("watchdog_timeout_ms", 200)

        # Mechanical limits, override A8 mini defaults if needed.
        n.declare_parameter("yaw_min_deg", float(A8MINI_YAW_MIN_DEG))
        n.declare_parameter("yaw_max_deg", float(A8MINI_YAW_MAX_DEG))
        n.declare_parameter("pitch_min_deg", float(A8MINI_PITCH_MIN_DEG))
        n.declare_parameter("pitch_max_deg", float(A8MINI_PITCH_MAX_DEG))

        n.declare_parameter("tracking_enabled_at_start", True)

        # Cache for hot path.
        self._max_yaw_rate_dps = float(n.get_parameter("max_yaw_rate_dps").value)
        self._max_pitch_rate_dps = float(n.get_parameter("max_pitch_rate_dps").value)
        self._feedforward_gain = float(n.get_parameter("feedforward_gain").value)
        self._watchdog_timeout_ms = int(n.get_parameter("watchdog_timeout_ms").value)
        self._yaw_min_deg = float(n.get_parameter("yaw_min_deg").value)
        self._yaw_max_deg = float(n.get_parameter("yaw_max_deg").value)
        self._pitch_min_deg = float(n.get_parameter("pitch_min_deg").value)
        self._pitch_max_deg = float(n.get_parameter("pitch_max_deg").value)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_rate_cmd(self, msg: GimbalRateCmd) -> None:
        if not self._enabled:
            return

        yaw_dps = float(msg.yaw_rate_dps)
        pitch_dps = float(msg.pitch_rate_dps)

        # Feedforward — add (scaled) body rate to the tracker's setpoint.
        if self._feedforward_gain != 0.0:
            body_yaw, body_pitch, _ = self._body_rate.snapshot()
            yaw_dps += self._feedforward_gain * body_yaw
            pitch_dps += self._feedforward_gain * body_pitch

        # Soft position clamp — block motion towards a saturated axis.
        gimbal_yaw, gimbal_pitch, _, _ = self._gimbal_state.snapshot()
        yaw_sat = False
        pitch_sat = False
        if yaw_dps > 0.0 and gimbal_yaw >= self._yaw_max_deg - _LIMIT_MARGIN_DEG:
            yaw_dps = 0.0
            yaw_sat = True
        elif yaw_dps < 0.0 and gimbal_yaw <= self._yaw_min_deg + _LIMIT_MARGIN_DEG:
            yaw_dps = 0.0
            yaw_sat = True
        if pitch_dps > 0.0 and gimbal_pitch >= self._pitch_max_deg - _LIMIT_MARGIN_DEG:
            pitch_dps = 0.0
            pitch_sat = True
        elif pitch_dps < 0.0 and gimbal_pitch <= self._pitch_min_deg + _LIMIT_MARGIN_DEG:
            pitch_dps = 0.0
            pitch_sat = True

        if yaw_sat or pitch_sat:
            self._publish_saturation(yaw_sat, pitch_sat, yaw_dps, pitch_dps)

        yaw_norm = self._normalise(yaw_dps, self._max_yaw_rate_dps)
        pitch_norm = self._normalise(pitch_dps, self._max_pitch_rate_dps)

        with self._lock:
            self._last_cmd_time_ns = self._node.get_clock().now().nanoseconds

        self._dispatch(yaw_norm, pitch_norm)

    def _watchdog_tick(self) -> None:
        with self._lock:
            last_cmd_ns = self._last_cmd_time_ns
            last_yaw = self._last_sent_yaw_norm
            last_pitch = self._last_sent_pitch_norm
            enabled = self._enabled

        if not enabled:
            return

        now_ns = self._node.get_clock().now().nanoseconds
        stale = (now_ns - last_cmd_ns) > self._watchdog_timeout_ms * 1_000_000

        if stale and (last_yaw != 0 or last_pitch != 0):
            self._node.get_logger().warning(
                f"Rate command watchdog tripped ({self._watchdog_timeout_ms} ms) — "
                "sending zero rate"
            )
            self._dispatch(0, 0)

    def _on_enable(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        with self._lock:
            self._enabled = bool(request.data)

        if not request.data:
            # Stop immediately on disable.
            self._dispatch(0, 0)

        response.success = True
        response.message = (
            f"Tracking {'enabled' if request.data else 'disabled'}. "
            f"in_flight={self._in_flight}, dropped={self._dropped}"
        )
        self._node.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _normalise(rate_dps: float, max_rate_dps: float) -> int:
        if max_rate_dps <= 0.0:
            return 0
        norm = rate_dps / max_rate_dps * 100.0
        return max(GIMBAL_RATE_CMD_MIN, min(GIMBAL_RATE_CMD_MAX, int(round(norm))))

    def _dispatch(self, yaw_norm: int, pitch_norm: int) -> None:
        """Fire-and-forget rate command. Non-blocking."""
        with self._lock:
            self._last_sent_yaw_norm = yaw_norm
            self._last_sent_pitch_norm = pitch_norm
            self._in_flight += 1

        fut = asyncio.run_coroutine_threadsafe(
            self._client.rotate_nowait(yaw_norm, pitch_norm), self._bridge.loop
        )
        fut.add_done_callback(self._on_dispatch_done)

    def _on_dispatch_done(self, fut) -> None:
        with self._lock:
            self._in_flight = max(0, self._in_flight - 1)
        exc = fut.exception()
        if exc is not None:
            self._node.get_logger().warning(
                f"Fire-and-forget rotate failed: {exc!r}"
            )

    def _publish_saturation(
        self, yaw_sat: bool, pitch_sat: bool, yaw_dps: float, pitch_dps: float
    ) -> None:
        msg = GimbalSaturation()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "siyi_gimbal"
        msg.yaw_saturated = yaw_sat
        msg.pitch_saturated = pitch_sat
        msg.commanded_yaw_deg = float(yaw_dps)
        msg.commanded_pitch_deg = float(pitch_dps)
        self._sat_pub.publish(msg)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def destroy(self) -> None:
        # Best-effort stop on shutdown.
        try:
            self._dispatch(0, 0)
            time.sleep(0.05)
        except Exception:
            pass
        self._node.destroy_timer(self._timer)
        self._node.destroy_subscription(self._sub)
        self._node.destroy_service(self._srv)
        self._node.destroy_publisher(self._sat_pub)
