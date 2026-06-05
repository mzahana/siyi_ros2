"""SIYI gimbal/camera ROS2 node."""

from __future__ import annotations

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from siyi_sdk import SIYIClient, configure_logging, connect_serial, connect_tcp, connect_udp
from siyi_sdk.models import DataStreamFreq, GimbalDataType

from siyi_ros2._async_bridge import AsyncBridge
from siyi_ros2.state import BodyRate, GimbalState

# Mapping from integer Hz to DataStreamFreq enum members.
_HZ_MAP: dict[int, DataStreamFreq] = {
    2:   DataStreamFreq.HZ2,
    4:   DataStreamFreq.HZ4,
    5:   DataStreamFreq.HZ5,
    10:  DataStreamFreq.HZ10,
    20:  DataStreamFreq.HZ20,
    50:  DataStreamFreq.HZ50,
    100: DataStreamFreq.HZ100,
}


class SIYINode(Node):
    """Main ROS2 node for interfacing with a SIYI gimbal/camera over siyi_sdk."""

    def __init__(self) -> None:
        super().__init__("siyi_node")
        self._bridge = AsyncBridge()
        self._client: SIYIClient | None = None

        # Shared state for command subscribers (limit clamping + feedforward).
        self._gimbal_state = GimbalState()
        self._body_rate = BodyRate()

        self._declare_parameters()
        self._connect()

        self._setup_publishers()
        self._setup_subscribers()
        self._setup_services()

    # ------------------------------------------------------------------
    # Parameter declaration
    # ------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        self.declare_parameter("host", "192.168.144.25")
        self.declare_parameter("port", 37260)
        self.declare_parameter("transport", "udp")
        self.declare_parameter("serial_device", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("timeout", 2.0)
        self.declare_parameter("auto_reconnect", False)
        # Default 50 Hz: tracking control loops need fresh feedback. Raise
        # to 100 if the gimbal firmware supports it stably on your unit.
        self.declare_parameter("attitude_stream_hz", 50)

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def _connect(self) -> None:
        host = self.get_parameter("host").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        transport = self.get_parameter("transport").get_parameter_value().string_value
        serial_device = (
            self.get_parameter("serial_device").get_parameter_value().string_value
        )
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        timeout = self.get_parameter("timeout").get_parameter_value().double_value
        auto_reconnect = (
            self.get_parameter("auto_reconnect").get_parameter_value().bool_value
        )
        attitude_stream_hz = (
            self.get_parameter("attitude_stream_hz").get_parameter_value().integer_value
        )

        try:
            if transport == "udp":
                self._client = self._bridge.run_async(
                    connect_udp(
                        ip=host,
                        port=port,
                        timeout=timeout,
                        auto_reconnect=auto_reconnect,
                    ),
                    timeout=timeout + 3.0,
                )
                self.get_logger().info(
                    f"Connected via UDP to {host}:{port}"
                )
            elif transport == "tcp":
                self._client = self._bridge.run_async(
                    connect_tcp(
                        ip=host,
                        port=port,
                        timeout=timeout,
                        auto_reconnect=auto_reconnect,
                    ),
                    timeout=timeout + 3.0,
                )
                self.get_logger().info(
                    f"Connected via TCP to {host}:{port}"
                )
            elif transport == "serial":
                self._client = self._bridge.run_async(
                    connect_serial(
                        device=serial_device,
                        baud=baud_rate,
                        timeout=timeout,
                        auto_reconnect=auto_reconnect,
                    ),
                    timeout=timeout + 3.0,
                )
                self.get_logger().info(
                    f"Connected via serial on {serial_device} @ {baud_rate} baud"
                )
            else:
                raise ValueError(
                    f"Unknown transport '{transport}'. "
                    "Expected one of: udp, tcp, serial."
                )
        except (ConnectionError, OSError, TimeoutError, ValueError) as exc:
            self.get_logger().error(f"Failed to connect to SIYI gimbal: {exc}")
            raise RuntimeError(f"SIYI connection failed: {exc}") from exc

        # Activate the attitude push stream.
        freq = _HZ_MAP.get(attitude_stream_hz, DataStreamFreq.HZ10)
        try:
            self._bridge.run_async(
                self._client.request_gimbal_stream(GimbalDataType.ATTITUDE, freq),
                timeout=timeout + 3.0,
            )
            self.get_logger().info(
                f"Attitude stream activated at {attitude_stream_hz} Hz "
                f"(DataStreamFreq.{freq.name})"
            )
        except (TimeoutError, OSError) as exc:
            self.get_logger().error(
                f"Failed to activate attitude stream: {exc}"
            )
            raise RuntimeError(f"Attitude stream activation failed: {exc}") from exc

    # ------------------------------------------------------------------
    # Setup stubs (filled in by later phases)
    # ------------------------------------------------------------------

    def _setup_publishers(self) -> None:
        from siyi_ros2.publishers.attitude import AttitudePublisher
        from siyi_ros2.publishers.laser import LaserPublisher
        from siyi_ros2.publishers.ai_tracking import AITrackingPublisher

        self._att_pub = AttitudePublisher(self, self._client, state=self._gimbal_state)
        self._laser_pub = LaserPublisher(self, self._client)
        self._ai_pub = AITrackingPublisher(self, self._client)

    def _setup_subscribers(self) -> None:
        from siyi_ros2.subscribers.aircraft_attitude import AircraftBodyRateSubscriber
        from siyi_ros2.subscribers.attitude_cmd import AttitudeCommandSubscriber
        from siyi_ros2.subscribers.rate_cmd import RateCommandSubscriber

        self._rate_sub = RateCommandSubscriber(
            self, self._bridge, self._client, self._gimbal_state, self._body_rate
        )
        self._att_cmd_sub = AttitudeCommandSubscriber(
            self, self._bridge, self._client, self._rate_sub
        )
        self._body_rate_sub = AircraftBodyRateSubscriber(self, self._body_rate)

    def _setup_services(self) -> None:
        from siyi_ros2.services.camera import CameraServices
        from siyi_ros2.services.gimbal import GimbalServices
        from siyi_ros2.services.system import SystemServices

        self._gimbal_services = GimbalServices(self, self._bridge, self._client)
        self._camera_services = CameraServices(self, self._bridge, self._client)
        self._system_services = SystemServices(self, self._bridge, self._client)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        if hasattr(self, '_rate_sub'):
            self._rate_sub.destroy()
        if hasattr(self, '_att_cmd_sub'):
            self._att_cmd_sub.destroy()
        if hasattr(self, '_body_rate_sub'):
            self._body_rate_sub.destroy()
        if hasattr(self, '_att_pub'):
            self._att_pub.destroy()
        if hasattr(self, '_laser_pub'):
            self._laser_pub.destroy()
        if hasattr(self, '_ai_pub'):
            self._ai_pub.destroy()
        if self._client is not None:
            try:
                self._bridge.run_async(self._client.close(), timeout=5.0)
            except (TimeoutError, OSError) as exc:
                self.get_logger().warning(f"Error closing SIYI client: {exc}")
        self._bridge.shutdown()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(args=None) -> None:
    configure_logging()
    rclpy.init(args=args)
    node = SIYINode()
    # MultiThreadedExecutor: keeps the rate-command subscriber and watchdog
    # timer from blocking each other or telemetry publishing. The asyncio
    # event loop in AsyncBridge handles I/O off the executor thread, so
    # fire-and-forget dispatches do not contend with rclpy callbacks.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
