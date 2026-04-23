---
name: 08-tests
description: Phase 8 — implements pytest-based tests for SIYINode parameter validation, service handlers with MockTransport, and publisher callbacks, using siyi_sdk's built-in mock transport
model: claude-sonnet-4-6
color: "#2ECC71"
---

# Phase 8 — Tests

## Context

Working directory: `~/ros2_ws/src/siyi_ros2/`
siyi_sdk location: `~/src/siyi_sdk`
Phases 1–7 must be complete.

You are writing tests in `siyi_ros2/test/`. Use `pytest` and `rclpy` test utilities.

Read before writing:
- `~/src/siyi_sdk/siyi_sdk/transport/mock.py` — `MockTransport` class
- `~/src/siyi_sdk/tests/` — see how the SDK's own tests use MockTransport
- `~/src/siyi_sdk/siyi_sdk/client.py` — `SIYIClient` constructor takes a transport

---

## How MockTransport works

The siyi_sdk includes a `MockTransport` for testing. Check its API:

```python
from siyi_sdk.transport.mock import MockTransport
```

The mock allows you to inject fake response frames without real hardware.
Read `~/src/siyi_sdk/tests/conftest.py` or `~/src/siyi_sdk/siyi_sdk/transport/mock.py`
to understand how to queue response frames.

---

## File 1: `siyi_ros2/test/conftest.py`

Shared fixtures for all tests.

```python
"""Shared pytest fixtures for siyi_ros2 tests."""

import asyncio
import pytest
import rclpy
from rclpy.context import Context


@pytest.fixture(scope="session", autouse=True)
def rclpy_init():
    """Initialize and shutdown rclpy once for the test session."""
    context = Context()
    rclpy.init(context=context)
    yield context
    rclpy.shutdown(context=context)


@pytest.fixture
def event_loop():
    """Create a fresh asyncio event loop for each test."""
    loop = asyncio.new_event_loop()
    yield loop
    loop.close()
```

---

## File 2: `siyi_ros2/test/test_node_params.py`

Test parameter declaration and validation in `SIYINode`.

**Strategy**: Don't actually connect to hardware. Test that parameters are declared
with correct defaults and types. Mock the `_connect` method to avoid network calls.

```python
"""Tests for SIYINode parameter declaration and defaults."""

import pytest
from unittest.mock import patch, MagicMock
import rclpy
from rclpy.node import Node


def test_default_host_parameter(rclpy_init):
    """SIYINode declares host parameter with correct default."""
    with patch("siyi_ros2.siyi_node.SIYINode._connect"):
        with patch("siyi_ros2.siyi_node.SIYINode._setup_publishers"):
            with patch("siyi_ros2.siyi_node.SIYINode._setup_services"):
                from siyi_ros2.siyi_node import SIYINode
                node = SIYINode()
                assert node.get_parameter("host").value == "192.168.144.25"
                node.destroy_node()


def test_default_transport_parameter(rclpy_init):
    """SIYINode declares transport parameter with default 'udp'."""
    with patch("siyi_ros2.siyi_node.SIYINode._connect"):
        with patch("siyi_ros2.siyi_node.SIYINode._setup_publishers"):
            with patch("siyi_ros2.siyi_node.SIYINode._setup_services"):
                from siyi_ros2.siyi_node import SIYINode
                node = SIYINode()
                assert node.get_parameter("transport").value == "udp"
                node.destroy_node()


def test_default_port_parameter(rclpy_init):
    """SIYINode declares port parameter with default 37260."""
    with patch("siyi_ros2.siyi_node.SIYINode._connect"):
        with patch("siyi_ros2.siyi_node.SIYINode._setup_publishers"):
            with patch("siyi_ros2.siyi_node.SIYINode._setup_services"):
                from siyi_ros2.siyi_node import SIYINode
                node = SIYINode()
                assert node.get_parameter("port").value == 37260
                node.destroy_node()


def test_default_attitude_stream_hz(rclpy_init):
    """SIYINode declares attitude_stream_hz with default 10."""
    with patch("siyi_ros2.siyi_node.SIYINode._connect"):
        with patch("siyi_ros2.siyi_node.SIYINode._setup_publishers"):
            with patch("siyi_ros2.siyi_node.SIYINode._setup_services"):
                from siyi_ros2.siyi_node import SIYINode
                node = SIYINode()
                assert node.get_parameter("attitude_stream_hz").value == 10
                node.destroy_node()
```

---

## File 3: `siyi_ros2/test/test_async_bridge.py`

Test the `AsyncBridge` class in isolation.

```python
"""Tests for AsyncBridge thread safety and lifecycle."""

import asyncio
import pytest
from siyi_ros2._async_bridge import AsyncBridge


def test_run_async_returns_value():
    """AsyncBridge.run_async returns the coroutine's result."""
    bridge = AsyncBridge()
    async def coro():
        return 42
    assert bridge.run_async(coro()) == 42
    bridge.shutdown()


def test_run_async_propagates_exception():
    """AsyncBridge.run_async re-raises exceptions from the coroutine."""
    bridge = AsyncBridge()
    async def failing_coro():
        raise ValueError("test error")
    with pytest.raises(ValueError, match="test error"):
        bridge.run_async(failing_coro())
    bridge.shutdown()


def test_run_async_timeout():
    """AsyncBridge.run_async raises TimeoutError on timeout."""
    bridge = AsyncBridge()
    async def slow_coro():
        await asyncio.sleep(10)
    with pytest.raises(TimeoutError):
        bridge.run_async(slow_coro(), timeout=0.1)
    bridge.shutdown()


def test_shutdown_idempotent():
    """Calling shutdown twice does not raise."""
    bridge = AsyncBridge()
    bridge.shutdown()
    bridge.shutdown()  # should not raise
```

---

## File 4: `siyi_ros2/test/test_publishers.py`

Test attitude publisher callback correctly maps SDK data to ROS2 message.

```python
"""Tests for telemetry publisher message construction."""

import pytest
from unittest.mock import MagicMock, patch
from siyi_sdk.models import GimbalAttitude, LaserDistance
from siyi_msgs.msg import GimbalAttitude as GimbalAttitudeMsg, LaserDistance as LaserDistanceMsg


def make_mock_node():
    node = MagicMock()
    node.get_clock.return_value.now.return_value.to_msg.return_value = MagicMock()
    pub = MagicMock()
    node.create_publisher.return_value = pub
    return node, pub


def test_attitude_publisher_maps_fields():
    """AttitudePublisher maps all GimbalAttitude fields correctly."""
    from siyi_ros2.publishers.attitude import AttitudePublisher

    mock_client = MagicMock()
    captured_cb = None
    def on_attitude(cb):
        nonlocal captured_cb
        captured_cb = cb
        return lambda: None
    mock_client.on_attitude = on_attitude

    node, pub = make_mock_node()
    publisher = AttitudePublisher(node, mock_client)

    att = GimbalAttitude(
        yaw_deg=45.0, pitch_deg=-20.0, roll_deg=1.5,
        yaw_rate_dps=10.0, pitch_rate_dps=-5.0, roll_rate_dps=0.5
    )
    captured_cb(att)

    pub.publish.assert_called_once()
    msg: GimbalAttitudeMsg = pub.publish.call_args[0][0]
    assert msg.yaw_deg == pytest.approx(45.0)
    assert msg.pitch_deg == pytest.approx(-20.0)
    assert msg.roll_deg == pytest.approx(1.5)


def test_laser_publisher_none_distance_sets_invalid():
    """LaserPublisher sets valid=False when distance_m is None."""
    from siyi_ros2.publishers.laser import LaserPublisher

    mock_client = MagicMock()
    captured_cb = None
    def on_laser(cb):
        nonlocal captured_cb
        captured_cb = cb
        return lambda: None
    mock_client.on_laser_distance = on_laser

    node, pub = make_mock_node()
    LaserPublisher(node, mock_client)

    captured_cb(LaserDistance(distance_m=None))

    msg: LaserDistanceMsg = pub.publish.call_args[0][0]
    assert msg.valid is False
    assert msg.distance_m == pytest.approx(0.0)


def test_laser_publisher_valid_distance():
    """LaserPublisher sets valid=True and correct distance."""
    from siyi_ros2.publishers.laser import LaserPublisher

    mock_client = MagicMock()
    captured_cb = None
    def on_laser(cb):
        nonlocal captured_cb
        captured_cb = cb
        return lambda: None
    mock_client.on_laser_distance = on_laser

    node, pub = make_mock_node()
    LaserPublisher(node, mock_client)

    captured_cb(LaserDistance(distance_m=42.5))

    msg: LaserDistanceMsg = pub.publish.call_args[0][0]
    assert msg.valid is True
    assert msg.distance_m == pytest.approx(42.5)
```

---

## File 5: `siyi_ros2/test/test_services.py`

Test service handlers with a mocked bridge and client.

```python
"""Tests for siyi_ros2 service handlers."""

import pytest
from unittest.mock import MagicMock, AsyncMock
from siyi_sdk.models import SetAttitudeAck, FirmwareVersion, LaserDistance


def make_bridge_and_client():
    bridge = MagicMock()
    client = MagicMock()
    return bridge, client


def make_mock_node():
    node = MagicMock()
    node.get_logger.return_value = MagicMock()
    node.create_service = MagicMock(return_value=MagicMock())
    return node


def test_set_attitude_success():
    """SetGimbalAttitude handler fills response fields on success."""
    from siyi_ros2.services.gimbal import GimbalServices
    from siyi_msgs.srv import SetGimbalAttitude

    bridge, client = make_bridge_and_client()
    ack = SetAttitudeAck(yaw_deg=45.0, pitch_deg=-20.0, roll_deg=0.0)
    bridge.run_async.return_value = ack

    node = make_mock_node()
    gs = GimbalServices(node, bridge, client)

    req = SetGimbalAttitude.Request()
    req.yaw_deg = 45.0
    req.pitch_deg = -20.0
    resp = gs._handle_set_attitude(req, SetGimbalAttitude.Response())

    assert resp.success is True
    assert resp.actual_yaw_deg == pytest.approx(45.0)
    assert resp.actual_pitch_deg == pytest.approx(-20.0)


def test_set_attitude_timeout():
    """SetGimbalAttitude returns success=False on timeout."""
    from siyi_ros2.services.gimbal import GimbalServices
    from siyi_msgs.srv import SetGimbalAttitude

    bridge, client = make_bridge_and_client()
    bridge.run_async.side_effect = TimeoutError

    node = make_mock_node()
    gs = GimbalServices(node, bridge, client)

    req = SetGimbalAttitude.Request()
    resp = gs._handle_set_attitude(req, SetGimbalAttitude.Response())

    assert resp.success is False
    assert "timeout" in resp.message.lower()


def test_get_firmware_version():
    """GetFirmwareVersion handler formats firmware words correctly."""
    from siyi_ros2.services.system import SystemServices
    from siyi_msgs.srv import GetFirmwareVersion

    bridge, client = make_bridge_and_client()
    fw = FirmwareVersion(camera=0x00030201, gimbal=0x00020301, zoom=0x00010101)
    bridge.run_async.return_value = fw

    node = make_mock_node()
    ss = SystemServices(node, bridge, client)

    req = GetFirmwareVersion.Request()
    resp = ss._handle_get_firmware_version(req, GetFirmwareVersion.Response())

    assert resp.success is True
    assert resp.camera_version.startswith("v")
    assert resp.gimbal_version.startswith("v")


def test_get_laser_distance_none():
    """GetLaserDistance returns valid=False when distance is None."""
    from siyi_ros2.services.system import SystemServices
    from siyi_msgs.srv import GetLaserDistance

    bridge, client = make_bridge_and_client()
    bridge.run_async.return_value = LaserDistance(distance_m=None)

    node = make_mock_node()
    ss = SystemServices(node, bridge, client)

    req = GetLaserDistance.Request()
    resp = ss._handle_get_laser_distance(req, GetLaserDistance.Response())

    assert resp.success is True
    assert resp.valid is False
    assert resp.distance_m == pytest.approx(0.0)
```

---

## Acceptance Criteria

- All 5 test files exist in `siyi_ros2/test/`
- Tests import from `siyi_ros2.*` correctly (no ImportError)
- `pytest siyi_ros2/test/ -v` passes all tests
- `colcon test --packages-select siyi_ros2` passes
- No tests require real hardware
