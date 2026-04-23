"""Tests for siyi_ros2 service handlers."""

import pytest
from unittest.mock import MagicMock
from siyi_sdk.models import SetAttitudeAck, FirmwareVersion, LaserDistance


def make_bridge_and_client():
    """Return a (bridge_mock, client_mock) pair."""
    bridge = MagicMock()
    client = MagicMock()
    return bridge, client


def make_mock_node():
    """Return a minimal mock ROS2 node."""
    node = MagicMock()
    node.get_logger.return_value = MagicMock()
    node.create_service = MagicMock(return_value=MagicMock())
    return node


# ---------------------------------------------------------------------------
# GimbalServices tests
# ---------------------------------------------------------------------------


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
    assert resp.actual_roll_deg == pytest.approx(0.0)


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


def test_set_attitude_generic_exception():
    """SetGimbalAttitude returns success=False on unexpected exception."""
    from siyi_ros2.services.gimbal import GimbalServices
    from siyi_msgs.srv import SetGimbalAttitude

    bridge, client = make_bridge_and_client()
    bridge.run_async.side_effect = RuntimeError("device error")

    node = make_mock_node()
    gs = GimbalServices(node, bridge, client)

    req = SetGimbalAttitude.Request()
    resp = gs._handle_set_attitude(req, SetGimbalAttitude.Response())

    assert resp.success is False
    assert "device error" in resp.message


def test_set_attitude_ok_message():
    """SetGimbalAttitude response message is 'OK' on success."""
    from siyi_ros2.services.gimbal import GimbalServices
    from siyi_msgs.srv import SetGimbalAttitude

    bridge, client = make_bridge_and_client()
    ack = SetAttitudeAck(yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0)
    bridge.run_async.return_value = ack

    node = make_mock_node()
    gs = GimbalServices(node, bridge, client)

    req = SetGimbalAttitude.Request()
    resp = gs._handle_set_attitude(req, SetGimbalAttitude.Response())

    assert resp.message == "OK"


# ---------------------------------------------------------------------------
# SystemServices tests
# ---------------------------------------------------------------------------


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
    assert resp.zoom_version.startswith("v")


def test_get_firmware_version_timeout():
    """GetFirmwareVersion returns success=False on timeout."""
    from siyi_ros2.services.system import SystemServices
    from siyi_msgs.srv import GetFirmwareVersion

    bridge, client = make_bridge_and_client()
    bridge.run_async.side_effect = TimeoutError

    node = make_mock_node()
    ss = SystemServices(node, bridge, client)

    req = GetFirmwareVersion.Request()
    resp = ss._handle_get_firmware_version(req, GetFirmwareVersion.Response())

    assert resp.success is False
    assert "timeout" in resp.message.lower()


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


def test_get_laser_distance_valid():
    """GetLaserDistance returns valid=True with correct distance."""
    from siyi_ros2.services.system import SystemServices
    from siyi_msgs.srv import GetLaserDistance

    bridge, client = make_bridge_and_client()
    bridge.run_async.return_value = LaserDistance(distance_m=150.5)

    node = make_mock_node()
    ss = SystemServices(node, bridge, client)

    req = GetLaserDistance.Request()
    resp = ss._handle_get_laser_distance(req, GetLaserDistance.Response())

    assert resp.success is True
    assert resp.valid is True
    assert resp.distance_m == pytest.approx(150.5)


def test_get_laser_distance_timeout():
    """GetLaserDistance returns success=False on timeout."""
    from siyi_ros2.services.system import SystemServices
    from siyi_msgs.srv import GetLaserDistance

    bridge, client = make_bridge_and_client()
    bridge.run_async.side_effect = TimeoutError

    node = make_mock_node()
    ss = SystemServices(node, bridge, client)

    req = GetLaserDistance.Request()
    resp = ss._handle_get_laser_distance(req, GetLaserDistance.Response())

    assert resp.success is False
    assert "timeout" in resp.message.lower()
