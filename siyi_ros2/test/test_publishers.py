"""Tests for telemetry publisher message construction."""

import pytest
from unittest.mock import MagicMock
from siyi_sdk.models import GimbalAttitude, LaserDistance
from siyi_msgs.msg import GimbalAttitude as GimbalAttitudeMsg, LaserDistance as LaserDistanceMsg


def make_mock_node():
    """Build a minimal mock ROS2 node and a captured publisher mock."""
    node = MagicMock()
    node.get_clock.return_value.now.return_value.to_msg.return_value = MagicMock()
    pub = MagicMock()
    node.create_publisher.return_value = pub
    return node, pub


# ---------------------------------------------------------------------------
# AttitudePublisher tests
# ---------------------------------------------------------------------------


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
        yaw_deg=45.0,
        pitch_deg=-20.0,
        roll_deg=1.5,
        yaw_rate_dps=10.0,
        pitch_rate_dps=-5.0,
        roll_rate_dps=0.5,
    )
    captured_cb(att)

    pub.publish.assert_called_once()
    msg: GimbalAttitudeMsg = pub.publish.call_args[0][0]
    assert msg.yaw_deg == pytest.approx(45.0)
    assert msg.pitch_deg == pytest.approx(-20.0)
    assert msg.roll_deg == pytest.approx(1.5)
    assert msg.yaw_rate_dps == pytest.approx(10.0)
    assert msg.pitch_rate_dps == pytest.approx(-5.0)
    assert msg.roll_rate_dps == pytest.approx(0.5)


def test_attitude_publisher_sets_frame_id():
    """AttitudePublisher sets header.frame_id to 'siyi_gimbal'."""
    from siyi_ros2.publishers.attitude import AttitudePublisher

    mock_client = MagicMock()
    captured_cb = None

    def on_attitude(cb):
        nonlocal captured_cb
        captured_cb = cb
        return lambda: None

    mock_client.on_attitude = on_attitude

    node, pub = make_mock_node()
    AttitudePublisher(node, mock_client)

    captured_cb(GimbalAttitude(
        yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0,
        yaw_rate_dps=0.0, pitch_rate_dps=0.0, roll_rate_dps=0.0,
    ))

    msg: GimbalAttitudeMsg = pub.publish.call_args[0][0]
    assert msg.header.frame_id == "siyi_gimbal"


def test_attitude_publisher_publishes_on_each_callback():
    """AttitudePublisher publishes a message for every SDK callback invocation."""
    from siyi_ros2.publishers.attitude import AttitudePublisher

    mock_client = MagicMock()
    captured_cb = None

    def on_attitude(cb):
        nonlocal captured_cb
        captured_cb = cb
        return lambda: None

    mock_client.on_attitude = on_attitude

    node, pub = make_mock_node()
    AttitudePublisher(node, mock_client)

    dummy = GimbalAttitude(
        yaw_deg=1.0, pitch_deg=2.0, roll_deg=3.0,
        yaw_rate_dps=0.0, pitch_rate_dps=0.0, roll_rate_dps=0.0,
    )
    for _ in range(5):
        captured_cb(dummy)

    assert pub.publish.call_count == 5


# ---------------------------------------------------------------------------
# LaserPublisher tests
# ---------------------------------------------------------------------------


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

    pub.publish.assert_called_once()
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

    pub.publish.assert_called_once()
    msg: LaserDistanceMsg = pub.publish.call_args[0][0]
    assert msg.valid is True
    assert msg.distance_m == pytest.approx(42.5)


def test_laser_publisher_sets_frame_id():
    """LaserPublisher sets header.frame_id to 'siyi_gimbal'."""
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

    captured_cb(LaserDistance(distance_m=10.0))

    msg: LaserDistanceMsg = pub.publish.call_args[0][0]
    assert msg.header.frame_id == "siyi_gimbal"


def test_laser_publisher_zero_distance_is_valid():
    """LaserPublisher treats distance_m=0.0 (not None) as valid."""
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

    captured_cb(LaserDistance(distance_m=0.0))

    msg: LaserDistanceMsg = pub.publish.call_args[0][0]
    assert msg.valid is True
    assert msg.distance_m == pytest.approx(0.0)
