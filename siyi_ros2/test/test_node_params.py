"""Tests for SIYINode parameter declaration and defaults."""

import pytest
from unittest.mock import patch


def _make_node():
    """Create a SIYINode with all hardware-touching methods patched out."""
    with patch("siyi_ros2.siyi_node.SIYINode._connect"):
        with patch("siyi_ros2.siyi_node.SIYINode._setup_publishers"):
            with patch("siyi_ros2.siyi_node.SIYINode._setup_services"):
                from siyi_ros2.siyi_node import SIYINode
                return SIYINode()


def test_default_host_parameter(rclpy_init):
    """SIYINode declares host parameter with correct default."""
    node = _make_node()
    try:
        assert node.get_parameter("host").value == "192.168.144.25"
    finally:
        node.destroy_node()


def test_default_transport_parameter(rclpy_init):
    """SIYINode declares transport parameter with default 'udp'."""
    node = _make_node()
    try:
        assert node.get_parameter("transport").value == "udp"
    finally:
        node.destroy_node()


def test_default_port_parameter(rclpy_init):
    """SIYINode declares port parameter with default 37260."""
    node = _make_node()
    try:
        assert node.get_parameter("port").value == 37260
    finally:
        node.destroy_node()


def test_default_attitude_stream_hz(rclpy_init):
    """SIYINode declares attitude_stream_hz with default 10."""
    node = _make_node()
    try:
        assert node.get_parameter("attitude_stream_hz").value == 10
    finally:
        node.destroy_node()


def test_default_serial_device_parameter(rclpy_init):
    """SIYINode declares serial_device parameter with correct default."""
    node = _make_node()
    try:
        assert node.get_parameter("serial_device").value == "/dev/ttyUSB0"
    finally:
        node.destroy_node()


def test_default_baud_rate_parameter(rclpy_init):
    """SIYINode declares baud_rate parameter with default 115200."""
    node = _make_node()
    try:
        assert node.get_parameter("baud_rate").value == 115200
    finally:
        node.destroy_node()


def test_default_timeout_parameter(rclpy_init):
    """SIYINode declares timeout parameter with default 2.0."""
    node = _make_node()
    try:
        assert node.get_parameter("timeout").value == pytest.approx(2.0)
    finally:
        node.destroy_node()


def test_default_auto_reconnect_parameter(rclpy_init):
    """SIYINode declares auto_reconnect parameter with default False."""
    node = _make_node()
    try:
        assert node.get_parameter("auto_reconnect").value is False
    finally:
        node.destroy_node()
