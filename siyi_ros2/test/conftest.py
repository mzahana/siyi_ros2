"""Shared pytest fixtures for siyi_ros2 tests."""

import asyncio
import pytest
import rclpy


@pytest.fixture(scope="session", autouse=True)
def rclpy_init():
    """Initialize and shutdown rclpy once for the test session.

    Uses the default global context so that nodes created with
    ``super().__init__("node_name")`` (no explicit context) can find
    an initialised ROS2 context.
    """
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def event_loop():
    """Create a fresh asyncio event loop for each test."""
    loop = asyncio.new_event_loop()
    yield loop
    loop.close()
