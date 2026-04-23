"""Tests for AsyncBridge thread safety and lifecycle."""

import asyncio
import pytest
from siyi_ros2._async_bridge import AsyncBridge


def test_run_async_returns_value():
    """AsyncBridge.run_async returns the coroutine's result."""
    bridge = AsyncBridge()
    try:
        async def coro():
            return 42

        assert bridge.run_async(coro()) == 42
    finally:
        bridge.shutdown()


def test_run_async_propagates_exception():
    """AsyncBridge.run_async re-raises exceptions from the coroutine."""
    bridge = AsyncBridge()
    try:
        async def failing_coro():
            raise ValueError("test error")

        with pytest.raises(ValueError, match="test error"):
            bridge.run_async(failing_coro())
    finally:
        bridge.shutdown()


def test_run_async_timeout():
    """AsyncBridge.run_async raises TimeoutError on timeout."""
    bridge = AsyncBridge()
    try:
        async def slow_coro():
            await asyncio.sleep(10)

        with pytest.raises(TimeoutError):
            bridge.run_async(slow_coro(), timeout=0.1)
    finally:
        bridge.shutdown()


def test_shutdown_idempotent():
    """Calling shutdown twice does not raise."""
    bridge = AsyncBridge()
    bridge.shutdown()
    bridge.shutdown()  # should not raise


def test_run_async_multiple_sequential():
    """AsyncBridge handles multiple sequential coroutines correctly."""
    bridge = AsyncBridge()
    try:
        async def add(a, b):
            return a + b

        assert bridge.run_async(add(1, 2)) == 3
        assert bridge.run_async(add(10, 20)) == 30
        assert bridge.run_async(add(100, 200)) == 300
    finally:
        bridge.shutdown()


def test_loop_property():
    """AsyncBridge exposes the underlying event loop via .loop."""
    bridge = AsyncBridge()
    try:
        loop = bridge.loop
        assert isinstance(loop, asyncio.AbstractEventLoop)
        assert loop.is_running()
    finally:
        bridge.shutdown()
