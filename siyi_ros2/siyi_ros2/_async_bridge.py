"""Asyncio event loop bridge for running siyi_sdk coroutines from ROS2 threads."""

from __future__ import annotations

import asyncio
import threading
from typing import Any, Coroutine, TypeVar

T = TypeVar("T")


class AsyncBridge:
    """Runs an asyncio event loop in a daemon thread.

    Allows ROS2 service/timer callbacks (running in the ROS2 executor thread)
    to submit coroutines to the asyncio loop and block until they complete.

    Usage::

        bridge = AsyncBridge()
        result = bridge.run_async(some_coroutine())
        bridge.shutdown()
    """

    def __init__(self) -> None:
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._run_loop, name="siyi_asyncio", daemon=True
        )
        self._thread.start()

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def run_async(self, coro: Coroutine[Any, Any, T], timeout: float = 5.0) -> T:
        """Submit *coro* to the asyncio loop and block until it completes.

        Args:
            coro: Coroutine to execute.
            timeout: Maximum seconds to wait before raising TimeoutError.

        Returns:
            The coroutine's return value.

        Raises:
            TimeoutError: If the coroutine does not complete within *timeout*.
            Exception: Any exception raised by the coroutine.
        """
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return future.result(timeout=timeout)

    @property
    def loop(self) -> asyncio.AbstractEventLoop:
        """The managed asyncio event loop."""
        return self._loop

    def shutdown(self) -> None:
        """Stop the event loop and join the asyncio thread."""
        if self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=5.0)
