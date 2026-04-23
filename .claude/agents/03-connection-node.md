---
name: 03-connection-node
description: Phase 3 — implements the asyncio/ROS2 thread bridge (_async_bridge.py) and the main SIYINode class with full connection lifecycle, parameter declaration, and reconnection logic
model: claude-sonnet-4-6
color: "#E67E22"
---

# Phase 3 — Async Bridge & Connection Node

## Context

Working directory: `~/ros2_ws/src/siyi_ros2/`
siyi_sdk is at `~/src/siyi_sdk`
Phase 1 and 2 must be complete.

You are implementing two files:
1. `siyi_ros2/siyi_ros2/_async_bridge.py` — the asyncio↔ROS2 threading bridge
2. `siyi_ros2/siyi_ros2/siyi_node.py` — the main `SIYINode` ROS2 node

---

## Threading Architecture

```
Main thread: rclpy.spin(node)
  ├── service callbacks → call bridge.run_async(coroutine)
  ├── timer callbacks  → call bridge.run_async(coroutine)
  └── shutdown → bridge.shutdown()

Asyncio thread (daemon):
  └── asyncio event loop
        ├── SIYIClient I/O (reader task)
        ├── heartbeat task (TCP only)
        └── stream push dispatch (attitude, laser, AI)
```

Callbacks registered with `client.on_attitude()` etc. are called from the asyncio
thread but call `publisher.publish()` which is thread-safe in ROS2.

---

## File 1: `siyi_ros2/siyi_ros2/_async_bridge.py`

Implement a clean thread bridge. Key requirements:
- Starts an asyncio event loop in a dedicated daemon thread on construction
- `run_async(coro, timeout=5.0)` — submits coroutine to the asyncio loop from any
  thread using `asyncio.run_coroutine_threadsafe`, blocks until done or raises
  `TimeoutError` / re-raises the coroutine's exception
- `shutdown()` — stops the event loop and joins the thread gracefully
- Must be safe to call `shutdown()` multiple times

```python
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
```

---

## File 2: `siyi_ros2/siyi_ros2/siyi_node.py`

Implement `SIYINode(rclpy.Node)`. Full implementation requirements:

### Parameters to declare

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `host` | str | `"192.168.144.25"` | Gimbal IP address |
| `port` | int | `37260` | UDP/TCP port |
| `transport` | str | `"udp"` | `"udp"` / `"tcp"` / `"serial"` |
| `serial_device` | str | `"/dev/ttyUSB0"` | Serial port |
| `baud_rate` | int | `115200` | Serial baud rate |
| `timeout` | float | `2.0` | Command timeout (s) |
| `auto_reconnect` | bool | `False` | Reconnect on failure |
| `attitude_stream_hz` | int | `10` | Attitude push rate |

### Connection logic

Use the convenience factories from `siyi_sdk`:

```python
from siyi_sdk import connect_udp, connect_tcp, connect_serial
```

- On `udp`: `connect_udp(ip=host, port=port, timeout=timeout, auto_reconnect=auto_reconnect)`
- On `tcp`: `connect_tcp(ip=host, port=port, timeout=timeout, auto_reconnect=auto_reconnect)`
- On `serial`: `connect_serial(device=serial_device, baud=baud_rate, timeout=timeout, auto_reconnect=auto_reconnect)`

Call via `self._bridge.run_async(connect_udp(...))`.

### Attitude stream activation

After connecting, activate the attitude push stream:

```python
from siyi_sdk.models import GimbalDataType, DataStreamFreq

hz_map = {
    1:  DataStreamFreq.FREQ_1HZ,
    2:  DataStreamFreq.FREQ_2HZ,
    5:  DataStreamFreq.FREQ_5HZ,
    10: DataStreamFreq.FREQ_10HZ,
    20: DataStreamFreq.FREQ_20HZ,
    50: DataStreamFreq.FREQ_50HZ,
}
freq = hz_map.get(attitude_stream_hz, DataStreamFreq.FREQ_10HZ)
self._bridge.run_async(
    self._client.request_gimbal_stream(GimbalDataType.ATTITUDE, freq)
)
```

### Shutdown

Override `destroy_node()` to:
1. Call `self._bridge.run_async(self._client.close())`
2. Call `self._bridge.shutdown()`
3. Call `super().destroy_node()`

### Structure

```python
class SIYINode(Node):
    def __init__(self) -> None:
        super().__init__("siyi_node")
        self._bridge = AsyncBridge()
        self._client: SIYIClient | None = None

        self._declare_parameters()
        self._connect()

        # Publishers, services, publishers registered after successful connect
        # (imported and wired in later phases — leave placeholders)
        self._setup_publishers()
        self._setup_services()

    def _declare_parameters(self) -> None: ...
    def _connect(self) -> None: ...
    def _setup_publishers(self) -> None:
        pass  # Phase 5 fills this
    def _setup_services(self) -> None:
        pass  # Phase 4 fills this

    def destroy_node(self) -> None: ...


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SIYINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Error handling

- If connection fails, log the error with `self.get_logger().error(...)` and raise
  `RuntimeError` (stops the node cleanly rather than silently continuing with `None` client)
- Log connection success: transport type, host/device, port

### Logging

Use `self.get_logger()` (ROS2 logger, not Python `logging`).

---

## Acceptance Criteria

- `_async_bridge.py` has `AsyncBridge` class with `run_async()` and `shutdown()`
- `siyi_node.py` imports `AsyncBridge`, `SIYIClient`, and the three connect factories
- `SIYINode.__init__` declares all parameters, connects, and calls setup stubs
- `main()` function exists and is correct for ROS2 entry point
- `colcon build --packages-select siyi_ros2` still succeeds
- No bare `except:` clauses — use specific exception types
