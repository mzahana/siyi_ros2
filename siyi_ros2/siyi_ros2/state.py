"""Shared mutable state between publishers and command subscribers.

The attitude publisher writes the latest gimbal attitude here; the rate
and attitude command subscribers read it to enforce mechanical limits
without each having to maintain its own DDS subscription.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field


@dataclass
class GimbalState:
    """Latest known gimbal attitude, thread-safe."""

    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    last_update_ns: int = 0
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def update(self, yaw_deg: float, pitch_deg: float, roll_deg: float, ts_ns: int) -> None:
        with self._lock:
            self.yaw_deg = yaw_deg
            self.pitch_deg = pitch_deg
            self.roll_deg = roll_deg
            self.last_update_ns = ts_ns

    def snapshot(self) -> tuple[float, float, float, int]:
        with self._lock:
            return (self.yaw_deg, self.pitch_deg, self.roll_deg, self.last_update_ns)


@dataclass
class BodyRate:
    """Latest aircraft body rate in the gimbal frame (rad/s), thread-safe."""

    yaw_rate_dps: float = 0.0
    pitch_rate_dps: float = 0.0
    last_update_ns: int = 0
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def update(self, yaw_rate_dps: float, pitch_rate_dps: float, ts_ns: int) -> None:
        with self._lock:
            self.yaw_rate_dps = yaw_rate_dps
            self.pitch_rate_dps = pitch_rate_dps
            self.last_update_ns = ts_ns

    def snapshot(self) -> tuple[float, float, int]:
        with self._lock:
            return (self.yaw_rate_dps, self.pitch_rate_dps, self.last_update_ns)
