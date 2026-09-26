# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""When the node needs wheel odometry at all. Pure logic: timestamps are passed in, no ROS.

Odometry only feeds the stop detector, and a stop can only follow commanded motion. So the
node subscribes when a velocity command arrives and unsubscribes once the commands have gone
quiet and the rover has been still for the stop detector's hold time - by then the stop has
been reported. An idle rover then costs the node no odometry messages at all: in rclpy every
message wakes the executor, and that wake-up, not the callback, is what costs CPU.
"""

from typing import Optional

from ..domain.motion import StopThresholds


class OdometryGate:

    def __init__(self, thresholds: StopThresholds, idle_timeout: float):
        self._t = thresholds
        self._idle_timeout = idle_timeout
        self._open = False
        self._last_command: Optional[float] = None
        self._still_since: Optional[float] = None

    @property
    def is_open(self) -> bool:
        return self._open

    def on_command(self, now: float) -> bool:
        """A velocity command arrived. True when the odometry subscription should open now."""
        self._last_command = now
        if self._open:
            return False
        self._open = True
        self._still_since = None
        return True

    def on_sample(self, linear: float, angular: float, now: float) -> None:
        """A speed sample while open, the same one the stop detector gets."""
        if abs(linear) < self._t.linear and abs(angular) < self._t.angular:
            if self._still_since is None:
                self._still_since = now
        else:
            self._still_since = None

    def should_close(self, now: float) -> bool:
        """True once, when the subscription can go: commands quiet and the stop reported."""
        if not self._open or self._last_command is None:
            return False
        if now - self._last_command < self._idle_timeout:
            return False
        if self._still_since is None or now - self._still_since < self._t.hold_time:
            return False
        self._open = False
        self._still_since = None
        return True
