# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""When has the rover come to rest? Pure logic: timestamps are passed in, no clock here."""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class StopThresholds:
    linear: float = 0.03   # m/s: below this the rover counts as still
    angular: float = 0.05  # rad/s
    hold_time: float = 0.5  # s it has to stay still before a stop is reported


class StopDetector:
    """Reports each moving -> stopped transition exactly once.

    A stop is only reported after the rover has actually moved (a rover that boots standing
    still has nothing new to save), and only once the speed has stayed below the thresholds
    for `hold_time`, so a momentary zero while reversing direction does not count.
    """

    def __init__(self, thresholds: StopThresholds = StopThresholds()):
        self._t = thresholds
        self._moved = False
        self._still_since: Optional[float] = None

    def update(self, linear: float, angular: float, now: float) -> bool:
        """Feed one speed sample; True exactly when a stop has just been confirmed."""
        still = abs(linear) < self._t.linear and abs(angular) < self._t.angular
        if not still:
            self._moved = True
            self._still_since = None
            return False
        if not self._moved:
            return False
        if self._still_since is None:
            self._still_since = now
            return False
        if now - self._still_since >= self._t.hold_time - 1e-9:  # float timestamps
            self._moved = False  # report once; the next stop needs motion first
            self._still_since = None
            return True
        return False
