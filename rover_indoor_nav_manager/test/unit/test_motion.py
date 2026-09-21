# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

from rover_indoor_nav_manager.domain.motion import StopDetector, StopThresholds


def feed(detector, samples):
    """samples: (linear, angular, t) -> list of t where a stop was reported."""
    return [t for v, w, t in samples if detector.update(v, w, t)]


def test_reports_one_stop_after_motion_once_still_for_hold_time():
    d = StopDetector(StopThresholds(linear=0.03, angular=0.05, hold_time=0.5))
    samples = [(0.5, 0.0, 0.0), (0.4, 0.0, 0.1), (0.0, 0.0, 0.2), (0.0, 0.0, 0.5),
               (0.0, 0.0, 0.7), (0.0, 0.0, 1.0), (0.0, 0.0, 2.0)]
    assert feed(d, samples) == [0.7]


def test_standing_still_from_boot_is_not_a_stop():
    d = StopDetector()
    assert feed(d, [(0.0, 0.0, t / 10) for t in range(30)]) == []


def test_brief_zero_while_reversing_does_not_count():
    d = StopDetector(StopThresholds(hold_time=0.5))
    samples = [(0.3, 0.0, 0.0), (0.0, 0.0, 0.1), (0.0, 0.0, 0.3), (-0.3, 0.0, 0.4),
               (-0.3, 0.0, 0.6), (0.0, 0.0, 0.7), (0.0, 0.0, 1.3)]
    assert feed(d, samples) == [1.3]


def test_turning_on_the_spot_counts_as_motion_and_jitter_is_still():
    d = StopDetector(StopThresholds(linear=0.03, angular=0.05, hold_time=0.5))
    samples = [(0.0, 0.8, 0.0), (0.01, -0.02, 0.1), (0.02, 0.01, 0.4), (0.0, 0.0, 0.7)]
    assert feed(d, samples) == [0.7]
    # A second stop needs motion first.
    assert feed(d, [(0.0, 0.0, 2.0), (0.0, 0.0, 3.0)]) == []
    assert feed(d, [(0.2, 0.0, 4.0), (0.0, 0.0, 4.1), (0.0, 0.0, 4.6)]) == [4.6]
