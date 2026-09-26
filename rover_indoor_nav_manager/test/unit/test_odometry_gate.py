# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""OdometryGate: odometry only around commanded motion, and never cut before the stop."""

from rover_indoor_nav_manager.domain.motion import StopDetector, StopThresholds
from rover_indoor_nav_manager.infrastructure.odometry_gate import OdometryGate

T = StopThresholds(linear=0.03, angular=0.05, hold_time=0.5)


def test_closed_until_the_first_command_and_opens_once():
    gate = OdometryGate(T, idle_timeout=5.0)
    assert not gate.is_open
    assert not gate.should_close(100.0)
    assert gate.on_command(1.0)
    assert gate.is_open
    assert not gate.on_command(1.1)  # already open: no second subscription


def test_stays_open_while_commands_keep_coming():
    gate = OdometryGate(T, idle_timeout=5.0)
    gate.on_command(0.0)
    for t in range(1, 20):
        gate.on_command(float(t))
        gate.on_sample(0.0, 0.0, float(t))  # still, e.g. zero commands during a pause
        assert not gate.should_close(float(t) + 0.5)


def test_closes_once_commands_are_quiet_and_the_rover_has_been_still():
    gate = OdometryGate(T, idle_timeout=5.0)
    gate.on_command(0.0)
    gate.on_sample(0.8, 0.0, 0.5)
    gate.on_sample(0.0, 0.0, 1.5)
    assert not gate.should_close(4.9)  # commands quiet for less than idle_timeout
    assert gate.should_close(5.0)
    assert not gate.is_open
    assert not gate.should_close(6.0)  # reports the close once


def test_does_not_close_while_the_rover_is_still_moving():
    # Commands stopped, but the rover is coasting or being driven by something else.
    gate = OdometryGate(T, idle_timeout=5.0)
    gate.on_command(0.0)
    for t in range(1, 10):
        gate.on_sample(0.4, 0.0, float(t))
        assert not gate.should_close(float(t))
    gate.on_sample(0.0, 0.0, 10.0)
    assert not gate.should_close(10.4)  # still for less than hold_time
    assert gate.should_close(10.5)


def test_never_closes_without_odometry():
    # A dead motion_topic must not be mistaken for a rover at rest.
    gate = OdometryGate(T, idle_timeout=5.0)
    gate.on_command(0.0)
    assert not gate.should_close(60.0)


def test_reopens_on_the_next_command():
    gate = OdometryGate(T, idle_timeout=5.0)
    gate.on_command(0.0)
    gate.on_sample(0.0, 0.0, 0.0)
    assert gate.should_close(5.0)
    assert gate.on_command(7.0)
    gate.on_sample(0.0, 0.0, 7.1)
    assert not gate.should_close(8.0)


def test_the_stop_detector_has_reported_the_stop_before_the_gate_closes():
    detector = StopDetector(T)
    gate = OdometryGate(T, idle_timeout=5.0)
    stops = []
    gate.on_command(0.0)
    closed_at = None
    t = 0.0
    while closed_at is None and t < 30.0:
        v = 0.5 if t < 2.0 else 0.0  # drive for 2 s, then stop; commands end at 2 s
        if t < 2.0:
            gate.on_command(t)
        gate.on_sample(v, 0.0, t)
        if detector.update(v, 0.0, t):
            stops.append(t)
        if gate.should_close(t):
            closed_at = t
        t = round(t + 0.02, 2)  # 50 Hz odometry
    assert stops and closed_at is not None
    assert stops[0] <= closed_at
