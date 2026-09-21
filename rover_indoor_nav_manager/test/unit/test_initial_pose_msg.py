# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

import math

import pytest

from rover_indoor_nav_manager.domain.model import Pose2D
from rover_indoor_nav_manager.infrastructure.ros_adapters import initial_pose_msg


def test_covariance_diagonal_and_orientation():
    msg = initial_pose_msg('rover/map', Pose2D(1.0, -2.0, math.pi / 2), 1.5, math.pi / 4)
    assert msg.header.frame_id == 'rover/map'
    assert (msg.pose.pose.position.x, msg.pose.pose.position.y) == (1.0, -2.0)
    q = msg.pose.pose.orientation
    assert math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z) == pytest.approx(math.pi / 2)
    cov = list(msg.pose.covariance)
    assert cov[0] == pytest.approx(2.25) and cov[7] == pytest.approx(2.25)
    assert cov[35] == pytest.approx((math.pi / 4) ** 2)
    assert sum(abs(c) for i, c in enumerate(cov) if i not in (0, 7, 35)) == 0.0
