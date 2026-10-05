# Copyright 2026 Mechatronics Academy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
