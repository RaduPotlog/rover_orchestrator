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

"""The teleop collision monitor's zones cover every command and keep clear of the rover.

nav2_collision_monitor picks a velocity_polygon's sub-polygon from the commanded velocity,
first match wins, and leaves the previous zone active when none matches. A gap in coverage
is therefore a silent failure, and a zone that overlaps the rover itself stops it for good.
"""

import itertools
import json
import math
from pathlib import Path

import pytest
import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[2]

# Padded Nav 2 footprint (rover_navigation bringup.launch.py): wheel edges + 0.04 m.
FOOTPRINT_HALF_LENGTH = 0.4566
FOOTPRINT_HALF_WIDTH = 0.4015
# What a turn in place sweeps: the footprint's corner radius.
CORNER_RADIUS = (FOOTPRINT_HALF_LENGTH ** 2 + FOOTPRINT_HALF_WIDTH ** 2) ** 0.5

# Commands the monitor may see: the UI caps at 1.0 m/s, the RC budget at 0.95 m/s and
# 1.5 rad/s; the config promises [-2, 2] x [-4, 4].
LINEAR_SAMPLES = [x / 100.0 for x in range(-200, 201, 1)]
ANGULAR_SAMPLES = [w / 10.0 for w in range(-40, 41, 1)]


@pytest.fixture(scope='module')
def params():
    config = yaml.safe_load(
        (PACKAGE_DIR / 'config' / 'teleop_collision_monitor.yaml').read_text())
    return config['/**']['teleop_collision_monitor']['ros__parameters']


def points(sub_polygon):
    return json.loads(sub_polygon['points'])


def select(polygon, linear, angular):
    """The sub-polygon nav2_collision_monitor's VelocityPolygon picks (non-holonomic)."""
    for name in polygon['velocity_polygons']:
        sub = polygon[name]
        if (sub['theta_min'] <= angular <= sub['theta_max']
                and sub['linear_min'] <= linear <= sub['linear_max']):
            return name
    return None


def on_boundary(polygon, x, y, eps=1e-9):
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        cross = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)
        within = min(x1, x2) - eps <= x <= max(x1, x2) + eps and \
            min(y1, y2) - eps <= y <= max(y1, y2) + eps
        if abs(cross) <= eps and within:
            return True
    return False


def contains(polygon, x, y):
    """Point in polygon, even-odd rule (boundary counts as inside)."""
    if on_boundary(polygon, x, y):
        return True
    inside = False
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y1 > y) != (y2 > y):
            x_cross = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x <= x_cross:
                inside = not inside
    return inside


def test_monitor_topics_match_the_manager(params):
    assert params['cmd_vel_in_topic'] == 'teleop_guard_in'
    assert params['cmd_vel_out_topic'] == 'teleop_guard_out'
    assert params['state_topic'] == 'teleop_collision_monitor_state'
    assert params['enable_stamped_cmd_vel'] is True


def test_stale_lidar_stops_the_rover(params):
    # 0 would disable the check and let the guard pass commands with no data at all.
    assert 0.0 < params['source_timeout'] <= 1.0


@pytest.mark.parametrize('polygon_name', ['teleop_stop', 'teleop_slow'])
def test_every_command_selects_a_zone(params, polygon_name):
    polygon = params[polygon_name]
    uncovered = [
        (v, w) for v, w in itertools.product(LINEAR_SAMPLES, ANGULAR_SAMPLES)
        if select(polygon, v, w) is None
    ]
    assert not uncovered, f'{polygon_name}: no zone for {uncovered[:5]}'


@pytest.mark.parametrize('polygon_name', ['teleop_stop', 'teleop_slow'])
def test_direction_picks_the_matching_zone(params, polygon_name):
    polygon = params[polygon_name]
    assert select(polygon, 0.0, 1.0) == 'rotate'
    assert select(polygon, 0.0, 0.0) == 'rotate'
    assert select(polygon, 0.5, 0.0) == 'forward'
    assert select(polygon, 0.5, 1.2) == 'forward'
    assert select(polygon, -0.3, 0.0) == 'reverse'


@pytest.mark.parametrize('polygon_name', ['teleop_stop', 'teleop_slow'])
@pytest.mark.parametrize('zone', ['forward', 'reverse'])
def test_driving_zones_stay_outside_the_footprint(params, polygon_name, zone):
    """Otherwise points on the rover itself (the lidar support post) would trip them."""
    for x, _ in points(params[polygon_name][zone]):
        assert abs(x) >= FOOTPRINT_HALF_LENGTH, f'{polygon_name}.{zone} reaches into the rover'


@pytest.mark.parametrize('polygon_name', ['teleop_stop', 'teleop_slow'])
def test_driving_zones_are_wider_than_the_rover(params, polygon_name):
    for zone in ('forward', 'reverse'):
        ys = [y for _, y in points(params[polygon_name][zone])]
        assert min(ys) < -FOOTPRINT_HALF_WIDTH and max(ys) > FOOTPRINT_HALF_WIDTH


@pytest.mark.parametrize('polygon_name', ['teleop_stop', 'teleop_slow'])
def test_rotate_zone_covers_what_a_turn_in_place_sweeps(params, polygon_name):
    rotate = points(params[polygon_name]['rotate'])
    for step in range(72):
        angle = step * math.pi / 36
        x, y = CORNER_RADIUS * math.cos(angle), CORNER_RADIUS * math.sin(angle)
        assert contains(rotate, x, y), f'{polygon_name}.rotate misses ({x:.2f}, {y:.2f})'


@pytest.mark.parametrize('zone', ['rotate', 'forward', 'reverse'])
def test_stop_zone_lies_inside_its_slow_zone(params, zone):
    """Something in the stop zone must also be slowing the rover down on approach."""
    slow = points(params['teleop_slow'][zone])
    for x, y in points(params['teleop_stop'][zone]):
        assert contains(slow, x, y), f'stop.{zone} corner ({x}, {y}) outside slow.{zone}'


def test_zone_ranges_match_between_stop_and_slow(params):
    for zone in ('rotate', 'forward', 'reverse'):
        stop, slow = params['teleop_stop'][zone], params['teleop_slow'][zone]
        for key in ('linear_min', 'linear_max', 'theta_min', 'theta_max'):
            assert stop[key] == slow[key], f'{zone}.{key}'


def test_names_do_not_clash_with_the_nav2_monitor(params):
    """Both monitors live in the rover namespace; polygon topics default to polygon names."""
    assert params['state_topic'] != 'collision_monitor_state'
    for name in params['polygons']:
        assert name.startswith('teleop_')
        assert params[name]['polygon_pub_topic'].startswith('teleop_')


def test_slow_down_actually_slows(params):
    assert 0.0 < params['teleop_slow']['slowdown_ratio'] < 1.0
