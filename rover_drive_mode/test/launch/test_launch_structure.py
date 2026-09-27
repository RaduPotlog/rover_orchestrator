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

"""The launch file's structure: the manager, the teleop guard and the guard's lifecycle manager."""

import importlib.util
from pathlib import Path

import pytest
import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[2]


@pytest.fixture
def drive_mode_launch(monkeypatch):
    path = PACKAGE_DIR / 'launch' / 'rover_drive_mode.launch.py'
    spec = importlib.util.spec_from_file_location('rover_drive_mode_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    captured = {}
    original_node = module.Node

    def capture(**kwargs):
        captured[kwargs['name']] = kwargs
        return original_node(**kwargs)

    monkeypatch.setattr(module, 'Node', capture)
    description = module.generate_launch_description()
    return module, captured, description


def test_launches_the_manager_the_guard_and_its_lifecycle_manager(drive_mode_launch):
    _, nodes, _ = drive_mode_launch
    assert set(nodes) == {
        'drive_mode_manager', 'teleop_collision_monitor', 'lifecycle_manager_teleop_guard'}

    assert nodes['drive_mode_manager']['package'] == 'rover_drive_mode'
    assert nodes['drive_mode_manager']['executable'] == 'drive_mode_manager'
    assert nodes['teleop_collision_monitor']['package'] == 'nav2_collision_monitor'
    assert nodes['teleop_collision_monitor']['executable'] == 'collision_monitor'


def test_lifecycle_manager_activates_the_teleop_guard(drive_mode_launch):
    """Without it the monitor stays unconfigured and ASSISTED blocks every command."""
    _, nodes, _ = drive_mode_launch
    params = nodes['lifecycle_manager_teleop_guard']['parameters'][0]
    assert params['node_names'] == ['teleop_collision_monitor']
    assert params['autostart'] is True


def test_guard_nodes_and_manager_follow_use_teleop_guard(drive_mode_launch):
    _, nodes, _ = drive_mode_launch
    assert nodes['teleop_collision_monitor']['condition'] is not None
    assert nodes['lifecycle_manager_teleop_guard']['condition'] is not None
    # The manager must know when the guard is not launched, or it would accept ASSISTED.
    overrides = nodes['drive_mode_manager']['parameters'][1]
    assert 'use_teleop_guard' in overrides
    assert 'default_mode' in overrides


def test_every_node_is_namespaced(drive_mode_launch):
    _, nodes, _ = drive_mode_launch
    for kwargs in nodes.values():
        assert 'namespace' in kwargs


def test_boot_mode_argument_excludes_automatic(drive_mode_launch):
    _, _, description = drive_mode_launch
    args = {arg.name: arg for arg in description.get_launch_arguments()}
    assert set(args['default_mode'].choices) == {'manual', 'assisted'}


def test_manager_config_is_keyed_on_the_launched_node_name():
    config = yaml.safe_load((PACKAGE_DIR / 'config' / 'drive_mode_manager.yaml').read_text())
    assert '/**/drive_mode_manager' in config


def test_guard_config_is_keyed_on_the_launched_node_name():
    config = yaml.safe_load(
        (PACKAGE_DIR / 'config' / 'teleop_collision_monitor.yaml').read_text())
    assert 'teleop_collision_monitor' in config['/**']
