# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

import logging
import sys

import pytest

from rover_indoor_nav_manager.domain.model import Pose2D
from rover_indoor_nav_manager.infrastructure.launch_localization_controller import (
    child_env,
    LaunchLocalizationController,
)


class Logger:
    def __init__(self):
        self._log = logging.getLogger('test')

    def info(self, msg):
        self._log.info(msg)

    def warning(self, msg):
        self._log.warning(msg)


def controller(**kw):
    return LaunchLocalizationController(Logger(), 'rover', '/tmp/params.yaml', False, **kw)


def test_mapping_command():
    cmd = controller().command('mapping')
    assert cmd[:4] == ['ros2', 'launch', 'rover_navigation', 'indoor_localization.launch.py']
    assert 'mode:=mapping' in cmd
    assert 'namespace:=rover' in cmd
    assert 'params_file:=/tmp/params.yaml' in cmd
    assert not any(a.startswith('map:=') for a in cmd)


def test_localization_command_carries_map_and_pose():
    cmd = controller().command('localization', '/maps/lab/map.yaml', Pose2D(1.0, -2.0, 0.5))
    assert 'map:=/maps/lab/map.yaml' in cmd
    assert 'initial_pose_x:=1.0' in cmd
    assert 'initial_pose_y:=-2.0' in cmd
    assert 'initial_pose_yaw:=0.5' in cmd


def test_process_group_lifecycle(monkeypatch):
    """Start/stop a stand-in child that ignores nothing; the whole group must be gone."""
    ctl = controller(stop_timeout=2.0)
    child = [sys.executable, '-c', 'import time, subprocess, sys; '
             'subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"]); '
             'time.sleep(60)']
    monkeypatch.setattr(ctl, 'command', lambda *a, **k: child)
    ctl.start_mapping()
    assert ctl.running()
    with pytest.raises(RuntimeError, match='already running'):
        ctl.start_mapping()
    ctl.stop()
    assert not ctl.running()
    ctl.stop()  # idempotent


def test_immediate_exit_is_reported(monkeypatch):
    ctl = controller()
    monkeypatch.setattr(ctl, 'command', lambda *a, **k: [sys.executable, '-c', 'raise SystemExit(3)'])
    with pytest.raises(RuntimeError, match='code 3'):
        ctl.start_mapping()


ZENOH = {'RMW_IMPLEMENTATION': 'rmw_zenoh_cpp', 'PATH': '/usr/bin'}


def test_child_env_sets_client_mode():
    env = child_env(ZENOH, zenoh_client=True)
    assert env['ZENOH_CONFIG_OVERRIDE'] == 'mode="client"'
    assert env['PATH'] == '/usr/bin'


def test_child_env_appends_to_existing_override():
    base = dict(ZENOH, ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/10.0.0.1:7447"]')
    env = child_env(base, zenoh_client=True)
    assert env['ZENOH_CONFIG_OVERRIDE'] == 'connect/endpoints=["tcp/10.0.0.1:7447"];mode="client"'
    assert 'mode=' not in base['ZENOH_CONFIG_OVERRIDE']  # the caller's mapping is not modified


@pytest.mark.parametrize('base, enabled', [
    (ZENOH, False),
    ({'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'}, True),
    ({}, True),
])
def test_child_env_unchanged(base, enabled):
    assert child_env(base, zenoh_client=enabled) == base


@pytest.mark.parametrize('enabled, expected', [(True, 'mode="client"'), (False, 'unset')])
def test_child_launch_runs_with_client_override(monkeypatch, tmp_path, enabled, expected):
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp')
    monkeypatch.delenv('ZENOH_CONFIG_OVERRIDE', raising=False)
    out = tmp_path / 'override'
    ctl = controller(zenoh_client=enabled)
    monkeypatch.setattr(ctl, 'command', lambda *a, **k: [
        sys.executable, '-c',
        f'import os, time; open({str(out)!r}, "w").write('
        'os.environ.get("ZENOH_CONFIG_OVERRIDE", "unset")); time.sleep(60)'])
    ctl.start_mapping()
    ctl.stop()
    assert out.read_text() == expected


class Recorder:
    def __init__(self, error=None):
        self.calls = []
        self.error = error

    def load(self, map_yaml):
        if self.error:
            raise RuntimeError(self.error)
        self.calls.append(map_yaml)

    def seed(self, pose, sigma_xy, sigma_yaw):
        self.calls.append((pose, sigma_xy, sigma_yaw))

    def cancel(self):
        pass


def test_switch_map_loads_then_seeds_with_amcl_default_spread(monkeypatch):
    loader, seeder = Recorder(), Recorder()
    ctl = controller(map_loader=loader, initial_pose_seeder=seeder)
    monkeypatch.setattr(ctl, 'running', lambda: True)
    ctl.switch_map('/maps/hall/map.yaml', Pose2D(1.0, 2.0, 0.3))
    assert loader.calls == ['/maps/hall/map.yaml']
    assert seeder.calls == [(Pose2D(1.0, 2.0, 0.3), 0.5, pytest.approx(0.2618, abs=1e-4))]


def test_switch_map_refuses_without_a_running_stack_or_loader(monkeypatch):
    with pytest.raises(RuntimeError, match='no map loader'):
        controller().switch_map('/m.yaml', Pose2D(0, 0, 0))
    seeder = Recorder()
    ctl = controller(map_loader=Recorder(), initial_pose_seeder=seeder)
    with pytest.raises(RuntimeError, match='not running'):
        ctl.switch_map('/m.yaml', Pose2D(0, 0, 0))
    monkeypatch.setattr(ctl, 'running', lambda: True)
    ctl._map_loader = Recorder(error='map_server could not load /m.yaml (result 1)')
    with pytest.raises(RuntimeError, match='result 1'):
        ctl.switch_map('/m.yaml', Pose2D(0, 0, 0))
    assert seeder.calls == []  # no seed for a map AMCL never got
