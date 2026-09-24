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
