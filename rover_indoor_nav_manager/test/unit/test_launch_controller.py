# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

import logging
import sys

import pytest

from rover_indoor_nav_manager.domain.model import Pose2D
from rover_indoor_nav_manager.infrastructure.launch_localization_controller import (
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
