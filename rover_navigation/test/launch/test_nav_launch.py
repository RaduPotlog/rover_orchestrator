#!/usr/bin/env python3

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

"""The Nav 2 command path ends at the collision monitor, in both composition branches.

velocity_smoother -> cmd_vel_smoothed -> collision_monitor -> nav_cmd_vel_guarded, which
rover_drive_mode forwards to the platform only in AUTOMATIC. If the smoother still published
nav_cmd_vel_stamped itself, Nav 2 would drive the rover in every mode and bypass the guard.
"""

import os

import pytest
import yaml

_NAV_SHARE = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


@pytest.fixture
def nav_launch(monkeypatch):
    import importlib.util

    path = os.path.join(_NAV_SHARE, "launch", "rover_nav.launch.py")
    spec = importlib.util.spec_from_file_location("rover_nav_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    captured = {"nodes": {}, "composable": {}}

    def capture(original, kind):
        def wrapper(**kwargs):
            # controller_server is launched without name=; its executable names it.
            name = kwargs.get("name") or kwargs.get("executable")
            captured[kind][name] = kwargs
            return original(**kwargs)
        return wrapper

    monkeypatch.setattr(module, "Node", capture(module.Node, "nodes"))
    monkeypatch.setattr(module, "ComposableNode", capture(module.ComposableNode, "composable"))
    module.generate_launch_description()
    return captured


@pytest.mark.parametrize("kind", ["nodes", "composable"])
def test_collision_monitor_is_launched_and_lifecycle_managed(nav_launch, kind):
    launched = nav_launch[kind]
    assert "collision_monitor" in launched

    manager = launched["lifecycle_manager_navigation"]
    params = manager["parameters"]
    node_names = next(
        (p["node_names"] for p in params if isinstance(p, dict) and "node_names" in p), None)
    assert node_names is not None
    # Launched but never activated, the monitor would swallow every Nav 2 command.
    assert "collision_monitor" in node_names
    assert node_names[-1] == "collision_monitor"


@pytest.mark.parametrize("kind", ["nodes", "composable"])
def test_velocity_smoother_no_longer_feeds_the_platform(nav_launch, kind):
    smoother = nav_launch[kind]["velocity_smoother"]
    targets = [dst for _, dst in smoother.get("remappings", [])]
    assert "nav_cmd_vel_stamped" not in targets
    assert ("cmd_vel_smoothed", "nav_cmd_vel_stamped") not in smoother.get("remappings", [])


def test_collision_monitor_config_chains_the_smoother_to_the_drive_mode_manager():
    with open(os.path.join(_NAV_SHARE, "config", "rover_nav_params.yaml")) as f:
        # <namespace>/ and the other tokens are plain strings to YAML.
        params = yaml.safe_load(f)["/**"]["collision_monitor"]["ros__parameters"]

    assert params["cmd_vel_in_topic"] == "cmd_vel_smoothed"
    assert params["cmd_vel_out_topic"] == "nav_cmd_vel_guarded"
    assert params["state_topic"] == "collision_monitor_state"
    assert params["enable_stamped_cmd_vel"] is True
    for name in params["polygons"]:
        assert name.startswith("nav_"), "clashes with rover_drive_mode's teleop_* polygons"
