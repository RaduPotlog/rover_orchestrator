#!/usr/bin/env python3

# Copyright 2025 Mechatronics Academy
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

"""Introspection tests for localization_source routing.

These build the LaunchDescriptions and inspect them; nothing is spawned, so the test
needs no ROS graph and no nav2 binaries.

Two things are worth locking down here:

1. `localization_source` is declared in three launch files that must agree by hand
   (bringup, localization, rover_mission_manager). A divergence between them does not
   fail at startup -- it fails in the field, as goals stamped in a frame Nav 2 is not
   planning in. test_choices_are_identical turns that into a build failure.
2. AMCL must appear only for localization_source:=amcl, and must be in
   lifecycle_manager's node_names when it does, otherwise it is launched but never
   configured and never publishes map -> odom.
"""

import importlib.util
import os

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.utilities import evaluate_parameters

_NAV_SHARE = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_MISSION_LAUNCH = os.path.join(
    _NAV_SHARE, "..", "rover_mission_manager", "launch", "rover_mission_manager.launch.py"
)

MODES = ["odom", "gps", "slam", "amcl"]

def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

def _nav_launch(name):
    return _load(os.path.join(_NAV_SHARE, "launch", name), name.replace(".", "_"))

def _perform(context, value):
    """Resolve a name that may be a plain str or a list of substitutions.

    Node stores `name=` verbatim when it is a literal; ComposableNode normalizes it into
    a substitution list. Both appear here.
    """
    if isinstance(value, str):
        return value
    return "".join(context.perform_substitution(s) for s in value)

def _declared_choices(launch_description, arg_name="localization_source"):
    for entity in launch_description.entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.name == arg_name:
            return entity.choices
    return None

def _localization_actions(source, use_composition):
    """Resolve localization.launch.py for one mode and report what it would start."""
    module = _nav_launch("localization.launch.py")
    launch_description = module.generate_launch_description()

    context = LaunchContext()
    # Seeded before the declarations are visited: DeclareLaunchArgument only supplies a
    # default when the configuration is unset, and `map` deliberately has no default.
    context.launch_configurations.update(
        {
            "localization_source": source,
            "namespace": "rover",
            "map": "/maps/map.yaml",
            "use_composition": use_composition,
            "initial_pose_x": "1.5",
            "initial_pose_y": "-2.25",
            "initial_pose_yaw": "0.75",
        }
    )
    for entity in launch_description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(context)

    opaque = next(
        e for e in launch_description.entities if isinstance(e, OpaqueFunction)
    )

    plain, composable, node_names, amcl_overrides = [], [], None, None
    for action in opaque.execute(context):
        if isinstance(action, GroupAction):
            for node in action._GroupAction__actions:
                if not isinstance(node, Node):
                    continue
                plain.append(_perform(context, node._Node__node_name))
                # launch_ros stores parameters YAML-encoded; evaluate_parameters decodes
                # them, so this is exactly what the node would receive.
                dicts = [p for p in (node._Node__parameters or []) if isinstance(p, dict)]
                for params in evaluate_parameters(context, dicts):
                    if "node_names" in params:
                        node_names = list(params["node_names"])
                    if "initial_pose.x" in params:
                        amcl_overrides = dict(params)
        elif isinstance(action, LoadComposableNodes):
            composable = [
                _perform(context, d.node_name)
                for d in action._LoadComposableNodes__composable_node_descriptions
            ]

    return plain, composable, node_names, amcl_overrides

def test_choices_are_identical_across_launch_files():
    """All three declarations of localization_source must offer the same modes."""
    per_file = {
        "bringup": _declared_choices(_nav_launch("bringup.launch.py").generate_launch_description()),
        "localization": _declared_choices(
            _nav_launch("localization.launch.py").generate_launch_description()
        ),
        "mission_manager": _declared_choices(
            _load(_MISSION_LAUNCH, "mission_launch").generate_launch_description()
        ),
    }
    assert all(c is not None for c in per_file.values()), per_file
    assert len({tuple(c) for c in per_file.values()}) == 1, per_file
    assert sorted(per_file["bringup"]) == sorted(MODES), per_file

@pytest.mark.parametrize("use_composition", ["True", "False"])
def test_amcl_starts_only_in_amcl_mode(use_composition):
    for source in MODES:
        plain, composable, _, _ = _localization_actions(source, use_composition)
        started = composable if use_composition == "True" else plain
        assert ("amcl" in started) == (source == "amcl"), (source, use_composition, started)
        assert "map_server" in started, (source, started)

@pytest.mark.parametrize("use_composition", ["True", "False"])
def test_lifecycle_manager_manages_amcl_only_in_amcl_mode(use_composition):
    for source in MODES:
        _, _, node_names, _ = _localization_actions(source, use_composition)
        expected = ["map_server", "amcl"] if source == "amcl" else ["map_server"]
        # Order matters: lifecycle_manager transitions in list order, and AMCL blocks
        # waiting for the map, so map_server has to be activated first.
        assert node_names == expected, (source, use_composition, node_names)

def test_initial_pose_overrides_are_floats_and_only_for_amcl():
    _, _, _, overrides = _localization_actions("amcl", "False")
    assert overrides == {
        "initial_pose.x": 1.5,
        "initial_pose.y": -2.25,
        "initial_pose.yaw": 0.75,
    }, overrides
    # AMCL declares these as doubles; launch substitutions would hand it strings.
    assert all(isinstance(v, float) for v in overrides.values()), overrides

    for source in ("odom", "gps", "slam"):
        _, _, _, other = _localization_actions(source, "False")
        assert other is None, (source, other)

def test_bringup_forwards_localization_source_to_localization_launch():
    """bringup must pass the mode down, or localization.launch.py silently defaults."""
    from launch.actions import IncludeLaunchDescription

    launch_description = _nav_launch("bringup.launch.py").generate_launch_description()
    forwarded = []
    for entity in launch_description.entities:
        if not isinstance(entity, GroupAction):
            continue
        for action in entity._GroupAction__actions:
            if isinstance(action, IncludeLaunchDescription):
                forwarded.extend(k for k, _ in (action.launch_arguments or []))
    assert "localization_source" in forwarded, forwarded
    for name in ("initial_pose_x", "initial_pose_y", "initial_pose_yaw"):
        assert name in forwarded, (name, forwarded)
