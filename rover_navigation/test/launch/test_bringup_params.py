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

"""Every `<token>` in rover_nav_params.yaml is one bringup.launch.py substitutes.

A token nobody replaces does not fail at startup: Nav 2 subscribes to a topic literally named
`<scan_topic>`, or a footprint fails to parse, and the rover drives blind.
"""

import importlib.util
import os
import re

_NAV_SHARE = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_TOKEN = re.compile(r"<[a-z_]+>")


def _bringup_replacements(monkeypatch):
    path = os.path.join(_NAV_SHARE, "launch", "bringup.launch.py")
    spec = importlib.util.spec_from_file_location("bringup_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    captured = []

    def capture(**kwargs):
        captured.append(kwargs["replacements"])
        return original(**kwargs)

    original = module.ReplaceString
    monkeypatch.setattr(module, "ReplaceString", capture)
    module.generate_launch_description()
    assert captured
    return captured


def test_every_params_token_is_substituted(monkeypatch):
    with open(os.path.join(_NAV_SHARE, "config", "rover_nav_params.yaml")) as f:
        # Comments may name tokens for the reader (<ns>, <costmap>); only values matter.
        values = "\n".join(line.split("#", 1)[0] for line in f)

    for replacements in _bringup_replacements(monkeypatch):
        remaining = values
        # Literal keys, exactly as ReplaceString applies them: "<namespace>/" only matches
        # with its slash.
        for key in replacements:
            remaining = remaining.replace(key, "")
        assert _TOKEN.findall(remaining) == []
