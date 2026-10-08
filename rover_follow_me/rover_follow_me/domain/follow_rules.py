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
"""
When the rover may start following, and when following must stop.

Following drives through Nav 2's Following server, whose velocity goes into cmd_vel_nav, the
same topic the controller_server uses for missions, and on through drive mode, which passes it
only in AUTOMATIC. So following needs AUTOMATIC and must never overlap a mission: a mission
that starts while the rover follows wins (a fleet order, a GoTo), and following stops.
"""

from dataclasses import dataclass
from enum import IntEnum
from typing import Optional


class DriveMode(IntEnum):
    # rover_msgs/DriveMode
    MANUAL = 1
    ASSISTED = 2
    AUTOMATIC = 3


class MissionState(IntEnum):
    # rover_msgs/MissionState
    IDLE = 0
    RUNNING = 1
    HELD = 2
    SUCCEEDED = 3
    FAILED = 4
    CANCELLED = 5


class TargetState(IntEnum):
    # rover_msgs/TrackedPerson
    NONE = 0
    TRACKING = 1
    COASTING = 2
    LOST = 3


# A mission in these states owns cmd_vel_nav.
ACTIVE_MISSION_STATES = (MissionState.RUNNING, MissionState.HELD)


def forwards_target(state: int) -> bool:
    """Positions the Following server should get: measured, or briefly predicted (occluded)."""
    return state in (TargetState.TRACKING, TargetState.COASTING)


@dataclass(frozen=True)
class Conditions:
    """What the rover reports, as the executor last heard it. None = never heard."""

    drive_mode: Optional[int] = None
    mission_state: Optional[int] = None
    target_state: Optional[int] = None
    target_fresh: bool = False
    server_ready: bool = False


def _mode_name(mode: Optional[int]) -> str:
    if mode is None:
        return 'unknown (no drive_mode)'
    try:
        return DriveMode(mode).name
    except ValueError:
        return str(mode)


def _mission_name(state: Optional[int]) -> str:
    try:
        return MissionState(state).name
    except (TypeError, ValueError):
        return str(state)


def start_refusal(c: Conditions) -> Optional[str]:
    """Why following cannot start now, or None."""
    if c.drive_mode != DriveMode.AUTOMATIC:
        return f'drive mode is {_mode_name(c.drive_mode)}; switch the rover to Automatic'
    if c.mission_state in ACTIVE_MISSION_STATES:
        return f'a mission is {_mission_name(c.mission_state)}; stop it first'
    if not c.server_ready:
        return 'the Nav 2 Following server (follow_object) is not available'
    if not c.target_fresh or c.target_state != TargetState.TRACKING:
        return 'no person tracked; stand about 1.5 m in front of the stopped rover'
    return None


def abort_reason(c: Conditions) -> Optional[str]:
    """Why running following must stop now, or None."""
    if c.drive_mode != DriveMode.AUTOMATIC:
        return f'drive mode changed to {_mode_name(c.drive_mode)}'
    if c.mission_state in ACTIVE_MISSION_STATES:
        return f'a mission started ({_mission_name(c.mission_state)})'
    return None
