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
import pytest

from rover_follow_me.domain.follow_rules import (
    abort_reason, Conditions, DriveMode, forwards_target, MissionState, start_refusal,
    TargetState)

READY = Conditions(drive_mode=DriveMode.AUTOMATIC, mission_state=MissionState.IDLE,
                   target_state=TargetState.TRACKING, target_fresh=True, server_ready=True)


def with_(**changes):
    return Conditions(**{**READY.__dict__, **changes})


def test_ready_rover_may_start():
    assert start_refusal(READY) is None
    assert abort_reason(READY) is None


@pytest.mark.parametrize('changes, words', [
    ({'drive_mode': DriveMode.ASSISTED}, 'ASSISTED'),
    ({'drive_mode': None}, 'unknown'),
    ({'mission_state': MissionState.RUNNING}, 'RUNNING'),
    ({'mission_state': MissionState.HELD}, 'HELD'),
    ({'server_ready': False}, 'Following server'),
    ({'target_state': TargetState.COASTING}, 'no person'),
    ({'target_fresh': False}, 'no person'),
])
def test_start_refusals(changes, words):
    refusal = start_refusal(with_(**changes))
    assert refusal and words in refusal


@pytest.mark.parametrize('state', [MissionState.SUCCEEDED, MissionState.FAILED,
                                   MissionState.CANCELLED, None])
def test_finished_or_unknown_missions_do_not_block(state):
    assert start_refusal(with_(mission_state=state)) is None


def test_abort_on_leaving_automatic_or_mission_start_but_not_on_losing_the_person():
    assert 'MANUAL' in abort_reason(with_(drive_mode=DriveMode.MANUAL))
    assert 'mission' in abort_reason(with_(mission_state=MissionState.RUNNING))
    # The Following server searches for a lost person itself.
    assert abort_reason(with_(target_state=TargetState.LOST, target_fresh=False)) is None


def test_only_measured_or_coasting_targets_are_forwarded():
    assert forwards_target(TargetState.TRACKING) and forwards_target(TargetState.COASTING)
    assert not forwards_target(TargetState.NONE) and not forwards_target(TargetState.LOST)
