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
from rover_follow_me.application.follow_executor import (
    ExecutorConfig, FollowExecutor, Phase)
from rover_follow_me.application.ports import FollowActionPort, Log
from rover_follow_me.domain.follow_rules import DriveMode, MissionState, TargetState


class FakeAction(FollowActionPort):

    def __init__(self):
        self.up = True
        self.goals = 0
        self.cancels = 0

    def ready(self):
        return self.up

    def send_goal(self):
        self.goals += 1

    def cancel(self):
        self.cancels += 1


class FakeLog(Log):

    def __init__(self):
        self.lines = []

    def info(self, message):
        self.lines.append(message)

    def warning(self, message):
        self.lines.append(message)


def ready_executor():
    action = FakeAction()
    ex = FollowExecutor(ExecutorConfig(target_timeout=1.0), action, FakeLog())
    ex.on_drive_mode(DriveMode.AUTOMATIC)
    ex.on_mission_state(MissionState.IDLE)
    ex.on_target(TargetState.TRACKING, 10.0)
    return ex, action


def following():
    ex, action = ready_executor()
    assert ex.start(10.1)[0]
    ex.on_goal_accepted()
    return ex, action


def test_start_sends_one_goal_and_reports_progress():
    ex, action = ready_executor()
    ok, _ = ex.start(10.1)
    assert ok and action.goals == 1 and ex.phase == Phase.STARTING
    ex.on_goal_accepted()
    assert ex.status() == 'FOLLOWING: waiting for the person'
    ex.on_feedback('CONTROLLING')
    assert ex.status() == 'FOLLOWING: following'
    ex.on_feedback('RETRY')
    assert ex.phase == Phase.SEARCHING
    ex.on_feedback('CONTROLLING')
    assert ex.phase == Phase.FOLLOWING
    assert not ex.start(11.0)[0]


def test_start_refused_without_automatic_or_with_a_stale_target():
    ex, action = ready_executor()
    ex.on_drive_mode(DriveMode.ASSISTED)
    ok, why = ex.start(10.1)
    assert not ok and 'Automatic' in why
    ex.on_drive_mode(DriveMode.AUTOMATIC)
    ok, why = ex.start(12.0)  # last target 2 s ago
    assert not ok and 'no person' in why
    assert action.goals == 0 and ex.phase == Phase.IDLE


def test_stop_cancels_and_result_returns_to_idle_keeping_the_reason():
    ex, action = following()
    ok, _ = ex.stop()
    assert ok and action.cancels == 1 and ex.phase == Phase.STOPPING
    assert ex.stop()[0]  # already stopping: no second cancel
    assert action.cancels == 1
    ex.on_result('canceled')
    assert ex.status() == 'IDLE: stopped on request'
    assert not ex.stop()[0]


def test_stop_before_acceptance_still_cancels():
    ex, action = ready_executor()
    ex.start(10.1)
    ex.stop()
    assert action.cancels == 1 and ex.phase == Phase.STOPPING
    ex.on_goal_rejected('rejected')
    assert ex.phase == Phase.IDLE


def test_leaving_automatic_or_a_mission_starting_cancels():
    for event in ('mode', 'mission'):
        ex, action = following()
        if event == 'mode':
            ex.on_drive_mode(DriveMode.ASSISTED)
        else:
            ex.on_mission_state(MissionState.RUNNING)
        ex.tick(11.0)
        assert action.cancels == 1 and ex.phase == Phase.STOPPING
        ex.on_result('canceled')
        assert ex.phase == Phase.IDLE
        assert ('ASSISTED' if event == 'mode' else 'mission') in ex.status()


def test_losing_the_person_is_left_to_the_server_until_it_gives_up():
    ex, action = following()
    ex.on_target(TargetState.LOST, 11.0)
    ex.tick(20.0)
    assert action.cancels == 0 and ex.phase == Phase.FOLLOWING
    ex.on_result('aborted', 'FAILED_TO_DETECT_OBJECT: lost detection')
    assert ex.phase == Phase.IDLE
    assert 'FAILED_TO_DETECT_OBJECT' in ex.status()


def test_rejected_goal_and_foreign_cancel_are_reported():
    ex, _ = ready_executor()
    ex.start(10.1)
    ex.on_goal_rejected('server busy')
    assert ex.phase == Phase.IDLE and 'server busy' in ex.status()
    ex, _ = following()
    ex.on_result('canceled')
    assert ex.phase == Phase.IDLE and 'someone else' in ex.status()


def test_status_line_format():
    ex, _ = ready_executor()
    assert ex.status() == 'IDLE: not started'
