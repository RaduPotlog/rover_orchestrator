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
Follow-me as one Nav 2 FollowObject goal: start it, watch it, stop it.

The Following server does the following itself (distance, heading, search rotation when the
person is lost). This use case only decides when a goal may be sent and when it must be
cancelled (domain/follow_rules.py), and turns the server's feedback and result into a status
line for the drive UI, the VDA 5050 adapter and the logs:

    IDLE | STARTING | FOLLOWING | SEARCHING | STOPPING, as "PHASE: detail"

Not thread-safe: the node calls it from one executor thread.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

from ..domain.follow_rules import abort_reason, Conditions, start_refusal
from .ports import FollowActionPort, Log


class Phase(str, Enum):
    IDLE = 'IDLE'
    STARTING = 'STARTING'    # goal sent, not accepted yet
    FOLLOWING = 'FOLLOWING'
    SEARCHING = 'SEARCHING'  # the server lost the person and is retrying / rotating to look
    STOPPING = 'STOPPING'    # cancel sent, waiting for the result


RUNNING_PHASES = (Phase.STARTING, Phase.FOLLOWING, Phase.SEARCHING)

# FollowObject feedback state -> (phase, detail)
_FEEDBACK = {
    'INITIAL_PERCEPTION': (Phase.FOLLOWING, 'waiting for the person'),
    'CONTROLLING': (Phase.FOLLOWING, 'following'),
    'STOPPING': (Phase.FOLLOWING, 'at following distance'),
    'RETRY': (Phase.SEARCHING, 'person lost; searching'),
}


@dataclass(frozen=True)
class ExecutorConfig:
    # A tracked_person older than this counts as no target.
    target_timeout: float = 1.0


class FollowExecutor:

    def __init__(self, config: ExecutorConfig, action: FollowActionPort, log: Log):
        self._config = config
        self._action = action
        self._log = log
        self._phase = Phase.IDLE
        self._detail = 'not started'
        self._drive_mode: Optional[int] = None
        self._mission_state: Optional[int] = None
        self._target_state: Optional[int] = None
        self._target_at: Optional[float] = None

    # --- inputs ---------------------------------------------------------------------------------

    def on_drive_mode(self, mode: int) -> None:
        self._drive_mode = mode

    def on_mission_state(self, state: int) -> None:
        self._mission_state = state

    def on_target(self, state: int, now: float) -> None:
        self._target_state = state
        self._target_at = now

    # --- commands -------------------------------------------------------------------------------

    def start(self, now: float) -> Tuple[bool, str]:
        if self._phase in RUNNING_PHASES:
            return False, 'already following'
        if self._phase == Phase.STOPPING:
            return False, 'still stopping the previous follow; try again'
        refusal = start_refusal(self._conditions(now))
        if refusal:
            self._log.warning(f'Follow-me not started: {refusal}')
            return False, refusal
        self._action.send_goal()
        self._set(Phase.STARTING, 'sending the FollowObject goal')
        return True, 'follow-me starting'

    def stop(self, reason: str = 'stopped on request') -> Tuple[bool, str]:
        if self._phase == Phase.STOPPING:
            return True, 'already stopping'
        if self._phase not in RUNNING_PHASES:
            return False, 'not following'
        self._action.cancel()
        self._set(Phase.STOPPING, reason)
        return True, reason

    def tick(self, now: float) -> None:
        if self._phase not in RUNNING_PHASES:
            return
        reason = abort_reason(self._conditions(now))
        if reason:
            self._log.warning(f'Follow-me stopped: {reason}')
            self.stop(reason)

    # --- action callbacks -----------------------------------------------------------------------

    def on_goal_accepted(self) -> None:
        if self._phase == Phase.STARTING:
            self._set(Phase.FOLLOWING, 'waiting for the person')

    def on_goal_rejected(self, why: str) -> None:
        if self._phase in (Phase.STARTING, Phase.STOPPING):
            self._set(Phase.IDLE, f'the Following server refused the goal: {why}')

    def on_feedback(self, state_name: str) -> None:
        if self._phase not in (Phase.FOLLOWING, Phase.SEARCHING):
            return
        phase, detail = _FEEDBACK.get(state_name, (Phase.FOLLOWING, state_name.lower()))
        self._set(phase, detail)

    def on_result(self, outcome: str, detail: str = '') -> None:
        """outcome: 'succeeded', 'canceled' or 'aborted'; detail: the server's error, if any."""
        if self._phase == Phase.STOPPING:
            self._set(Phase.IDLE, self._detail)  # keep why it was stopped
            return
        if self._phase == Phase.IDLE:
            return
        if outcome == 'succeeded':
            text = 'following ended (max duration)'
        elif outcome == 'canceled':
            text = 'following cancelled by someone else'
        else:
            text = f'following ended: {detail}' if detail else 'following ended'
        self._log.warning(f'Follow-me: {text}')
        self._set(Phase.IDLE, text)

    # --- outputs --------------------------------------------------------------------------------

    @property
    def phase(self) -> Phase:
        return self._phase

    def status(self) -> str:
        return f'{self._phase.value}: {self._detail}'

    # --- internals ------------------------------------------------------------------------------

    def _conditions(self, now: float) -> Conditions:
        fresh = (self._target_at is not None
                 and now - self._target_at <= self._config.target_timeout)
        return Conditions(
            drive_mode=self._drive_mode,
            mission_state=self._mission_state,
            target_state=self._target_state,
            target_fresh=fresh,
            server_ready=self._action.ready())

    def _set(self, phase: Phase, detail: str) -> None:
        if (phase, detail) != (self._phase, self._detail):
            self._log.info(f'follow_me: {phase.value}: {detail}')
        self._phase = phase
        self._detail = detail
