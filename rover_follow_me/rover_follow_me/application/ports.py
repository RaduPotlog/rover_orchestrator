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
"""Ports the follow executor drives; the ROS node implements them."""

from abc import ABC, abstractmethod


class FollowActionPort(ABC):
    """The FollowObject action of Nav 2's Following server, seen from the executor.

    Asynchronous: the implementation reports back through the executor's on_goal_accepted,
    on_goal_rejected, on_feedback and on_result.
    """

    @abstractmethod
    def ready(self) -> bool:
        """The action server is up."""

    @abstractmethod
    def send_goal(self) -> None:
        """Ask the server to follow the target pose topic, until cancelled."""

    @abstractmethod
    def cancel(self) -> None:
        """Cancel the goal sent last (also when it has not been accepted yet)."""


class Log(ABC):

    @abstractmethod
    def info(self, message: str) -> None: ...

    @abstractmethod
    def warning(self, message: str) -> None: ...
