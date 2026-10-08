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
Lifecycle node follow_me: start/stop follow-me as a Nav 2 FollowObject goal.

- tracked_person (rover_msgs/TrackedPerson, from rover_perception's fmoc or another tracker) is
  republished as follow_me/target_pose (PoseStamped), the pose topic the goal names.
- follow_me/start and follow_me/stop (std_srvs/Trigger) send and cancel the goal; the drive UI
  and rover_vda5050's startFollowing / stopFollowing call them.
- follow_me/status (std_msgs/String, latched): "PHASE: detail".

The node owns the pose bridge, so if it dies the Following server stops getting poses, searches
and gives up on its own; on Ctrl-C / SIGTERM it cancels the goal first.
"""

import signal
import time
from typing import Optional

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowObject
import rclpy
from rclpy.action import ActionClient
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rover_follow_me.application.follow_executor import ExecutorConfig, FollowExecutor
from rover_follow_me.application.ports import FollowActionPort, Log
from rover_follow_me.domain.follow_rules import forwards_target
from rover_msgs.msg import DriveMode, MissionState, TrackedPerson


def _constant_names(cls) -> dict:
    """Map the uint constants of a generated message class to their names."""
    names = {}
    for name in dir(cls):
        # Skip the generated field defaults (ERROR_CODE__DEFAULT ...), which share value 0.
        if name.isupper() and '__' not in name and isinstance(getattr(cls, name), int):
            names.setdefault(getattr(cls, name), name)
    return names


_RESULT_ERRORS = _constant_names(FollowObject.Result)
_FEEDBACK_STATES = _constant_names(FollowObject.Feedback)
_OUTCOMES = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_CANCELED: 'canceled',
    GoalStatus.STATUS_ABORTED: 'aborted',
}


class _RosLog(Log):

    def __init__(self, logger):
        self._logger = logger

    def info(self, message: str) -> None:
        self._logger.info(message)

    def warning(self, message: str) -> None:
        self._logger.warning(message)


class _FollowObjectClient(FollowActionPort):
    """FollowActionPort over rclpy's ActionClient; reports back to the executor."""

    def __init__(self, node: Node, action_name: str, pose_topic: str):
        self._node = node
        self._client = ActionClient(node, FollowObject, action_name)
        self._pose_topic = pose_topic
        self.executor: Optional[FollowExecutor] = None
        self._goal_handle = None
        self._cancel_pending = False

    def ready(self) -> bool:
        return self._client.server_is_ready()

    def send_goal(self) -> None:
        goal = FollowObject.Goal()
        goal.pose_topic = self._pose_topic
        goal.max_duration = DurationMsg(sec=0, nanosec=0)  # until cancelled
        self._goal_handle = None
        self._cancel_pending = False
        future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def cancel(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        else:
            self._cancel_pending = True  # not accepted yet: cancel as soon as it is

    def active(self) -> bool:
        return self._goal_handle is not None

    def destroy(self) -> None:
        self._client.destroy()

    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as e:  # noqa: B902 - the server went away mid-request
            self.executor.on_goal_rejected(str(e))
            return
        if not handle.accepted:
            self.executor.on_goal_rejected('goal rejected')
            return
        self._goal_handle = handle
        self.executor.on_goal_accepted()
        if self._cancel_pending:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, msg) -> None:
        state = msg.feedback.state
        self.executor.on_feedback(_FEEDBACK_STATES.get(state, str(state)))

    def _on_result(self, future) -> None:
        self._goal_handle = None
        try:
            response = future.result()
        except Exception as e:  # noqa: B902
            self.executor.on_result('aborted', str(e))
            return
        result = response.result
        detail = ''
        if result.error_code:
            name = _RESULT_ERRORS.get(result.error_code, str(result.error_code))
            detail = f'{name}: {result.error_msg}' if result.error_msg else name
        self.executor.on_result(_OUTCOMES.get(response.status, 'aborted'), detail)


class FollowMeNode(Node):

    def __init__(self):
        super().__init__('follow_me')
        self.declare_parameter('autostart', True)
        self.declare_parameter('action_name', 'follow_object')
        self.declare_parameter('target_timeout', 1.0)
        self.declare_parameter('update_rate', 5.0)
        self._executor_uc: Optional[FollowExecutor] = None
        self._client: Optional[_FollowObjectClient] = None
        self._subs = []
        self._services = []
        self._timer = None
        self._pose_pub = None
        self._status_pub = None
        self._last_status = ''

    # --- lifecycle ------------------------------------------------------------------------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        p = self.get_parameter
        if p('target_timeout').value <= 0.0:
            self.get_logger().error('target_timeout must be positive')
            return TransitionCallbackReturn.FAILURE
        self._config = ExecutorConfig(target_timeout=p('target_timeout').value)
        self._period = 1.0 / max(p('update_rate').value, 1.0)
        # The goal names the pose topic absolutely: the server runs in its own node.
        self._pose_topic = self.resolve_topic_name('follow_me/target_pose')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._client = _FollowObjectClient(
            self, self.get_parameter('action_name').value, self._pose_topic)
        self._executor_uc = FollowExecutor(self._config, self._client, _RosLog(self.get_logger()))
        self._client.executor = self._executor_uc

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pose_pub = self.create_publisher(PoseStamped, 'follow_me/target_pose', 1)
        self._status_pub = self.create_publisher(String, 'follow_me/status', latched)
        self._subs = [
            self.create_subscription(TrackedPerson, 'tracked_person', self._on_target, 10),
            # drive_mode and mission_state are latched by their publishers.
            self.create_subscription(DriveMode, 'drive_mode',
                                     lambda m: self._executor_uc.on_drive_mode(m.mode), latched),
            self.create_subscription(MissionState, 'mission_state',
                                     lambda m: self._executor_uc.on_mission_state(m.state),
                                     latched),
        ]
        self._services = [
            self.create_service(Trigger, 'follow_me/start', self._on_start),
            self.create_service(Trigger, 'follow_me/stop', self._on_stop),
        ]
        # Also keeps the executor waking up, so SIGINT is noticed (no-timer nodes did not).
        self._timer = self.create_timer(self._period, self._on_timer)
        self._last_status = ''
        self._publish_status()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.cancel_following('follow_me deactivated')
        for sub in self._subs:
            self.destroy_subscription(sub)
        for srv in self._services:
            self.destroy_service(srv)
        if self._timer is not None:
            self.destroy_timer(self._timer)
        for pub in (self._pose_pub, self._status_pub):
            if pub is not None:
                self.destroy_publisher(pub)
        if self._client is not None:
            self._client.destroy()
        self._subs, self._services = [], []
        self._timer = self._pose_pub = self._status_pub = None
        self._client = None
        self._executor_uc = None
        return TransitionCallbackReturn.SUCCESS

    def cancel_following(self, reason: str) -> bool:
        """Cancel a running goal (deactivate, Ctrl-C). True if a cancel was sent."""
        if self._executor_uc is None:
            return False
        sent, _ = self._executor_uc.stop(reason)
        return sent and self._client is not None and self._client.active()

    # --- callbacks ------------------------------------------------------------------------------

    def _on_target(self, msg: TrackedPerson):
        self._executor_uc.on_target(msg.state, time.monotonic())
        if not forwards_target(msg.state):
            return  # no pose: the Following server's detection_timeout and search take over
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.position
        pose.pose.orientation.w = 1.0  # skip_orientation: the server faces the position
        self._pose_pub.publish(pose)

    def _on_start(self, request, response):
        response.success, response.message = self._executor_uc.start(time.monotonic())
        self._publish_status()
        return response

    def _on_stop(self, request, response):
        response.success, response.message = self._executor_uc.stop()
        self._publish_status()
        return response

    def _on_timer(self):
        self._executor_uc.tick(time.monotonic())
        self._publish_status()

    def _publish_status(self):
        if self._executor_uc is None or self._status_pub is None:
            return
        text = self._executor_uc.status()
        if text != self._last_status:
            self._status_pub.publish(String(data=text))
            self._last_status = text


def _sigterm_as_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # rclpy's own SIGINT handler shuts the context down at once, before a goal could be
    # cancelled: take SIGINT (and SIGTERM) ourselves, cancel, spin briefly to send it, then quit.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _sigterm_as_interrupt)
    node = FollowMeNode()
    if node.get_parameter('autostart').value:
        if node.trigger_configure() == TransitionCallbackReturn.SUCCESS:
            node.trigger_activate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        if node.cancel_following('follow_me shut down'):
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
