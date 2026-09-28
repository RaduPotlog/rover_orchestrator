// Copyright 2025 Mechatronics Academy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rover_mission_manager/infrastructure/nav2_navigation_adapter.hpp"

#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <utility>

namespace rover_mission_manager::infrastructure
{

using domain::ports::NavigationResult;

Nav2NavigationAdapter::Nav2NavigationAdapter(
    rclcpp::Node * node,
    const std::string & action_name,
    std::string goal_frame_id,
    std::chrono::duration<double> server_timeout)
: node_(node),
  goal_frame_id_(std::move(goal_frame_id)),
  server_timeout_(server_timeout),
  result_(NavigationResult::kIdle)
{
    client_ = rclcpp_action::create_client<NavigateToPose>(node_, action_name);
}

bool Nav2NavigationAdapter::goTo(const domain::Waypoint & waypoint)
{
    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(server_timeout_);

    if (!client_->wait_for_action_server(timeout)) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "navigate_to_pose action server unavailable.");
        return false;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = goal_frame_id_;
    goal.pose.header.stamp = node_->now();
    goal.pose.pose.position.x = waypoint.x;
    goal.pose.pose.position.y = waypoint.y;
    goal.pose.pose.orientation.z = std::sin(waypoint.yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(waypoint.yaw / 2.0);

    std::uint64_t generation = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        generation = ++generation_;
        goal_handle_.reset();
        endDrainLocked();
        result_ = NavigationResult::kPending;
    }

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
        [this, generation](const GoalHandle::SharedPtr & handle) {
            goalResponseCb(handle, generation);
        };
    options.result_callback =
        [this, generation](const GoalHandle::WrappedResult & wrapped) {
            resultCb(wrapped, generation);
        };

    client_->async_send_goal(goal, options);

    return true;
}

void Nav2NavigationAdapter::goalResponseCb(
    const GoalHandle::SharedPtr & goal_handle, std::uint64_t generation)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (generation == generation_) {
            goal_handle_ = goal_handle;

            if (goal_handle == nullptr) {
                result_ = NavigationResult::kFailed;
            }
        } else if (draining_ && generation == draining_generation_) {
            if (goal_handle == nullptr) {
                // Rejected: there is nothing for Nav 2 to wind down.
                endDrainLocked();
                return;
            }
            // Accepted after cancel(): keep the handle so its result ends the drain.
            draining_handle_ = goal_handle;
        } else if (goal_handle == nullptr) {
            return;
        }
    }

    if (goal_handle == nullptr) {
        RCLCPP_WARN(node_->get_logger(), "navigate_to_pose goal was rejected by the server.");
        return;
    }

    if (generation != currentGeneration()) {
        // Cancelled or replaced before the server answered, when there was no handle to
        // cancel yet. Cancel it now, or Nav 2 keeps driving a goal nobody is waiting on.
        client_->async_cancel_goal(goal_handle);
    }
}

void Nav2NavigationAdapter::resultCb(
    const GoalHandle::WrappedResult & wrapped_result, std::uint64_t generation)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (generation != generation_) {
        // A goal we already cancelled or replaced. Its ABORTED or CANCELED says nothing
        // about the goal in flight now, if any - but it does mean Nav 2 is done with it.
        if (draining_ && generation == draining_generation_) {
            endDrainLocked();
        }
        return;
    }

    goal_handle_.reset();

    switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            result_ = NavigationResult::kReached;
            break;

        case rclcpp_action::ResultCode::CANCELED:
            // Not ours -- ours bump the generation first -- so someone else cancelled the goal
            // (a second client, or an operator on the CLI). The mission did not reach the
            // waypoint, and treating it as idle would silently re-dispatch it.
            result_ = NavigationResult::kFailed;
            break;

        case rclcpp_action::ResultCode::ABORTED:
        case rclcpp_action::ResultCode::UNKNOWN:
        default:
            result_ = NavigationResult::kFailed;
            break;
    }
}

void Nav2NavigationAdapter::cancel()
{
    GoalHandle::SharedPtr handle;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto cancelled_generation = generation_++;
        handle = std::move(goal_handle_);
        goal_handle_.reset();

        // Only a goal still in flight needs winding down; a finished one, or a second cancel()
        // during a drain, leaves nothing new for Nav 2 to process.
        if (result_ == NavigationResult::kPending && !draining_) {
            draining_ = true;
            draining_generation_ = cancelled_generation;
            draining_since_ = std::chrono::steady_clock::now();
            draining_handle_ = handle;   // null when the goal response is still on its way
        } else if (!draining_) {
            result_ = NavigationResult::kIdle;
        }
    }

    if (handle != nullptr) {
        client_->async_cancel_goal(handle);
    }
}

void Nav2NavigationAdapter::endDrainLocked() const
{
    if (!draining_) {
        return;
    }

    draining_ = false;
    draining_handle_.reset();

    // Idle only if nothing was dispatched since: goTo() ends a drain itself.
    if (generation_ != draining_generation_ && goal_handle_ == nullptr &&
        result_ == NavigationResult::kPending)
    {
        result_ = NavigationResult::kIdle;
    }
}

std::uint64_t Nav2NavigationAdapter::currentGeneration() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return generation_;
}

NavigationResult Nav2NavigationAdapter::result() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (draining_ && std::chrono::steady_clock::now() - draining_since_ > kDrainTimeout) {
        // No result for the cancelled goal (Nav 2 restarting, a lost message): stop waiting
        // rather than hold the mission forever. Worst case the next goal is rejected, which
        // fails the mission visibly.
        RCLCPP_WARN(
            node_->get_logger(),
            "No result for the cancelled navigate_to_pose goal after %lld s; dispatching anyway.",
            static_cast<long long>(kDrainTimeout.count()));
        endDrainLocked();
    }

    return result_.load();
}

}  // namespace rover_mission_manager::infrastructure
