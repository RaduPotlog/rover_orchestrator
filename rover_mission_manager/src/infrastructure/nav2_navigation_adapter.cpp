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
#include <exception>
#include <mutex>
#include <string>
#include <utility>

namespace rover_mission_manager::infrastructure
{

using domain::ports::DispatchResult;
using domain::ports::NavigationResult;

Nav2NavigationAdapter::Nav2NavigationAdapter(
    rclcpp_lifecycle::LifecycleNode * node,
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

Nav2NavigationAdapter::~Nav2NavigationAdapter()
{
    // Normally a no-op: the manager cancels on deactivate and on shutdown. Best effort here,
    // since the context may already be gone, and a destructor must not throw.
    try {
        cancel();
    } catch (const std::exception & e) {
        RCLCPP_WARN(
            node_->get_logger(), "Could not cancel the navigate_to_pose goal: %s", e.what());
    }
}

DispatchResult Nav2NavigationAdapter::goTo(const domain::Waypoint & waypoint)
{
    // Called from the manager's timer, which also ticks the tree and serves this node's
    // subscriptions, so it never waits for the server: it reports kNotReady and the use case
    // retries on the next tick until the grace period runs out.
    if (!client_->action_server_is_ready()) {
        const auto now = std::chrono::steady_clock::now();

        if (!unavailable_since_) {
            unavailable_since_ = now;
        }

        if (now - *unavailable_since_ >= server_timeout_) {
            RCLCPP_WARN(
                node_->get_logger(), "navigate_to_pose action server unavailable for %.1f s.",
                server_timeout_.count());
            unavailable_since_.reset();
            return DispatchResult::kUnreachable;
        }

        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "Waiting for the navigate_to_pose action server.");
        return DispatchResult::kNotReady;
    }

    unavailable_since_.reset();

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = goal_frame_id_;
    goal.pose.header.stamp = node_->now();
    goal.pose.pose.position.x = waypoint.x;
    goal.pose.pose.position.y = waypoint.y;
    goal.pose.pose.orientation.z = std::sin(waypoint.yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(waypoint.yaw / 2.0);

    // A new goal supersedes the previous one. Nav 2 preempts it on the server side; here it
    // only has to stop counting, so its late response or result cannot overwrite this one's.
    std::uint64_t generation;
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        generation = ++generation_;
        goal_handle_.reset();
    }

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
        [this, generation](const GoalHandle::SharedPtr & handle) {
            goalResponseCb(generation, handle);
        };
    options.result_callback =
        [this, generation](const GoalHandle::WrappedResult & wrapped) {
            resultCb(generation, wrapped);
        };

    result_ = NavigationResult::kPending;
    client_->async_send_goal(goal, options);

    return DispatchResult::kDispatched;
}

void Nav2NavigationAdapter::goalResponseCb(
    std::uint64_t generation, const GoalHandle::SharedPtr & goal_handle)
{
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);

        if (generation != generation_) {
            // cancel() (or a newer goal) ran before the server answered. The mission has
            // already moved on, so a goal the server did accept must not be left driving.
            if (goal_handle != nullptr) {
                RCLCPP_INFO(
                    node_->get_logger(),
                    "Cancelling a navigate_to_pose goal that was accepted after it was "
                    "abandoned.");
                client_->async_cancel_goal(goal_handle);
            }
            return;
        }

        goal_handle_ = goal_handle;
    }

    if (goal_handle == nullptr) {
        RCLCPP_WARN(node_->get_logger(), "navigate_to_pose goal was rejected by the server.");
        result_ = NavigationResult::kFailed;
    }
}

void Nav2NavigationAdapter::resultCb(
    std::uint64_t generation, const GoalHandle::WrappedResult & wrapped_result)
{
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);

        // The outcome of an abandoned goal, including the CANCELED our own cancel() causes.
        // The use case has already moved on and may have a newer goal in flight.
        if (generation != generation_) {
            return;
        }

        goal_handle_.reset();
    }

    switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            result_ = NavigationResult::kReached;
            break;

        case rclcpp_action::ResultCode::CANCELED:
            // Our own cancels start a new generation and never get here, so this goal was
            // cancelled by another client (an RViz panel, a CLI). Reporting kIdle would make
            // the use case send the same waypoint straight back to Nav 2, overriding the
            // operator. Fail the mission instead.
            RCLCPP_WARN(
                node_->get_logger(),
                "navigate_to_pose goal was cancelled by another client; failing the mission.");
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
        std::lock_guard<std::mutex> lock(goal_mutex_);
        // Invalidates the goal in flight even when its handle has not arrived yet:
        // goalResponseCb then cancels it on arrival.
        ++generation_;
        handle = std::move(goal_handle_);
        goal_handle_.reset();
    }

    if (handle != nullptr) {
        client_->async_cancel_goal(handle);
    }

    unavailable_since_.reset();
    result_ = NavigationResult::kIdle;
}

NavigationResult Nav2NavigationAdapter::result() const { return result_.load(); }

}  // namespace rover_mission_manager::infrastructure
