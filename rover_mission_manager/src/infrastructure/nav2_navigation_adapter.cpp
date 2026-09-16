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

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
        [this](const GoalHandle::SharedPtr & handle) { goalResponseCb(handle); };
    options.result_callback =
        [this](const GoalHandle::WrappedResult & wrapped) { resultCb(wrapped); };

    result_ = NavigationResult::kPending;
    client_->async_send_goal(goal, options);

    return true;
}

void Nav2NavigationAdapter::goalResponseCb(const GoalHandle::SharedPtr & goal_handle)
{
    {
        std::lock_guard<std::mutex> lock(goal_handle_mutex_);
        goal_handle_ = goal_handle;
    }

    if (goal_handle == nullptr) {
        RCLCPP_WARN(node_->get_logger(), "navigate_to_pose goal was rejected by the server.");
        result_ = NavigationResult::kFailed;
    }
}

void Nav2NavigationAdapter::resultCb(const GoalHandle::WrappedResult & wrapped_result)
{
    {
        std::lock_guard<std::mutex> lock(goal_handle_mutex_);
        goal_handle_.reset();
    }

    switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            result_ = NavigationResult::kReached;
            break;

        case rclcpp_action::ResultCode::CANCELED:
            // A cancel is always our own doing (hold, abort or a new mission), and the use
            // case has already moved on. Reporting kFailed here would abort that mission.
            result_ = NavigationResult::kIdle;
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
        std::lock_guard<std::mutex> lock(goal_handle_mutex_);
        handle = goal_handle_;
    }

    if (handle != nullptr) {
        client_->async_cancel_goal(handle);
    }

    result_ = NavigationResult::kIdle;
}

NavigationResult Nav2NavigationAdapter::result() const { return result_.load(); }

}  // namespace rover_mission_manager::infrastructure
