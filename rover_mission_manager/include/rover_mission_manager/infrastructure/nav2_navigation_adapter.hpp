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

#ifndef ROVER_MISSION_MANAGER_INFRASTRUCTURE_NAV2_NAVIGATION_ADAPTER_HPP_
#define ROVER_MISSION_MANAGER_INFRASTRUCTURE_NAV2_NAVIGATION_ADAPTER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "rover_mission_manager/domain/ports/navigation_port.hpp"

namespace rover_mission_manager::infrastructure
{

/** @brief NavigationPort backed by Nav 2's `navigate_to_pose` action. */
class Nav2NavigationAdapter : public domain::ports::NavigationPort
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    Nav2NavigationAdapter(
        rclcpp::Node * node,
        const std::string & action_name,
        std::string goal_frame_id,
        std::chrono::duration<double> server_timeout);

    domain::ports::DispatchResult goTo(const domain::Waypoint & waypoint) override;
    void cancel() override;
    domain::ports::NavigationResult result() const override;

private:
    void goalResponseCb(const GoalHandle::SharedPtr & goal_handle);
    void resultCb(const GoalHandle::WrappedResult & wrapped_result);

    rclcpp::Node * node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    std::string goal_frame_id_;
    // Grace period for the action server to come up before goTo() reports kUnreachable.
    std::chrono::duration<double> server_timeout_;
    // When goTo() first found the server down; reset by a dispatch or a cancel. Only touched
    // from the manager's timer thread.
    std::optional<std::chrono::steady_clock::time_point> unavailable_since_;

    // Written from action-client callbacks, read from the manager's timer thread.
    std::atomic<domain::ports::NavigationResult> result_;
    GoalHandle::SharedPtr goal_handle_;
    mutable std::mutex goal_handle_mutex_;
};

}  // namespace rover_mission_manager::infrastructure

#endif  // ROVER_MISSION_MANAGER_INFRASTRUCTURE_NAV2_NAVIGATION_ADAPTER_HPP_
