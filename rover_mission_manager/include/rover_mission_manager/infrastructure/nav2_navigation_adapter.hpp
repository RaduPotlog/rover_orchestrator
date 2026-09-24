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
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "rover_mission_manager/domain/ports/navigation_port.hpp"

namespace rover_mission_manager::infrastructure
{

/**
 * @brief NavigationPort backed by Nav 2's `navigate_to_pose` action.
 *
 * goTo() never waits for the server to accept a goal, so the goal handle only exists once
 * the goal response arrives, possibly several ticks later. Every goal therefore carries a
 * generation number. cancel() and each new goTo() start a new generation, and a response or
 * result from an older one is stale: a stale goal that the server accepted is cancelled on
 * arrival, and a stale result is dropped. Without this, a hold or abort decided before the
 * server answered cancelled nothing, and Nav 2 kept driving to a goal the mission had
 * already given up.
 */
class Nav2NavigationAdapter : public domain::ports::NavigationPort
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    Nav2NavigationAdapter(
        rclcpp_lifecycle::LifecycleNode * node,
        const std::string & action_name,
        std::string goal_frame_id,
        std::chrono::duration<double> server_timeout);

    /** @brief Cancels the goal in flight, so a torn-down manager never leaves Nav 2 driving. */
    ~Nav2NavigationAdapter() override;

    domain::ports::DispatchResult goTo(const domain::Waypoint & waypoint) override;
    void cancel() override;
    domain::ports::NavigationResult result() const override;

private:
    void goalResponseCb(std::uint64_t generation, const GoalHandle::SharedPtr & goal_handle);
    void resultCb(std::uint64_t generation, const GoalHandle::WrappedResult & wrapped_result);

    rclcpp_lifecycle::LifecycleNode * node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    std::string goal_frame_id_;
    // Grace period for the action server to come up before goTo() reports kUnreachable.
    std::chrono::duration<double> server_timeout_;
    // When goTo() first found the server down; reset by a dispatch or a cancel. Only touched
    // from the manager's timer thread.
    std::optional<std::chrono::steady_clock::time_point> unavailable_since_;

    // Written from action-client callbacks, read from the manager's timer thread.
    std::atomic<domain::ports::NavigationResult> result_;

    // Guarded by goal_mutex_. goal_handle_ belongs to the current generation, and is null
    // until the server accepts that goal and again once it finishes or is cancelled.
    std::uint64_t generation_ = 0;
    GoalHandle::SharedPtr goal_handle_;
    mutable std::mutex goal_mutex_;
};

}  // namespace rover_mission_manager::infrastructure

#endif  // ROVER_MISSION_MANAGER_INFRASTRUCTURE_NAV2_NAVIGATION_ADAPTER_HPP_
